#!/usr/bin/env python3
"""state_machine: coordinador de la navegación de Parte B (consigna 1.11) y, fusionado,
el controlador de seguimiento (1.6), el ángulo final (1.7), el re-planeo (1.8) y la
evasión reactiva (1.9). El planeo global (1.5) lo hace `global_planner`, al que esta
máquina le pide rutas por /plan_request.

Máquina de estados (ver docs/context/04_arquitectura_parte_b.md):

    IDLE --initialpose--> LOCALIZING --pose estable--> WAITING_GOAL
    WAITING_GOAL --goal_pose--> PLANNING --plan OK--> FOLLOWING
    FOLLOWING --llegó a la posición--> ALIGNING_ANGLE --orientación OK--> GOAL_REACHED
    FOLLOWING --obstáculo no mapeado--> AVOIDING --despejado--> PLANNING (re-planea)
    FOLLOWING/ALIGNING/GOAL_REACHED --goal_pose nuevo--> PLANNING (re-planea, 1.8)
    GOAL_REACHED --> WAITING_GOAL

Entradas: goals, plan, obstacle/monitor health, /scan, /mcl_pose y TF map->base.
Salidas:  /cmd_vel, /nav_state (String, para RViz/consola), /plan_request.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, Empty
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

from tp_b_navigation.safety_gates import (
    SafetyGateConfig,
    localization_gate,
    monitor_gate,
    scan_gate,
)
from tp_b_navigation.utils import angle_diff, yaw_from_quaternion


# Estados
IDLE = 'IDLE'
LOCALIZING = 'LOCALIZING'
WAITING_GOAL = 'WAITING_GOAL'
PLANNING = 'PLANNING'
FOLLOWING = 'FOLLOWING'
AVOIDING = 'AVOIDING'
ALIGNING_ANGLE = 'ALIGNING_ANGLE'
GOAL_REACHED = 'GOAL_REACHED'


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        # --- parámetros de control ---
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('v_max', 0.18)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('lookahead', 0.30)        # carrot pure-pursuit (m)
        self.declare_parameter('k_w', 1.6)               # ganancia angular
        self.declare_parameter('slow_angle', 0.8)        # err. angular que frena el avance (rad)
        self.declare_parameter('goal_xy_tol', 0.12)      # tolerancia de posición (m)
        self.declare_parameter('goal_yaw_tol', 0.08)     # tolerancia de ángulo final (rad)
        self.declare_parameter('localize_settle', 1.5)   # tiempo con TF estable -> localizado (s)
        self.declare_parameter('plan_timeout', 4.0)      # espera de /plan (s)
        self.declare_parameter('avoid_time', 1.5)        # duración de la maniobra de evasión (s)
        self.declare_parameter('avoid_w', 0.9)           # giro de evasión (rad/s)
        self.declare_parameter('avoid_back_v', -0.07)    # retroceso de evasión (m/s)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('enable_safety_gates', False)
        self.declare_parameter('max_mcl_pose_age', 1.0)
        self.declare_parameter('max_scan_age', 1.0)
        self.declare_parameter('max_monitor_age', 1.0)
        self.declare_parameter('max_position_covariance', 0.25)
        self.declare_parameter('max_yaw_covariance', 0.5)

        g = self.get_parameter
        self.v_max = float(g('v_max').value)
        self.w_max = float(g('w_max').value)
        self.lookahead = float(g('lookahead').value)
        self.k_w = float(g('k_w').value)
        self.slow_angle = float(g('slow_angle').value)
        self.xy_tol = float(g('goal_xy_tol').value)
        self.yaw_tol = float(g('goal_yaw_tol').value)
        self.localize_settle = float(g('localize_settle').value)
        self.plan_timeout = float(g('plan_timeout').value)
        self.avoid_time = float(g('avoid_time').value)
        self.avoid_w = float(g('avoid_w').value)
        self.avoid_back_v = float(g('avoid_back_v').value)
        self.global_frame = g('global_frame').value
        self.base_frame = g('base_frame').value
        self.safety_config = SafetyGateConfig(
            enabled=bool(g('enable_safety_gates').value),
            max_mcl_pose_age=float(g('max_mcl_pose_age').value),
            max_scan_age=float(g('max_scan_age').value),
            max_monitor_age=float(g('max_monitor_age').value),
            max_position_covariance=float(g('max_position_covariance').value),
            max_yaw_covariance=float(g('max_yaw_covariance').value),
        )

        # --- estado interno ---
        self.state = IDLE
        self.goal = None            # PoseStamped objetivo actual
        self.goal_source = None     # 'manual' o 'mission'
        self.new_goal = False       # llegó un goal nuevo (dispara re-planeo)
        self.path = None            # lista [(x,y)] de la ruta actual
        self.path_idx = 0
        self.obstacle = False
        self.t_enter = self.now()   # tiempo de entrada al estado actual
        self.tf_ok_since = None     # primer instante con TF map->base disponible
        self.last_mcl_pose_stamp = None
        self.last_mcl_covariance = None
        self.last_scan_stamp = None
        self.last_monitor_stamp = None
        self.monitor_healthy = None
        self.last_safety_reason = None

        # --- pub/sub ---
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.initialpose_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(PoseStamped, '/mission_goal', self.mission_goal_cb, 10)
        self.create_subscription(Empty, '/mission_cancel', self.mission_cancel_cb, 10)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(Bool, '/plan_status', self.plan_status_cb, 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_cb, 10)
        self.create_subscription(
            Bool, '/obstacle_monitor_healthy', self.monitor_health_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/mcl_pose',
                                 self.mcl_pose_cb, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/nav_state', 10)
        self.pub_request = self.create_publisher(PoseStamped, '/plan_request', 10)
        self.pub_result = self.create_publisher(String, '/navigation_result', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.plan_ok = None  # None=esperando, True/False=resultado del último request

        rate = float(g('control_rate').value)
        self.create_timer(1.0 / rate, self.loop)
        self.create_timer(0.5, self.publish_state)  # estado a RViz/consola

        self.get_logger().info('state_machine en IDLE. Esperá "2D Pose Estimate".')

    # --------------------------------------------------------------- utilidades
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'[FSM] {self.state} -> {s}')
            self.state = s
            self.t_enter = self.now()

    def robot_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        return (tr.transform.translation.x, tr.transform.translation.y,
                yaw_from_quaternion(tr.transform.rotation))

    def stop(self):
        self.pub_cmd.publish(Twist())

    def drive(self, v, w):
        t = Twist()
        t.linear.x = float(max(-self.v_max, min(self.v_max, v)))
        t.angular.z = float(max(-self.w_max, min(self.w_max, w)))
        self.pub_cmd.publish(t)

    def _stamp_to_seconds(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def localization_block_reason(self):
        return localization_gate(
            now_s=self.now(),
            last_pose_stamp_s=self.last_mcl_pose_stamp,
            covariance=self.last_mcl_covariance,
            config=self.safety_config,
        )

    def localization_safe(self):
        reason = self.localization_block_reason()
        if reason is None:
            self.last_safety_reason = None
            return True
        if reason != self.last_safety_reason:
            self.get_logger().warn(
                f'Safety gate bloquea movimiento autonomo: {reason}')
            self.last_safety_reason = reason
        return False

    def navigation_block_reason(self):
        now = self.now()
        reason = localization_gate(
            now, self.last_mcl_pose_stamp, self.last_mcl_covariance,
            self.safety_config)
        if reason is None:
            reason = scan_gate(now, self.last_scan_stamp, self.safety_config)
        if reason is None:
            reason = monitor_gate(
                now, self.last_monitor_stamp, self.monitor_healthy,
                self.safety_config)
        return reason

    def navigation_safe(self):
        reason = self.navigation_block_reason()
        if reason is None:
            self.last_safety_reason = None
            return True
        if reason != self.last_safety_reason:
            self.get_logger().warn(
                f'Safety gate bloquea movimiento autonomo: {reason}')
            self.last_safety_reason = reason
        return False

    # --------------------------------------------------------------- callbacks
    def initialpose_cb(self, msg):
        # nueva localización inicial: reiniciar a LOCALIZING
        self.tf_ok_since = None
        self.goal = None
        self.goal_source = None
        self.path = None
        self.set_state(LOCALIZING)

    def goal_cb(self, msg: PoseStamped):
        self._accept_goal(msg, 'manual')

    def mission_goal_cb(self, msg: PoseStamped):
        if self.goal_source == 'mission' and self.goal is not None:
            self._publish_result('PREEMPTED')
        self._accept_goal(msg, 'mission')

    def _accept_goal(self, msg, source):
        self.goal = msg
        self.goal_source = source
        self.new_goal = True
        self.get_logger().info(
            f'Goal {source} recibido: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}).')

    def mission_cancel_cb(self, _msg):
        if self.goal_source == 'mission' and self.goal is not None:
            self._publish_result('PREEMPTED')
        self.goal = None
        self.goal_source = None
        self.new_goal = False
        self.path = None
        self.stop()
        if self.state not in (IDLE, LOCALIZING):
            self.set_state(WAITING_GOAL)

    def plan_cb(self, msg: Path):
        if msg.poses:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.path_idx = 0
            self.plan_ok = True
        else:
            self.path = None
            self.plan_ok = False

    def plan_status_cb(self, msg: Bool):
        if not msg.data:
            self.plan_ok = False

    def obstacle_cb(self, msg: Bool):
        self.obstacle = bool(msg.data)

    def monitor_health_cb(self, msg: Bool):
        self.last_monitor_stamp = self.now()
        self.monitor_healthy = bool(msg.data)

    def scan_cb(self, msg: LaserScan):
        self.last_scan_stamp = self._stamp_to_seconds(msg.header.stamp)

    def mcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        self.last_mcl_pose_stamp = self._stamp_to_seconds(msg.header.stamp)
        self.last_mcl_covariance = msg.pose.covariance

    # --------------------------------------------------------------- request de plan
    def request_plan(self):
        if self.goal is None:
            return
        self.plan_ok = None
        self.path = None
        if not self.navigation_safe():
            self.stop()
            return
        req = PoseStamped()
        req.header.frame_id = self.global_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.pose = self.goal.pose
        self.pub_request.publish(req)

    # --------------------------------------------------------------- bucle FSM
    def loop(self):
        if (self.safety_config.enabled
                and self.state in (PLANNING, FOLLOWING, AVOIDING, ALIGNING_ANGLE)
                and not self.navigation_safe()):
            self.stop()
            return

        # re-planeo por goal nuevo (1.8): salvo si todavía estamos localizando
        if self.new_goal and self.state not in (IDLE, LOCALIZING):
            self.new_goal = False
            self.request_plan()
            self.set_state(PLANNING)
            return

        if self.state == IDLE:
            self.stop()

        elif self.state == LOCALIZING:
            self.stop()
            pose = self.robot_pose()
            if pose is not None:
                if self.tf_ok_since is None:
                    self.tf_ok_since = self.now()
                elif self.now() - self.tf_ok_since >= self.localize_settle:
                    self.set_state(WAITING_GOAL)
            else:
                self.tf_ok_since = None

        elif self.state == WAITING_GOAL:
            # Espera pasiva: un goal nuevo lo levanta el manejador de new_goal de arriba.
            self.stop()

        elif self.state == PLANNING:
            self.stop()
            if not self.navigation_safe():
                return
            if self.plan_ok is True and self.path:
                self.set_state(FOLLOWING)
            elif self.plan_ok is False or (self.now() - self.t_enter > self.plan_timeout):
                self.get_logger().warn('No se pudo planear; vuelvo a esperar goal.')
                if self.goal_source == 'mission':
                    result = 'PLAN_FAILED' if self.plan_ok is False else 'TIMEOUT'
                    self._publish_result(result)
                    self.goal = None
                    self.goal_source = None
                self.set_state(WAITING_GOAL)

        elif self.state == FOLLOWING:
            if self.obstacle:
                self.set_state(AVOIDING)
                return
            self.follow()

        elif self.state == AVOIDING:
            self.avoid()

        elif self.state == ALIGNING_ANGLE:
            self.align_final()

        elif self.state == GOAL_REACHED:
            # Objetivo cumplido: limpiar para no re-planear sobre un goal ya alcanzado.
            self.stop()
            if self.goal_source == 'mission':
                self._publish_result('REACHED')
            self.goal = None
            self.goal_source = None
            self.path = None
            self.set_state(WAITING_GOAL)

    # --------------------------------------------------------------- comportamientos
    def follow(self):
        if not self.navigation_safe():
            self.stop()
            return
        pose = self.robot_pose()
        if pose is None or not self.path:
            self.stop()
            return
        rx, ry, rth = pose
        gx, gy = self.path[-1]
        # ¿llegamos a la posición final? -> alinear ángulo
        if math.hypot(gx - rx, gy - ry) <= self.xy_tol:
            self.stop()
            self.set_state(ALIGNING_ANGLE)
            return

        # avanzar el carrot (lookahead) sobre la ruta
        while (self.path_idx < len(self.path) - 1 and
               math.hypot(self.path[self.path_idx][0] - rx,
                          self.path[self.path_idx][1] - ry) < self.lookahead):
            self.path_idx += 1
        cx, cy = self.path[self.path_idx]

        heading = math.atan2(cy - ry, cx - rx)
        err = angle_diff(heading, rth)
        # avanzar más lento cuanto mayor sea el error angular (giro primero, suave)
        v = self.v_max * max(0.0, 1.0 - abs(err) / self.slow_angle)
        w = self.k_w * err
        self.drive(v, w)

    def avoid(self):
        if not self.navigation_safe():
            self.stop()
            return
        if self.robot_pose() is None:
            self.stop()
            return
        # maniobra reactiva: frenar, retroceder y girar un tiempo; luego re-planear
        elapsed = self.now() - self.t_enter
        if elapsed < self.avoid_time:
            self.drive(self.avoid_back_v, self.avoid_w)
        else:
            self.obstacle = False
            self.request_plan()
            self.set_state(PLANNING)

    def align_final(self):
        if not self.navigation_safe():
            self.stop()
            return
        pose = self.robot_pose()
        if pose is None or self.goal is None:
            self.stop()
            return
        _, _, rth = pose
        goal_yaw = yaw_from_quaternion(self.goal.pose.orientation)
        err = angle_diff(goal_yaw, rth)
        if abs(err) <= self.yaw_tol:
            self.stop()
            self.get_logger().info('Objetivo alcanzado (posición + ángulo).')
            self.set_state(GOAL_REACHED)
        else:
            self.drive(0.0, self.k_w * err)

    def publish_state(self):
        self.pub_state.publish(String(data=self.state))

    def _publish_result(self, result):
        self.pub_result.publish(String(data=result))


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
