#!/usr/bin/env python3
"""obstacle_monitor: detecta **obstáculos NO mapeados** (consigna 1.9).

No alcanza con "hay algo cerca en el LIDAR": las paredes del mapa también lo están. La
clave es distinguir lo que YA está en el mapa de lo que NO. Para cada retorno del LIDAR
dentro de un cono frontal y a corta distancia, se proyecta el punto al frame `map`
(usando la TF map->base que cierra el MCL) y se consulta la celda del mapa:

  - si la celda está OCUPADA en el mapa  -> es una pared conocida -> se ignora.
  - si la celda está LIBRE en el mapa     -> es un obstáculo nuevo -> cuenta.

Si hay suficientes puntos "nuevos" cerca y al frente, se publica /obstacle_detected=True.

Entradas: /scan, /map (latcheado), TF map->base_frame.
Salidas: /obstacle_detected, /obstacle_monitor_healthy y /dynamic_obstacles.
"""

import math

import numpy as np

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from tp_b_navigation.dynamic_obstacles import mark_dynamic_obstacles, same_static_occupancy
from tp_b_navigation.safety_gates import (
    SafetyGateConfig,
    localization_gate,
    scan_gate,
)
from tp_b_navigation.scan_projection import transform_scan_points
from tp_b_navigation.utils import yaw_from_quaternion


class ObstacleMonitor(Node):
    def __init__(self):
        super().__init__('obstacle_monitor')

        self.declare_parameter('danger_dist', 0.45)     # alcance de alerta (m)
        self.declare_parameter('cone_halfangle', 0.6)   # semicono frontal (rad) ~34°
        self.declare_parameter('min_points', 3)         # puntos nuevos para disparar
        self.declare_parameter('dynamic_inflation_radius', 0.08)
        # La capa dinámica NO es acumulativa: cada celda marcada "caduca" si no se
        # vuelve a observar dentro de este tiempo. Sin esto, todo retorno no mapeado
        # quedaría grabado para siempre (sellando corredores y, con el ruido del MCL,
        # engordando los conos hasta volver el objetivo inalcanzable). El TTL se elige
        # > a la maniobra de evasión para que A* alcance a rodear el obstáculo antes
        # de que se borre.
        self.declare_parameter('dynamic_ttl', 4.0)       # vida de una celda dinámica (s)
        # El cono rojo es un objeto NO mapeado: sin esto, el monitor lo marcaría
        # como obstáculo dinámico y sellaría su entorno, bloqueando la aproximación.
        self.declare_parameter('cone_clear_radius', 0.30)  # disco eximido (m)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('enable_safety_gates', False)
        self.declare_parameter('max_mcl_pose_age', 1.0)
        self.declare_parameter('max_scan_age', 1.0)
        self.declare_parameter('max_position_covariance', 0.25)
        self.declare_parameter('max_yaw_covariance', 0.5)

        self.danger = float(self.get_parameter('danger_dist').value)
        self.cone = float(self.get_parameter('cone_halfangle').value)
        self.min_pts = int(self.get_parameter('min_points').value)
        self.dynamic_radius = float(self.get_parameter('dynamic_inflation_radius').value)
        self.dynamic_ttl = float(self.get_parameter('dynamic_ttl').value)
        self.cone_clear_radius = float(self.get_parameter('cone_clear_radius').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.safety_config = SafetyGateConfig(
            enabled=bool(self.get_parameter('enable_safety_gates').value),
            max_mcl_pose_age=float(self.get_parameter('max_mcl_pose_age').value),
            max_scan_age=float(self.get_parameter('max_scan_age').value),
            max_position_covariance=float(
                self.get_parameter('max_position_covariance').value),
            max_yaw_covariance=float(self.get_parameter('max_yaw_covariance').value),
        )

        self.map = None
        self.res = None
        self.ox = self.oy = 0.0
        self.W = self.H = 0
        self.occ = None  # bool [H,W] ocupado en el mapa
        self.dynamic = None
        self.last_seen = None  # float [H,W]: instante (s) de última observación por celda
        self.exempt = None  # bool [H,W]: celdas eximidas (cono rojo confirmado)
        self.last_mcl_pose_stamp = None
        self.last_mcl_covariance = None
        self.last_safety_reason = None

        qos_latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_latched)
        self.create_subscription(LaserScan, '/scan', self.scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/mcl_pose',
                                 self.mcl_pose_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/red_cone_pose',
                                 self.cone_cb, 10)
        self.pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.health_pub = self.create_publisher(
            Bool, '/obstacle_monitor_healthy', 10)
        self.dynamic_pub = self.create_publisher(
            OccupancyGrid, '/dynamic_obstacles', qos_latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scans_skipped_no_tf = 0
        self.points_rejected_invalid_ranges = 0

        self.get_logger().info(
            f'obstacle_monitor activo (danger={self.danger} m, cono=±{math.degrees(self.cone):.0f}°).')

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.W = msg.info.width
        self.H = msg.info.height
        occ = (np.array(msg.data, dtype=np.int16).reshape(self.H, self.W) == 100)
        if same_static_occupancy(self.occ, occ):
            return
        self.occ = occ
        self.dynamic = np.zeros((self.H, self.W), dtype=bool)
        # -inf efectivo: ninguna celda está "vista" hasta el primer barrido.
        self.last_seen = np.full((self.H, self.W), -1e9, dtype=float)
        self.exempt = np.zeros((self.H, self.W), dtype=bool)
        self._publish_dynamic()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _warn_safety_once(self, reason):
        if reason == self.last_safety_reason:
            return
        self.get_logger().warn(
            f'Safety gate bloquea insercion de obstaculos dinamicos: {reason}')
        self.last_safety_reason = reason

    def _safety_block_reason(self, scan):
        now = self._now()
        scan_stamp = self._stamp_to_seconds(scan.header.stamp)
        reason = scan_gate(now, scan_stamp, self.safety_config)
        if reason is not None:
            return reason
        return localization_gate(
            now_s=now,
            last_pose_stamp_s=self.last_mcl_pose_stamp,
            covariance=self.last_mcl_covariance,
            config=self.safety_config,
        )

    def mcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        self.last_mcl_pose_stamp = self._stamp_to_seconds(msg.header.stamp)
        self.last_mcl_covariance = msg.pose.covariance

    @staticmethod
    def _transform_to_planar_pose(transform):
        tr = transform.transform
        return (
            tr.translation.x,
            tr.translation.y,
            yaw_from_quaternion(tr.rotation),
        )

    @staticmethod
    def _declared_beam_count(scan):
        if scan.angle_increment <= 0.0:
            return 0
        span = scan.angle_max - scan.angle_min
        if span < 0.0:
            return 0
        declared = int(math.floor(span / scan.angle_increment + 1e-9)) + 1
        return min(len(scan.ranges), declared)

    def _lookup_scan_poses(self, scan):
        sensor_frame = scan.header.frame_id or self.base_frame
        stamp = rclpy.time.Time.from_msg(scan.header.stamp)
        base_tf = self.tf_buffer.lookup_transform(
            self.base_frame, sensor_frame, stamp)
        map_tf = self.tf_buffer.lookup_transform(
            self.global_frame, sensor_frame, stamp)
        return (
            self._transform_to_planar_pose(base_tf),
            self._transform_to_planar_pose(map_tf),
        )

    def scan_cb(self, scan: LaserScan):
        if self.occ is None:
            self.health_pub.publish(Bool(data=False))
            return
        reason = self._safety_block_reason(scan)
        if reason is not None:
            self.pub.publish(Bool(data=False))
            self.health_pub.publish(Bool(data=False))
            self._warn_safety_once(reason)
            return
        self.last_safety_reason = None
        try:
            base_from_sensor, map_from_sensor = self._lookup_scan_poses(scan)
        except Exception as exc:  # noqa: BLE001
            self.scans_skipped_no_tf += 1
            self.health_pub.publish(Bool(data=False))
            self.get_logger().warn(
                f'TF para proyectar scan {scan.header.frame_id} no disponible: {exc}',
                throttle_duration_sec=2.0,
            )
            return  # sin localización todavía: no podemos discriminar mapeado/no-mapeado
        self.health_pub.publish(Bool(data=True))

        ranges = np.asarray(scan.ranges, dtype=float)
        if ranges.size == 0:
            return

        projected = transform_scan_points(
            ranges=ranges,
            angle_min=scan.angle_min,
            angle_max=scan.angle_max,
            angle_increment=scan.angle_increment,
            range_min=scan.range_min,
            range_max=scan.range_max,
            base_from_sensor=base_from_sensor,
            map_from_sensor=map_from_sensor,
        )
        self.points_rejected_invalid_ranges += (
            self._declared_beam_count(scan) - len(projected))
        if not projected:
            return

        # cono frontal en base_frame, luego consulta de celda en global_frame.
        selected = []
        for point in projected:
            base_range = math.hypot(point.base_x, point.base_y)
            if base_range >= self.danger:
                continue
            bearing = math.atan2(point.base_y, point.base_x)
            if abs(math.atan2(math.sin(bearing), math.cos(bearing))) > self.cone:
                continue
            selected.append(point)
        if not selected:
            self.pub.publish(Bool(data=False))
            return

        mx = np.asarray([point.map_x for point in selected], dtype=float)
        my = np.asarray([point.map_y for point in selected], dtype=float)

        cols = ((mx - self.ox) / self.res).astype(int)
        rows = ((my - self.oy) / self.res).astype(int)
        inside = (rows >= 0) & (rows < self.H) & (cols >= 0) & (cols < self.W)

        # Marcar SOLO lo visto en este barrido sobre una grilla temporal (no se
        # acumula sobre la capa persistente) y refrescar su instante de observación.
        inflation_cells = max(1, int(math.ceil(self.dynamic_radius / self.res)))
        seen_now = np.zeros((self.H, self.W), dtype=bool)
        _changed, new_pts = mark_dynamic_obstacles(
            seen_now, self.occ, rows[inside], cols[inside], inflation_cells)
        now = self._now()
        self.last_seen[seen_now] = now

        # La capa dinámica = celdas observadas dentro del TTL, menos lo eximido
        # (cono rojo confirmado). Las celdas que dejan de verse caducan solas, así
        # nunca se sella el mapa de forma permanente ni se acumula el smear del MCL.
        fresh = (now - self.last_seen) <= self.dynamic_ttl
        new_dynamic = fresh & ~self.exempt

        detected = new_pts >= self.min_pts
        if not np.array_equal(new_dynamic, self.dynamic):
            self.dynamic = new_dynamic
            self._publish_dynamic()
        self.pub.publish(Bool(data=bool(detected)))

    def cone_cb(self, msg: PoseWithCovarianceStamped):
        """Exime un disco alrededor del cono confirmado de la capa dinámica.

        El cono es el objetivo de la misión, no un obstáculo a rodear: si quedara
        marcado en /dynamic_obstacles, el global_planner sellaría su entorno y la
        aproximación nunca podría planearse. Limpiamos lo ya marcado y evitamos
        que se vuelva a marcar.
        """
        if self.exempt is None or self.res is None:
            return
        col = int((msg.pose.pose.position.x - self.ox) / self.res)
        row = int((msg.pose.pose.position.y - self.oy) / self.res)
        radius = max(1, int(math.ceil(self.cone_clear_radius / self.res)))
        changed = False
        for rr in range(max(0, row - radius), min(self.H, row + radius + 1)):
            for cc in range(max(0, col - radius), min(self.W, col + radius + 1)):
                if (rr - row) ** 2 + (cc - col) ** 2 > radius ** 2:
                    continue
                if not self.exempt[rr, cc]:
                    self.exempt[rr, cc] = True
                    changed = True
                if self.dynamic[rr, cc]:
                    self.dynamic[rr, cc] = False
                    changed = True
        if changed:
            self._publish_dynamic()

    def _publish_dynamic(self):
        if self.map is None or self.dynamic is None:
            return
        msg = OccupancyGrid()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map.info
        data = np.zeros((self.H, self.W), dtype=np.int8)
        data[self.dynamic] = 100
        msg.data = data.reshape(-1).astype(int).tolist()
        self.dynamic_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitor()
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
