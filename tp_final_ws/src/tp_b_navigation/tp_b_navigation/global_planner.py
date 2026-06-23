#!/usr/bin/env python3
"""global_planner: planificación de ruta inicio->objetivo sobre la grilla de ocupación
(consigna 1.5). A* 8-conectado sobre `/map`, con **inflado de obstáculos** (margen de
seguridad por el radio del robot) y una penalización de cercanía para preferir rutas que
no rocen las paredes ("ruta segura").

Entradas:
  - /map            (nav_msgs/OccupancyGrid, latcheado)  -> grilla
  - /plan_request   (geometry_msgs/PoseStamped)          -> objetivo a planear
  - TF map->base_frame                                   -> pose de inicio (del MCL)

Salidas:
  - /plan           (nav_msgs/Path)        -> ruta en frame map (vacía si falla)
  - /plan_status    (std_msgs/Bool)        -> True si encontró ruta

Es un nodo "puro": no comanda al robot. La máquina de estados (state_machine) le pide
planes publicando en /plan_request y consume /plan.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from tp_b_navigation.dynamic_obstacles import apply_dynamic_obstacles
from tp_b_navigation.utils import yaw_from_quaternion, quaternion_from_yaw
from tp_b_navigation.planner_core import GridPlannerCore


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')

        self.declare_parameter('robot_radius', 0.18)      # radio + margen (m)
        self.declare_parameter('clearance_weight', 0.6)   # peso de la penalización de cercanía
        self.declare_parameter('clearance_max', 0.5)      # distancia (m) a partir de la cual no penaliza
        self.declare_parameter('allow_unknown', False)    # tratar desconocido como obstáculo
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.clear_w = float(self.get_parameter('clearance_weight').value)
        self.clear_max = float(self.get_parameter('clearance_max').value)
        self.allow_unknown = bool(self.get_parameter('allow_unknown').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.map = None
        self.static_data = None
        self.dynamic_data = None
        self.blocked = None      # bool [H,W] celdas no navegables (obstáculo inflado)
        self.cost = None         # float [H,W] penalización de cercanía (m)
        self.core = None
        self.res = None
        self.ox = self.oy = 0.0
        self.W = self.H = 0

        qos_latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_latched)
        self.create_subscription(OccupancyGrid, '/dynamic_obstacles',
                                 self.dynamic_cb, qos_latched)
        self.create_subscription(PoseStamped, '/plan_request', self.request_cb, 10)

        self.pub_plan = self.create_publisher(Path, '/plan', qos_latched)
        self.pub_status = self.create_publisher(Bool, '/plan_status', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('global_planner listo. Esperando /map y /plan_request.')

    # ------------------------------------------------------------------ mapa
    def map_cb(self, msg: OccupancyGrid):
        self.map = msg
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.W = msg.info.width
        self.H = msg.info.height
        self.static_data = np.array(msg.data, dtype=np.int16).reshape(self.H, self.W)
        self._rebuild_core()
        self.get_logger().info(
            f'Mapa recibido ({self.W}x{self.H}). Inflado ~{self.robot_radius / self.res:.1f} '
            f'celdas ({self.robot_radius:.2f} m).')

    def dynamic_cb(self, msg: OccupancyGrid):
        if self.static_data is None:
            return
        same_geometry = (
            msg.info.width == self.W and msg.info.height == self.H and
            abs(msg.info.resolution - self.res) < 1e-9 and
            abs(msg.info.origin.position.x - self.ox) < 1e-9 and
            abs(msg.info.origin.position.y - self.oy) < 1e-9)
        if not same_geometry:
            self.get_logger().warn('Ignoro /dynamic_obstacles con geometría distinta al /map.')
            return
        self.dynamic_data = np.array(msg.data, dtype=np.int16).reshape(self.H, self.W)
        self._rebuild_core()

    def _rebuild_core(self):
        if self.static_data is None:
            return
        data = apply_dynamic_obstacles(self.static_data, self.dynamic_data)
        self.core = GridPlannerCore.from_occupancy(
            data, self.res, self.ox, self.oy,
            robot_radius=self.robot_radius,
            clearance_weight=self.clear_w,
            clearance_max=self.clear_max,
            allow_unknown=self.allow_unknown,
        )
        self.blocked = self.core.blocked
        self.cost = self.core.cost

    # ------------------------------------------------------------- conversiones
    def world_to_cell(self, x, y):
        return self.core.world_to_cell(x, y)

    def cell_to_world(self, r, c):
        return self.core.cell_to_world((r, c))

    def _nearest_free(self, r, c, max_radius=12):
        return self.core.nearest_free((r, c), max_radius=max_radius)

    # ------------------------------------------------------------- A*
    def _astar(self, start, goal):
        return self.core.plan_cells(start, goal, simplify=False)

    def _shortcut(self, cells):
        return self.core.simplify(cells)

    def _line_clear(self, a, b):
        return self.core.line_clear(a, b)

    # ------------------------------------------------------------- request
    def request_cb(self, goal: PoseStamped):
        if self.map is None:
            self.get_logger().warn('Sin /map todavía; no puedo planear.')
            self._publish_failure()
            return
        try:
            tr = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Sin TF {self.global_frame}->{self.base_frame}: {e}')
            self._publish_failure()
            return

        sx = tr.transform.translation.x
        sy = tr.transform.translation.y
        gx = goal.pose.position.x
        gy = goal.pose.position.y

        start = self._nearest_free(*self.world_to_cell(sx, sy))
        goalc = self._nearest_free(*self.world_to_cell(gx, gy))
        if start is None or goalc is None:
            self.get_logger().warn('Inicio u objetivo fuera del mapa / en obstáculo.')
            self._publish_failure()
            return

        cells = self._astar(start, goalc)
        if cells is None:
            self.get_logger().warn('A* no encontró ruta al objetivo.')
            self._publish_failure()
            return

        cells = self._shortcut(cells)
        path = self._build_path(cells, goal)
        self.pub_plan.publish(path)
        self.pub_status.publish(Bool(data=True))
        self.get_logger().info(
            f'Ruta planeada: {len(cells)} waypoints, '
            f'inicio ({sx:.2f},{sy:.2f}) -> objetivo ({gx:.2f},{gy:.2f}).')

    def _build_path(self, cells, goal):
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for k, (r, c) in enumerate(cells):
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame
            x, y = self.cell_to_world(r, c)
            ps.pose.position.x = x
            ps.pose.position.y = y
            # orientación de cada waypoint = hacia el siguiente (estética/diagnóstico)
            if k < len(cells) - 1:
                nr, nc = cells[k + 1]
                nx, ny = self.cell_to_world(nr, nc)
                ps.pose.orientation = quaternion_from_yaw(math.atan2(ny - y, nx - x))
            else:
                ps.pose.orientation = goal.pose.orientation
            path.poses.append(ps)
        # forzar el último punto exactamente al objetivo pedido (posición + ángulo final)
        if path.poses:
            path.poses[-1].pose.position.x = goal.pose.position.x
            path.poses[-1].pose.position.y = goal.pose.position.y
            path.poses[-1].pose.orientation = goal.pose.orientation
        return path

    def _publish_failure(self):
        empty = Path()
        empty.header.frame_id = self.global_frame
        empty.header.stamp = self.get_clock().now().to_msg()
        self.pub_plan.publish(empty)
        self.pub_status.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
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
