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
from collections import deque
import heapq

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from tp_b_navigation.utils import yaw_from_quaternion, quaternion_from_yaw


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
        self.blocked = None      # bool [H,W] celdas no navegables (obstáculo inflado)
        self.cost = None         # float [H,W] penalización de cercanía (m)
        self.res = None
        self.ox = self.oy = 0.0
        self.W = self.H = 0

        qos_latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_latched)
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
        data = np.array(msg.data, dtype=np.int16).reshape(self.H, self.W)

        obst = (data == 100)
        if not self.allow_unknown:
            obst |= (data == -1)

        # Distancia (en celdas) de cada celda libre al obstáculo más cercano (BFS multi-fuente).
        dist_cells = self._distance_transform(obst)
        dist_m = dist_cells * self.res

        inflate_cells = self.robot_radius / self.res
        self.blocked = dist_cells < inflate_cells

        # Penalización de cercanía: alta cerca de obstáculos, 0 más allá de clearance_max.
        clr = np.clip(self.clear_max - dist_m, 0.0, self.clear_max)
        self.cost = self.clear_w * clr
        self.get_logger().info(
            f'Mapa recibido ({self.W}x{self.H}). Inflado ~{inflate_cells:.1f} celdas '
            f'({self.robot_radius:.2f} m).')

    @staticmethod
    def _distance_transform(obst):
        """Distancia de Chebyshev (en celdas, 8-conexa) al obstáculo más cercano."""
        H, W = obst.shape
        INF = float('inf')
        dist = np.full((H, W), INF, dtype=float)
        dq = deque()
        ys, xs = np.where(obst)
        for r, c in zip(ys.tolist(), xs.tolist()):
            dist[r, c] = 0.0
            dq.append((r, c))
        nbrs = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))
        while dq:
            r, c = dq.popleft()
            d1 = dist[r, c] + 1.0
            for dr, dc in nbrs:
                nr, nc = r + dr, c + dc
                if 0 <= nr < H and 0 <= nc < W and dist[nr, nc] > d1:
                    dist[nr, nc] = d1
                    dq.append((nr, nc))
        return dist

    # ------------------------------------------------------------- conversiones
    def world_to_cell(self, x, y):
        c = int((x - self.ox) / self.res)
        r = int((y - self.oy) / self.res)
        return r, c

    def cell_to_world(self, r, c):
        x = self.ox + (c + 0.5) * self.res
        y = self.oy + (r + 0.5) * self.res
        return x, y

    def _nearest_free(self, r, c, max_radius=12):
        """Si (r,c) cae en zona bloqueada, busca la celda libre más cercana (espiral BFS)."""
        if not (0 <= r < self.H and 0 <= c < self.W):
            return None
        if not self.blocked[r, c]:
            return (r, c)
        seen = {(r, c)}
        dq = deque([(r, c, 0)])
        while dq:
            cr, cc, d = dq.popleft()
            if d > max_radius:
                break
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = cr + dr, cc + dc
                if (nr, nc) in seen or not (0 <= nr < self.H and 0 <= nc < self.W):
                    continue
                seen.add((nr, nc))
                if not self.blocked[nr, nc]:
                    return (nr, nc)
                dq.append((nr, nc, d + 1))
        return None

    # ------------------------------------------------------------- A*
    def _astar(self, start, goal):
        H, W = self.H, self.W
        blocked = self.blocked
        cost = self.cost
        res = self.res

        def heur(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1]) * res

        nbrs = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, 1.41421356), (-1, 1, 1.41421356),
                (1, -1, 1.41421356), (1, 1, 1.41421356))

        open_heap = [(heur(start, goal), 0.0, start)]
        g = {start: 0.0}
        came = {}
        INF = float('inf')

        while open_heap:
            f, gc, cur = heapq.heappop(open_heap)
            if cur == goal:
                break
            if gc > g.get(cur, INF):
                continue
            r, c = cur
            for dr, dc, base in nbrs:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < H and 0 <= nc < W) or blocked[nr, nc]:
                    continue
                # evitar cortar esquinas en diagonal
                if dr != 0 and dc != 0 and (blocked[r + dr, c] or blocked[r, c + dc]):
                    continue
                ng = gc + base * res + cost[nr, nc] * base
                if ng < g.get((nr, nc), INF):
                    g[(nr, nc)] = ng
                    came[(nr, nc)] = cur
                    heapq.heappush(open_heap, (ng + heur((nr, nc), goal), ng, (nr, nc)))

        if goal not in came and goal != start:
            return None
        # reconstruir
        path = [goal]
        node = goal
        while node != start:
            node = came[node]
            path.append(node)
        path.reverse()
        return path

    def _shortcut(self, cells):
        """Simplifica la ruta con line-of-sight greedy (rutas más rectas y suaves, 1.6)."""
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        i = 0
        n = len(cells)
        while i < n - 1:
            j = n - 1
            while j > i + 1:
                if self._line_clear(cells[i], cells[j]):
                    break
                j -= 1
            out.append(cells[j])
            i = j
        return out

    def _line_clear(self, a, b):
        """Bresenham entre celdas: True si no atraviesa zona bloqueada."""
        r0, c0 = a
        r1, c1 = b
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            if self.blocked[r, c]:
                return False
            if r == r1 and c == c1:
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

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
