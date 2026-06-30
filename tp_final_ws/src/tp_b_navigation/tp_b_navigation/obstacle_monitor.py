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
Salidas:  /obstacle_detected (std_msgs/Bool), /dynamic_obstacles (OccupancyGrid).
"""

import math

import numpy as np

import rclpy
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

        self.danger = float(self.get_parameter('danger_dist').value)
        self.cone = float(self.get_parameter('cone_halfangle').value)
        self.min_pts = int(self.get_parameter('min_points').value)
        self.dynamic_radius = float(self.get_parameter('dynamic_inflation_radius').value)
        self.dynamic_ttl = float(self.get_parameter('dynamic_ttl').value)
        self.cone_clear_radius = float(self.get_parameter('cone_clear_radius').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.map = None
        self.res = None
        self.ox = self.oy = 0.0
        self.W = self.H = 0
        self.occ = None  # bool [H,W] ocupado en el mapa
        self.dynamic = None
        self.last_seen = None  # float [H,W]: instante (s) de última observación por celda
        self.exempt = None  # bool [H,W]: celdas eximidas (cono rojo confirmado)

        qos_latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_latched)
        self.create_subscription(LaserScan, '/scan', self.scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/red_cone_pose',
                                 self.cone_cb, 10)
        self.pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.dynamic_pub = self.create_publisher(
            OccupancyGrid, '/dynamic_obstacles', qos_latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def scan_cb(self, scan: LaserScan):
        if self.occ is None:
            return
        try:
            tr = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return  # sin localización todavía: no podemos discriminar mapeado/no-mapeado

        rx = tr.transform.translation.x
        ry = tr.transform.translation.y
        rth = yaw_from_quaternion(tr.transform.rotation)

        ranges = np.asarray(scan.ranges, dtype=float)
        n = ranges.size
        if n == 0:
            return
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        # cono frontal: |angulo| <= cono ; rango válido y < danger
        valid = (np.isfinite(ranges) & (ranges > scan.range_min) &
                 (ranges < self.danger))
        # normalizar ángulo del haz a [-pi,pi] para el test de cono frontal
        a_norm = np.arctan2(np.sin(angles), np.cos(angles))
        frontal = np.abs(a_norm) <= self.cone
        sel = valid & frontal

        r = ranges[sel]
        a = angles[sel]
        # punto en frame robot -> frame map
        bx = r * np.cos(a)
        by = r * np.sin(a)
        cos_t, sin_t = math.cos(rth), math.sin(rth)
        mx = rx + bx * cos_t - by * sin_t
        my = ry + bx * sin_t + by * cos_t

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
