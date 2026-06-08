#!/usr/bin/env python3
"""
occupancy_grid_node.py
======================
SEGUNDA PASADA del SLAM (Opción 3). Con la trayectoria ya corregida por el
Graph SLAM (leída del JSON), proyecta cada barrido LIDAR en una grilla de
ocupación métrica usando inverse sensor model + log-odds.

FLUJO DE USO:
  1ª pasada: ros2 launch tp_slam_aruco parte_a_slam.launch.py ...
             → genera runs/trayectoria.json con poses + timestamps
  2ª pasada: ros2 launch tp_slam_aruco parte_a_mapa.launch.py ...
             (con el bag corriendo en paralelo: ros2 bag play ... --clock)
             → construye /map en tiempo real
             Ctrl+C → exporta .pgm + .yaml

ALGORITMO (inverse sensor model + log-odds):
  Para cada rayo del LIDAR desde una pose interpolada:
    - Celdas libres (Bresenham desde robot hasta impacto-1): log-odds += L_FREE
    - Celda de impacto: log-odds += L_OCC
  Saturar en [L_MIN, L_MAX] para evitar desbordamiento numérico.
  Al exportar, convertir log-odds a probabilidad y umbralizar a 0/100/-1.
"""

import bisect
import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

_L_OCC  =  0.85   # evidencia de celda ocupada (≈ log(0.70/0.30))
_L_FREE = -0.40   # evidencia de celda libre (asimétrico: más conservador al limpiar)
_L_MIN  = -5.0
_L_MAX  =  5.0


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        self.declare_parameter('resolution',    0.05)
        self.declare_parameter('width',         300)
        self.declare_parameter('height',        300)
        self.declare_parameter('origin_x',     -4.0)
        self.declare_parameter('origin_y',     -1.5)
        self.declare_parameter('scan_topic',   'tb4_0/scan')
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('map_output',   '/tmp/mapa')
        self.declare_parameter('publish_every', 50)
        self.declare_parameter('lidar_tx',     -0.04)   # rplidar_link en base_link (TF real)
        self.declare_parameter('lidar_ty',      0.0)
        self.declare_parameter('lidar_yaw',     0.0)

        self.resolution    = self.get_parameter('resolution').value
        self.width         = self.get_parameter('width').value
        self.height        = self.get_parameter('height').value
        self.origin_x      = self.get_parameter('origin_x').value
        self.origin_y      = self.get_parameter('origin_y').value
        self.map_output    = self.get_parameter('map_output').value
        self.publish_every = self.get_parameter('publish_every').value
        self.lidar_tx      = self.get_parameter('lidar_tx').value
        self.lidar_ty      = self.get_parameter('lidar_ty').value
        self.lidar_yaw     = self.get_parameter('lidar_yaw').value

        # Cargar trayectoria corregida desde JSON
        traj_path = self.get_parameter('trajectory_file').value
        self.trajectory  = []
        self.traj_stamps = []
        if traj_path and os.path.exists(traj_path):
            with open(traj_path) as f:
                data = json.load(f)
            self.trajectory = data['trajectory']
            self.traj_stamps = [p['stamp'] for p in self.trajectory]
            self.get_logger().info(
                f'Trayectoria cargada: {len(self.trajectory)} poses desde {traj_path}')
        else:
            self.get_logger().error(
                f'trajectory_file no encontrado: "{traj_path}". '
                'Ejecutar primero la 1ª pasada (parte_a_slam.launch.py).')

        # Grilla log-odds
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.scan_count = 0

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_cb,
            best_effort,
        )

        self.get_logger().info(
            f'occupancy_grid_node activo. '
            f'Grilla {self.width}x{self.height} @ {self.resolution} m/cel '
            f'({self.width * self.resolution:.1f}x{self.height * self.resolution:.1f} m). '
            f'Origen ({self.origin_x}, {self.origin_y}). '
            f'LIDAR offset tx={self.lidar_tx} ty={self.lidar_ty}.'
        )

    # ------------------------------------------------------------------ #
    #  Utilidades de coordenadas y geometría                               #
    # ------------------------------------------------------------------ #

    def _world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Genera las celdas de la línea de (x0,y0) a (x1,y1) inclusive."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        x, y = x0, y0
        if dx > dy:
            err = dx // 2
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy // 2
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        cells.append((x1, y1))
        return cells

    def _interpolate_pose(self, t):
        """Devuelve (x, y, theta) interpolado linealmente para el timestamp t (segundos)."""
        if not self.traj_stamps:
            return None
        if t <= self.traj_stamps[0]:
            p = self.trajectory[0]
            return p['x'], p['y'], p['theta']
        if t >= self.traj_stamps[-1]:
            p = self.trajectory[-1]
            return p['x'], p['y'], p['theta']

        hi = bisect.bisect_right(self.traj_stamps, t)
        lo = hi - 1
        t0, t1 = self.traj_stamps[lo], self.traj_stamps[hi]
        alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0.0

        p0, p1 = self.trajectory[lo], self.trajectory[hi]
        x = p0['x'] + alpha * (p1['x'] - p0['x'])
        y = p0['y'] + alpha * (p1['y'] - p0['y'])
        dth = math.atan2(
            math.sin(p1['theta'] - p0['theta']),
            math.cos(p1['theta'] - p0['theta']))
        theta = p0['theta'] + alpha * dth
        return x, y, theta

    # ------------------------------------------------------------------ #
    #  Integración de un barrido LIDAR                                     #
    # ------------------------------------------------------------------ #

    def integrate_scan(self, robot_pose, scan: LaserScan):
        """Proyecta un barrido LaserScan en la grilla log-odds desde la pose dada."""
        rx, ry, rth = robot_pose

        # Posición del LIDAR en frame map (componer offset del sensor)
        lx  = rx + self.lidar_tx * math.cos(rth) - self.lidar_ty * math.sin(rth)
        ly  = ry + self.lidar_tx * math.sin(rth) + self.lidar_ty * math.cos(rth)
        lth = rth + self.lidar_yaw

        gx0, gy0 = self._world_to_grid(lx, ly)

        angle = scan.angle_min
        for r in scan.ranges:
            if not (math.isnan(r) or math.isinf(r) or
                    r < scan.range_min or r > scan.range_max):
                ray_angle = lth + angle
                wx = lx + r * math.cos(ray_angle)
                wy = ly + r * math.sin(ray_angle)
                gx1, gy1 = self._world_to_grid(wx, wy)

                cells = self._bresenham(gx0, gy0, gx1, gy1)

                # Celdas libres (todas menos la de impacto)
                for cx, cy in cells[:-1]:
                    if self._in_bounds(cx, cy):
                        v = self.log_odds[cy, cx] + _L_FREE
                        self.log_odds[cy, cx] = max(_L_MIN, v)

                # Celda de impacto → ocupada
                cx, cy = cells[-1]
                if self._in_bounds(cx, cy):
                    v = self.log_odds[cy, cx] + _L_OCC
                    self.log_odds[cy, cx] = min(_L_MAX, v)

            angle += scan.angle_increment

    # ------------------------------------------------------------------ #
    #  Callback de scan                                                    #
    # ------------------------------------------------------------------ #

    def scan_cb(self, msg: LaserScan):
        if not self.trajectory:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self._interpolate_pose(t)
        if pose is None:
            return
        self.integrate_scan(pose, msg)
        self.scan_count += 1
        if self.scan_count % self.publish_every == 0:
            self.get_logger().info(f'Scans integrados: {self.scan_count}')
            self.publish_map()

    # ------------------------------------------------------------------ #
    #  Publicación y exportación del mapa                                  #
    # ------------------------------------------------------------------ #

    def publish_map(self):
        prob = (1.0 / (1.0 + np.exp(-self.log_odds))).astype(np.float32)

        occ = np.full(self.height * self.width, -1, dtype=np.int8)
        flat = prob.flatten()
        occ[flat >= 0.6] = 100
        occ[flat <= 0.4] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occ.tolist()
        self.map_pub.publish(msg)

    def export_map(self, path_prefix: str):
        """Exporta la grilla a <prefix>.pgm + <prefix>.yaml (formato nav2 map_server)."""
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))

        # Convención PGM de ROS: 0=negro(ocupado), 205=gris(desconocido), 254=blanco(libre)
        pgm = np.full((self.height, self.width), 205, dtype=np.uint8)
        pgm[prob >= 0.6] = 0
        pgm[prob <= 0.4] = 254

        # ROS: fila 0 es y mínimo. PGM: fila 0 es y máximo → invertir verticalmente
        pgm = np.flipud(pgm)

        pgm_path = path_prefix + '.pgm'
        os.makedirs(os.path.dirname(pgm_path) if os.path.dirname(pgm_path) else '.', exist_ok=True)
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{self.width} {self.height}\n255\n'.encode('ascii'))
            f.write(pgm.tobytes())

        yaml_path = path_prefix + '.yaml'
        pgm_basename = os.path.basename(pgm_path)
        with open(yaml_path, 'w') as f:
            f.write(f'image: {pgm_basename}\n')
            f.write(f'resolution: {self.resolution}\n')
            f.write(f'origin: [{self.origin_x:.4f}, {self.origin_y:.4f}, 0.0]\n')
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.196\n')

        self.get_logger().info(
            f'Mapa exportado → {pgm_path} '
            f'({self.width}x{self.height} px, '
            f'{self.width * self.resolution:.1f}x{self.height * self.resolution:.1f} m). '
            f'Scans totales integrados: {self.scan_count}.')


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_map()
        if node.map_output:
            node.export_map(node.map_output)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
