#!/usr/bin/env python3
"""sim_mapper: re-mapeo del entorno SIMULADO con la lógica de mapeo de Parte A.

Consigna (Sistema 3): "Con esos landmarks ficticios, repetir el proceso de mapeo
para tener un mapa del entorno simulado coherente". Parte A genera su mapa de
ocupación a partir del rosbag del TurtleBot4 (mundo real); este nodo **repite el
mismo proceso de mapeo en Gazebo** para producir el mapa del entorno simulado que
después consume Parte B (planner + MCL).

Reusa el algoritmo de `tp_slam_aruco/occupancy_grid_node.py` (Parte A):
modelo de sensor inverso + log-odds + Bresenham + export .pgm/.yaml (mismas
constantes y umbrales, ver _L_OCC/_L_FREE y _PGM_*). El core está PORTADO acá
(igual que `landmark_sensor` está portado de tp5) para mantener Parte B
autocontenida; los valores se mantienen idénticos a la Parte A a propósito.

Diferencia con la Parte A: la pose de cada barrido NO viene de un JSON de
trayectoria del Graph SLAM (en Gazebo no hay ArUco), sino de la **TF en vivo**
de la simulación (`mapping_frame` → frame del LIDAR). En Gazebo la odometría es
~ground-truth, así que `odom` ≡ `map` en el origen y el mapa queda alineado con
lo que espera Parte B (map≡odom en el origen).

USO:
  # Terminal 1: simulación
  ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
  # Terminal 2: este nodo (mapea mientras el robot recorre la casa)
  ros2 run tp_b_navigation sim_mapper --ros-args -p use_sim_time:=true \
       -p map_output:=<ruta>/map_sim
  # Recorrer la casa (driver de cobertura o navegación de Parte B).
  # Ctrl+C → exporta map_sim.pgm + map_sim.yaml
"""

import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# --- Constantes del modelo de sensor inverso (idénticas a occupancy_grid_node de Parte A) ---
_L_OCC = 0.85    # evidencia de celda ocupada (≈ log(0.70/0.30))
_L_FREE = -0.40  # evidencia de celda libre (asimétrico: más conservador al limpiar)
_L_MIN = -5.0
_L_MAX = 5.0


def world_to_grid(wx, wy, origin_x, origin_y, resolution):
    """Portado de tp_slam_aruco/slam_mapping.py (Parte A)."""
    gx = math.floor((wx - origin_x) / resolution + 1e-9)
    gy = math.floor((wy - origin_y) / resolution + 1e-9)
    return gx, gy


def bresenham_cells(x0, y0, x1, y1):
    """Celdas de la línea (x0,y0)->(x1,y1) inclusive. Portado de Parte A (slam_mapping.py)."""
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


def yaw_from_quaternion(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


class SimMapper(Node):
    # Umbrales de generación del PGM (idénticos a occupancy_grid_node de Parte A).
    # Con negate:0, map_server calcula p=(255-pixel)/255.
    _PGM_OCC_THRESH = 0.60
    _PGM_FREE_THRESH = 0.40
    _PGM_UNKNOWN_PX = 127

    def __init__(self):
        super().__init__('sim_mapper')

        # Frame en el que se construye el mapa. En Gazebo la odometría es
        # ~ground-truth, así que 'odom' ≡ 'map' en el origen (lo que asume Parte B).
        self.declare_parameter('mapping_frame', 'odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('resolution', 0.05)
        # Grilla fija dimensionada para cubrir custom_casa (~7x7 m) con margen.
        self.declare_parameter('origin_x', -5.0)
        self.declare_parameter('origin_y', -5.0)
        self.declare_parameter('width', 220)
        self.declare_parameter('height', 220)
        self.declare_parameter('map_output', os.path.join(
            os.path.expanduser('~'),
            'Documents', 'GitHub', 'TP-Final-Rob', 'mapas', 'map_sim'))
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('log_every', 100)

        self.mapping_frame = self.get_parameter('mapping_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.map_output = self.get_parameter('map_output').value
        self.log_every = int(self.get_parameter('log_every').value)

        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.scan_count = 0
        self.scans_skipped_no_tf = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.scan_cb, best_effort)

        # /map latcheado para RViz / inspección en vivo.
        qos_latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_latched)
        period = float(self.get_parameter('publish_period_sec').value)
        if period > 0.0:
            self.create_timer(period, self.publish_map)

        self.get_logger().info(
            f'sim_mapper activo. Grilla {self.width}x{self.height} @ '
            f'{self.resolution} m/cel '
            f'({self.width * self.resolution:.1f}x{self.height * self.resolution:.1f} m), '
            f'origen ({self.origin_x}, {self.origin_y}), frame {self.mapping_frame}. '
            f'Export → {self.map_output}.{{pgm,yaml}}')

    # ------------------------------------------------------------------ #
    def _world_to_grid(self, wx, wy):
        return world_to_grid(wx, wy, self.origin_x, self.origin_y, self.resolution)

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def _sensor_pose(self, scan: LaserScan):
        """Pose del LIDAR en mapping_frame vía TF (mapping_frame <- frame del scan).

        Devuelve (lx, ly, lyaw) o None si la TF no está disponible. Usa el último
        TF disponible (Time(0)): a velocidad de mapeo el desfase es despreciable y
        evita el parpadeo de extrapolación de sim-time.
        """
        sensor_frame = scan.header.frame_id or 'base_scan'
        try:
            tf = self.tf_buffer.lookup_transform(
                self.mapping_frame, sensor_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF {self.mapping_frame}<-{sensor_frame} no disponible: {e}',
                                   throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def scan_cb(self, msg: LaserScan):
        pose = self._sensor_pose(msg)
        if pose is None:
            self.scans_skipped_no_tf += 1
            return
        self.integrate_scan(pose, msg)
        self.scan_count += 1
        if self.log_every and self.scan_count % self.log_every == 0:
            self.get_logger().info(
                f'Scans integrados: {self.scan_count} '
                f'(descartados sin TF: {self.scans_skipped_no_tf})')

    def integrate_scan(self, sensor_pose, scan: LaserScan):
        """Proyecta un LaserScan en la grilla log-odds (modelo de sensor inverso).

        Idéntico a occupancy_grid_node.integrate_scan de Parte A, pero la pose ya
        es la del SENSOR en mapping_frame (la TF resuelve los extrínsecos del LIDAR),
        así que no hace falta componer offsets a mano.
        """
        lx, ly, lth = sensor_pose
        gx0, gy0 = self._world_to_grid(lx, ly)

        angle = scan.angle_min
        for r in scan.ranges:
            if not (math.isnan(r) or math.isinf(r) or
                    r < scan.range_min or r > scan.range_max):
                ray_angle = lth + angle
                wx = lx + r * math.cos(ray_angle)
                wy = ly + r * math.sin(ray_angle)
                gx1, gy1 = self._world_to_grid(wx, wy)

                cells = bresenham_cells(gx0, gy0, gx1, gy1)
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
    def publish_map(self):
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))
        grid = np.full(prob.shape, -1, dtype=np.int8)
        grid[prob >= self._PGM_OCC_THRESH] = 100
        grid[prob <= self._PGM_FREE_THRESH] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten(order='C').tolist()
        self.map_pub.publish(msg)

    def export_map(self, path_prefix: str):
        """Exporta a <prefix>.pgm + <prefix>.yaml (formato nav2 map_server).

        Idéntico al export de occupancy_grid_node de Parte A (mismos umbrales y
        semántica negate:0), para que map_loader/map_server lo lean igual.
        """
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))
        pgm = np.full((self.height, self.width), self._PGM_UNKNOWN_PX, dtype=np.uint8)
        pgm[prob >= self._PGM_OCC_THRESH] = 0
        pgm[prob <= self._PGM_FREE_THRESH] = 254
        # ROS: fila 0 = y mínimo; PGM: fila 0 = y máximo → invertir verticalmente.
        pgm = np.flipud(pgm)

        pgm_path = path_prefix + '.pgm'
        out_dir = os.path.dirname(pgm_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{self.width} {self.height}\n255\n'.encode('ascii'))
            f.write(pgm.tobytes())

        yaml_path = path_prefix + '.yaml'
        with open(yaml_path, 'w') as f:
            f.write(f'image: {os.path.basename(pgm_path)}\n')
            f.write(f'resolution: {self.resolution}\n')
            f.write(f'origin: [{self.origin_x:.4f}, {self.origin_y:.4f}, 0.0]\n')
            f.write('negate: 0\n')
            f.write(f'occupied_thresh: {self._PGM_OCC_THRESH}\n')
            f.write(f'free_thresh: {self._PGM_FREE_THRESH}\n')

        self.get_logger().info(
            f'Mapa exportado → {pgm_path} ({self.width}x{self.height} px). '
            f'Scans totales integrados: {self.scan_count}.')


def main(args=None):
    rclpy.init(args=args)
    node = SimMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.export_map(node.map_output)
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f'No se pudo exportar el mapa: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
