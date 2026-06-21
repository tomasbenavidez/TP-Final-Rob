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

POSE DE CADA SCAN (clave para un mapa nitido):
  El Graph SLAM entrega una trayectoria CORRECTA pero SPARSE (un keyframe cada
  ~0.15 m o ~0.6 rad). Interpolar linealmente esos keyframes para cada scan
  falsea la pose durante los giros (entre keyframes el robot rota de forma no
  uniforme) y emborrona las paredes. En su lugar componemos:
      pose_map(t) = correccion_SLAM_interpolada(t) ∘ odom_densa(t)
  donde la correccion SLAM (map<-odom) varia lento y suave, y la odometria densa
  (20 Hz) aporta el detalle de alta frecuencia. Asi el mapa hereda la correccion
  de deriva del SLAM sin perder resolucion temporal. (Modo legacy: si el JSON no
  trae la pose de odom por keyframe, se cae a interpolacion lineal de keyframes.)

ALGORITMO (inverse sensor model + log-odds):
  Para cada rayo del LIDAR desde la pose resuelta:
    - Celdas libres (Bresenham desde robot hasta impacto-1): log-odds += L_FREE
    - Celda de impacto: log-odds += L_OCC
  Saturar en [L_MIN, L_MAX] para evitar desbordamiento numérico.
  Al exportar, convertir log-odds a probabilidad y umbralizar a 0/100/-1.
"""

import bisect
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

from tp_slam_aruco.motion_model import yaw_from_quaternion, normalize_angle
from tp_slam_aruco.slam_geometry import (
    se2_compose,
    se2_interpolate,
    se2_inverse,
)
from tp_slam_aruco.slam_mapping import (
    bresenham_cells,
    interpolate_pose,
    log_odds_to_occupancy,
    world_to_grid,
)
from tp_slam_aruco.slam_io import read_trajectory_json

_L_OCC  =  0.85   # evidencia de celda ocupada (≈ log(0.70/0.30))
_L_FREE = -0.40   # evidencia de celda libre (asimétrico: más conservador al limpiar)
_L_MIN  = -5.0
_L_MAX  =  5.0


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        self.declare_parameter('resolution',    0.05)
        # width / height / origin_x / origin_y son fallback si no hay trayectoria.
        # Con trayectoria cargada se auto-calculan desde el bounding box de las poses.
        self.declare_parameter('width',         300)
        self.declare_parameter('height',        300)
        self.declare_parameter('origin_x',     -4.0)
        self.declare_parameter('origin_y',     -1.5)
        # Margen añadido a cada lado del bounding box de la trayectoria (metros).
        # Debe cubrir el rango máximo del LIDAR hacia afuera del recorrido.
        self.declare_parameter('map_margin',    3.0)
        self.declare_parameter('scan_topic',   'tb4_0/scan')
        self.declare_parameter('odom_topic',   'tb4_0/odom')
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('map_output',   '/tmp/mapa')
        self.declare_parameter('publish_every', 50)
        # Extrinsecos LIDAR->base_link tomados del TF estatico del bag:
        #   base_link -> shell_link (yaw 0) -> rplidar_link (t=-0.04, yaw +90deg)
        # => el RPLIDAR esta montado rotado +pi/2 respecto de base_link. Ignorar
        # este yaw rota cada barrido y, como el robot gira, smearea las paredes.
        self.declare_parameter('lidar_tx',     -0.04)
        self.declare_parameter('lidar_ty',      0.0)
        self.declare_parameter('lidar_yaw',     math.pi / 2)
        # No integrar scans cuando el robot gira rapido: durante un giro la pose
        # angular es la menos confiable y un barrido de 360deg se distorsiona.
        # rad/s; <=0 desactiva el gate. Se mide sobre la odometria densa.
        self.declare_parameter('max_angular_velocity', 0.0)

        self.resolution    = self.get_parameter('resolution').value
        self.map_output    = self.get_parameter('map_output').value
        self.publish_every = self.get_parameter('publish_every').value
        self.lidar_tx      = self.get_parameter('lidar_tx').value
        self.lidar_ty      = self.get_parameter('lidar_ty').value
        self.lidar_yaw     = self.get_parameter('lidar_yaw').value
        self.max_angular_velocity = float(self.get_parameter('max_angular_velocity').value)

        # Cargar trayectoria corregida desde JSON
        traj_path = self.get_parameter('trajectory_file').value
        self.trajectory  = []
        self.traj_stamps = []
        if traj_path and os.path.exists(traj_path):
            try:
                self.trajectory, self.landmarks = read_trajectory_json(traj_path)
            except Exception as exc:
                self.get_logger().error(f'trajectory_file invalido: {exc}')
                self.trajectory = []
                self.landmarks = {}
            self.traj_stamps = [p['stamp'] for p in self.trajectory]
            self.get_logger().info(
                f'Trayectoria cargada: {len(self.trajectory)} poses desde {traj_path}')
        else:
            self.landmarks = {}
            self.get_logger().error(
                f'trajectory_file no encontrado: "{traj_path}". '
                'Ejecutar primero la 1ª pasada (parte_a_slam.launch.py).')

        # Correccion SLAM por keyframe: T_corr_i = T_map_kf_i ∘ (T_odom_kf_i)^-1.
        # Si la trayectoria trae la pose de odom de cada keyframe podemos componer
        # la correccion (lenta, suave) sobre la odometria densa (alta frecuencia)
        # en lugar de interpolar linealmente keyframes sparse, que durante los giros
        # falsea la pose de cada scan y emborrona el mapa.
        self.corr_stamps = []
        self.corrections = []
        for pose in self.trajectory:
            odom = pose.get('odom')
            if not odom:
                continue
            corr = se2_compose(
                (pose['x'], pose['y'], pose['theta']),
                se2_inverse((odom['x'], odom['y'], odom['theta'])),
            )
            self.corr_stamps.append(pose['stamp'])
            self.corrections.append(corr)
        self.use_dense_odom = len(self.corrections) >= 2
        if self.use_dense_odom:
            self.get_logger().info(
                f'Modo odometria-densa+correccion-SLAM activo '
                f'({len(self.corrections)} correcciones de keyframe).')
        elif self.trajectory:
            self.get_logger().warning(
                'La trayectoria no trae pose de odom por keyframe; usando '
                'interpolacion lineal de keyframes (modo legacy, menos preciso). '
                'Re-generar el JSON con la 1ª pasada actualizada para mejor mapa.')

        # Buffer de odometria densa (ordenado por timestamp).
        self.odom_t = []
        self.odom_pose = []

        # Dimensiones de la grilla: auto-calculadas desde la trayectoria si está disponible,
        # usando los parámetros explícitos como fallback.
        if self.trajectory:
            margin = self.get_parameter('map_margin').value
            xs = [p['x'] for p in self.trajectory]
            ys = [p['y'] for p in self.trajectory]
            self.origin_x = min(xs) - margin
            self.origin_y = min(ys) - margin
            self.width  = math.ceil((max(xs) + margin - self.origin_x) / self.resolution)
            self.height = math.ceil((max(ys) + margin - self.origin_y) / self.resolution)
            self.get_logger().info(
                f'Grilla auto-calculada: bbox x=[{min(xs):.2f}, {max(xs):.2f}] '
                f'y=[{min(ys):.2f}, {max(ys):.2f}] + margen {margin} m')
        else:
            self.width    = self.get_parameter('width').value
            self.height   = self.get_parameter('height').value
            self.origin_x = self.get_parameter('origin_x').value
            self.origin_y = self.get_parameter('origin_y').value

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
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_cb,
            best_effort,
        )

        self.scans_skipped_no_pose = 0
        self.scans_skipped_turning = 0

        self.get_logger().info(
            f'occupancy_grid_node activo. '
            f'Grilla {self.width}x{self.height} @ {self.resolution} m/cel '
            f'({self.width * self.resolution:.1f}x{self.height * self.resolution:.1f} m). '
            f'Origen ({self.origin_x}, {self.origin_y}). '
            f'LIDAR offset tx={self.lidar_tx} ty={self.lidar_ty} yaw={self.lidar_yaw:.4f}. '
            f'max_angular_velocity={self.max_angular_velocity}.'
        )

    # ------------------------------------------------------------------ #
    #  Utilidades de coordenadas y geometría                               #
    # ------------------------------------------------------------------ #

    def _world_to_grid(self, wx, wy):
        return world_to_grid(wx, wy, self.origin_x, self.origin_y, self.resolution)

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Genera las celdas de la línea de (x0,y0) a (x1,y1) inclusive."""
        return bresenham_cells(x0, y0, x1, y1)

    def _interpolate_pose(self, t):
        """Devuelve (x, y, theta) interpolado linealmente para el timestamp t (segundos)."""
        return interpolate_pose(self.trajectory, t)

    def odom_cb(self, msg: Odometry):
        """Acumula la odometria densa para corregirla con la solucion del SLAM."""
        p = msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # El bag llega en orden; si no, ignoramos la muestra fuera de orden.
        if self.odom_t and t < self.odom_t[-1]:
            return
        self.odom_t.append(t)
        self.odom_pose.append((p.position.x, p.position.y, yaw))

    def _odom_pose_at(self, t):
        """Interpola la odometria densa en el timestamp t. None si no hay datos suficientes."""
        if len(self.odom_t) < 2:
            return None
        if t <= self.odom_t[0]:
            return self.odom_pose[0]
        if t >= self.odom_t[-1]:
            return self.odom_pose[-1]
        hi = bisect.bisect_right(self.odom_t, t)
        lo = hi - 1
        t0, t1 = self.odom_t[lo], self.odom_t[hi]
        alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0.0
        return se2_interpolate(self.odom_pose[lo], self.odom_pose[hi], alpha)

    def _correction_at(self, t):
        """Interpola la correccion SLAM (map<-odom) en el timestamp t."""
        if t <= self.corr_stamps[0]:
            return self.corrections[0]
        if t >= self.corr_stamps[-1]:
            return self.corrections[-1]
        hi = bisect.bisect_right(self.corr_stamps, t)
        lo = hi - 1
        t0, t1 = self.corr_stamps[lo], self.corr_stamps[hi]
        alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0.0
        return se2_interpolate(self.corrections[lo], self.corrections[hi], alpha)

    def _angular_velocity_at(self, t):
        """Velocidad angular (rad/s) de la odometria densa cerca de t. 0 si no hay datos."""
        if len(self.odom_t) < 2:
            return 0.0
        hi = bisect.bisect_right(self.odom_t, t)
        hi = min(max(hi, 1), len(self.odom_t) - 1)
        lo = hi - 1
        dt = self.odom_t[hi] - self.odom_t[lo]
        if dt <= 0:
            return 0.0
        dth = normalize_angle(self.odom_pose[hi][2] - self.odom_pose[lo][2])
        return dth / dt

    def _resolve_pose(self, t):
        """Mejor pose disponible para el scan en t.

        Modo preferido (use_dense_odom): odometria densa corregida por la
        solucion del SLAM => detalle de alta frecuencia sin smearing en giros.
        Fallback legacy: interpolacion lineal de los keyframes sparse.
        """
        if self.use_dense_odom:
            odom = self._odom_pose_at(t)
            if odom is None:
                return None
            return se2_compose(self._correction_at(t), odom)
        return self._interpolate_pose(t)

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

        # Gate de rotacion: no integrar mientras el robot gira rapido.
        if self.max_angular_velocity > 0.0:
            if abs(self._angular_velocity_at(t)) > self.max_angular_velocity:
                self.scans_skipped_turning += 1
                return

        pose = self._resolve_pose(t)
        if pose is None:
            # Odometria densa todavia no disponible para este scan; se descarta.
            self.scans_skipped_no_pose += 1
            return
        self.integrate_scan(pose, msg)
        self.scan_count += 1
        if self.scan_count % self.publish_every == 0:
            self.get_logger().info(
                f'Scans integrados: {self.scan_count} '
                f'(descartados: sin_pose={self.scans_skipped_no_pose}, '
                f'girando={self.scans_skipped_turning})')
            self.publish_map()

    # ------------------------------------------------------------------ #
    #  Publicación y exportación del mapa                                  #
    # ------------------------------------------------------------------ #

    def publish_map(self):
        occ = log_odds_to_occupancy(self.log_odds.flatten())

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
        msg.data = occ
        self.map_pub.publish(msg)

    # Umbrales de generación del PGM. Los mismos valores se escriben en el YAML
    # para que nav2/map_server interprete el mapa exactamente como fue generado.
    # Con negate:0, map_server calcula p=(255-pixel)/255, luego:
    #   pixel   0 → p=1.00 > 0.60 → OCCUPIED
    #   pixel 127 → p=0.50, entre 0.40 y 0.60 → UNKNOWN
    #   pixel 254 → p=0.00 < 0.40 → FREE
    _PGM_OCC_THRESH  = 0.60
    _PGM_FREE_THRESH = 0.40
    _PGM_UNKNOWN_PX  = 127   # p=0.498 ≈ 0.5, cae exactamente entre los dos umbrales

    def export_map(self, path_prefix: str):
        """Exporta la grilla a <prefix>.pgm + <prefix>.yaml (formato nav2 map_server)."""
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))

        pgm = np.full((self.height, self.width), self._PGM_UNKNOWN_PX, dtype=np.uint8)
        pgm[prob >= self._PGM_OCC_THRESH]  = 0
        pgm[prob <= self._PGM_FREE_THRESH] = 254

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
            f.write(f'occupied_thresh: {self._PGM_OCC_THRESH}\n')
            f.write(f'free_thresh: {self._PGM_FREE_THRESH}\n')

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
        if node.map_output:
            node.export_map(node.map_output)
        try:
            node.publish_map()
        except Exception as exc:
            node.get_logger().warn(f'No se pudo publicar el mapa final durante shutdown: {exc}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
