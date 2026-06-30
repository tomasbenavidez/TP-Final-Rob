#!/usr/bin/env python3
"""landmark_sensor: sensor virtual de landmarks (Sistema 3).

Emula el detector de cámara/ArUco que NO existe en Gazebo. Para cada landmark conocido
(/landmarks, frame map) calcula la observación range/bearing en el frame del robot, y
publica /observed_landmarks aplicando:

  - FOV frontal y rango de una cámara virtual configurable.
  - Oclusión / línea de visión conservadora usando el rayo central del LIDAR.
  - Ruido gaussiano en range y bearing (modelo de medición).

Formato de salida (compatible con el consumidor MCL, igual que tp4/tp5):
  Pose.position.x = range r   (con ruido)
  Pose.position.z = bearing θ (con ruido)
  Pose.position.y = 0
  Landmark no visible / ocluido -> (0, 0, 0).

Portado de tps_viejos/tp5/tp5/feature_finder.py, parametrizando ruido y frames.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from tp_b_navigation.landmark_visibility import (
    camera_point_from_base,
    visibility_reason,
)


def do_transform_point_2d(point, transform):
    """Transforma un punto 3D aplicando sólo traslación + rotación en yaw (caso planar).

    Equivale a tf2_geometry_msgs.do_transform_point para el plano, evitando dependencias
    pesadas. 'transform' es un TransformStamped (target <- source).
    """
    t = transform.transform.translation
    q = transform.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    x = point.point.x
    y = point.point.y
    out = PointStamped()
    out.header = transform.header
    out.point.x = cos_y * x - sin_y * y + t.x
    out.point.y = sin_y * x + cos_y * y + t.y
    out.point.z = point.point.z + t.z
    return out


class LandmarkSensor(Node):
    def __init__(self):
        super().__init__('landmark_sensor')

        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('sigma_range', 0.05)      # σ ruido de distancia [m]
        self.declare_parameter('sigma_bearing', 0.05)    # σ ruido de ángulo [rad]
        self.declare_parameter('occlusion_tol', 0.08)
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('camera_tx', -0.06)
        self.declare_parameter('camera_ty', 0.0)
        self.declare_parameter('camera_yaw', 0.0)
        self.declare_parameter('camera_fov', 1.05)
        self.declare_parameter('camera_max_range', 3.0)
        self.declare_parameter('log_rejections', False)
        self.declare_parameter('publish_markers', True)
        # Frame de pose VERDADERA para emular la cámara. Una cámara real ve los landmarks
        # según la pose real del robot, NO según la estimación del filtro. Usar la estimación
        # (map->robot) realimenta el MCL consigo mismo y lo hace diverger. En Gazebo la
        # odometría es ~ground-truth y map≡odom en el origen, así que proyectamos los
        # landmarks (coords en 'map') usando truth_frame->robot.
        self.declare_parameter('truth_frame', 'odom')

        self.robot_frame = self.get_parameter('robot_frame').value
        self.truth_frame = self.get_parameter('truth_frame').value
        self.sigma_r = float(self.get_parameter('sigma_range').value)
        self.sigma_b = float(self.get_parameter('sigma_bearing').value)
        self.occ_tol = float(self.get_parameter('occlusion_tol').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.camera_tx = float(self.get_parameter('camera_tx').value)
        self.camera_ty = float(self.get_parameter('camera_ty').value)
        self.camera_yaw = float(self.get_parameter('camera_yaw').value)
        self.camera_fov = float(self.get_parameter('camera_fov').value)
        self.camera_max_range = float(
            self.get_parameter('camera_max_range').value)
        self.log_rejections = bool(self.get_parameter('log_rejections').value)
        self.publish_markers = bool(self.get_parameter('publish_markers').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # /landmarks es latcheado -> mismo QoS para recibirlo aunque arranque después.
        qos_latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_lm = self.create_subscription(
            PoseArray, '/landmarks', self.landmarks_callback, qos_latched)

        self.pub_obs = self.create_publisher(PoseArray, '/observed_landmarks', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/observed_landmarks_markers', 10)

        self.map_landmarks = None
        self.get_logger().info(
            f'Sensor virtual de landmarks activo (robot_frame={self.robot_frame}, '
            f'camera_frame={self.camera_frame or "fallback"}, '
            f'FOV={self.camera_fov:.2f} rad, σ_r={self.sigma_r}, '
            f'σ_θ={self.sigma_b}).')

    def landmarks_callback(self, msg: PoseArray):
        self.map_landmarks = msg

    def scan_callback(self, msg: LaserScan):
        if self.map_landmarks is None:
            return

        # Transformación pose-verdadera -> robot. Se usa truth_frame (odom, ground-truth en
        # Gazebo) en vez de map para NO realimentar el MCL con su propia estimación.
        # Los landmarks vienen en 'map', que coincide con truth_frame en el origen.
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.robot_frame,
                source_frame=self.truth_frame,
                time=rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF no disponible aún: {e}', throttle_duration_sec=2.0)
            return

        camera_in_base = self._camera_in_base()

        obs = PoseArray()
        # Estampar con el tiempo de la TF que REALMENTE usamos (último disponible), no con el
        # stamp del /scan: el LIDAR de Gazebo estampa el scan ~20-60 ms adelante de su propia
        # TF odom->base, así que publicar con el stamp del scan deja la observación "en el
        # futuro" para la TF y RViz parpadea con "extrapolation into the future". Usar el stamp
        # de la transform garantiza que la cadena map->odom->base sea válida en ese instante.
        obs.header.stamp = transform.header.stamp
        obs.header.frame_id = self.robot_frame

        angle_min = msg.angle_min
        r_min = msg.range_min
        r_max = msg.range_max

        for landmark in self.map_landmarks.poses:
            pt = PointStamped()
            pt.header = self.map_landmarks.header
            pt.point = landmark.position
            pr = do_transform_point_2d(pt, transform)
            lx, ly = pr.point.x, pr.point.y

            r = math.hypot(lx, ly)
            theta = math.atan2(ly, lx)
            camera_point = camera_point_from_base(
                (lx, ly), *camera_in_base)

            pose = Pose()
            reason = visibility_reason(
                base_point=(lx, ly),
                camera_point=camera_point,
                ranges=msg.ranges,
                angle_min=angle_min,
                angle_increment=msg.angle_increment,
                range_min=r_min,
                range_max=r_max,
                camera_fov=self.camera_fov,
                camera_max_range=self.camera_max_range,
                occlusion_tol=self.occ_tol,
            )
            if reason == 'visible':
                pose.position.x = float(r + np.random.normal(0.0, self.sigma_r))
                pose.position.y = 0.0
                pose.position.z = float(theta + np.random.normal(0.0, self.sigma_b))
            else:
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                if self.log_rejections:
                    self.get_logger().debug(
                        f'Landmark rechazado: {reason}, '
                        f'r={r:.2f}, bearing={theta:.2f}')
            obs.poses.append(pose)

        self.pub_obs.publish(obs)
        if self.publish_markers:
            self._publish_markers(obs)

    def _camera_in_base(self):
        """Retorna la extrínseca planar cámara en base, prefiriendo TF real."""
        if self.camera_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    self.camera_frame,
                    rclpy.time.Time(),
                )
                t = transform.transform.translation
                q = transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                return float(t.x), float(t.y), float(yaw)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f'Sin TF {self.camera_frame}→{self.robot_frame}; '
                    f'usando extrínseca fallback: {exc}',
                    throttle_duration_sec=2.0,
                )
        return self.camera_tx, self.camera_ty, self.camera_yaw

    def _publish_markers(self, obs: PoseArray):
        ma = MarkerArray()
        m = Marker()
        m.header = obs.header
        m.ns = 'observed_landmarks'
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.12
        m.scale.y = 0.12
        m.color.r = 1.0
        m.color.g = 0.6
        m.color.b = 0.0
        m.color.a = 1.0
        for pose in obs.poses:
            r = pose.position.x
            theta = pose.position.z
            if r == 0.0 and theta == 0.0:
                continue
            pt = PointStamped().point
            pt.x = r * math.cos(theta)
            pt.y = r * math.sin(theta)
            pt.z = 0.1
            m.points.append(pt)
        ma.markers.append(m)
        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkSensor()
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
