#!/usr/bin/env python3
"""landmark_publisher: publica las posiciones FIJAS de los landmarks virtuales del
mundo (Sistema 3) como geometry_msgs/PoseArray en /landmarks, en el frame 'map'.

En Parte B los landmarks son CONOCIDOS (no se estiman): emulan los marcadores ArUco
del mundo real de la Parte A. La cantidad y distribución deben ser coherentes con la
densidad real observada (lineamiento de la consigna). Las coordenadas se cargan desde
un parámetro 'landmarks' (lista plana [x1,y1,x2,y2,...] en metros, frame map), que se
provee normalmente desde config/landmarks.yaml.

Publica con QoS transient_local (latcheado) y además re-publica periódicamente, para
que el sensor virtual y RViz lo reciban aunque arranquen después.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray


# Set por defecto si no se pasa el parámetro 'landmarks'. Extraídos de las esquinas de las
# paredes de mapas/map.pgm: deben estar SOBRE paredes para que el sensor virtual (línea de
# visión) los observe. AJUSTAR para igualar la densidad real de ArUco de la Parte A.
DEFAULT_LANDMARKS = [
    -2.96, -2.14,  -2.91, 2.86,  -2.86, -0.69,  -2.86, 1.71,
    -2.21, -0.19,  -1.91, -1.99, -1.56, -0.89,  -1.01, 0.56,
    -0.91, -2.49,  -0.86, -1.44,  0.14, 1.71,    0.39, -2.99,
     0.79,  0.31,   0.99, -2.54,
]


class LandmarkPublisher(Node):
    def __init__(self):
        super().__init__('landmark_publisher')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('landmarks', DEFAULT_LANDMARKS)

        self.frame_id = self.get_parameter('frame_id').value
        flat = list(self.get_parameter('landmarks').value)
        if len(flat) % 2 != 0:
            self.get_logger().warn('Lista de landmarks impar; ignoro el último valor.')
            flat = flat[:-1]
        self.points = np.array(flat, dtype=float).reshape((-1, 2))

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(PoseArray, '/landmarks', qos)
        # Marcadores sólo para visualización en RViz (no los consume el filtro).
        self.marker_pub = self.create_publisher(MarkerArray, '/landmarks_markers', qos)

        self.pose_array = self._build_pose_array()
        self.marker_array = self._build_markers()
        self._publish()

        period = float(self.get_parameter('publish_period_sec').value)
        if period > 0.0:
            self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Publicados {len(self.points)} landmarks virtuales en /landmarks '
            f'(frame {self.frame_id}).')

    def _build_pose_array(self):
        msg = PoseArray()
        msg.header.frame_id = self.frame_id
        for x, y in self.points:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 0.0
            p.orientation.w = 1.0
            msg.poses.append(p)
        return msg

    def _build_markers(self):
        ma = MarkerArray()
        for i, (x, y) in enumerate(self.points):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.ns = 'landmarks_gt'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.12
            m.scale.y = 0.12
            m.scale.z = 0.2
            m.color.r = 0.1
            m.color.g = 0.8
            m.color.b = 0.1
            m.color.a = 0.9
            ma.markers.append(m)
        return ma

    def _publish(self):
        now = self.get_clock().now().to_msg()
        self.pose_array.header.stamp = now
        for m in self.marker_array.markers:
            m.header.stamp = now
        self.pub.publish(self.pose_array)
        self.marker_pub.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkPublisher()
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
