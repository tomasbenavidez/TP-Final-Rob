#!/usr/bin/env python3
"""
aruco_detector_node.py
======================
Detecta marcadores ArUco en el stream de la cámara del TurtleBot4 y estima
la pose relativa (posición + orientación) de cada marcador respecto del robot.

POR QUÉ ESTE NODO EXISTE:
Los ArUco son nuestros LANDMARKS (hitos). Cada vez que el robot "ve" un
marcador con ID conocido, eso es una OBSERVACIÓN: una restricción que dice
"desde mi pose actual, el landmark #ID está a esta distancia y ángulo".
Esas observaciones son las que permiten al Graph SLAM cerrar lazos
(loop closure): si vuelvo a ver el mismo ID que vi hace 100 poses, sé que
estoy cerca de donde estaba antes, y eso corrige la deriva acumulada.

ENTRADA  : sensor_msgs/Image en el tópico de cámara
           + matriz K y coeficientes de distorsión (calibración)
SALIDA   : detecciones (id, tvec, rvec) -> por ahora logueadas/publicadas
           como markers para RViz.

NOTA (enunciado 3): hay que lidiar con motion blur y baja densidad de tags.
La versión robusta (anti motion-blur, rechazo de outliers) es la tarea F2-B
en el tablero; este archivo arranca con la detección base sólida (F2-A).
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# cv_bridge convierte mensajes Image de ROS <-> arrays de OpenCV
from cv_bridge import CvBridge
import cv2

from tp_slam_aruco.aruco_utils import load_camera_calibration, estimate_marker_poses


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # --- Parámetros ---
        self.declare_parameter('image_topic', 'tb4_0/oakd/rgb/image_raw')
        self.declare_parameter('calibration_file', '')  # ruta al yaml de K y coefs
        self.declare_parameter('marker_length', 0.15)    # lado del ArUco en metros
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')

        image_topic = self.get_parameter('image_topic').value
        calib_file = self.get_parameter('calibration_file').value
        self.marker_length = self.get_parameter('marker_length').value
        dict_name = self.get_parameter('aruco_dict').value

        # Cargamos la calibración de la cámara (matriz K + coeficientes de
        # distorsión). Vienen provistas por la cátedra (TB4 #0).
        if calib_file:
            self.camera_matrix, self.dist_coeffs = load_camera_calibration(calib_file)
            self.get_logger().info(f'Calibración cargada desde {calib_file}')
        else:
            # Fallback: sin calibración no podemos estimar distancias reales.
            self.camera_matrix, self.dist_coeffs = None, None
            self.get_logger().warn(
                'Sin archivo de calibración. La detección 2D funcionará, '
                'pero NO la estimación de pose 3D. Pasá calibration_file.'
            )

        # Diccionario de ArUco. El TB4 de la cátedra usa un diccionario
        # concreto; ajustar el parámetro aruco_dict si no detecta nada.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dict_name)
        )
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.detector_params
        )

        self.bridge = CvBridge()

        # Suscripción a la imagen y publicador de markers para visualizar
        # las detecciones en RViz (tópico /landmarks del enunciado).
        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/landmarks', 10)

        self.get_logger().info(
            f'aruco_detector_node escuchando imágenes en "{image_topic}". '
            f'Diccionario={dict_name}, lado={self.marker_length} m.'
        )

    def image_callback(self, msg: Image):
        """Procesa cada frame de la cámara."""
        # Convertimos el mensaje ROS a una imagen OpenCV (BGR).
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detección 2D: devuelve las esquinas de cada marcador y sus IDs.
        corners, ids, _rejected = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return  # ningún marcador en este frame

        # Estimación de pose 3D de cada marcador (requiere calibración).
        detections = []
        if self.camera_matrix is not None:
            detections = estimate_marker_poses(
                corners, ids, self.marker_length,
                self.camera_matrix, self.dist_coeffs
            )
            for det in detections:
                # tvec = (x, y, z) del marcador en el frame de la cámara.
                # z es la profundidad (distancia hacia adelante).
                self.get_logger().info(
                    f'ArUco id={det["id"]}  '
                    f'pos_cam=({det["tvec"][0]:+.2f}, '
                    f'{det["tvec"][1]:+.2f}, {det["tvec"][2]:+.2f}) m'
                )
        else:
            ids_list = [int(i) for i in ids.flatten()]
            self.get_logger().info(f'Detectados IDs (sin pose 3D): {ids_list}')

        # Publicamos los markers para RViz.
        self._publish_markers(detections, msg.header)

    def _publish_markers(self, detections, header):
        """Convierte las detecciones en MarkerArray para visualizar en RViz."""
        marker_array = MarkerArray()
        for det in detections:
            m = Marker()
            m.header = header
            m.ns = 'aruco'
            m.id = int(det['id'])
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(
                x=float(det['tvec'][0]),
                y=float(det['tvec'][1]),
                z=float(det['tvec'][2]),
            )
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.a = 0.9
            marker_array.markers.append(m)
        if marker_array.markers:
            self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
