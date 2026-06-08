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
from sensor_msgs.msg import Image, CameraInfo
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
        self.declare_parameter('image_topic', 'tb4_0/oakd/rgb/preview/image_raw')
        self.declare_parameter('calibration_file', '')  # ruta al yaml de K y coefs (fallback)
        self.declare_parameter('marker_length', 0.0889)  # lado del ArUco en metros
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        # frame_id del frame de la cámara; si está vacío se usa el del mensaje.
        self.declare_parameter('camera_frame', '')

        image_topic = self.get_parameter('image_topic').value
        calib_file = self.get_parameter('calibration_file').value
        self.marker_length = self.get_parameter('marker_length').value
        dict_name = self.get_parameter('aruco_dict').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # Calibración de la cámara: se obtiene del tópico camera_info (fuente
        # canónica y siempre consistente con el bag). El YAML es fallback por
        # si el tópico no está disponible. Nota: el modelo rational_polynomial
        # del OAK-D usa 8 coeficientes de distorsión, no 5.
        self.camera_matrix = None
        self.dist_coeffs = None
        self._calib_from_topic = False  # True cuando ya recibimos camera_info

        if calib_file:
            try:
                self.camera_matrix, self.dist_coeffs, marker_size = load_camera_calibration(calib_file)
                self.get_logger().info(f'Calibración fallback cargada desde {calib_file}')
                if marker_size is not None:
                    try:
                        self.marker_length = float(marker_size)
                    except Exception:
                        pass
            except Exception as e:
                self.get_logger().warn(f'No se pudo cargar calibración YAML: {e}')

        # Tópico camera_info: se infiere del tópico de imagen (convención ROS).
        # Si la imagen es /foo/image_raw, el camera_info es /foo/camera_info.
        info_topic = image_topic.replace('image_raw', 'camera_info')
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
        _best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CameraInfo, info_topic, self._camera_info_cb, _best_effort)
        self.get_logger().info(f'Esperando calibración desde "{info_topic}"...')

        # Diccionario de ArUco. El TB4 de la cátedra usa un diccionario
        # concreto; ajustar el parámetro aruco_dict si no detecta nada.
        # Soporte para OpenCV 4.5 (API legacy) y 4.6+ (API nueva con ArucoDetector).
        aruco_dict_id = getattr(cv2.aruco, dict_name)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            # OpenCV >= 4.6
            _dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
            _params = cv2.aruco.DetectorParameters()
            _det = cv2.aruco.ArucoDetector(_dict, _params)
            self._detect_markers = _det.detectMarkers
        else:
            # OpenCV 4.5.x — API legacy todavía funcional
            _dict = cv2.aruco.Dictionary_get(aruco_dict_id)
            _params = cv2.aruco.DetectorParameters_create()
            self._detect_markers = lambda gray: cv2.aruco.detectMarkers(
                gray, _dict, parameters=_params
            )

        self.bridge = CvBridge()

        # Suscripción a la imagen y publicador de markers para visualizar
        # las detecciones en RViz (tópico /landmarks del enunciado).
        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/landmarks', 10)

        self.get_logger().info(
            f'aruco_detector_node escuchando imágenes en "{image_topic}". '
            f'Diccionario={dict_name}, lado={self.marker_length} m.'
        )

    def _camera_info_cb(self, msg: CameraInfo):
        """Toma la calibración directamente del tópico camera_info del driver.

        Es la fuente canónica: siempre coincide con el bag/cámara real,
        soporta el modelo rational_polynomial (8 coeficientes) del OAK-D,
        y evita errores por YAML desactualizado. Se ejecuta solo hasta
        recibir el primer mensaje válido.
        """
        if self._calib_from_topic:
            return  # ya inicializado; no recalcular en cada frame

        K = msg.k  # float64[9], row-major
        D = msg.d  # float64[], variable length (5 o 8 según el modelo)

        if len(K) != 9 or len(D) == 0:
            return

        self.camera_matrix = np.array(K, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(D, dtype=np.float64).reshape(1, -1)
        self._calib_from_topic = True

        self.get_logger().info(
            f'Calibración recibida desde camera_info '
            f'(modelo={msg.distortion_model}, {len(D)} coefs). '
            f'fx={K[0]:.2f} fy={K[4]:.2f} cx={K[2]:.2f} cy={K[5]:.2f}'
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
        corners, ids, _rejected = self._detect_markers(gray)

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
            # Usamos el frame explícito si fue configurado; si no, confiamos
            # en el frame_id que venga en el header de la imagen (lo provee
            # el driver de cámara). graph_slam_node usa este frame_id para
            # hacer el TF lookup cámara→base_link.
            if self.camera_frame:
                m.header.frame_id = self.camera_frame
            m.ns = 'aruco'
            m.id = int(det['id'])
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(
                x=float(det['tvec'][0]),
                y=float(det['tvec'][1]),
                z=float(det['tvec'][2]),
            )
            # Orientación real del marcador: convertimos el rvec (Rodrigues)
            # a quaternion para visualizar la pose completa en RViz.
            qx, qy, qz, qw = self._rvec_to_quaternion(det['rvec'])
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.a = 0.9
            marker_array.markers.append(m)
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    @staticmethod
    def _rvec_to_quaternion(rvec):
        """Convierte un vector de Rodrigues a quaternion (x, y, z, w)."""
        R, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64))
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return x, y, z, w


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
