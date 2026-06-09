#!/usr/bin/env python3

import numpy as np

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from tp_slam_interfaces.msg import (
    LandmarkObservation,
    LandmarkObservationArray,
    VisualObservability,
)
from tp_slam_aruco.aruco_diagnostics import DetectionStatsWindow
from tp_slam_aruco.aruco_runtime import (
    calibration_is_ready,
    calibration_mismatch_summary,
    quadrilateral_area,
    reprojection_error_px,
)
from tp_slam_aruco.aruco_utils import load_camera_calibration, estimate_marker_poses
from tp_slam_aruco.slam_geometry import (
    CameraExtrinsics,
    TB4_CAMERA_EXTRINSICS,
    fallback_camera_to_base_xy,
    transform_stamped_to_base_xy,
)
from tp_slam_aruco.slam_publish import build_accepted_landmark_markers
from tp_slam_aruco.slam_publish import build_raw_landmark_markers


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('image_topic', 'tb4_0/oakd/rgb/preview/image_raw')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('allow_yaml_fallback', False)
        self.declare_parameter('allow_fallback_tf', False)
        self.declare_parameter('marker_length', 0.0889)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('raw_landmarks_topic', '/landmarks_raw')
        self.declare_parameter('accepted_landmarks_topic', '/landmarks_accepted')
        self.declare_parameter('observations_topic', '/landmark_observations')
        self.declare_parameter('min_marker_area_px2', 120.0)
        self.declare_parameter('max_reprojection_error_px', 4.0)
        self.declare_parameter('diagnostics_window_frames', 50)
        self.declare_parameter('diagnostics_topic', '/landmark_detection_stats')
        self.declare_parameter('camera_tx', TB4_CAMERA_EXTRINSICS.tx)
        self.declare_parameter('camera_ty', TB4_CAMERA_EXTRINSICS.ty)
        self.declare_parameter('camera_yaw', TB4_CAMERA_EXTRINSICS.yaw)

        image_topic = self.get_parameter('image_topic').value
        calib_file = self.get_parameter('calibration_file').value
        self.allow_yaml_fallback = self.get_parameter('allow_yaml_fallback').value
        self.allow_fallback_tf = self.get_parameter('allow_fallback_tf').value
        self.marker_length = float(self.get_parameter('marker_length').value)
        dict_name = self.get_parameter('aruco_dict').value
        self.camera_frame = self.get_parameter('camera_frame').value.strip()
        self.base_frame = self.get_parameter('base_frame').value
        raw_landmarks_topic = self.get_parameter('raw_landmarks_topic').value
        accepted_landmarks_topic = self.get_parameter(
            'accepted_landmarks_topic'
        ).value
        observations_topic = self.get_parameter('observations_topic').value
        self.min_marker_area_px2 = float(self.get_parameter('min_marker_area_px2').value)
        self.max_reprojection_error_px = float(
            self.get_parameter('max_reprojection_error_px').value
        )
        diagnostics_window_frames = int(
            self.get_parameter('diagnostics_window_frames').value
        )
        diagnostics_topic = self.get_parameter('diagnostics_topic').value
        self.camera_extrinsics = CameraExtrinsics(
            tx=self.get_parameter('camera_tx').value,
            ty=self.get_parameter('camera_ty').value,
            yaw=self.get_parameter('camera_yaw').value,
        )

        self.camera_matrix = None
        self.dist_coeffs = None
        self.yaml_camera_matrix = None
        self.yaml_dist_coeffs = None
        self._calib_from_topic = False
        self._warned_missing_calibration = False
        self._warned_tf_fallback = False
        self._reported_yaml_mismatch = False
        self._warned_single_landmark_window = False
        self._diagnostic_frame_count = 0
        self.detection_stats = DetectionStatsWindow(
            window_size=diagnostics_window_frames
        )

        if calib_file:
            try:
                yaml_k, yaml_d, marker_size = load_camera_calibration(calib_file)
                self.yaml_camera_matrix = yaml_k
                self.yaml_dist_coeffs = yaml_d
                if self.allow_yaml_fallback:
                    self.camera_matrix = yaml_k
                    self.dist_coeffs = yaml_d
                self.get_logger().info(
                    f'Calibración YAML cargada desde {calib_file} '
                    f'(fallback habilitado={self.allow_yaml_fallback}).'
                )
                if marker_size is not None:
                    self.marker_length = float(marker_size)
            except Exception as exc:
                self.get_logger().warning(f'No se pudo cargar calibración YAML: {exc}')

        half = self.marker_length / 2.0
        self.marker_object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        info_topic = image_topic.replace('image_raw', 'camera_info')
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CameraInfo, info_topic, self._camera_info_cb, best_effort)

        aruco_dict_id = getattr(cv2.aruco, dict_name)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            self._detect_markers = detector.detectMarkers
        else:
            dictionary = cv2.aruco.Dictionary_get(aruco_dict_id)
            params = cv2.aruco.DetectorParameters_create()
            self._detect_markers = lambda gray: cv2.aruco.detectMarkers(
                gray, dictionary, parameters=params
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.raw_marker_pub = self.create_publisher(
            MarkerArray, raw_landmarks_topic, 10
        )
        self.accepted_marker_pub = self.create_publisher(
            MarkerArray, accepted_landmarks_topic, 10
        )
        self.obs_pub = self.create_publisher(LandmarkObservationArray, observations_topic, 10)
        self.diagnostics_pub = self.create_publisher(VisualObservability, diagnostics_topic, 10)

        camera_frame_mode = (
            f'override={self.camera_frame}'
            if self.camera_frame
            else 'override=<disabled; using image.header.frame_id>'
        )
        self.get_logger().info(
            f'aruco_detector_node activo. image={image_topic} camera_info={info_topic} '
            f'raw={raw_landmarks_topic} observations={observations_topic} '
            f'accepted={accepted_landmarks_topic} diagnostics={diagnostics_topic} '
            f'camera_frame={camera_frame_mode} qos(image/camera_info)=BEST_EFFORT'
        )

    def _camera_info_cb(self, msg):
        if self._calib_from_topic:
            return

        if len(msg.k) != 9 or len(msg.d) == 0:
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)
        self._calib_from_topic = True

        self.get_logger().info(
            f'Calibración recibida desde camera_info '
            f'(modelo={msg.distortion_model}, {len(msg.d)} coefs). '
            f'fx={msg.k[0]:.2f} fy={msg.k[4]:.2f} cx={msg.k[2]:.2f} cy={msg.k[5]:.2f}'
        )

        if (
            self.yaml_camera_matrix is not None
            and self.yaml_dist_coeffs is not None
            and not self._reported_yaml_mismatch
        ):
            summary = calibration_mismatch_summary(
                yaml_camera_matrix=self.yaml_camera_matrix,
                yaml_dist_coeffs=self.yaml_dist_coeffs,
                topic_camera_matrix=self.camera_matrix,
                topic_dist_coeffs=self.dist_coeffs,
            )
            if summary:
                self.get_logger().warning(
                    f'La calibración YAML difiere del camera_info real: {summary}'
                )
            self._reported_yaml_mismatch = True

    def image_callback(self, msg):
        active_camera_frame = self.camera_frame or msg.header.frame_id

        if not calibration_is_ready(
            has_topic_calibration=self._calib_from_topic,
            has_yaml_calibration=self.yaml_camera_matrix is not None,
            allow_yaml_fallback=self.allow_yaml_fallback,
        ):
            if not self._warned_missing_calibration:
                self.get_logger().warning(
                    'Todavía no hay camera_info válido; se descartan detecciones ArUco.'
                )
                self._warned_missing_calibration = True
            self._publish_detection_diagnostics(
                stamp=msg.header.stamp,
                raw_count=0,
                raw_ids=[],
                valid_ids=[],
                rejected_no_calibration=1,
            )
            self.raw_marker_pub.publish(
                build_raw_landmark_markers(active_camera_frame, msg.header.stamp, [])
            )
            self.accepted_marker_pub.publish(
                build_accepted_landmark_markers(
                    active_camera_frame, msg.header.stamp, []
                )
            )
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Error convirtiendo imagen: {exc}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._detect_markers(gray)

        if ids is None or len(ids) == 0:
            self._publish_detection_diagnostics(
                stamp=msg.header.stamp,
                raw_count=0,
                raw_ids=[],
                valid_ids=[],
            )
            self.raw_marker_pub.publish(
                build_raw_landmark_markers(active_camera_frame, msg.header.stamp, [])
            )
            self.accepted_marker_pub.publish(
                build_accepted_landmark_markers(
                    active_camera_frame, msg.header.stamp, []
                )
            )
            return

        raw_count = len(ids)
        raw_ids = [int(tag_id[0]) for tag_id in ids]
        detections = estimate_marker_poses(
            corners,
            ids,
            self.marker_length,
            self.camera_matrix,
            self.dist_coeffs,
        )
        self.raw_marker_pub.publish(
            build_raw_landmark_markers(active_camera_frame, msg.header.stamp, detections)
        )

        accepted_detections = []
        accepted_ids = []
        rejected_area_ids = []
        rejected_depth_ids = []
        rejected_reprojection_ids = []
        rejected_tf_ids = []
        rejected_area = 0
        rejected_depth = 0
        rejected_reprojection = 0
        rejected_tf = 0
        observations = LandmarkObservationArray()
        observations.header.stamp = msg.header.stamp
        observations.header.frame_id = self.base_frame

        for det in detections:
            area_px2 = quadrilateral_area(det['image_points'])
            if area_px2 < self.min_marker_area_px2:
                rejected_area += 1
                rejected_area_ids.append(int(det['id']))
                continue

            tvec = det['tvec']
            tz = float(tvec[2])
            if tz <= 0.15 or tz > 5.0:
                rejected_depth += 1
                rejected_depth_ids.append(int(det['id']))
                continue

            reproj_error = reprojection_error_px(
                object_points=self.marker_object_points,
                image_points=det['image_points'],
                rvec=det['rvec'],
                tvec=det['tvec'],
                camera_matrix=self.camera_matrix,
                dist_coeffs=self.dist_coeffs,
            )
            if reproj_error > self.max_reprojection_error_px:
                rejected_reprojection += 1
                rejected_reprojection_ids.append(int(det['id']))
                continue

            try:
                x_base, y_base, used_fallback_tf = self._transform_detection_to_base(
                    frame_id=active_camera_frame,
                    stamp=msg.header.stamp,
                    tvec=det['tvec'],
                )
            except RuntimeError:
                rejected_tf += 1
                rejected_tf_ids.append(int(det['id']))
                continue

            observation = LandmarkObservation()
            observation.header.stamp = msg.header.stamp
            observation.header.frame_id = self.base_frame
            observation.landmark_id = int(det['id'])
            observation.range_m = float(np.hypot(x_base, y_base))
            observation.bearing_rad = float(np.arctan2(y_base, x_base))
            observation.x_base = float(x_base)
            observation.y_base = float(y_base)
            observation.depth_m = tz
            observation.reprojection_error_px = float(reproj_error)
            observation.used_fallback_tf = bool(used_fallback_tf)
            observation.source_frame = active_camera_frame
            observations.observations.append(observation)
            accepted_detections.append(det)
            accepted_ids.append(int(det['id']))

        self._publish_detection_diagnostics(
            stamp=msg.header.stamp,
            raw_count=raw_count,
            raw_ids=raw_ids,
            valid_ids=accepted_ids,
            rejected_area=rejected_area,
            rejected_area_ids=rejected_area_ids,
            rejected_depth=rejected_depth,
            rejected_depth_ids=rejected_depth_ids,
            rejected_reprojection=rejected_reprojection,
            rejected_reprojection_ids=rejected_reprojection_ids,
            rejected_tf=rejected_tf,
            rejected_tf_ids=rejected_tf_ids,
        )
        self.accepted_marker_pub.publish(
            build_accepted_landmark_markers(
                active_camera_frame, msg.header.stamp, accepted_detections
            )
        )
        if observations.observations:
            self.obs_pub.publish(observations)

    def _publish_detection_diagnostics(
        self,
        stamp,
        raw_count,
        raw_ids,
        valid_ids,
        rejected_area=0,
        rejected_area_ids=None,
        rejected_depth=0,
        rejected_depth_ids=None,
        rejected_reprojection=0,
        rejected_reprojection_ids=None,
        rejected_tf=0,
        rejected_tf_ids=None,
        rejected_no_calibration=0,
    ):
        self._diagnostic_frame_count += 1
        msg = VisualObservability()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.raw_count = int(raw_count)
        msg.valid_count = len(valid_ids)
        msg.valid_unique_count = len(set(valid_ids))
        msg.rejected_area = int(rejected_area)
        msg.rejected_depth = int(rejected_depth)
        msg.rejected_reprojection = int(rejected_reprojection)
        msg.rejected_tf = int(rejected_tf)
        msg.rejected_no_calibration = int(rejected_no_calibration)
        msg.raw_ids = [int(x) for x in (raw_ids or [])]
        msg.valid_ids = [int(x) for x in (valid_ids or [])]
        msg.rejected_area_ids = [int(x) for x in (rejected_area_ids or [])]
        msg.rejected_depth_ids = [int(x) for x in (rejected_depth_ids or [])]
        msg.rejected_reprojection_ids = [
            int(x) for x in (rejected_reprojection_ids or [])
        ]
        msg.rejected_tf_ids = [int(x) for x in (rejected_tf_ids or [])]
        self.detection_stats.observe(frame=msg)
        self.diagnostics_pub.publish(msg)

        summary = self.detection_stats.summary()
        if (
            summary.frames >= self.detection_stats.window_size
            and self._diagnostic_frame_count % self.detection_stats.window_size == 0
        ):
            self.get_logger().info(
                'Diagnóstico ArUco: '
                f'frames={summary.frames} '
                f'avg_raw={summary.avg_raw:.2f} '
                f'avg_valid={summary.avg_valid:.2f} '
                f'max_valid={summary.max_valid}'
            )
            if self.detection_stats.should_warn_low_multilandmark():
                if not self._warned_single_landmark_window:
                    self.get_logger().warning(
                        'En la ventana reciente casi no hubo múltiples landmarks válidos '
                        'simultáneos (max_valid <= 1).'
                    )
                    self._warned_single_landmark_window = True
            else:
                self._warned_single_landmark_window = False

    def _transform_detection_to_base(self, frame_id, stamp, tvec):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame_id,
                stamp,
                timeout=Duration(seconds=0.05),
            )
            x_base, y_base = transform_stamped_to_base_xy(transform, tuple(tvec))
            return x_base, y_base, False
        except Exception:
            if not self.allow_fallback_tf:
                raise RuntimeError('camera tf unavailable')

        if not self._warned_tf_fallback:
            self.get_logger().warning(
                'No se pudo resolver TF cámara->base_link; usando extrínsecos fallback.'
            )
            self._warned_tf_fallback = True

        x_base, y_base = fallback_camera_to_base_xy(
            tx=float(tvec[0]),
            tz=float(tvec[2]),
            extrinsics=self.camera_extrinsics,
        )
        return x_base, y_base, True


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
