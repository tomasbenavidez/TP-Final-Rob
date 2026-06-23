#!/usr/bin/env python3
"""Detector ROS de conos rojos con depth y fallback monocular."""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from tp_c_mission.cone_perception import (
    ConeTracker, detect_red_regions, estimate_range, pixel_to_camera,
)


class ConeDetector(Node):
    def __init__(self):
        super().__init__('red_cone_detector')
        defaults = {
            'rgb_topic': '/camera/rgb/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/rgb/camera_info',
            'global_frame': 'map',
            'cone_height_m': 0.30,
            'min_area_px': 150,
            'required_hits': 3,
            'window_size': 5,
            'max_center_distance_px': 25.0,
            'min_depth_m': 0.20,
            'max_depth_m': 5.0,
            'depth_scale': 0.001,
            'max_depth_age': 0.20,
            'hue_low': 10, 'hue_high': 170,
            'min_saturation': 100, 'min_value': 70,
            'morphology_radius': 1,
            # Validación geométrica 3D para descartar distractores (consigna 1.2):
            # el rojo del HospitalBot es indistinguible por color/forma, sí por geometría.
            'cone_height_tol_m': 0.13,   # |altura_real - cono| permitido (cono medido ~0.31 m)
            'max_base_height_m': 0.20,   # z máx de la base en 'map' para estar apoyada en el piso
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        self.global_frame = str(get('global_frame'))
        self.cone_height = float(get('cone_height_m'))
        self.min_area = int(get('min_area_px'))
        self.min_depth = float(get('min_depth_m'))
        self.max_depth = float(get('max_depth_m'))
        self.depth_scale = float(get('depth_scale'))
        self.max_depth_age = float(get('max_depth_age'))
        self.hsv = (
            int(get('hue_low')), int(get('hue_high')),
            int(get('min_saturation')), int(get('min_value')),
        )
        self.morphology_radius = int(get('morphology_radius'))
        self.cone_height_tol = float(get('cone_height_tol_m'))
        self.max_base_height = float(get('max_base_height_m'))
        self.tracker = ConeTracker(
            required_hits=int(get('required_hits')),
            window_size=int(get('window_size')),
            max_center_distance_px=float(get('max_center_distance_px')),
        )

        self.bridge = CvBridge()
        self.depth = None
        self.depth_stamp = None
        self.camera_info = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, str(get('depth_topic')), self.depth_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(get('camera_info_topic')), self.info_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, str(get('rgb_topic')), self.rgb_cb,
                                 qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              '/red_cone_pose', 10)
        self.debug_pub = self.create_publisher(Image, '/red_cone/debug_image',
                                               qos_profile_sensor_data)
        self.mask_pub = self.create_publisher(Image, '/red_cone/mask',
                                              qos_profile_sensor_data)
        self.ready_pub = self.create_publisher(Bool, '/red_cone/vision_ready', 10)

    def depth_cb(self, msg):
        depth = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
        scale = self.depth_scale if np.issubdtype(depth.dtype, np.integer) else 1.0
        self.depth = depth.astype(np.float32) * scale
        self.depth_stamp = _stamp_seconds(msg.header.stamp)

    def info_cb(self, msg):
        if len(msg.k) == 9 and msg.k[0] > 0 and msg.k[4] > 0:
            self.camera_info = msg

    def rgb_cb(self, msg):
        image = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        ready = self.camera_info is not None and self.tf_buffer.can_transform(
            self.global_frame, msg.header.frame_id, rclpy.time.Time())
        self.ready_pub.publish(Bool(data=bool(ready)))
        detections, mask = detect_red_regions(
            image, min_area=self.min_area, hue_low=self.hsv[0],
            hue_high=self.hsv[1], min_saturation=self.hsv[2],
            min_value=self.hsv[3], morphology_radius=self.morphology_radius)
        region = detections[0] if detections else None
        confirmed = self.tracker.update(
            None if region is None else (region.center_u, region.center_v))

        debug = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug.header = msg.header
        self.debug_pub.publish(debug)
        mask_msg = self.bridge.cv2_to_imgmsg(mask.astype(np.uint8) * 255, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        if confirmed is None or region is None or self.camera_info is None:
            return
        depth_values = np.array([], dtype=float)
        rgb_stamp = _stamp_seconds(msg.header.stamp)
        depth_fresh = (
            self.depth_stamp is not None
            and abs(rgb_stamp - self.depth_stamp) <= self.max_depth_age)
        if self.depth is not None and self.depth.shape == mask.shape and depth_fresh:
            rows = np.fromiter((item[0] for item in region.pixels), dtype=int)
            cols = np.fromiter((item[1] for item in region.pixels), dtype=int)
            depth_values = self.depth[rows, cols]
        k = self.camera_info.k
        distance, source = estimate_range(
            depth_values, region.height, k[4], self.cone_height,
            self.min_depth, self.max_depth,
        )
        if distance is None:
            return
        # --- Validación geométrica 3D: descartar distractores rojos (consigna 1.2) ---
        # El rojo del HospitalBot es indistinguible por color/forma, pero su geometría no.
        # (1) Altura real implícita ~ altura del cono (sólo confiable con depth real).
        if source == 'depth':
            real_height = distance * region.height / k[4]
            if abs(real_height - self.cone_height) > self.cone_height_tol:
                self.get_logger().info(
                    f'Descarto rojo no-cono: altura real {real_height:.2f} m '
                    f'(cono {self.cone_height:.2f}±{self.cone_height_tol:.2f}).',
                    throttle_duration_sec=2.0)
                return
        camera_point = pixel_to_camera(
            confirmed[0], confirmed[1], distance, k[0], k[4], k[2], k[5])
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Sin TF cámara→{self.global_frame}: {exc}',
                                   throttle_duration_sec=2.0)
            return
        map_point = _transform_point(camera_point, transform)
        # (2) Base apoyada en el piso (z≈0 en 'map'): el cono se para en el suelo,
        #     la cara roja del HospitalBot está elevada. Usa el tercio inferior de la región.
        if source == 'depth':
            base_row = int(rows.max())
            band = rows >= base_row - max(2, region.height // 5)
            base_depths = depth_values[band]
            base_depths = base_depths[np.isfinite(base_depths)
                                      & (base_depths > self.min_depth)
                                      & (base_depths < self.max_depth)]
            if base_depths.size:
                base_cam = pixel_to_camera(
                    float(cols[band].mean()), float(base_row),
                    float(np.median(base_depths)), k[0], k[4], k[2], k[5])
                base_z = _transform_point(base_cam, transform)[2]
                if abs(base_z) > self.max_base_height:
                    self.get_logger().info(
                        f'Descarto rojo no-cono: base a z={base_z:.2f} m '
                        f'(no apoyada en el piso).', throttle_duration_sec=2.0)
                    return
        output = PoseWithCovarianceStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.global_frame
        output.pose.pose.position.x = map_point[0]
        output.pose.pose.position.y = map_point[1]
        output.pose.pose.position.z = map_point[2]
        output.pose.pose.orientation.w = 1.0
        sigma = 0.05 if source == 'depth' else 0.20
        output.pose.covariance[0] = sigma ** 2
        output.pose.covariance[7] = sigma ** 2
        output.pose.covariance[14] = (2.0 * sigma) ** 2
        self.pose_pub.publish(output)


def _transform_point(point, transform):
    x, y, z = point
    q = transform.transform.rotation
    translation = transform.transform.translation
    tx, ty, tz = translation.x, translation.y, translation.z
    # Quaternion-vector rotation, expanded to avoid an extra runtime dependency.
    ix = q.w * x + q.y * z - q.z * y
    iy = q.w * y + q.z * x - q.x * z
    iz = q.w * z + q.x * y - q.y * x
    iw = -q.x * x - q.y * y - q.z * z
    return (
        ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y + tx,
        iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z + ty,
        iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x + tz,
    )


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
