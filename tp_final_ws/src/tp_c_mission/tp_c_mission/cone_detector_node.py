#!/usr/bin/env python3
"""Detector ROS de conos rojos con depth, LIDAR o fallback monocular."""

import math
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from tp_c_mission.cone_perception import (
    ConeTracker,
    detect_red_regions,
    estimate_range,
    pixel_bearing_rad,
    pixel_to_camera,
    select_lidar_range,
)


def evaluate_vision_readiness(
    *,
    rgb_fresh,
    camera_info_valid,
    rgb_shape,
    camera_info_shape,
    depth_shape,
    rgb_stamp,
    depth_stamp,
    max_depth_age,
    range_source,
    scan_fresh,
    tf_available,
    require_aligned_depth,
):
    if not rgb_fresh:
        return 'rgb_stale'
    if not camera_info_valid:
        return 'camera_info_invalid'
    if camera_info_shape != rgb_shape:
        return 'camera_info_misaligned'
    if range_source == 'lidar' and not scan_fresh:
        return 'scan_stale'
    if require_aligned_depth or range_source == 'depth':
        if depth_shape is None or depth_stamp is None:
            return 'depth_missing'
        if depth_shape != rgb_shape:
            return 'depth_misaligned'
        if abs(rgb_stamp - depth_stamp) > max_depth_age:
            return 'depth_stale'
    if not tf_available:
        return 'tf_unavailable'
    return None


class ConeDetector(Node):
    def __init__(self):
        super().__init__('red_cone_detector')
        defaults = {
            'rgb_topic': '/camera/rgb/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/rgb/camera_info',
            'scan_topic': '/scan',
            'global_frame': 'map',
            'range_source': 'depth',
            'cone_height_m': 0.30,
            'min_area_px': 150,
            'required_hits': 3,
            'window_size': 5,
            'max_center_distance_px': 25.0,
            'min_depth_m': 0.20,
            'max_depth_m': 5.0,
            'depth_scale': 0.001,
            'max_depth_age': 0.20,
            'max_scan_age': 0.30,
            'lidar_bearing_window_rad': 0.08,
            'min_lidar_points': 1,
            'require_aligned_depth': False,
            'allow_latest_tf_fallback': False,
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
        self.max_scan_age = float(get('max_scan_age'))
        self.lidar_bearing_window = float(get('lidar_bearing_window_rad'))
        self.min_lidar_points = int(get('min_lidar_points'))
        self.range_source = str(get('range_source')).lower()
        if self.range_source not in ('depth', 'lidar', 'monocular'):
            raise ValueError('range_source debe ser depth, lidar o monocular')
        self.require_aligned_depth = bool(get('require_aligned_depth'))
        self.allow_latest_tf_fallback = bool(get('allow_latest_tf_fallback'))
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
        self.last_scan = None
        self.last_scan_stamp = None
        self.camera_info = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.range_source == 'depth':
            self.create_subscription(Image, str(get('depth_topic')), self.depth_cb,
                                     qos_profile_sensor_data)
        if self.range_source == 'lidar':
            self.create_subscription(LaserScan, str(get('scan_topic')), self.scan_cb,
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

    def scan_cb(self, msg):
        self.last_scan = msg
        self.last_scan_stamp = _stamp_seconds(msg.header.stamp)

    def depth_cb(self, msg):
        depth = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
        scale = self.depth_scale if np.issubdtype(depth.dtype, np.integer) else 1.0
        self.depth = depth.astype(np.float32) * scale
        self.depth_stamp = _stamp_seconds(msg.header.stamp)

    def info_cb(self, msg):
        if (
            len(msg.k) == 9
            and msg.k[0] > 0
            and msg.k[4] > 0
            and msg.width > 0
            and msg.height > 0
        ):
            self.camera_info = msg

    def rgb_cb(self, msg):
        image = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        measurement_time = rclpy.time.Time.from_msg(msg.header.stamp)
        tf_available = self.tf_buffer.can_transform(
            self.global_frame,
            msg.header.frame_id,
            measurement_time,
        )
        if not tf_available and self.allow_latest_tf_fallback:
            tf_available = self.tf_buffer.can_transform(
                self.global_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
            )
        if self.range_source == 'lidar' and self.last_scan is not None:
            scan_time = rclpy.time.Time.from_msg(self.last_scan.header.stamp)
            scan_frame = self.last_scan.header.frame_id
            tf_available = (
                tf_available
                and self.tf_buffer.can_transform(
                    scan_frame,
                    msg.header.frame_id,
                    measurement_time,
                )
                and self.tf_buffer.can_transform(
                    self.global_frame,
                    scan_frame,
                    scan_time,
                )
            )
        camera_info_shape = (
            None if self.camera_info is None
            else (self.camera_info.height, self.camera_info.width)
        )
        rgb_stamp = _stamp_seconds(msg.header.stamp)
        scan_fresh = (
            self.last_scan_stamp is not None
            and abs(rgb_stamp - self.last_scan_stamp) <= self.max_scan_age
        )
        ready_reason = evaluate_vision_readiness(
            rgb_fresh=True,
            camera_info_valid=self.camera_info is not None,
            rgb_shape=image.shape[:2],
            camera_info_shape=camera_info_shape,
            depth_shape=None if self.depth is None else self.depth.shape,
            rgb_stamp=rgb_stamp,
            depth_stamp=self.depth_stamp,
            max_depth_age=self.max_depth_age,
            range_source=self.range_source,
            scan_fresh=scan_fresh,
            tf_available=tf_available,
            require_aligned_depth=self.require_aligned_depth,
        )
        self.ready_pub.publish(Bool(data=ready_reason is None))
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

        if (
            confirmed is None
            or region is None
            or self.camera_info is None
            or (
                (self.require_aligned_depth or self.range_source == 'lidar')
                and ready_reason is not None
            )
        ):
            return
        depth_values = np.array([], dtype=float)
        depth_fresh = (
            self.depth_stamp is not None
            and abs(rgb_stamp - self.depth_stamp) <= self.max_depth_age)
        if self.depth is not None and self.depth.shape == mask.shape and depth_fresh:
            rows = np.fromiter((item[0] for item in region.pixels), dtype=int)
            cols = np.fromiter((item[1] for item in region.pixels), dtype=int)
            depth_values = self.depth[rows, cols]
        k = self.camera_info.k
        if self.range_source == 'lidar':
            map_point = self._map_point_from_lidar(
                confirmed,
                k,
                msg.header.frame_id,
                measurement_time,
            )
            source = 'lidar'
            if map_point is None:
                return
        else:
            distance, source = estimate_range(
                depth_values, region.height, k[4], self.cone_height,
                self.min_depth, self.max_depth,
            )
            if self.range_source == 'depth' and source != 'depth':
                return
            if self.range_source == 'monocular' and source == 'depth':
                source = 'monocular'
            if distance is None:
                return
            camera_point = pixel_to_camera(
                confirmed[0], confirmed[1], distance, k[0], k[4], k[2], k[5])
            try:
                transform = self._lookup_transform(
                    msg.header.frame_id,
                    measurement_time,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Sin TF cámara→{self.global_frame}: {exc}',
                                       throttle_duration_sec=2.0)
                return
            map_point = _transform_point(camera_point, transform)
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

    def _map_point_from_lidar(self, confirmed, camera_k, camera_frame, measurement_time):
        if self.last_scan is None:
            return None
        camera_bearing = pixel_bearing_rad(confirmed[0], camera_k[0], camera_k[2])
        if camera_bearing is None:
            return None
        ray_camera = (
            math.sin(camera_bearing),
            0.0,
            math.cos(camera_bearing),
        )
        scan = self.last_scan
        scan_frame = scan.header.frame_id
        try:
            camera_to_scan = self.tf_buffer.lookup_transform(
                scan_frame,
                camera_frame,
                measurement_time,
            )
            scan_to_map = self.tf_buffer.lookup_transform(
                self.global_frame,
                scan_frame,
                rclpy.time.Time.from_msg(scan.header.stamp),
            )
        except Exception as exc:  # noqa: BLE001
            if not self.allow_latest_tf_fallback:
                self.get_logger().warn(
                    f'Sin TF cámara/LIDAR→{self.global_frame}: {exc}',
                    throttle_duration_sec=2.0)
                return None
            camera_to_scan = self.tf_buffer.lookup_transform(
                scan_frame,
                camera_frame,
                rclpy.time.Time(),
            )
            scan_to_map = self.tf_buffer.lookup_transform(
                self.global_frame,
                scan_frame,
                rclpy.time.Time(),
            )
        origin_scan = _transform_point((0.0, 0.0, 0.0), camera_to_scan)
        ray_scan = _transform_point(ray_camera, camera_to_scan)
        dx = ray_scan[0] - origin_scan[0]
        dy = ray_scan[1] - origin_scan[1]
        target_bearing = math.atan2(dy, dx)
        distance = select_lidar_range(
            scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            target_bearing=target_bearing,
            window_rad=self.lidar_bearing_window,
            range_min=max(scan.range_min, self.min_depth),
            range_max=min(scan.range_max, self.max_depth),
            min_points=self.min_lidar_points,
        )
        if distance is None:
            return None
        scan_point = (
            distance * math.cos(target_bearing),
            distance * math.sin(target_bearing),
            0.0,
        )
        return _transform_point(scan_point, scan_to_map)

    def _lookup_transform(self, source_frame, measurement_time):
        try:
            return self.tf_buffer.lookup_transform(
                self.global_frame,
                source_frame,
                measurement_time,
            )
        except Exception:
            if not self.allow_latest_tf_fallback:
                raise
            return self.tf_buffer.lookup_transform(
                self.global_frame,
                source_frame,
                rclpy.time.Time(),
            )


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
