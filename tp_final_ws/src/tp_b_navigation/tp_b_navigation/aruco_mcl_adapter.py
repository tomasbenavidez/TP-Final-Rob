#!/usr/bin/env python3
"""Convierte detecciones ArUco de Parte A al contrato identificado de MCL."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from tp_interfaces.msg import LandmarkObservation, LandmarkObservationArray


def _transform_point(point, transform):
    x, y, z = point
    q = transform.transform.rotation
    translation = transform.transform.translation
    tx, ty, tz = translation.x, translation.y, translation.z
    ix = q.w * x + q.y * z - q.z * y
    iy = q.w * y + q.z * x - q.x * z
    iz = q.w * z + q.x * y - q.y * x
    iw = -q.x * x - q.y * y - q.z * z
    return (
        ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y + tx,
        iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z + ty,
        iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x + tz,
    )


class ArucoMclAdapter(Node):
    def __init__(self):
        super().__init__('aruco_mcl_adapter')
        self.declare_parameter('detections_topic', '/aruco_detections')
        self.declare_parameter('output_topic', '/observed_landmark_ids')
        self.declare_parameter('base_frame', 'base_link')
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            LandmarkObservationArray, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            MarkerArray, self.get_parameter('detections_topic').value,
            self.detections_cb, 10)

    def detections_cb(self, msg):
        output = LandmarkObservationArray()
        if not msg.markers:
            output.header.frame_id = self.base_frame
            output.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(output)
            return
        output.header.stamp = msg.markers[0].header.stamp
        output.header.frame_id = self.base_frame
        for marker in msg.markers:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, marker.header.frame_id, rclpy.time.Time())
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f'Sin TF {marker.header.frame_id}->{self.base_frame}: {exc}',
                    throttle_duration_sec=2.0)
                continue
            point = _transform_point(
                (marker.pose.position.x, marker.pose.position.y,
                 marker.pose.position.z),
                transform)
            observation = LandmarkObservation()
            observation.header = marker.header
            observation.landmark_id = int(marker.id)
            observation.x_base = float(point[0])
            observation.y_base = float(point[1])
            observation.range_m = float(math.hypot(point[0], point[1]))
            observation.bearing_rad = float(math.atan2(point[1], point[0]))
            observation.depth_m = float(marker.pose.position.z)
            observation.reprojection_error_px = 0.0
            observation.used_fallback_tf = False
            observation.source_frame = marker.header.frame_id
            output.observations.append(observation)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMclAdapter()
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
