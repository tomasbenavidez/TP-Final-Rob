#!/usr/bin/env python3
"""Convierte detecciones ArUco de Parte A al contrato identificado de MCL."""

import csv
import math
import os
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from tp_interfaces.msg import LandmarkObservation, LandmarkObservationArray


@dataclass(frozen=True)
class CompensationResult:
    point: tuple[float, float, float]
    relative_dx: float
    relative_dy: float
    relative_dyaw: float


def _yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _planar_pose_from_transform(transform):
    tr = transform.transform.translation
    rot = transform.transform.rotation
    return float(tr.x), float(tr.y), _yaw_from_quaternion(rot)


def _apply_planar_pose(pose, point):
    x, y, yaw = pose
    px, py, pz = point
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        x + c * px - s * py,
        y + s * px + c * py,
        pz,
    )


def _apply_inverse_planar_pose(pose, point):
    x, y, yaw = pose
    px, py, pz = point
    dx = px - x
    dy = py - y
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        pz,
    )


def _relative_planar_motion(odom_base_obs, odom_base_now):
    obs_x, obs_y, obs_yaw = odom_base_obs
    now_x, now_y, now_yaw = odom_base_now
    rel_x, rel_y, _ = _apply_inverse_planar_pose(
        odom_base_obs,
        (now_x, now_y, 0.0),
    )
    return rel_x, rel_y, _normalize_angle(now_yaw - obs_yaw)


def compensate_point_base_obs_to_base_now(
    point_base_obs,
    odom_base_obs,
    odom_base_now,
):
    """Return the same point expressed in base(now) instead of base(t_obs)."""
    point_odom = _apply_planar_pose(odom_base_obs, point_base_obs)
    return _apply_inverse_planar_pose(odom_base_now, point_odom)


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
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('allow_latest_tf_fallback', False)
        self.declare_parameter('compensate_delayed_observations', True)
        self.declare_parameter('max_compensation_age', 4.0)
        self.declare_parameter('max_compensation_translation', 2.0)
        self.declare_parameter('max_compensation_rotation', 6.28)
        self.declare_parameter(
            'compensation_diagnostics_csv',
            '/tmp/aruco_mcl_compensation.csv',
        )
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.allow_latest_tf_fallback = bool(
            self.get_parameter('allow_latest_tf_fallback').value)
        self.compensate_delayed_observations = bool(
            self.get_parameter('compensate_delayed_observations').value)
        self.max_compensation_age = float(
            self.get_parameter('max_compensation_age').value)
        self.max_compensation_translation = float(
            self.get_parameter('max_compensation_translation').value)
        self.max_compensation_rotation = float(
            self.get_parameter('max_compensation_rotation').value)
        self.compensation_diagnostics_csv = str(
            self.get_parameter('compensation_diagnostics_csv').value)
        self._comp_diag_handle = None
        self._comp_diag_writer = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            LandmarkObservationArray, self.get_parameter('output_topic').value, 1)
        self.create_subscription(
            MarkerArray, self.get_parameter('detections_topic').value,
            self.detections_cb, 1)

    def detections_cb(self, msg):
        output = LandmarkObservationArray()
        now_time = self.get_clock().now()
        now_msg = now_time.to_msg()
        if not msg.markers:
            output.header.frame_id = self.base_frame
            output.header.stamp = now_msg
            self.publisher.publish(output)
            return
        if getattr(self, 'compensate_delayed_observations', True):
            output.header.stamp = now_msg
        else:
            output.header.stamp = msg.markers[0].header.stamp
        output.header.frame_id = self.base_frame
        for marker in msg.markers:
            observation_time = rclpy.time.Time.from_msg(marker.header.stamp)
            age = self._age_seconds(observation_time, now_time)
            try:
                point, used_fallback = self.transform_marker(marker)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f'Sin TF {marker.header.frame_id}->{self.base_frame}: {exc}',
                    throttle_duration_sec=2.0)
                continue
            raw_point = point
            compensated = False
            relative_dx = 0.0
            relative_dy = 0.0
            relative_dyaw = 0.0
            if getattr(self, 'compensate_delayed_observations', True):
                if age > getattr(self, 'max_compensation_age', 4.0):
                    self._write_compensation_diagnostic(
                        original_stamp=self._time_to_sec(observation_time),
                        publish_stamp=self._time_to_sec(now_time),
                        age_sec=age,
                        landmark_id=int(marker.id),
                        compensated=False,
                        drop_reason='too_old',
                        raw_point=raw_point,
                        compensated_point=None,
                        relative_dx=0.0,
                        relative_dy=0.0,
                        relative_dyaw=0.0,
                    )
                    continue
                try:
                    compensation = self.compensate_point_to_now(
                        raw_point,
                        observation_time,
                        now_time,
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f'Sin TF {self.odom_frame}->{self.base_frame} para '
                        f'compensar ArUco {marker.id}: {exc}',
                        throttle_duration_sec=2.0)
                    self._write_compensation_diagnostic(
                        original_stamp=self._time_to_sec(observation_time),
                        publish_stamp=self._time_to_sec(now_time),
                        age_sec=age,
                        landmark_id=int(marker.id),
                        compensated=False,
                        drop_reason='missing_odom_tf',
                        raw_point=raw_point,
                        compensated_point=None,
                        relative_dx=0.0,
                        relative_dy=0.0,
                        relative_dyaw=0.0,
                    )
                    continue
                relative_dx = compensation.relative_dx
                relative_dy = compensation.relative_dy
                relative_dyaw = compensation.relative_dyaw
                relative_translation = math.hypot(relative_dx, relative_dy)
                if (
                    relative_translation >
                    getattr(self, 'max_compensation_translation', 2.0)
                ):
                    self._write_compensation_diagnostic(
                        original_stamp=self._time_to_sec(observation_time),
                        publish_stamp=self._time_to_sec(now_time),
                        age_sec=age,
                        landmark_id=int(marker.id),
                        compensated=False,
                        drop_reason='translation_too_large',
                        raw_point=raw_point,
                        compensated_point=compensation.point,
                        relative_dx=relative_dx,
                        relative_dy=relative_dy,
                        relative_dyaw=relative_dyaw,
                    )
                    continue
                if (
                    abs(relative_dyaw) >
                    getattr(self, 'max_compensation_rotation', 6.28)
                ):
                    self._write_compensation_diagnostic(
                        original_stamp=self._time_to_sec(observation_time),
                        publish_stamp=self._time_to_sec(now_time),
                        age_sec=age,
                        landmark_id=int(marker.id),
                        compensated=False,
                        drop_reason='rotation_too_large',
                        raw_point=raw_point,
                        compensated_point=compensation.point,
                        relative_dx=relative_dx,
                        relative_dy=relative_dy,
                        relative_dyaw=relative_dyaw,
                    )
                    continue
                point = compensation.point
                compensated = True
            observation = LandmarkObservation()
            observation.header = marker.header
            observation.landmark_id = int(marker.id)
            observation.x_base = float(point[0])
            observation.y_base = float(point[1])
            observation.range_m = float(math.hypot(point[0], point[1]))
            observation.bearing_rad = float(math.atan2(point[1], point[0]))
            observation.depth_m = float(marker.pose.position.z)
            observation.reprojection_error_px = 0.0
            observation.used_fallback_tf = used_fallback
            observation.source_frame = marker.header.frame_id
            output.observations.append(observation)
            if getattr(self, 'compensate_delayed_observations', True):
                self._write_compensation_diagnostic(
                    original_stamp=self._time_to_sec(observation_time),
                    publish_stamp=self._time_to_sec(now_time),
                    age_sec=age,
                    landmark_id=int(marker.id),
                    compensated=compensated,
                    drop_reason='',
                    raw_point=raw_point,
                    compensated_point=point,
                    relative_dx=relative_dx,
                    relative_dy=relative_dy,
                    relative_dyaw=relative_dyaw,
                )
        if output.observations:
            self.publisher.publish(output)

    def transform_marker(self, marker):
        measurement_time = rclpy.time.Time.from_msg(marker.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                marker.header.frame_id,
                measurement_time,
            )
            used_fallback = False
        except Exception:
            if not self.allow_latest_tf_fallback:
                raise
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                marker.header.frame_id,
                rclpy.time.Time(),
            )
            used_fallback = True
        point = _transform_point(
            (
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            ),
            transform,
        )
        return point, used_fallback

    def compensate_point_to_now(self, point_base_obs, observation_time, now_time):
        transform_obs = self.tf_buffer.lookup_transform(
            self.odom_frame,
            self.base_frame,
            observation_time,
        )
        try:
            transform_now = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                now_time,
            )
        except Exception:
            transform_now = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        odom_base_obs = _planar_pose_from_transform(transform_obs)
        odom_base_now = _planar_pose_from_transform(transform_now)
        relative_dx, relative_dy, relative_dyaw = _relative_planar_motion(
            odom_base_obs,
            odom_base_now,
        )
        return CompensationResult(
            point=compensate_point_base_obs_to_base_now(
                point_base_obs,
                odom_base_obs,
                odom_base_now,
            ),
            relative_dx=relative_dx,
            relative_dy=relative_dy,
            relative_dyaw=relative_dyaw,
        )

    @staticmethod
    def _time_to_sec(time):
        return time.nanoseconds * 1e-9

    @staticmethod
    def _age_seconds(observation_time, now_time):
        return (now_time.nanoseconds - observation_time.nanoseconds) * 1e-9

    def _ensure_compensation_diagnostics_writer(self):
        path = getattr(self, 'compensation_diagnostics_csv', '')
        if not path:
            return None
        if getattr(self, '_comp_diag_writer', None) is None:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            self._comp_diag_handle = open(path, 'w', newline='')
            self._comp_diag_writer = csv.DictWriter(
                self._comp_diag_handle,
                fieldnames=[
                    'original_stamp',
                    'publish_stamp',
                    'age_sec',
                    'id',
                    'compensated',
                    'drop_reason',
                    'x_base_raw',
                    'y_base_raw',
                    'x_base_compensated',
                    'y_base_compensated',
                    'relative_dx',
                    'relative_dy',
                    'relative_dyaw',
                ],
            )
            self._comp_diag_writer.writeheader()
            self._comp_diag_handle.flush()
            self.get_logger().info(
                f'Diagnostico compensacion ArUco escribiendo en {path}')
        return self._comp_diag_writer

    def _write_compensation_diagnostic(
        self,
        *,
        original_stamp,
        publish_stamp,
        age_sec,
        landmark_id,
        compensated,
        drop_reason,
        raw_point,
        compensated_point,
        relative_dx,
        relative_dy,
        relative_dyaw,
    ):
        writer = self._ensure_compensation_diagnostics_writer()
        if writer is None:
            return
        comp_x = '' if compensated_point is None else f'{compensated_point[0]:.9g}'
        comp_y = '' if compensated_point is None else f'{compensated_point[1]:.9g}'
        writer.writerow({
            'original_stamp': f'{original_stamp:.9f}',
            'publish_stamp': f'{publish_stamp:.9f}',
            'age_sec': f'{age_sec:.9g}',
            'id': int(landmark_id),
            'compensated': int(bool(compensated)),
            'drop_reason': str(drop_reason),
            'x_base_raw': f'{raw_point[0]:.9g}',
            'y_base_raw': f'{raw_point[1]:.9g}',
            'x_base_compensated': comp_x,
            'y_base_compensated': comp_y,
            'relative_dx': f'{relative_dx:.9g}',
            'relative_dy': f'{relative_dy:.9g}',
            'relative_dyaw': f'{relative_dyaw:.9g}',
        })
        self._comp_diag_handle.flush()

    def close_diagnostics(self):
        if getattr(self, '_comp_diag_handle', None) is not None:
            self._comp_diag_handle.close()
            self._comp_diag_handle = None
            self._comp_diag_writer = None

    def destroy_node(self):
        self.close_diagnostics()
        return super().destroy_node()


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
