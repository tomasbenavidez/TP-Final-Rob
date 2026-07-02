from types import SimpleNamespace

import math
import pytest
import rclpy
from builtin_interfaces.msg import Time
from std_msgs.msg import Header


def _identity_transform():
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _planar_transform(x, y, yaw):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=0.0),
            rotation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        ),
    )


def _marker(stamp, marker_id=7):
    header = Header()
    header.stamp = stamp
    header.frame_id = 'camera'
    return SimpleNamespace(
        header=header,
        id=marker_id,
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=0.0, z=2.0)),
    )


def test_compensate_point_for_forward_motion():
    from tp_b_navigation.aruco_mcl_adapter import compensate_point_base_obs_to_base_now

    point = compensate_point_base_obs_to_base_now(
        (2.0, 0.0, 0.0),
        odom_base_obs=(0.0, 0.0, 0.0),
        odom_base_now=(1.0, 0.0, 0.0),
    )

    assert point == pytest.approx((1.0, 0.0, 0.0))


def test_compensate_point_for_robot_rotation():
    from tp_b_navigation.aruco_mcl_adapter import compensate_point_base_obs_to_base_now

    point = compensate_point_base_obs_to_base_now(
        (1.0, 0.0, 0.0),
        odom_base_obs=(0.0, 0.0, 0.0),
        odom_base_now=(0.0, 0.0, math.pi / 2.0),
    )

    assert point == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)


def test_compensate_point_without_relative_motion_is_identity():
    from tp_b_navigation.aruco_mcl_adapter import compensate_point_base_obs_to_base_now

    point = compensate_point_base_obs_to_base_now(
        (0.4, -0.2, 1.3),
        odom_base_obs=(2.0, 3.0, -0.4),
        odom_base_now=(2.0, 3.0, -0.4),
    )

    assert point == pytest.approx((0.4, -0.2, 1.3))


def test_transform_marker_uses_measurement_timestamp():
    from tp_b_navigation.aruco_mcl_adapter import ArucoMclAdapter

    requested = []
    adapter = ArucoMclAdapter.__new__(ArucoMclAdapter)
    adapter.base_frame = 'base_link'
    adapter.allow_latest_tf_fallback = False
    adapter.tf_buffer = SimpleNamespace(
        lookup_transform=lambda target, source, stamp: (
            requested.append(stamp) or _identity_transform()
        ),
    )
    stamp = Time(sec=12, nanosec=300_000_000)

    point, used_fallback = adapter.transform_marker(_marker(stamp))

    assert requested == [rclpy.time.Time.from_msg(stamp)]
    assert point == (1.0, 0.0, 2.0)
    assert not used_fallback


def test_latest_tf_fallback_is_explicit_and_reported():
    from tp_b_navigation.aruco_mcl_adapter import ArucoMclAdapter

    requested = []

    def lookup(_target, _source, stamp):
        requested.append(stamp)
        if stamp.nanoseconds:
            raise RuntimeError('timestamp unavailable')
        return _identity_transform()

    adapter = ArucoMclAdapter.__new__(ArucoMclAdapter)
    adapter.base_frame = 'base_link'
    adapter.allow_latest_tf_fallback = True
    adapter.tf_buffer = SimpleNamespace(lookup_transform=lookup)

    point, used_fallback = adapter.transform_marker(
        _marker(Time(sec=12, nanosec=0)),
    )

    assert len(requested) == 2
    assert requested[0].nanoseconds == 12_000_000_000
    assert requested[1].nanoseconds == 0
    assert point == (1.0, 0.0, 2.0)
    assert used_fallback


def test_compensate_point_to_now_uses_odom_tf_at_observation_and_now():
    from tp_b_navigation.aruco_mcl_adapter import ArucoMclAdapter

    requested = []
    obs_stamp = Time(sec=10, nanosec=0)
    now_time = rclpy.time.Time(seconds=12.0)

    def lookup(target, source, stamp):
        requested.append((target, source, stamp.nanoseconds))
        if stamp.nanoseconds == 10_000_000_000:
            return _planar_transform(0.0, 0.0, 0.0)
        if stamp.nanoseconds == 12_000_000_000:
            return _planar_transform(1.0, 0.0, 0.0)
        raise AssertionError(stamp.nanoseconds)

    adapter = ArucoMclAdapter.__new__(ArucoMclAdapter)
    adapter.odom_frame = 'odom'
    adapter.base_frame = 'base_link'
    adapter.tf_buffer = SimpleNamespace(lookup_transform=lookup)

    result = adapter.compensate_point_to_now(
        (2.0, 0.0, 0.0),
        rclpy.time.Time.from_msg(obs_stamp),
        now_time,
    )

    assert result.point == pytest.approx((1.0, 0.0, 0.0))
    assert result.relative_dx == pytest.approx(1.0)
    assert result.relative_dy == pytest.approx(0.0)
    assert result.relative_dyaw == pytest.approx(0.0)
    assert requested == [
        ('odom', 'base_link', 10_000_000_000),
        ('odom', 'base_link', 12_000_000_000),
    ]


def test_compensate_point_to_now_falls_back_to_latest_for_current_tf_only():
    from tp_b_navigation.aruco_mcl_adapter import ArucoMclAdapter

    requested = []
    now_time = rclpy.time.Time(seconds=12.0)

    def lookup(target, source, stamp):
        requested.append((target, source, stamp.nanoseconds))
        if stamp.nanoseconds == 10_000_000_000:
            return _planar_transform(0.0, 0.0, 0.0)
        if stamp.nanoseconds == 12_000_000_000:
            raise RuntimeError('current tf is slightly in the future')
        if stamp.nanoseconds == 0:
            return _planar_transform(1.0, 0.0, 0.0)
        raise AssertionError(stamp.nanoseconds)

    adapter = ArucoMclAdapter.__new__(ArucoMclAdapter)
    adapter.odom_frame = 'odom'
    adapter.base_frame = 'base_link'
    adapter.tf_buffer = SimpleNamespace(lookup_transform=lookup)

    result = adapter.compensate_point_to_now(
        (2.0, 0.0, 0.0),
        rclpy.time.Time(seconds=10.0),
        now_time,
    )

    assert result.point == pytest.approx((1.0, 0.0, 0.0))
    assert requested == [
        ('odom', 'base_link', 10_000_000_000),
        ('odom', 'base_link', 12_000_000_000),
        ('odom', 'base_link', 0),
    ]


def test_callback_publishes_compensated_observation_with_current_array_stamp():
    from tp_b_navigation.aruco_mcl_adapter import ArucoMclAdapter, CompensationResult

    published = []
    adapter = ArucoMclAdapter.__new__(ArucoMclAdapter)
    adapter.base_frame = 'base_link'
    adapter.compensate_delayed_observations = True
    adapter.max_compensation_age = 4.0
    adapter.max_compensation_translation = 2.0
    adapter.max_compensation_rotation = 6.28
    adapter.transform_marker = lambda marker: ((2.0, 0.0, 0.0), False)
    adapter.compensate_point_to_now = lambda point, obs, now: CompensationResult(
        point=(1.0, 0.0, 0.0),
        relative_dx=1.0,
        relative_dy=0.0,
        relative_dyaw=0.0,
    )
    adapter.publisher = SimpleNamespace(publish=published.append)
    adapter._write_compensation_diagnostic = lambda **_kwargs: None
    adapter.get_clock = lambda: SimpleNamespace(
        now=lambda: rclpy.time.Time(seconds=12.0),
    )

    adapter.detections_cb(SimpleNamespace(markers=[
        _marker(Time(sec=10, nanosec=0), marker_id=27),
    ]))

    assert len(published) == 1
    msg = published[0]
    assert msg.header.frame_id == 'base_link'
    assert msg.header.stamp.sec == 12
    assert msg.observations[0].header.stamp.sec == 10
    assert msg.observations[0].landmark_id == 27
    assert msg.observations[0].x_base == pytest.approx(1.0)
    assert msg.observations[0].range_m == pytest.approx(1.0)


def test_callback_drops_observation_older_than_max_compensation_age():
    from tp_b_navigation.aruco_mcl_adapter import ArucoMclAdapter

    published = []
    diagnostics = []
    adapter = ArucoMclAdapter.__new__(ArucoMclAdapter)
    adapter.base_frame = 'base_link'
    adapter.compensate_delayed_observations = True
    adapter.max_compensation_age = 1.0
    adapter.transform_marker = lambda marker: ((2.0, 0.0, 0.0), False)
    adapter.publisher = SimpleNamespace(publish=published.append)
    adapter._write_compensation_diagnostic = lambda **kwargs: diagnostics.append(kwargs)
    adapter.get_clock = lambda: SimpleNamespace(
        now=lambda: rclpy.time.Time(seconds=12.0),
    )

    adapter.detections_cb(SimpleNamespace(markers=[
        _marker(Time(sec=10, nanosec=0), marker_id=27),
    ]))

    assert published == []
    assert diagnostics[0]['drop_reason'] == 'too_old'
