import math
from types import SimpleNamespace

import pytest

from tp_a_slam_aruco.scan_projection import (
    fallback_sensor_pose_in_map,
    iter_valid_scan_points,
    iter_mapping_scan_points,
    sensor_pose_in_map,
)


def make_scan(**overrides):
    values = {
        'angle_min': 0.0,
        'angle_max': math.pi / 2.0,
        'angle_increment': math.pi / 4.0,
        'range_min': 0.2,
        'range_max': 4.0,
        'ranges': [1.0, float('nan'), float('inf'), 0.1, 4.5, 2.0],
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_iter_valid_scan_points_honors_angles_and_range_limits():
    scan = make_scan()

    points = list(iter_valid_scan_points(scan))

    assert points == pytest.approx([
        (1.0, 0.0, 0.0, 1.0),
    ])


def test_iter_valid_scan_points_accepts_last_declared_angle():
    scan = make_scan(ranges=[1.0, 2.0, 3.0])

    points = list(iter_valid_scan_points(scan))

    expected = [
        (1.0, 0.0, 0.0, 1.0),
        (math.sqrt(2.0), math.sqrt(2.0), math.pi / 4.0, 2.0),
        (0.0, 3.0, math.pi / 2.0, 3.0),
    ]
    assert len(points) == len(expected)
    for point, expected_point in zip(points, expected):
        assert point == pytest.approx(expected_point)


def test_iter_mapping_scan_points_limits_obstacles_and_raytracing_separately():
    scan = make_scan(
        angle_max=math.pi / 4.0,
        ranges=[1.0, 2.3],
    )

    points = list(iter_mapping_scan_points(
        scan,
        max_obstacle_range=2.0,
        max_raytrace_range=2.5,
    ))

    assert len(points) == 2
    assert points[0] == pytest.approx((1.0, 0.0, 0.0, 1.0, True))
    assert points[1] == pytest.approx((
        2.3 / math.sqrt(2.0),
        2.3 / math.sqrt(2.0),
        math.pi / 4.0,
        2.3,
        False,
    ))


def test_sensor_pose_in_map_composes_base_pose_and_tf_extrinsics():
    pose = sensor_pose_in_map(
        base_pose=(1.0, 2.0, math.pi / 2.0),
        base_from_sensor=(0.25, -0.10, -math.pi / 2.0),
    )

    assert pose == pytest.approx((1.10, 2.25, 0.0))


def test_fallback_sensor_pose_in_map_uses_legacy_tb4_extrinsics():
    pose = fallback_sensor_pose_in_map(
        base_pose=(1.0, 2.0, math.pi / 2.0),
        lidar_tx=-0.04,
        lidar_ty=0.0,
        lidar_yaw=math.pi / 2.0,
    )

    assert pose == pytest.approx((1.0, 1.96, math.pi))
