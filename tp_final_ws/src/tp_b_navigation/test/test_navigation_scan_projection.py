import math

import pytest

from tp_b_navigation.scan_projection import transform_scan_points


def test_transform_scan_points_uses_scan_frame_before_base_filtering():
    points = transform_scan_points(
        ranges=[1.0],
        angle_min=0.0,
        angle_max=0.0,
        angle_increment=1.0,
        range_min=0.2,
        range_max=4.0,
        base_from_sensor=(0.0, 0.0, math.pi / 2.0),
        map_from_sensor=(2.0, 3.0, math.pi / 2.0),
    )

    assert len(points) == 1
    point = points[0]
    assert point.base_x == pytest.approx(0.0)
    assert point.base_y == pytest.approx(1.0)
    assert point.map_x == pytest.approx(2.0)
    assert point.map_y == pytest.approx(4.0)


def test_transform_scan_points_drops_invalid_ranges_and_extra_angles():
    points = transform_scan_points(
        ranges=[1.0, 2.0, float('nan'), 0.1, 5.0],
        angle_min=0.0,
        angle_max=math.pi / 2.0,
        angle_increment=math.pi / 2.0,
        range_min=0.2,
        range_max=4.0,
        base_from_sensor=(0.0, 0.0, 0.0),
        map_from_sensor=(0.0, 0.0, 0.0),
    )

    expected = [
        (1.0, 0.0),
        (0.0, 2.0),
    ]
    assert len(points) == len(expected)
    for point, expected_point in zip(points, expected):
        assert (point.base_x, point.base_y) == pytest.approx(expected_point)
