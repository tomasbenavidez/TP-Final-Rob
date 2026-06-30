import math

import pytest

from tp_a_slam_aruco.slam_motion_filter import (
    MotionFilterConfig,
    MotionSample,
    estimate_motion_sample,
    should_integrate_scan,
)


def test_estimate_motion_sample_is_straight_when_heading_is_constant():
    sample = estimate_motion_sample(
        previous_pose=(0.0, 0.0, 0.0),
        current_pose=(0.2, 0.0, 0.0),
        next_pose=(0.4, 0.0, 0.0),
        dt=1.0,
    )

    assert sample.linear_speed_m_s == pytest.approx(0.2)
    assert sample.angular_speed_rad_s == pytest.approx(0.0)
    assert sample.curvature_rad_m == pytest.approx(0.0)
    assert sample.lateral_change_m == pytest.approx(0.0)


def test_estimate_motion_sample_detects_pure_turn():
    sample = estimate_motion_sample(
        previous_pose=(0.0, 0.0, 0.0),
        current_pose=(0.0, 0.0, math.pi / 4.0),
        next_pose=(0.0, 0.0, math.pi / 2.0),
        dt=1.0,
    )

    assert sample.linear_speed_m_s == pytest.approx(0.0)
    assert sample.angular_speed_rad_s == pytest.approx(math.pi / 4.0)


def test_should_integrate_scan_when_robot_is_still():
    config = MotionFilterConfig()

    allowed, reason = should_integrate_scan(
        MotionSample(
            linear_speed_m_s=0.01,
            angular_speed_rad_s=0.25,
            curvature_rad_m=float('inf'),
            lateral_change_m=0.02,
        ),
        config,
    )

    assert allowed
    assert reason == 'still'


def test_should_reject_scan_when_turning_too_fast():
    config = MotionFilterConfig(max_angular_speed_rad_s=0.10)

    allowed, reason = should_integrate_scan(
        MotionSample(
            linear_speed_m_s=0.15,
            angular_speed_rad_s=0.20,
            curvature_rad_m=0.20,
            lateral_change_m=0.01,
        ),
        config,
    )

    assert not allowed
    assert reason == 'turn_rate'


def test_should_reject_scan_when_path_curvature_is_high():
    config = MotionFilterConfig(max_path_curvature_rad_m=0.35)

    allowed, reason = should_integrate_scan(
        MotionSample(
            linear_speed_m_s=0.20,
            angular_speed_rad_s=0.04,
            curvature_rad_m=0.60,
            lateral_change_m=0.01,
        ),
        config,
    )

    assert not allowed
    assert reason == 'curvature'

