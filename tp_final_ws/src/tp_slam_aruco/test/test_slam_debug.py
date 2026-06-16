import math

from tp_slam_aruco.slam_debug import (
    ArucoGeometryObservation,
    build_geometry_debug_row,
    range_residual,
)


def test_range_residual_compares_observing_pose_with_landmark():
    residual = range_residual(
        pose_x=1.0,
        pose_y=2.0,
        landmark_x=1.6,
        landmark_y=2.8,
        measured_range=1.0,
    )

    assert math.isclose(residual, 0.0, abs_tol=1e-9)


def test_build_geometry_debug_row_keeps_tf_source_and_residual():
    observation = ArucoGeometryObservation(
        stamp=123.5,
        marker_id=10,
        frame_id='oakd_rgb_camera_optical_frame',
        tf_source='tf',
        tx=0.1,
        ty=-0.2,
        tz=0.7,
        x_base=0.65,
        y_base=-0.1,
        range_=0.66,
        bearing=-0.15,
        pose_index=42,
        pose_x=1.0,
        pose_y=2.0,
        pose_theta=0.3,
        predicted_landmark_x=1.5,
        predicted_landmark_y=2.7,
        spatial_jump=0.15,
        reject_reason='spatial_jump',
    )

    row = build_geometry_debug_row(
        observation,
        landmark_x=1.6,
        landmark_y=2.8,
    )

    assert row['stamp'] == '123.500000000'
    assert row['id'] == 10
    assert row['frame_id'] == 'oakd_rgb_camera_optical_frame'
    assert row['tf_source'] == 'tf'
    assert row['pose_index'] == 42
    assert row['landmark_x'] == '1.600000'
    assert row['landmark_y'] == '2.800000'
    assert row['residual_range'] == '0.340000'
    assert row['predicted_landmark_x'] == '1.500000'
    assert row['predicted_landmark_y'] == '2.700000'
    assert row['spatial_jump'] == '0.150000'
    assert row['reject_reason'] == 'spatial_jump'
