from tp_a_slam_aruco.aruco_runtime import (
    calibration_is_ready,
    calibration_mismatch_summary,
)


def test_calibration_requires_camera_info_when_yaml_fallback_disabled():
    assert not calibration_is_ready(
        has_topic_calibration=False,
        has_yaml_calibration=True,
        allow_yaml_fallback=False,
    )
    assert calibration_is_ready(
        has_topic_calibration=True,
        has_yaml_calibration=False,
        allow_yaml_fallback=False,
    )


def test_calibration_mismatch_summary_flags_large_intrinsic_difference():
    summary = calibration_mismatch_summary(
        yaml_camera_matrix=[
            [203.14, 0.0, 122.57],
            [0.0, 361.13, 123.33],
            [0.0, 0.0, 1.0],
        ],
        yaml_dist_coeffs=[
            -0.9904393553733826,
            -47.16939926147461,
            -0.0007601691759191453,
            -0.00031758102704770863,
            306.0343933105469,
        ],
        topic_camera_matrix=[
            [203.137, 0.0, 122.573],
            [0.0, 203.137, 124.059],
            [0.0, 0.0, 1.0],
        ],
        topic_dist_coeffs=[
            -0.9904393553733826,
            -47.16939926147461,
            -0.0007601691759191453,
            -0.00031758102704770863,
            306.0343933105469,
            -1.1441469192504883,
            -45.59364700317383,
            299.4920654296875,
        ],
    )

    assert summary is not None
    assert 'fy' in summary
    assert 'distortion' in summary
