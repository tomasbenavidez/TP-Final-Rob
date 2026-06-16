import math

from tp_slam_aruco.slam_geometry import (
    CameraExtrinsics,
    fallback_camera_to_base_xy,
    predict_landmark_from_observation,
    spatial_landmark_jump,
    transform_point_to_base_xy,
)


def test_tb4_camera_fallback_matches_static_transform_equivalence():
    optical_to_base_translation = (-0.0596, 0.0, 0.2436)
    optical_to_base_rotation = (0.5, -0.5, 0.5, -0.5)
    point_in_camera_frame = (0.12, 0.05, 1.40)

    from_tf = transform_point_to_base_xy(
        translation=optical_to_base_translation,
        rotation=optical_to_base_rotation,
        point=point_in_camera_frame,
    )
    from_fallback = fallback_camera_to_base_xy(
        tx=point_in_camera_frame[0],
        tz=point_in_camera_frame[2],
        extrinsics=CameraExtrinsics(tx=-0.0596, ty=0.0, yaw=0.0),
    )

    assert from_tf == from_fallback


def test_predict_landmark_from_observation_translates_robot_local_measurement():
    pred_x, pred_y = predict_landmark_from_observation(
        pose_x=1.0,
        pose_y=2.0,
        pose_theta=0.0,
        x_base=3.0,
        y_base=4.0,
    )

    assert pred_x == 4.0
    assert pred_y == 6.0


def test_predict_landmark_from_observation_rotates_robot_local_measurement():
    pred_x, pred_y = predict_landmark_from_observation(
        pose_x=1.0,
        pose_y=2.0,
        pose_theta=math.pi / 2.0,
        x_base=3.0,
        y_base=0.0,
    )

    assert math.isclose(pred_x, 1.0, abs_tol=1e-9)
    assert math.isclose(pred_y, 5.0, abs_tol=1e-9)


def test_spatial_landmark_jump_is_euclidean_distance():
    assert spatial_landmark_jump(
        pred_x=1.0,
        pred_y=2.0,
        landmark_x=4.0,
        landmark_y=6.0,
    ) == 5.0
