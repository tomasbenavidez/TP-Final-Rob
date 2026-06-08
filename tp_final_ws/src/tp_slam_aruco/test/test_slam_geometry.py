from tp_slam_aruco.slam_geometry import (
    CameraExtrinsics,
    fallback_camera_to_base_xy,
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
