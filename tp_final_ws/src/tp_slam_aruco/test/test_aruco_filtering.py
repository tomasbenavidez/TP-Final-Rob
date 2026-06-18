import math

from tp_slam_aruco.aruco_filtering import (
    detection_rejection_reason,
    marker_area_px,
    parse_allowed_marker_ids,
)


def test_marker_area_px_uses_corner_polygon_area():
    corners = [[10, 20], [30, 20], [30, 40], [10, 40]]

    assert marker_area_px(corners) == 400.0


def test_parse_allowed_marker_ids_accepts_csv_and_empty_values():
    assert parse_allowed_marker_ids('') == set()
    assert parse_allowed_marker_ids(' 4, 7,11 ') == {4, 7, 11}


def test_detection_rejection_reason_accepts_good_detection():
    detection = {
        'id': 7,
        'tvec': [0.1, 0.0, 1.2],
        'area_px': 1200.0,
        'reprojection_error_px': 1.5,
    }

    assert detection_rejection_reason(
        detection,
        min_area_px=250.0,
        min_depth=0.15,
        max_depth=3.0,
        max_reprojection_error_px=4.0,
        allowed_marker_ids={7, 9},
    ) is None


def test_detection_rejection_reason_rejects_suspicious_detections():
    base = {
        'id': 7,
        'tvec': [0.1, 0.0, 1.2],
        'area_px': 1200.0,
        'reprojection_error_px': 1.5,
    }

    small = {**base, 'area_px': 100.0}
    far = {**base, 'tvec': [0.1, 0.0, 3.5]}
    noisy = {**base, 'reprojection_error_px': 8.0}
    wrong_id = {**base, 'id': 12}
    missing_pose = {**base, 'tvec': None}

    assert detection_rejection_reason(small, min_area_px=250.0) == 'area_too_small'
    assert detection_rejection_reason(far, max_depth=3.0) == 'depth_too_far'
    assert detection_rejection_reason(
        noisy, max_reprojection_error_px=4.0
    ) == 'reprojection_error_too_high'
    assert detection_rejection_reason(
        wrong_id, allowed_marker_ids={7}
    ) == 'id_not_allowed'
    assert detection_rejection_reason(
        missing_pose, max_depth=math.inf
    ) == 'missing_pose'
