import numpy as np
import cv2


def calibration_is_ready(has_topic_calibration, has_yaml_calibration, allow_yaml_fallback):
    return bool(has_topic_calibration or (allow_yaml_fallback and has_yaml_calibration))


def calibration_mismatch_summary(
    yaml_camera_matrix,
    yaml_dist_coeffs,
    topic_camera_matrix,
    topic_dist_coeffs,
    focal_tolerance_px=5.0,
    principal_point_tolerance_px=5.0,
):
    yaml_k = np.asarray(yaml_camera_matrix, dtype=np.float64).reshape(3, 3)
    topic_k = np.asarray(topic_camera_matrix, dtype=np.float64).reshape(3, 3)
    yaml_d = np.asarray(yaml_dist_coeffs, dtype=np.float64).reshape(-1)
    topic_d = np.asarray(topic_dist_coeffs, dtype=np.float64).reshape(-1)

    diffs = []
    if abs(float(yaml_k[0, 0] - topic_k[0, 0])) > focal_tolerance_px:
        diffs.append(f'fx mismatch ({yaml_k[0, 0]:.2f} vs {topic_k[0, 0]:.2f})')
    if abs(float(yaml_k[1, 1] - topic_k[1, 1])) > focal_tolerance_px:
        diffs.append(f'fy mismatch ({yaml_k[1, 1]:.2f} vs {topic_k[1, 1]:.2f})')
    if abs(float(yaml_k[0, 2] - topic_k[0, 2])) > principal_point_tolerance_px:
        diffs.append(f'cx mismatch ({yaml_k[0, 2]:.2f} vs {topic_k[0, 2]:.2f})')
    if abs(float(yaml_k[1, 2] - topic_k[1, 2])) > principal_point_tolerance_px:
        diffs.append(f'cy mismatch ({yaml_k[1, 2]:.2f} vs {topic_k[1, 2]:.2f})')
    if yaml_d.size != topic_d.size:
        diffs.append(
            f'distortion length mismatch ({yaml_d.size} vs {topic_d.size})'
        )
    return '; '.join(diffs) if diffs else None


def quadrilateral_area(corners):
    points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    x = points[:, 0]
    y = points[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def reprojection_error_px(
    object_points,
    image_points,
    rvec,
    tvec,
    camera_matrix,
    dist_coeffs,
):
    reprojected, _ = cv2.projectPoints(
        np.asarray(object_points, dtype=np.float64),
        np.asarray(rvec, dtype=np.float64),
        np.asarray(tvec, dtype=np.float64),
        np.asarray(camera_matrix, dtype=np.float64),
        np.asarray(dist_coeffs, dtype=np.float64),
    )
    reproj = reprojected.reshape(-1, 2)
    img = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    return float(np.linalg.norm(reproj - img, axis=1).mean())
