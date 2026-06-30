import math

import numpy as np

from tp_a_slam_aruco.motion_model import normalize_angle
from tp_a_slam_aruco.slam_geometry import (
    predict_landmark_from_observation,
    spatial_landmark_jump,
)


def observation_sigmas(range_):
    return 0.10 + 0.05 * range_, 0.0239 + 0.0315 * range_ ** 2


def resolve_gate_state(result, initial, pose_key, landmark_key):
    pose = None
    landmark = None

    if initial is not None:
        try:
            pose = initial.atPose2(pose_key)
        except Exception:
            pass
        try:
            landmark = initial.atPoint2(landmark_key)
        except Exception:
            pass

    if result is not None:
        try:
            pose = result.atPose2(pose_key)
        except Exception:
            pass
        try:
            landmark = result.atPoint2(landmark_key)
        except Exception:
            pass

    if pose is None or landmark is None:
        return None
    return pose, landmark


def innovation_mahalanobis_sq(pose, landmark, bearing, range_):
    dx = landmark[0] - pose.x()
    dy = landmark[1] - pose.y()
    pred_range = math.hypot(dx, dy)
    pred_bearing = normalize_angle(math.atan2(dy, dx) - pose.theta())

    innov = np.array([
        normalize_angle(bearing - pred_bearing),
        range_ - pred_range,
    ])
    sigma_bearing, sigma_range = observation_sigmas(range_)
    s_inv = np.diag([
        1.0 / sigma_bearing ** 2,
        1.0 / sigma_range ** 2,
    ])
    return float(innov @ s_inv @ innov)


def innovation_gate(pose, landmark, bearing, range_, maha_threshold):
    return innovation_mahalanobis_sq(
        pose=pose,
        landmark=landmark,
        bearing=bearing,
        range_=range_,
    ) < maha_threshold


def innovation_gate_from_values(
    result,
    initial,
    pose_key,
    landmark_key,
    bearing,
    range_,
    maha_threshold,
):
    state = resolve_gate_state(
        result=result,
        initial=initial,
        pose_key=pose_key,
        landmark_key=landmark_key,
    )
    if state is None:
        return True

    pose, landmark = state
    return innovation_gate(
        pose=pose,
        landmark=landmark,
        bearing=bearing,
        range_=range_,
        maha_threshold=maha_threshold,
    )


def spatial_landmark_gate_from_values(
    result,
    initial,
    pose_key,
    landmark_key,
    x_base,
    y_base,
    max_jump,
):
    state = resolve_gate_state(
        result=result,
        initial=initial,
        pose_key=pose_key,
        landmark_key=landmark_key,
    )
    if state is None:
        return True, None, None, None, None, None

    pose, landmark = state
    pred_x, pred_y = predict_landmark_from_observation(
        pose_x=pose.x(),
        pose_y=pose.y(),
        pose_theta=pose.theta(),
        x_base=x_base,
        y_base=y_base,
    )
    landmark_x = float(landmark[0])
    landmark_y = float(landmark[1])
    jump = spatial_landmark_jump(pred_x, pred_y, landmark_x, landmark_y)
    return jump <= max_jump, pred_x, pred_y, jump, landmark_x, landmark_y
