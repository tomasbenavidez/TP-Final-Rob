import math
from dataclasses import dataclass

import numpy as np

from tp_slam_aruco.motion_model import normalize_angle


@dataclass(frozen=True)
class InnovationDiagnostics:
    pred_range: float
    pred_bearing: float
    range_residual: float
    bearing_residual: float
    maha_sq: float


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
    return innovation_diagnostics(
        pose=pose,
        landmark=landmark,
        bearing=bearing,
        range_=range_,
    ).maha_sq


def innovation_diagnostics(pose, landmark, bearing, range_):
    dx = landmark[0] - pose.x()
    dy = landmark[1] - pose.y()
    pred_range = math.hypot(dx, dy)
    pred_bearing = normalize_angle(math.atan2(dy, dx) - pose.theta())

    bearing_residual = normalize_angle(bearing - pred_bearing)
    range_residual = range_ - pred_range
    innov = np.array([bearing_residual, range_residual])
    sigma_bearing, sigma_range = observation_sigmas(range_)
    s_inv = np.diag([
        1.0 / sigma_bearing ** 2,
        1.0 / sigma_range ** 2,
    ])
    maha_sq = float(innov @ s_inv @ innov)
    return InnovationDiagnostics(
        pred_range=pred_range,
        pred_bearing=pred_bearing,
        range_residual=range_residual,
        bearing_residual=bearing_residual,
        maha_sq=maha_sq,
    )


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


def classify_innovation_diagnostics(diag, angular_scale=0.30, range_scale=0.20):
    bearing_score = abs(diag.bearing_residual) / angular_scale
    range_score = abs(diag.range_residual) / range_scale
    if bearing_score >= range_score * 1.5:
        return 'bearing_dominant'
    if range_score >= bearing_score * 1.5:
        return 'range_dominant'
    return 'mixed'
