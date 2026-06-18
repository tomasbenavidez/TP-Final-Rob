"""Pure helpers for ArUco geometry diagnostics."""

from __future__ import annotations

import math
from dataclasses import dataclass

from tp_slam_aruco.slam_geometry import (
    predict_landmark_from_observation,
    spatial_landmark_jump,
)


@dataclass(frozen=True)
class ArucoGeometryObservation:
    stamp: float
    marker_id: int
    frame_id: str
    tf_source: str
    tx: float
    ty: float
    tz: float
    x_base: float
    y_base: float
    range_: float
    bearing: float
    pose_index: int
    pose_x: float
    pose_y: float
    pose_theta: float
    predicted_landmark_x: float | None = None
    predicted_landmark_y: float | None = None
    spatial_jump: float | None = None
    reject_reason: str = ''


def range_residual(pose_x, pose_y, landmark_x, landmark_y, measured_range):
    expected = math.hypot(float(landmark_x) - float(pose_x), float(landmark_y) - float(pose_y))
    return expected - float(measured_range)


def build_geometry_debug_row(observation, landmark_x=None, landmark_y=None):
    residual = ''
    if landmark_x is not None and landmark_y is not None:
        residual = f'{range_residual(observation.pose_x, observation.pose_y, landmark_x, landmark_y, observation.range_):.6f}'

    predicted_landmark_x = observation.predicted_landmark_x
    predicted_landmark_y = observation.predicted_landmark_y
    if predicted_landmark_x is None or predicted_landmark_y is None:
        predicted_landmark_x, predicted_landmark_y = predict_landmark_from_observation(
            observation.pose_x,
            observation.pose_y,
            observation.pose_theta,
            observation.x_base,
            observation.y_base,
        )

    jump = observation.spatial_jump
    if jump is None and landmark_x is not None and landmark_y is not None:
        jump = spatial_landmark_jump(
            predicted_landmark_x,
            predicted_landmark_y,
            landmark_x,
            landmark_y,
        )

    return {
        'stamp': f'{observation.stamp:.9f}',
        'id': int(observation.marker_id),
        'frame_id': observation.frame_id,
        'tf_source': observation.tf_source,
        'tx': f'{observation.tx:.6f}',
        'ty': f'{observation.ty:.6f}',
        'tz': f'{observation.tz:.6f}',
        'x_base': f'{observation.x_base:.6f}',
        'y_base': f'{observation.y_base:.6f}',
        'range': f'{observation.range_:.6f}',
        'bearing': f'{observation.bearing:.6f}',
        'pose_index': int(observation.pose_index),
        'pose_x': f'{observation.pose_x:.6f}',
        'pose_y': f'{observation.pose_y:.6f}',
        'pose_theta': f'{observation.pose_theta:.6f}',
        'landmark_x': f'{float(landmark_x):.6f}' if landmark_x is not None else '',
        'landmark_y': f'{float(landmark_y):.6f}' if landmark_y is not None else '',
        'residual_range': residual,
        'predicted_landmark_x': f'{float(predicted_landmark_x):.6f}',
        'predicted_landmark_y': f'{float(predicted_landmark_y):.6f}',
        'spatial_jump': f'{float(jump):.6f}' if jump is not None else '',
        'reject_reason': observation.reject_reason,
    }
