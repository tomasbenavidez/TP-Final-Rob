"""Pure helpers for rejecting suspicious ArUco detections."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def marker_area_px(corners) -> float:
    """Return the polygon area of a detected marker in image pixels."""
    pts = np.asarray(corners, dtype=float).reshape(4, 2)
    x = pts[:, 0]
    y = pts[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) * 0.5)


def parse_allowed_marker_ids(value) -> set[int]:
    """Parse comma-separated or iterable marker IDs into a set."""
    if value is None or value == '':
        return set()
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(',')]
        return {int(part) for part in parts if part}
    if isinstance(value, Iterable):
        return {int(part) for part in value}
    return {int(value)}


def detection_rejection_reason(
    detection: dict,
    *,
    min_area_px: float = 0.0,
    min_depth: float = 0.0,
    max_depth: float = math.inf,
    max_reprojection_error_px: float = math.inf,
    allowed_marker_ids: set[int] | None = None,
) -> str | None:
    """Return a short rejection reason, or None when the detection is usable."""
    marker_id = int(detection['id'])
    if allowed_marker_ids and marker_id not in allowed_marker_ids:
        return 'id_not_allowed'

    area_px = float(detection.get('area_px', math.inf))
    if area_px < min_area_px:
        return 'area_too_small'

    tvec = detection.get('tvec')
    if tvec is None:
        return 'missing_pose'

    depth = float(tvec[2])
    if depth <= min_depth:
        return 'depth_too_near'
    if depth > max_depth:
        return 'depth_too_far'

    reproj = detection.get('reprojection_error_px')
    if reproj is not None and float(reproj) > max_reprojection_error_px:
        return 'reprojection_error_too_high'

    return None
