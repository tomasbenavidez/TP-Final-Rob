"""Geometría pura para el sensor virtual de landmarks."""

import math

import numpy as np


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def camera_point_from_base(base_point, camera_tx, camera_ty, camera_yaw):
    """Expresa un punto XY de ``base`` en el frame planar de la cámara."""
    dx = float(base_point[0]) - float(camera_tx)
    dy = float(base_point[1]) - float(camera_ty)
    cy = math.cos(float(camera_yaw))
    sy = math.sin(float(camera_yaw))
    return cy * dx + sy * dy, -sy * dx + cy * dy


def _scan_index(bearing, angle_min, angle_increment, count):
    if count <= 0 or angle_increment <= 0.0:
        return None
    span = angle_increment * count
    angle = float(bearing)
    while angle < angle_min:
        angle += 2.0 * math.pi
    while angle >= angle_min + span:
        angle -= 2.0 * math.pi
    index = int(round((angle - angle_min) / angle_increment))
    if index < 0 or index >= count:
        return None
    return index


def visibility_reason(
        base_point, camera_point, ranges, angle_min, angle_increment,
        range_min, range_max, camera_fov, camera_max_range, occlusion_tol):
    """Devuelve ``visible`` o el motivo por el que se rechaza el landmark."""
    camera_range = math.hypot(*camera_point)
    camera_bearing = wrap_angle(math.atan2(camera_point[1], camera_point[0]))
    if abs(camera_bearing) > float(camera_fov) / 2.0:
        return 'outside_fov'
    if camera_range < float(range_min) or camera_range > min(
            float(range_max), float(camera_max_range)):
        return 'outside_range'

    base_range = math.hypot(*base_point)
    base_bearing = math.atan2(base_point[1], base_point[0])
    index = _scan_index(base_bearing, float(angle_min), float(angle_increment),
                        len(ranges))
    if index is None:
        return 'invalid_scan'
    measured = float(ranges[index])
    if not np.isfinite(measured) or measured < float(range_min):
        return 'invalid_scan'
    if measured + float(occlusion_tol) < base_range:
        return 'occluded'
    return 'visible'
