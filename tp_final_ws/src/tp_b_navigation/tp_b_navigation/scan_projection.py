from dataclasses import dataclass
import math


_ANGLE_EPS = 1e-9


@dataclass(frozen=True)
class ProjectedScanPoint:
    base_x: float
    base_y: float
    map_x: float
    map_y: float
    range: float
    angle: float


def _transform_xy(x, y, pose):
    px, py, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return px + x * c - y * s, py + x * s + y * c


def transform_scan_points(
        ranges, angle_min, angle_max, angle_increment, range_min, range_max,
        base_from_sensor, map_from_sensor):
    """Project valid scan returns through sensor->base and sensor->map poses."""
    angle = float(angle_min)
    increment = float(angle_increment)
    if increment <= 0.0:
        return []
    points = []
    for value in ranges:
        if angle > float(angle_max) + _ANGLE_EPS:
            break
        r = float(value)
        if math.isfinite(r) and float(range_min) <= r <= float(range_max):
            sx = r * math.cos(angle)
            sy = r * math.sin(angle)
            bx, by = _transform_xy(sx, sy, base_from_sensor)
            mx, my = _transform_xy(sx, sy, map_from_sensor)
            points.append(ProjectedScanPoint(
                base_x=bx,
                base_y=by,
                map_x=mx,
                map_y=my,
                range=r,
                angle=angle,
            ))
        angle += increment
    return points
