import math
from dataclasses import dataclass


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class MotionFilterConfig:
    enable_motion_filter: bool = True
    max_angular_speed_rad_s: float = 0.10
    max_path_curvature_rad_m: float = 0.35
    max_lateral_pose_change_m: float = 0.03
    still_linear_speed_m_s: float = 0.03


@dataclass(frozen=True)
class MotionSample:
    linear_speed_m_s: float
    angular_speed_rad_s: float
    curvature_rad_m: float
    lateral_change_m: float


def estimate_motion_sample(previous_pose, current_pose, next_pose, dt):
    if dt <= 0.0:
        raise ValueError('dt must be positive')

    prev_x, prev_y, prev_theta = previous_pose
    curr_x, curr_y, curr_theta = current_pose
    next_x, next_y, next_theta = next_pose

    dx = next_x - prev_x
    dy = next_y - prev_y
    span = math.hypot(dx, dy)
    linear_speed = span / (2.0 * dt)
    angular_speed = _normalize_angle(next_theta - prev_theta) / (2.0 * dt)

    if span <= 1e-9:
        lateral_change = 0.0
    else:
        lateral_change = abs(
            dx * (curr_y - prev_y) - dy * (curr_x - prev_x)
        ) / span

    if linear_speed <= 1e-9:
        curvature = 0.0 if abs(angular_speed) <= 1e-9 else float('inf')
    else:
        curvature = abs(angular_speed) / linear_speed

    return MotionSample(
        linear_speed_m_s=linear_speed,
        angular_speed_rad_s=angular_speed,
        curvature_rad_m=curvature,
        lateral_change_m=lateral_change,
    )


def should_integrate_scan(sample, config):
    if not config.enable_motion_filter:
        return True, 'disabled'

    if sample.linear_speed_m_s <= config.still_linear_speed_m_s:
        return True, 'still'

    if abs(sample.angular_speed_rad_s) > config.max_angular_speed_rad_s:
        return False, 'turn_rate'

    if sample.lateral_change_m > config.max_lateral_pose_change_m:
        return False, 'lateral'

    if sample.curvature_rad_m > config.max_path_curvature_rad_m:
        return False, 'curvature'

    return True, 'straight'
