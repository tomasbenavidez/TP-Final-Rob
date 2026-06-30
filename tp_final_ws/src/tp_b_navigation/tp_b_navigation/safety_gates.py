from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SafetyGateConfig:
    enabled: bool = True
    max_mcl_pose_age: float = 1.0
    max_scan_age: float = 1.0
    max_monitor_age: float = 1.0
    max_position_covariance: float = 0.25
    max_yaw_covariance: float = 0.5


def _finite_nonnegative(value):
    return value is not None and math.isfinite(value) and value >= 0.0


def _covariance_values(covariance):
    values = list(covariance or [])
    if len(values) >= 36:
        return values[0], values[7], values[35]
    if len(values) >= 3:
        return values[0], values[1], values[2]
    return math.inf, math.inf, math.inf


def localization_gate(now_s, last_pose_stamp_s, covariance, config):
    if not config.enabled:
        return None
    if last_pose_stamp_s is None:
        return 'mcl_pose_missing'
    age = now_s - last_pose_stamp_s
    if not _finite_nonnegative(age) or age > config.max_mcl_pose_age:
        return 'mcl_pose_stale'

    cov_x, cov_y, cov_yaw = _covariance_values(covariance)
    max_position_cov = max(cov_x, cov_y)
    if (
        not _finite_nonnegative(max_position_cov)
        or not _finite_nonnegative(cov_yaw)
        or max_position_cov > config.max_position_covariance
        or cov_yaw > config.max_yaw_covariance
    ):
        return 'mcl_pose_covariance_high'
    return None


def scan_gate(now_s, scan_stamp_s, config):
    if not config.enabled:
        return None
    if scan_stamp_s is None:
        return 'scan_stamp_missing'
    age = now_s - scan_stamp_s
    if not _finite_nonnegative(age) or age > config.max_scan_age:
        return 'scan_stale'
    return None


def monitor_gate(now_s, last_monitor_stamp_s, monitor_healthy, config):
    if not config.enabled:
        return None
    if last_monitor_stamp_s is None:
        return 'obstacle_monitor_missing'
    age = now_s - last_monitor_stamp_s
    if not _finite_nonnegative(age) or age > config.max_monitor_age:
        return 'obstacle_monitor_stale'
    if monitor_healthy is not True:
        return 'obstacle_monitor_unhealthy'
    return None
