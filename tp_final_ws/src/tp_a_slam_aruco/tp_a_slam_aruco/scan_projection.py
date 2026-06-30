import math


_ANGLE_EPS = 1e-9


def iter_valid_scan_points(scan):
    """Yield valid LaserScan beams as (x, y, angle, range) in the scan frame."""
    angle = float(scan.angle_min)
    angle_increment = float(scan.angle_increment)
    angle_max = float(scan.angle_max)
    if angle_increment <= 0.0:
        return
    for r in scan.ranges:
        if angle > angle_max + _ANGLE_EPS:
            break
        value = float(r)
        if (math.isfinite(value) and
                float(scan.range_min) <= value <= float(scan.range_max)):
            yield (
                value * math.cos(angle),
                value * math.sin(angle),
                angle,
                value,
            )
        angle += angle_increment


def sensor_pose_in_map(base_pose, base_from_sensor):
    """Compose map<-base pose with base<-sensor planar extrinsics."""
    bx, by, bth = base_pose
    sx, sy, sth = base_from_sensor
    cb = math.cos(bth)
    sb = math.sin(bth)
    return (
        bx + sx * cb - sy * sb,
        by + sx * sb + sy * cb,
        bth + sth,
    )


def fallback_sensor_pose_in_map(base_pose, lidar_tx, lidar_ty, lidar_yaw):
    return sensor_pose_in_map(
        base_pose,
        (float(lidar_tx), float(lidar_ty), float(lidar_yaw)),
    )
