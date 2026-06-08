import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraExtrinsics:
    tx: float
    ty: float
    yaw: float


TB4_CAMERA_EXTRINSICS = CameraExtrinsics(tx=-0.0596, ty=0.0, yaw=0.0)


def fallback_camera_to_base_xy(tx, tz, extrinsics=TB4_CAMERA_EXTRINSICS):
    p_cam_2d = (tz, -tx)
    cy = math.cos(extrinsics.yaw)
    sy = math.sin(extrinsics.yaw)
    x_base = cy * p_cam_2d[0] - sy * p_cam_2d[1] + extrinsics.tx
    y_base = sy * p_cam_2d[0] + cy * p_cam_2d[1] + extrinsics.ty
    return x_base, y_base


def transform_point_to_base_xy(translation, rotation, point):
    tx, ty, tz = translation
    qx, qy, qz, qw = rotation
    x, y, z = point

    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)

    x_base = r00 * x + r01 * y + r02 * z + tx
    y_base = r10 * x + r11 * y + r12 * z + ty
    return x_base, y_base


def transform_stamped_to_base_xy(transform, point):
    return transform_point_to_base_xy(
        translation=(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ),
        rotation=(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ),
        point=point,
    )
