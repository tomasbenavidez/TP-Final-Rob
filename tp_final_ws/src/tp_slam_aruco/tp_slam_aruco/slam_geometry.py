import math
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraExtrinsics:
    tx: float
    ty: float
    yaw: float


@dataclass(frozen=True)
class PlanarTransform2D:
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


def quaternion_to_yaw(rotation):
    qx, qy, qz, qw = rotation
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def planar_transform_from_xyz_quat(translation, rotation):
    return PlanarTransform2D(
        tx=float(translation[0]),
        ty=float(translation[1]),
        yaw=float(quaternion_to_yaw(rotation)),
    )


def invert_planar_transform(transform):
    cy = math.cos(transform.yaw)
    sy = math.sin(transform.yaw)
    tx = -(cy * transform.tx + sy * transform.ty)
    ty = -(-sy * transform.tx + cy * transform.ty)
    return PlanarTransform2D(tx=tx, ty=ty, yaw=-transform.yaw)


def compose_planar_transforms(first, second):
    cy = math.cos(first.yaw)
    sy = math.sin(first.yaw)
    tx = first.tx + cy * second.tx - sy * second.ty
    ty = first.ty + sy * second.tx + cy * second.ty
    return PlanarTransform2D(
        tx=tx,
        ty=ty,
        yaw=math.atan2(
            math.sin(first.yaw + second.yaw),
            math.cos(first.yaw + second.yaw),
        ),
    )


def lookup_planar_transform(graph, source_frame, target_frame):
    if source_frame == target_frame:
        return PlanarTransform2D(tx=0.0, ty=0.0, yaw=0.0)

    neighbors = {}
    for (parent, child), transform in graph.items():
        neighbors.setdefault(parent, []).append((child, transform))
        neighbors.setdefault(child, []).append((parent, invert_planar_transform(transform)))

    queue = deque([(source_frame, PlanarTransform2D(tx=0.0, ty=0.0, yaw=0.0))])
    visited = {source_frame}

    while queue:
        frame, accumulated = queue.popleft()
        for next_frame, edge in neighbors.get(frame, []):
            if next_frame in visited:
                continue
            composed = compose_planar_transforms(accumulated, edge)
            if next_frame == target_frame:
                return composed
            visited.add(next_frame)
            queue.append((next_frame, composed))

    raise KeyError(f'No planar transform chain from {source_frame} to {target_frame}')


def planar_graph_from_transforms(transforms):
    graph = {}
    for transform in transforms:
        graph[(transform.header.frame_id, transform.child_frame_id)] = (
            planar_transform_from_xyz_quat(
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
            )
        )
    return graph
