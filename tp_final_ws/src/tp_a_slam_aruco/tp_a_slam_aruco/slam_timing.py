import bisect
import math
from dataclasses import dataclass

from tp_a_slam_aruco.motion_model import normalize_angle


@dataclass(frozen=True)
class TimedPose2:
    stamp: float
    x: float
    y: float
    theta: float


def interpolate_timed_pose(start, end, stamp):
    if end.stamp <= start.stamp:
        return TimedPose2(stamp=stamp, x=start.x, y=start.y, theta=start.theta)

    alpha = (stamp - start.stamp) / (end.stamp - start.stamp)
    alpha = min(max(alpha, 0.0), 1.0)
    dtheta = math.atan2(
        math.sin(end.theta - start.theta),
        math.cos(end.theta - start.theta),
    )
    return TimedPose2(
        stamp=stamp,
        x=start.x + alpha * (end.x - start.x),
        y=start.y + alpha * (end.y - start.y),
        theta=normalize_angle(start.theta + alpha * dtheta),
    )


def observation_requires_new_keyframe(
    last_keyframe,
    observation_pose,
    kf_dist,
    kf_angle_max,
):
    moved = math.hypot(
        observation_pose.x - last_keyframe.x,
        observation_pose.y - last_keyframe.y,
    )
    turned = abs(normalize_angle(observation_pose.theta - last_keyframe.theta))
    return moved >= kf_dist or turned >= kf_angle_max


def should_create_visual_keyframe(
    last_keyframe,
    observation_pose,
    kf_dist,
    kf_angle_max,
    valid_landmark_count,
    min_visual_landmarks,
):
    if valid_landmark_count < min_visual_landmarks:
        return False
    return observation_requires_new_keyframe(
        last_keyframe=last_keyframe,
        observation_pose=observation_pose,
        kf_dist=kf_dist,
        kf_angle_max=kf_angle_max,
    )


class OdomPoseBuffer:
    def __init__(self, max_size=4096):
        self._poses = []
        self._stamps = []
        self._max_size = max_size

    def append(self, pose):
        if self._poses and pose.stamp < self._poses[-1].stamp:
            idx = bisect.bisect_left(self._stamps, pose.stamp)
            self._stamps.insert(idx, pose.stamp)
            self._poses.insert(idx, pose)
        else:
            self._stamps.append(pose.stamp)
            self._poses.append(pose)

        if len(self._poses) > self._max_size:
            extra = len(self._poses) - self._max_size
            del self._poses[:extra]
            del self._stamps[:extra]

    def latest(self):
        return self._poses[-1] if self._poses else None

    def pose_at(self, stamp):
        if not self._poses:
            return None
        if stamp <= self._poses[0].stamp:
            return self._poses[0]
        if stamp >= self._poses[-1].stamp:
            return self._poses[-1]

        hi = bisect.bisect_right(self._stamps, stamp)
        lo = hi - 1
        return interpolate_timed_pose(self._poses[lo], self._poses[hi], stamp)
