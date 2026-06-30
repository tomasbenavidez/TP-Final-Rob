import math

from tp_a_slam_aruco.slam_timing import (
    TimedPose2,
    interpolate_timed_pose,
    observation_requires_new_keyframe,
)


def test_interpolate_timed_pose_uses_observation_timestamp():
    pose = interpolate_timed_pose(
        TimedPose2(stamp=10.0, x=0.0, y=0.0, theta=0.0),
        TimedPose2(stamp=12.0, x=2.0, y=0.0, theta=0.0),
        stamp=11.0,
    )

    assert pose.stamp == 11.0
    assert pose.x == 1.0
    assert pose.y == 0.0
    assert pose.theta == 0.0


def test_observation_requires_new_keyframe_only_when_threshold_is_crossed():
    last_keyframe = TimedPose2(stamp=10.0, x=0.0, y=0.0, theta=0.0)

    assert not observation_requires_new_keyframe(
        last_keyframe=last_keyframe,
        observation_pose=TimedPose2(stamp=10.1, x=0.05, y=0.0, theta=0.1),
        kf_dist=0.15,
        kf_angle_max=0.60,
    )
    assert observation_requires_new_keyframe(
        last_keyframe=last_keyframe,
        observation_pose=TimedPose2(stamp=10.2, x=0.20, y=0.0, theta=0.0),
        kf_dist=0.15,
        kf_angle_max=0.60,
    )
    assert observation_requires_new_keyframe(
        last_keyframe=last_keyframe,
        observation_pose=TimedPose2(
            stamp=10.3, x=0.0, y=0.0, theta=math.radians(40.0)),
        kf_dist=0.15,
        kf_angle_max=0.60,
    )
