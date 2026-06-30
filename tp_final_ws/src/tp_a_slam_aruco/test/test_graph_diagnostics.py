import math

import pytest
from builtin_interfaces.msg import Time

from tp_a_slam_aruco.slam_graph_diagnostics import GraphRunDiagnostics
from tp_a_slam_aruco.slam_timing import TimedPose2, should_create_visual_keyframe
from tp_a_slam_aruco.visual_observability import FrameObservability


def test_graph_run_diagnostics_tracks_recurrent_gating_rejections():
    diagnostics = GraphRunDiagnostics()

    diagnostics.record_gating_rejection(landmark_id=4, maha_sq=19.19)
    diagnostics.record_gating_rejection(landmark_id=4, maha_sq=11.2)
    diagnostics.record_gating_acceptance()

    summary = diagnostics.summary()

    assert summary['gating_rejections'] == 2
    assert summary['gating_acceptances'] == 1
    assert summary['rejected_landmarks']['4'] == 2
    assert summary['max_maha_sq'] == pytest.approx(19.19)


def test_visual_keyframe_policy_requires_enough_landmarks_and_translation():
    last_keyframe = TimedPose2(stamp=10.0, x=0.0, y=0.0, theta=0.0)
    observation_pose = TimedPose2(stamp=10.5, x=0.12, y=0.0, theta=math.radians(5.0))

    assert not should_create_visual_keyframe(
        last_keyframe=last_keyframe,
        observation_pose=observation_pose,
        kf_dist=0.15,
        kf_angle_max=0.60,
        valid_landmark_count=1,
        min_visual_landmarks=2,
    )

    assert should_create_visual_keyframe(
        last_keyframe=last_keyframe,
        observation_pose=TimedPose2(stamp=10.7, x=0.25, y=0.0, theta=0.0),
        kf_dist=0.15,
        kf_angle_max=0.60,
        valid_landmark_count=3,
        min_visual_landmarks=2,
    )


def test_graph_diagnostics_accept_full_visual_observability_frame():
    diagnostics = GraphRunDiagnostics()

    diagnostics.record_visual_observability(
        FrameObservability(
            stamp=1.5,
            raw_count=2,
            valid_count=1,
            valid_unique_count=1,
            rejected_area=1,
            rejected_depth=0,
            rejected_reprojection=0,
            rejected_tf=0,
            rejected_no_calibration=0,
            raw_ids=(4, 9),
            valid_ids=(4,),
            rejected_area_ids=(9,),
            rejected_depth_ids=(),
            rejected_reprojection_ids=(),
            rejected_tf_ids=(),
        )
    )

    summary = diagnostics.summary()

    assert summary['visual_observability']['summary']['frame_count'] == 1
    assert summary['visual_observability']['frames'][0]['raw_ids'] == (4, 9)
