import pytest

from tp_a_slam_aruco.graph_slam_node import GraphSlamNode


def _node_for_temporal_tests():
    node = GraphSlamNode.__new__(GraphSlamNode)
    node.odom_samples = []
    node.pending_detections = []
    node.max_detection_wait_seconds = 1.0
    node.detection_keyframe_tolerance = 0.10
    node.max_pending_detections = 500
    node.pose_count = 1
    node.kf_stamps = [1.0]
    node.kf_odom_poses = [(0.0, 0.0, 0.0)]
    node.last_kf_pose = (0.0, 0.0, 0.0)
    node.added_observations = []
    node.created_keyframes = []
    node.dropped_observations = []
    node._add_observation = (
        lambda pose_index, pending:
        node.added_observations.append((pose_index, dict(pending)))
    )

    def _fake_add_keyframe(prev_odom_pose, curr_odom_pose, stamp, pose_source):
        pose_index = node.pose_count
        node.pose_count += 1
        node.kf_stamps.append(float(stamp))
        node.kf_odom_poses.append(tuple(curr_odom_pose))
        node.last_kf_pose = tuple(curr_odom_pose)
        node.created_keyframes.append(
            (pose_index, tuple(prev_odom_pose), tuple(curr_odom_pose), float(stamp), pose_source)
        )
        return pose_index

    node._add_keyframe = _fake_add_keyframe
    node._after_observations_added = lambda _ids, _pose_index: None
    node._record_dropped_observation = (
        lambda pending, reason, pose_index=None, pose_stamp=None:
        node.dropped_observations.append((dict(pending), reason, pose_index, pose_stamp))
    )
    return node


def _pending(marker_id=7, stamp=1.5, range_=0.5):
    return {
        'id': marker_id,
        'stamp': stamp,
        'frame_id': 'camera',
        'tf_source': 'tf',
        'tx': 0.0,
        'ty': 0.0,
        'tz': range_,
        'x_base': range_,
        'y_base': 0.0,
        'range': range_,
        'bearing': 0.0,
    }


def test_detection_uses_interpolated_odom_at_image_timestamp():
    node = _node_for_temporal_tests()
    node.odom_samples = [
        (1.0, (0.0, 0.0, 0.0)),
        (2.0, (2.0, 0.0, 0.0)),
    ]
    node.pending_detections = [_pending(stamp=1.5)]

    node._process_ready_detections(newest_odom_stamp=2.0)

    assert node.created_keyframes == [
        (1, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 1.5, 'created_detection_keyframe')
    ]
    assert node.added_observations[0][0] == 1
    assert node.added_observations[0][1]['pose_stamp'] == pytest.approx(1.5)
    assert node.added_observations[0][1]['detection_pose_dt'] == pytest.approx(0.0)
    assert node.added_observations[0][1]['odom_interpolation_gap_ms'] == pytest.approx(1000.0)


def test_detection_near_existing_keyframe_reuses_that_pose():
    node = _node_for_temporal_tests()
    node.odom_samples = [
        (1.0, (0.0, 0.0, 0.0)),
        (1.2, (0.2, 0.0, 0.0)),
    ]
    node.pending_detections = [_pending(stamp=1.05)]

    node._process_ready_detections(newest_odom_stamp=1.2)

    assert node.created_keyframes == []
    assert node.added_observations[0][0] == 0
    assert node.added_observations[0][1]['pose_source'] == 'existing_keyframe'
    assert node.added_observations[0][1]['detection_pose_dt'] == pytest.approx(-0.05)


def test_old_detection_without_near_keyframe_is_dropped_as_stale():
    node = _node_for_temporal_tests()
    node.kf_stamps = [1.0, 2.0]
    node.pose_count = 2
    node.odom_samples = [
        (1.0, (0.0, 0.0, 0.0)),
        (2.0, (1.0, 0.0, 0.0)),
    ]
    node.pending_detections = [_pending(stamp=1.5)]

    node._process_ready_detections(newest_odom_stamp=2.0)

    assert node.added_observations == []
    assert node.created_keyframes == []
    assert node.dropped_observations[0][1] == 'dropped_stale'


def test_same_pose_and_marker_keeps_nearest_detection_only():
    node = _node_for_temporal_tests()
    node.odom_samples = [
        (1.0, (0.0, 0.0, 0.0)),
        (1.2, (0.2, 0.0, 0.0)),
    ]
    node.pending_detections = [
        _pending(marker_id=9, stamp=1.04, range_=1.0),
        _pending(marker_id=9, stamp=1.06, range_=0.4),
    ]

    node._process_ready_detections(newest_odom_stamp=1.2)

    assert len(node.added_observations) == 1
    assert node.added_observations[0][1]['range'] == pytest.approx(0.4)
