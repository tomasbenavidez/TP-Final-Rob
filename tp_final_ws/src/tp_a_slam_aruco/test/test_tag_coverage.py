from tp_a_slam_aruco.tag_coverage import TagCoverageTracker


def test_tag_coverage_tracks_raw_valid_seeded_and_rejected_counts():
    tracker = TagCoverageTracker(expected_tag_count=50)

    tracker.record_raw(7)
    tracker.record_raw(7)
    tracker.record_valid(7)
    tracker.record_detector_rejection(7, 'area')
    tracker.record_seeded(7)
    tracker.record_confirmed(7)
    tracker.record_gating_rejection(7)

    summary = tracker.summary()

    assert summary['expected_tag_count'] == 50
    assert summary['unique_raw_ids_seen'] == 1
    assert summary['unique_valid_ids_seen'] == 1
    assert summary['unique_ids_seeded_in_graph'] == 1
    assert summary['unique_ids_confirmed'] == 1
    assert summary['missing_ids_estimate'] == 49
    assert summary['per_id']['7']['raw'] == 2
    assert summary['per_id']['7']['valid'] == 1
    assert summary['per_id']['7']['seeded'] == 1
    assert summary['per_id']['7']['confirmed'] == 1
    assert summary['per_id']['7']['detector_rejections']['area'] == 1
    assert summary['per_id']['7']['gating_rejections'] == 1

