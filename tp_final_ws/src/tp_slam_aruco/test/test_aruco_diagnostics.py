from tp_slam_aruco.aruco_diagnostics import DetectionStatsWindow


def test_detection_stats_window_counts_multiple_valid_landmarks():
    window = DetectionStatsWindow(window_size=3)

    window.observe(raw_count=4, valid_ids=[1, 7, 7, 9])
    summary = window.summary()

    assert summary.frames == 1
    assert summary.avg_raw == 4.0
    assert summary.avg_valid == 3.0
    assert summary.max_valid == 3


def test_detection_stats_window_ignores_rejected_detections_in_valid_count():
    window = DetectionStatsWindow(window_size=4)

    window.observe(raw_count=5, valid_ids=[])
    window.observe(raw_count=3, valid_ids=[2])
    summary = window.summary()

    assert summary.frames == 2
    assert summary.avg_raw == 4.0
    assert summary.avg_valid == 0.5
    assert summary.max_valid == 1


def test_detection_stats_window_flags_poor_multilandmark_visibility():
    window = DetectionStatsWindow(window_size=5)

    for _ in range(5):
        window.observe(raw_count=2, valid_ids=[3])

    assert window.should_warn_low_multilandmark()
