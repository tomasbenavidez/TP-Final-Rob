import pytest

from tp_a_slam_aruco.landmark_coherence import summarize_landmark_clearance


def test_landmark_coherence_flags_crossed_landmark():
    trajectory = [
        {'x': 0.0, 'y': 0.0},
        {'x': 1.0, 'y': 0.0},
        {'x': 2.0, 'y': 0.0},
    ]
    landmarks = {
        '10': {'x': 1.0, 'y': 0.02},
        '11': {'x': 1.0, 'y': 0.50},
    }

    summary = summarize_landmark_clearance(
        trajectory=trajectory,
        landmarks=landmarks,
        min_landmark_clearance_m=0.20,
    )

    assert summary['crossed_landmark_count'] == 1
    assert summary['per_landmark']['10']['trajectory_crossing_flag']
    assert not summary['per_landmark']['11']['trajectory_crossing_flag']
    assert summary['per_landmark']['10']['min_distance_to_trajectory'] == pytest.approx(0.02)

