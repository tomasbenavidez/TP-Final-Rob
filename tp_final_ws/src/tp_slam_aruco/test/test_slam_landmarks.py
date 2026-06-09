from tp_slam_aruco.slam_landmarks import LandmarkCandidate, LandmarkCandidateManager


def test_candidate_landmark_is_not_confirmed_on_single_weak_observation():
    manager = LandmarkCandidateManager(
        min_candidate_observations=2,
        min_candidate_parallax_m=0.20,
        max_candidate_reprojection_error_px=3.0,
    )

    confirmed = manager.observe(
        landmark_id=7,
        pose_xy=(0.0, 0.0),
        projected_xy=(1.0, 0.0),
        reprojection_error_px=4.0,
    )

    assert confirmed is None
    assert manager.is_candidate(7)
    assert not manager.is_confirmed(7)


def test_candidate_landmark_is_confirmed_after_two_consistent_observations():
    manager = LandmarkCandidateManager(
        min_candidate_observations=2,
        min_candidate_parallax_m=0.20,
        max_candidate_reprojection_error_px=4.0,
    )

    assert manager.observe(
        landmark_id=7,
        pose_xy=(0.0, 0.0),
        projected_xy=(1.0, 0.0),
        reprojection_error_px=2.5,
    ) is None

    confirmed = manager.observe(
        landmark_id=7,
        pose_xy=(0.3, 0.0),
        projected_xy=(1.1, 0.05),
        reprojection_error_px=2.0,
    )

    assert confirmed is not None
    assert manager.is_confirmed(7)
    assert confirmed.landmark_id == 7
    assert confirmed.observation_count == 2


def test_candidate_landmark_can_be_confirmed_with_smaller_but_consistent_parallax():
    manager = LandmarkCandidateManager(
        min_candidate_observations=2,
        min_candidate_parallax_m=0.12,
        max_candidate_reprojection_error_px=4.0,
        min_candidate_pose_separation_m=0.08,
    )

    assert manager.observe(
        landmark_id=11,
        pose_xy=(0.0, 0.0),
        projected_xy=(1.0, 0.0),
        reprojection_error_px=2.8,
        observation_age_s=0.02,
        max_observation_age_s=0.20,
    ) is None

    confirmed = manager.observe(
        landmark_id=11,
        pose_xy=(0.12, 0.0),
        projected_xy=(1.05, 0.03),
        reprojection_error_px=3.2,
        observation_age_s=0.18,
        max_observation_age_s=0.20,
    )

    assert confirmed is not None
    assert manager.is_confirmed(11)


def test_candidate_manager_accepts_older_but_still_recent_observation_window():
    manager = LandmarkCandidateManager(
        min_candidate_observations=2,
        min_candidate_parallax_m=0.12,
        max_candidate_reprojection_error_px=4.0,
        min_candidate_pose_separation_m=0.08,
    )

    assert manager.observe(
        landmark_id=12,
        pose_xy=(0.0, 0.0),
        projected_xy=(1.0, 0.1),
        reprojection_error_px=3.0,
        observation_age_s=0.19,
        max_observation_age_s=0.20,
    ) is None

    confirmed = manager.observe(
        landmark_id=12,
        pose_xy=(0.13, 0.0),
        projected_xy=(1.02, 0.08),
        reprojection_error_px=3.1,
        observation_age_s=0.20,
        max_observation_age_s=0.20,
    )

    assert confirmed is not None


def test_candidate_manager_rejects_reprojection_outlier_before_confirmation():
    manager = LandmarkCandidateManager(
        min_candidate_observations=2,
        min_candidate_parallax_m=0.10,
        max_candidate_reprojection_error_px=3.0,
        candidate_reset_on_bad_observation=True,
    )

    manager.observe(
        landmark_id=4,
        pose_xy=(0.0, 0.0),
        projected_xy=(1.5, 0.0),
        reprojection_error_px=2.0,
    )
    confirmed = manager.observe(
        landmark_id=4,
        pose_xy=(0.4, 0.0),
        projected_xy=(1.6, 0.0),
        reprojection_error_px=5.0,
    )

    assert confirmed is None
    assert not manager.is_candidate(4)
    assert not manager.is_confirmed(4)
    stats = manager.stats()
    assert stats['discarded_bad_quality'] == 1


def test_candidate_manager_resets_after_max_buffer_without_confirmation():
    manager = LandmarkCandidateManager(
        min_candidate_observations=2,
        min_candidate_parallax_m=0.20,
        max_candidate_reprojection_error_px=3.0,
        max_candidate_buffer_observations=3,
        candidate_reset_on_bad_observation=True,
        min_candidate_pose_separation_m=0.15,
    )

    manager.observe(
        landmark_id=9,
        pose_xy=(0.0, 0.0),
        projected_xy=(1.0, 0.0),
        reprojection_error_px=2.0,
        observation_age_s=0.01,
        max_observation_age_s=0.10,
    )
    manager.observe(
        landmark_id=9,
        pose_xy=(0.01, 0.0),
        projected_xy=(1.0, 0.0),
        reprojection_error_px=2.0,
        observation_age_s=0.01,
        max_observation_age_s=0.10,
    )
    manager.observe(
        landmark_id=9,
        pose_xy=(0.02, 0.0),
        projected_xy=(1.0, 0.0),
        reprojection_error_px=2.0,
        observation_age_s=0.01,
        max_observation_age_s=0.10,
    )

    assert not manager.is_candidate(9)
    stats = manager.stats()
    assert stats['discarded_buffer_exhausted'] == 1


def test_candidate_manager_discards_old_observation_for_new_landmark():
    manager = LandmarkCandidateManager(
        min_candidate_observations=2,
        min_candidate_parallax_m=0.20,
        max_candidate_reprojection_error_px=3.0,
        max_candidate_buffer_observations=5,
        candidate_reset_on_bad_observation=True,
        min_candidate_pose_separation_m=0.15,
    )

    confirmed = manager.observe(
        landmark_id=15,
        pose_xy=(0.0, 0.0),
        projected_xy=(1.0, 0.0),
        reprojection_error_px=2.0,
        observation_age_s=0.25,
        max_observation_age_s=0.10,
    )

    assert confirmed is None
    assert not manager.is_candidate(15)
    stats = manager.stats()
    assert stats['discarded_age_dominated'] == 1
