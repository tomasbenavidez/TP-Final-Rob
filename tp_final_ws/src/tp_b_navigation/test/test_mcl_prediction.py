from types import SimpleNamespace

import numpy as np


def _odom(x, y=0.0, yaw=0.0):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=np.sin(yaw / 2.0),
                    w=np.cos(yaw / 2.0),
                ),
            ),
        ),
    )


def test_zero_noise_prediction_moves_particles_deterministically():
    from tp_b_navigation.mcl_localization import predict_particles

    particles = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])
    predicted = predict_particles(
        particles,
        previous_odom=(0.0, 0.0, 0.0),
        current_odom=(0.2, 0.0, 0.0),
        alphas=(0.0, 0.0, 0.0, 0.0),
        rng=np.random.default_rng(7),
    )

    assert np.allclose(predicted[:, 0], [0.2, 0.3])
    assert np.allclose(predicted[:, 1:], particles[:, 1:])
    assert np.allclose(particles[:, 0], [0.0, 0.1])


def test_map_to_odom_stays_stable_when_prediction_matches_odometry():
    from tp_b_navigation.mcl_localization import map_to_odom_pose

    before = map_to_odom_pose(
        estimate=(1.0, 2.0, 0.3),
        odom_pose=(0.0, 0.0, 0.0),
    )
    map_dx = 0.2 * np.cos(0.3)
    map_dy = 0.2 * np.sin(0.3)
    after = map_to_odom_pose(
        estimate=(1.0 + map_dx, 2.0 + map_dy, 0.3),
        odom_pose=(0.2, 0.0, 0.0),
    )

    assert np.allclose(before, after)


def test_odometry_prediction_refreshes_estimate_and_publications():
    from tp_b_navigation.mcl_localization import MCL

    mcl = MCL.__new__(MCL)
    mcl.initialized = True
    mcl.last_motion_odom = (0.0, 0.0, 0.0)
    mcl.particles = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])
    mcl.weights = np.array([0.5, 0.5])
    mcl.N = 2
    mcl.a1 = mcl.a2 = mcl.a3 = mcl.a4 = 0.0
    mcl.accum_d = 0.0
    mcl.accum_a = 0.0
    calls = []
    mcl._update_estimate = lambda: calls.append('estimate')
    mcl.publish_particles = lambda: calls.append('particles')
    mcl.publish_pose = lambda: calls.append('pose')

    mcl.motion_odom_cb(_odom(0.2))

    assert calls == ['estimate', 'particles', 'pose']
    assert np.allclose(mcl.particles[:, 0], [0.2, 0.3])
    assert mcl.last_motion_odom == (0.2, 0.0, 0.0)


def test_empty_and_unknown_measurements_do_not_consume_motion():
    from tp_b_navigation.mcl_localization import MCL

    mcl = MCL.__new__(MCL)
    mcl.N = 2
    mcl.particles = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])
    mcl.weights = np.array([0.5, 0.5])
    mcl.landmarks_by_id = {7: (1.0, 0.0)}
    mcl.sigma_r = 0.2
    mcl.sigma_b = 0.15
    mcl._update_estimate = lambda: None
    mcl.publish_particles = lambda: None
    mcl.publish_pose = lambda: None

    assert mcl._correct_measurements([]) == 0
    assert mcl._correct_measurements([(99, 1.0, 0.0)]) == 0
    assert mcl._correct_measurements([(7, 1.0, 0.0)]) == 1


def test_empty_identified_callback_preserves_accumulated_motion():
    from tp_b_navigation.mcl_localization import MCL

    mcl = MCL.__new__(MCL)
    mcl.initialized = True
    mcl.landmarks_by_id = {7: (1.0, 0.0)}
    mcl.accum_d = 0.10
    mcl.accum_a = 0.04
    mcl.update_min_d = 0.02
    mcl.update_min_a = 0.02
    mcl.allow_stationary_identified_correction = False
    mcl._correct_measurements = lambda measurements, *args, **kwargs: 0

    mcl.identified_observation_cb(SimpleNamespace(observations=[]))

    assert mcl.accum_d == 0.10
    assert mcl.accum_a == 0.04


def test_only_first_identified_correction_is_allowed_while_stationary():
    from tp_b_navigation.mcl_localization import MCL

    mcl = MCL.__new__(MCL)
    mcl.initialized = True
    mcl.landmarks_by_id = {7: (1.0, 0.0)}
    mcl.accum_d = 0.0
    mcl.accum_a = 0.0
    mcl.update_min_d = 0.02
    mcl.update_min_a = 0.02
    mcl.allow_stationary_identified_correction = True
    calls = []
    mcl._correct_measurements = (
        lambda measurements, *args, **kwargs: calls.append(measurements) or 1
    )
    observation = SimpleNamespace(landmark_id=7, range_m=1.0, bearing_rad=0.0)
    message = SimpleNamespace(observations=[observation])

    mcl.identified_observation_cb(message)
    mcl.identified_observation_cb(message)

    assert len(calls) == 1
    assert not mcl.allow_stationary_identified_correction


def test_stale_identified_observation_is_ignored_without_consuming_motion():
    from tp_b_navigation.mcl_localization import MCL

    mcl = MCL.__new__(MCL)
    mcl.initialized = True
    mcl.landmarks_by_id = {7: (1.0, 0.0)}
    mcl.accum_d = 0.10
    mcl.accum_a = 0.04
    mcl.update_min_d = 0.02
    mcl.update_min_a = 0.02
    mcl.max_landmark_measurement_age = 0.5
    mcl.allow_stationary_identified_correction = False
    calls = []
    mcl._correct_measurements = (
        lambda measurements, *args, **kwargs: calls.append(measurements) or 1
    )
    mcl._now_sec = lambda: 12.0
    observation = SimpleNamespace(landmark_id=7, range_m=1.0, bearing_rad=0.0)
    stamp = SimpleNamespace(sec=10, nanosec=0)
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        observations=[observation],
    )

    mcl.identified_observation_cb(message)

    assert calls == []
    assert mcl.accum_d == 0.10
    assert mcl.accum_a == 0.04


def test_legacy_landmarks_are_ignored_after_identified_measurements_arrive():
    from tp_b_navigation.mcl_localization import MCL

    mcl = MCL.__new__(MCL)
    mcl.received_identified_landmarks = True
    mcl.initialized = True
    mcl.landmarks = np.array([[1.0, 0.0]])
    mcl.accum_d = 1.0
    mcl.accum_a = 0.0
    mcl.update_min_d = 0.02
    mcl.update_min_a = 0.02
    calls = []
    mcl._correct = lambda msg: calls.append(msg) or 1
    message = SimpleNamespace(
        poses=[SimpleNamespace(position=SimpleNamespace(x=1.0, y=0.0, z=0.0))]
    )

    mcl.observation_cb(message)

    assert calls == []


def test_motion_and_reference_odom_accept_best_effort_robot_qos():
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1] / 'tp_b_navigation'
        / 'mcl_localization.py').read_text()

    assert 'qos_profile_sensor_data' in source
    assert 'self.motion_odom_cb, qos_profile_sensor_data' in source
    assert 'self.reference_odom_cb, qos_profile_sensor_data' in source


def test_landmark_pose_array_does_not_overwrite_loaded_id_map():
    from tp_b_navigation.mcl_localization import MCL

    mcl = MCL.__new__(MCL)
    mcl.landmarks_by_id = {7: (1.0, 0.0)}
    mcl.landmark_map_file = '/tmp/parte_a_trayectoria.json'
    message = SimpleNamespace(
        poses=[
            SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0)),
        ],
    )

    mcl.landmarks_cb(message)

    assert mcl.landmarks_by_id == {7: (1.0, 0.0)}


def test_mcl_subscribes_to_map_and_scan_for_laser_likelihood():
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1] / 'tp_b_navigation'
        / 'mcl_localization.py').read_text()

    assert 'OccupancyGrid' in source
    assert "LaserScan, '/scan'" in source
    assert "OccupancyGrid, '/map'" in source
    assert 'qos_profile_sensor_data' in source


def test_mcl_exposes_optional_csv_diagnostics():
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1] / 'tp_b_navigation'
        / 'mcl_localization.py').read_text()

    assert "declare_parameter('diagnostics_csv'" in source
    assert 'MclDiagnosticsCsv' in source
    assert '_record_diagnostic(' in source


def test_oos_landmark_update_corrects_old_particles_and_replays_to_now():
    from tp_b_navigation.mcl_localization import (
        MCL,
        OosMotionStep,
        OosSnapshot,
    )
    from tp_b_navigation.mcl_models import LandmarkMeasurement

    mcl = MCL.__new__(MCL)
    mcl.N = 2
    mcl.particles = np.array([[1.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    mcl.weights = np.array([0.5, 0.5])
    mcl.estimate = (2.0, 0.0, 0.0)
    mcl.landmarks_by_id = {7: (1.0, 0.0)}
    mcl.sigma_r = 0.05
    mcl.sigma_b = 0.05
    mcl.landmark_log_weight = 1.0
    mcl.rough_xy = 0.0
    mcl.rough_yaw = 0.0
    mcl.oos_max_snapshot_gap = 0.15
    mcl.oos_replay_deterministic = True
    mcl.diagnostics = None
    mcl.accum_d = 0.2
    mcl.accum_a = 0.0
    mcl.laser_accum_d = 0.2
    mcl.laser_accum_a = 0.0
    mcl.publish_particles = lambda: None
    mcl.publish_pose = lambda: None
    mcl._now_sec = lambda: 12.0
    mcl._oos_snapshots = [
        OosSnapshot(
            stamp=10.0,
            odom_pose=(0.0, 0.0, 0.0),
            particles=np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
            weights=np.array([0.5, 0.5]),
            estimate=(1.0, 0.0, 0.0),
        )
    ]
    mcl._oos_motion_steps = [
        OosMotionStep(
            start_stamp=10.0,
            end_stamp=11.0,
            previous_odom=(0.0, 0.0, 0.0),
            current_odom=(1.0, 0.0, 0.0),
        )
    ]

    used = mcl._correct_measurements_oos(
        [LandmarkMeasurement(7, 1.0, 0.0, 'identified')],
        source='identified',
        measurement_stamp=10.02,
    )

    assert used == 1
    assert mcl.estimate[0] < 1.25
    assert np.allclose(mcl.particles[:, 0], [1.0, 3.0])
    assert mcl.weights[0] > 0.99


def test_oos_landmark_update_rejects_large_snapshot_gap():
    from tp_b_navigation.mcl_localization import MCL, OosSnapshot
    from tp_b_navigation.mcl_models import LandmarkMeasurement

    mcl = MCL.__new__(MCL)
    mcl.N = 1
    mcl.particles = np.array([[4.0, 0.0, 0.0]])
    mcl.weights = np.array([1.0])
    mcl.estimate = (4.0, 0.0, 0.0)
    mcl.landmarks_by_id = {7: (1.0, 0.0)}
    mcl.sigma_r = 0.05
    mcl.sigma_b = 0.05
    mcl.landmark_log_weight = 1.0
    mcl.oos_max_snapshot_gap = 0.05
    mcl.diagnostics = None
    mcl._now_sec = lambda: 12.0
    mcl._oos_snapshots = [
        OosSnapshot(
            stamp=10.0,
            odom_pose=(0.0, 0.0, 0.0),
            particles=np.array([[0.0, 0.0, 0.0]]),
            weights=np.array([1.0]),
            estimate=(0.0, 0.0, 0.0),
        )
    ]
    mcl._oos_motion_steps = []

    used = mcl._correct_measurements_oos(
        [LandmarkMeasurement(7, 1.0, 0.0, 'identified')],
        source='identified',
        measurement_stamp=10.20,
    )

    assert used == 0
    assert np.allclose(mcl.particles, [[4.0, 0.0, 0.0]])


def test_oos_landmark_update_refreshes_future_history_snapshots():
    from tp_b_navigation.mcl_localization import (
        MCL,
        OosMotionStep,
        OosSnapshot,
    )
    from tp_b_navigation.mcl_models import LandmarkMeasurement

    mcl = MCL.__new__(MCL)
    mcl.N = 2
    mcl.particles = np.array([[1.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    mcl.weights = np.array([0.5, 0.5])
    mcl.estimate = (2.0, 0.0, 0.0)
    mcl.landmarks_by_id = {7: (1.0, 0.0)}
    mcl.sigma_r = 0.05
    mcl.sigma_b = 0.05
    mcl.landmark_log_weight = 1.0
    mcl.rough_xy = 0.0
    mcl.rough_yaw = 0.0
    mcl.oos_max_snapshot_gap = 0.15
    mcl.oos_history_duration = 6.0
    mcl.diagnostics = None
    mcl.accum_d = 0.2
    mcl.accum_a = 0.0
    mcl.laser_accum_d = 0.2
    mcl.laser_accum_a = 0.0
    mcl.publish_particles = lambda: None
    mcl.publish_pose = lambda: None
    mcl._now_sec = lambda: 12.0
    mcl.last_motion_stamp = 11.0
    mcl.last_motion_odom = (1.0, 0.0, 0.0)
    mcl._oos_snapshots = [
        OosSnapshot(
            stamp=10.0,
            odom_pose=(0.0, 0.0, 0.0),
            particles=np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
            weights=np.array([0.5, 0.5]),
            estimate=(1.0, 0.0, 0.0),
        ),
        OosSnapshot(
            stamp=11.0,
            odom_pose=(1.0, 0.0, 0.0),
            particles=np.array([[9.0, 0.0, 0.0], [9.5, 0.0, 0.0]]),
            weights=np.array([0.5, 0.5]),
            estimate=(9.25, 0.0, 0.0),
        ),
    ]
    mcl._oos_motion_steps = [
        OosMotionStep(
            start_stamp=10.0,
            end_stamp=11.0,
            previous_odom=(0.0, 0.0, 0.0),
            current_odom=(1.0, 0.0, 0.0),
        )
    ]

    used = mcl._correct_measurements_oos(
        [LandmarkMeasurement(7, 1.0, 0.0, 'identified')],
        source='identified',
        measurement_stamp=10.02,
    )

    assert used == 1
    latest_snapshot = list(mcl._oos_snapshots)[-1]
    assert latest_snapshot.stamp == 11.0
    assert np.allclose(latest_snapshot.particles[:, 0], mcl.particles[:, 0])
    assert latest_snapshot.weights[0] > 0.99
