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
    mcl._correct_measurements = lambda measurements: 0

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
    mcl._correct_measurements = lambda measurements: calls.append(measurements) or 1
    observation = SimpleNamespace(landmark_id=7, range_m=1.0, bearing_rad=0.0)
    message = SimpleNamespace(observations=[observation])

    mcl.identified_observation_cb(message)
    mcl.identified_observation_cb(message)

    assert len(calls) == 1
    assert not mcl.allow_stationary_identified_correction


def test_motion_and_reference_odom_accept_best_effort_robot_qos():
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1] / 'tp_b_navigation'
        / 'mcl_localization.py').read_text()

    assert 'qos_profile_sensor_data' in source
    assert 'self.motion_odom_cb, qos_profile_sensor_data' in source
    assert 'self.reference_odom_cb, qos_profile_sensor_data' in source
