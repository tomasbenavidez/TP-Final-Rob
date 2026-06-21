from tp_slam_aruco.slam_landmarks import LandmarkObservationGate


def test_landmark_observation_gate_promotes_after_minimum_count():
    gate = LandmarkObservationGate(min_observations=3)

    assert gate.register(45) == (False, 1)
    assert gate.register(45) == (False, 2)
    assert gate.register(45) == (True, 3)
    assert gate.register(45) == (True, 4)


def test_landmark_observation_gate_tracks_ids_independently_and_can_clear():
    gate = LandmarkObservationGate(min_observations=2)

    assert gate.register(1) == (False, 1)
    assert gate.register(2) == (False, 1)
    assert gate.register(1) == (True, 2)

    gate.clear(1)

    assert gate.register(1) == (False, 1)
