import gtsam
from gtsam.symbol_shorthand import L, X

from tp_a_slam_aruco.slam_gating import (
    innovation_gate_from_values,
    spatial_landmark_gate_from_values,
)


def test_innovation_gate_uses_initial_pose_when_result_is_missing_new_pose():
    initial = gtsam.Values()
    initial.insert(X(1), gtsam.Pose2(0.0, 0.0, 0.0))
    initial.insert(L(7), gtsam.Point2(2.0, 0.0))

    result = gtsam.Values()
    result.insert(L(7), gtsam.Point2(2.0, 0.0))

    accepted = innovation_gate_from_values(
        result=result,
        initial=initial,
        pose_key=X(1),
        landmark_key=L(7),
        bearing=0.0,
        range_=2.0,
        maha_threshold=5.99,
    )

    assert accepted


def test_innovation_gate_rejects_outlier_even_for_new_pose_not_in_result():
    initial = gtsam.Values()
    initial.insert(X(3), gtsam.Pose2(0.0, 0.0, 0.0))
    initial.insert(L(19), gtsam.Point2(2.0, 0.0))

    result = gtsam.Values()
    result.insert(L(19), gtsam.Point2(2.0, 0.0))

    accepted = innovation_gate_from_values(
        result=result,
        initial=initial,
        pose_key=X(3),
        landmark_key=L(19),
        bearing=1.2,
        range_=4.0,
        maha_threshold=5.99,
    )

    assert not accepted


def test_spatial_landmark_gate_accepts_small_position_jump():
    initial = gtsam.Values()
    initial.insert(X(2), gtsam.Pose2(1.0, 2.0, 0.0))
    initial.insert(L(17), gtsam.Point2(4.2, 6.1))

    accepted, pred_x, pred_y, jump, landmark_x, landmark_y = spatial_landmark_gate_from_values(
        result=None,
        initial=initial,
        pose_key=X(2),
        landmark_key=L(17),
        x_base=3.0,
        y_base=4.0,
        max_jump=0.75,
    )

    assert accepted
    assert pred_x == 4.0
    assert pred_y == 6.0
    assert jump < 0.75
    assert landmark_x == 4.2
    assert landmark_y == 6.1


def test_spatial_landmark_gate_rejects_large_position_jump():
    initial = gtsam.Values()
    initial.insert(X(2), gtsam.Pose2(1.0, 2.0, 0.0))
    initial.insert(L(17), gtsam.Point2(8.0, 10.0))

    accepted, pred_x, pred_y, jump, landmark_x, landmark_y = spatial_landmark_gate_from_values(
        result=None,
        initial=initial,
        pose_key=X(2),
        landmark_key=L(17),
        x_base=3.0,
        y_base=4.0,
        max_jump=0.75,
    )

    assert not accepted
    assert pred_x == 4.0
    assert pred_y == 6.0
    assert jump > 0.75
    assert landmark_x == 8.0
    assert landmark_y == 10.0
