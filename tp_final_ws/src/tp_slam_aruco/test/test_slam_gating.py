import gtsam
from gtsam.symbol_shorthand import L, X

from tp_slam_aruco.slam_gating import innovation_gate_from_values


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
