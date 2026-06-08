import gtsam

from gtsam.symbol_shorthand import L, X


def rebuild_initial_from_result(result, pose_count, landmark_ids):
    new_initial = gtsam.Values()
    for i in range(pose_count):
        try:
            new_initial.insert(X(i), result.atPose2(X(i)))
        except Exception:
            pass

    for lm_id in landmark_ids:
        try:
            new_initial.insert(L(lm_id), result.atPoint2(L(lm_id)))
        except Exception:
            pass

    return new_initial


def optimize_graph(graph, initial, pose_count, landmark_ids):
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    err0 = graph.error(initial)
    result = optimizer.optimize()
    err1 = graph.error(result)
    new_initial = rebuild_initial_from_result(result, pose_count, landmark_ids)
    return result, new_initial, err0, err1
