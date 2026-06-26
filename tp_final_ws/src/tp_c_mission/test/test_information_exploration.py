from pathlib import Path
import math
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tp_b_navigation'))
sys.path.insert(0, str(ROOT / 'tp_c_mission'))

from tp_b_navigation.planner_core import GridPlannerCore  # noqa: E402
from tp_c_mission.information_exploration import (  # noqa: E402
    CandidateAction,
    CoverageBelief,
    frontier_cells,
    InformationPolicy,
    map_signature,
    observed_free_points,
    select_frontier_action,
    select_approach_pose,
)


class InformationExplorationTest(unittest.TestCase):
    def test_information_gain_can_outweigh_shorter_path(self):
        policy = InformationPolicy()
        nearby = CandidateAction((1.0, 0.0, 0.0), coverage_gain=0.1,
                                 localization_gain=0.0, path_cost=0.1, risk=0.0)
        informative = CandidateAction((3.0, 0.0, 0.0), coverage_gain=0.9,
                                      localization_gain=0.2, path_cost=0.5, risk=0.0)

        selected = policy.select([nearby, informative])

        self.assertEqual(selected, informative)

    def test_high_pose_uncertainty_rewards_landmark_revisit(self):
        policy = InformationPolicy()
        explore = CandidateAction((2.0, 0.0, 0.0), coverage_gain=0.5,
                                  localization_gain=0.0, path_cost=0.2, risk=0.0)
        revisit = CandidateAction((0.0, 2.0, 1.57), coverage_gain=0.1,
                                  localization_gain=1.0, path_cost=0.2, risk=0.0)

        selected = policy.select([explore, revisit], covariance_scale=2.0)

        self.assertEqual(selected, revisit)

    def test_cone_approach_path_goes_around_wall(self):
        blocked = np.zeros((7, 8), dtype=bool)
        blocked[0, :] = blocked[6, :] = True
        blocked[1:6, 3] = True
        blocked[5, 3] = False
        core = GridPlannerCore.from_blocked(
            blocked, 1.0, 0.0, 0.0, robot_radius=0.0,
            clearance_weight=0.0, clearance_max=0.0,
        )

        result = select_approach_pose(
            core, robot_xy=(1.5, 2.5), cone_xy=(5.5, 2.5),
            standoff=1.0, samples=16,
        )

        self.assertIsNotNone(result)
        pose, path = result
        self.assertGreater(core.path_length(path), 6.0)
        self.assertAlmostEqual(
            pose[2], math.atan2(2.5 - pose[1], 5.5 - pose[0]), places=6,
        )
        self.assertTrue(all(not blocked[cell] for cell in path))

    def test_frontier_cells_track_boundary_between_seen_and_unseen_space(self):
        blocked = np.ones((5, 5), dtype=bool)
        blocked[1:4, 1:4] = False
        core = GridPlannerCore.from_blocked(
            blocked, 1.0, 0.0, 0.0, robot_radius=0.0,
            clearance_weight=0.0, clearance_max=0.0,
        )
        coverage = CoverageBelief(core)
        coverage.observed[:, :] = True
        coverage.observed[2, 2] = False

        cells = frontier_cells(core, coverage.observed)

        self.assertEqual(cells, {(2, 2)})

    def test_frontier_fallback_selects_reachable_view_when_local_gain_is_exhausted(self):
        blocked = np.zeros((7, 7), dtype=bool)
        blocked[0, :] = blocked[6, :] = True
        blocked[:, 0] = blocked[:, 6] = True
        core = GridPlannerCore.from_blocked(
            blocked, 1.0, 0.0, 0.0, robot_radius=0.0,
            clearance_weight=0.0, clearance_max=0.0,
        )
        coverage = CoverageBelief(core)
        coverage.observed[:, :] = True
        coverage.observed[2:5, 4] = False
        poses = [
            (2.5, 2.5, 0.0),
            (4.5, 2.5, math.pi),
        ]

        selected, debug = select_frontier_action(
            core, coverage, robot_xy=(2.5, 2.5), candidate_poses=poses,
            fov=1.2, max_range=3.0,
            exhausted_cells={core.world_to_cell(*poses[0][:2])},
        )

        self.assertIsNotNone(selected)
        self.assertGreater(selected.coverage_gain, 0.0)
        self.assertEqual(selected.pose, poses[1])
        self.assertTrue(debug)

    def test_frontier_fallback_skips_blocked_and_unreachable_candidates(self):
        blocked = np.zeros((7, 8), dtype=bool)
        blocked[0, :] = blocked[6, :] = True
        blocked[:, 0] = blocked[:, 7] = True
        blocked[1:6, 3] = True
        core = GridPlannerCore.from_blocked(
            blocked, 1.0, 0.0, 0.0, robot_radius=0.0,
            clearance_weight=0.0, clearance_max=0.0,
        )
        coverage = CoverageBelief(core)
        coverage.observed[:, :] = True
        coverage.observed[2:5, 5] = False
        poses = [
            (5.5, 2.5, math.pi),
            (3.5, 2.5, 0.0),
        ]

        selected, debug = select_frontier_action(
            core, coverage, robot_xy=(1.5, 2.5), candidate_poses=poses,
            fov=1.2, max_range=3.0,
        )

        self.assertIsNone(selected)
        self.assertEqual(debug, [])

    def test_identical_map_signature_preserves_coverage_and_exhausted_state(self):
        data = np.zeros((4, 4), dtype=np.int16)
        first = map_signature(data, 0.05, -1.0, -1.0)
        second = map_signature(data.copy(), 0.05, -1.0, -1.0)

        self.assertEqual(first, second)

        exhausted = {(2, 2)}

        self.assertEqual(exhausted, {(2, 2)})

    def test_changed_map_signature_rebuilds_coverage_with_only_blocked_cells_seen(self):
        data = np.zeros((4, 4), dtype=np.int16)
        changed = data.copy()
        changed[1, 1] = 100

        self.assertNotEqual(
            map_signature(data, 0.05, -1.0, -1.0),
            map_signature(changed, 0.05, -1.0, -1.0),
        )

        core = GridPlannerCore.from_blocked(
            changed == 100, 1.0, 0.0, 0.0, robot_radius=0.0,
            clearance_weight=0.0, clearance_max=0.0,
        )
        rebuilt = CoverageBelief(core)

        self.assertTrue(rebuilt.observed[1, 1])
        self.assertFalse(rebuilt.observed[1, 2])

    def test_policy_prefers_unvisited_frontier_over_repeating_local_view(self):
        blocked = np.zeros((7, 7), dtype=bool)
        blocked[0, :] = blocked[6, :] = True
        blocked[:, 0] = blocked[:, 6] = True
        core = GridPlannerCore.from_blocked(
            blocked, 1.0, 0.0, 0.0, robot_radius=0.0,
            clearance_weight=0.0, clearance_max=0.0,
        )
        coverage = CoverageBelief(core)
        coverage.observed[:, :] = True
        coverage.observed[2:5, 4] = False
        repeated = (2.5, 2.5, 0.0)
        frontier = (4.5, 2.5, math.pi)

        selected, _debug = select_frontier_action(
            core, coverage, robot_xy=(2.5, 2.5),
            candidate_poses=[repeated, frontier], fov=1.2, max_range=3.0,
            exhausted_cells={core.world_to_cell(*repeated[:2])},
        )

        self.assertEqual(selected.pose, frontier)

    def test_coverage_visualization_points_are_accumulated_observed_free_cells(self):
        blocked = np.zeros((4, 4), dtype=bool)
        blocked[0, 0] = True
        core = GridPlannerCore.from_blocked(
            blocked, 1.0, 0.0, 0.0, robot_radius=0.0,
            clearance_weight=0.0, clearance_max=0.0,
        )
        coverage = CoverageBelief(core)
        coverage.observed[:, :] = False
        coverage.observed[0, 0] = True
        coverage.observed[1, 1] = True
        coverage.observed[2, 2] = True

        points = observed_free_points(core, coverage)

        self.assertEqual(points, [core.cell_to_world((1, 1)), core.cell_to_world((2, 2))])


if __name__ == '__main__':
    unittest.main()
