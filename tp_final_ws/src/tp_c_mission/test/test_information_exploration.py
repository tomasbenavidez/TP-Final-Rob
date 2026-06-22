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
    InformationPolicy,
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


if __name__ == '__main__':
    unittest.main()
