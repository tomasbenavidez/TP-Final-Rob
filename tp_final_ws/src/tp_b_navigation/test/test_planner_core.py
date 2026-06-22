import math
from pathlib import Path
import sys
import unittest

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from tp_b_navigation.planner_core import GridPlannerCore  # noqa: E402


def make_core(blocked, resolution=1.0):
    return GridPlannerCore.from_blocked(
        np.asarray(blocked, dtype=bool),
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        robot_radius=0.0,
        clearance_weight=0.0,
        clearance_max=0.0,
    )


class GridPlannerCoreTest(unittest.TestCase):
    def test_astar_routes_around_wall_opening(self):
        blocked = np.zeros((7, 7), dtype=bool)
        blocked[0, :] = True
        blocked[6, :] = True
        blocked[1:6, 3] = True
        blocked[5, 3] = False
        core = make_core(blocked)

        path = core.plan_cells((2, 1), (2, 5), simplify=False)

        self.assertIsNotNone(path)
        self.assertIn((5, 3), path)
        self.assertTrue(all(not blocked[r, c] for r, c in path))

    def test_raycast_stops_at_known_wall(self):
        blocked = np.zeros((5, 7), dtype=bool)
        blocked[:, 4] = True
        core = make_core(blocked)

        visible = core.visible_cells((2.5, 2.5, 0.0), fov=0.6, max_range=6.0)

        self.assertIn((2, 3), visible)
        self.assertNotIn((2, 4), visible)
        self.assertNotIn((2, 5), visible)

    def test_path_cost_matches_metric_length(self):
        core = make_core(np.zeros((5, 5), dtype=bool), resolution=0.5)

        path = core.plan_cells((1, 1), (4, 1), simplify=False)

        self.assertTrue(math.isclose(core.path_length(path), 1.5, rel_tol=1e-6))

    def test_nearest_free_rejects_outside_map(self):
        core = make_core(np.zeros((3, 3), dtype=bool))

        self.assertIsNone(core.nearest_free((-1, 0)))
        self.assertIsNone(core.nearest_free((5, 5)))


if __name__ == '__main__':
    unittest.main()
