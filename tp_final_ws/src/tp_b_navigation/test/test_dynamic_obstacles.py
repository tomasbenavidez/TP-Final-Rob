from pathlib import Path
import sys
import unittest

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from tp_b_navigation.dynamic_obstacles import (  # noqa: E402
    apply_dynamic_obstacles,
    mark_dynamic_obstacles,
    same_static_occupancy,
)
from tp_b_navigation.planner_core import GridPlannerCore  # noqa: E402


class DynamicObstaclesTest(unittest.TestCase):
    def test_planner_routes_around_dynamic_obstacle_layer(self):
        static = np.zeros((7, 8), dtype=np.int16)
        static[0, :] = static[6, :] = 100
        dynamic = np.zeros_like(static)
        dynamic[1:6, 3] = 100
        dynamic[5, 3] = 0

        merged = apply_dynamic_obstacles(static, dynamic)
        core = GridPlannerCore.from_occupancy(
            merged, 1.0, 0.0, 0.0, robot_radius=0.0,
            clearance_weight=0.0, clearance_max=0.0,
            allow_unknown=False,
        )
        path = core.plan_cells((2, 1), (2, 5), simplify=False)

        self.assertIsNotNone(path)
        self.assertIn((5, 3), path)
        self.assertNotIn((2, 3), path)

    def test_new_lidar_points_mark_persistent_dynamic_cells(self):
        static_occupied = np.zeros((8, 8), dtype=bool)
        dynamic = np.zeros((8, 8), dtype=bool)
        rows = np.array([3, 3, 3])
        cols = np.array([4, 5, 6])

        changed, count = mark_dynamic_obstacles(
            dynamic, static_occupied, rows, cols, inflation_cells=1)

        self.assertTrue(changed)
        self.assertEqual(count, 3)
        self.assertTrue(dynamic[3, 5])
        self.assertTrue(dynamic[2, 5])
        self.assertTrue(dynamic[4, 5])

        changed_again, count_again = mark_dynamic_obstacles(
            dynamic, static_occupied, rows, cols, inflation_cells=1)

        self.assertFalse(changed_again)
        self.assertEqual(count_again, 3)

    def test_dynamic_inflation_uses_disk_not_square_corners(self):
        static_occupied = np.zeros((9, 9), dtype=bool)
        dynamic = np.zeros((9, 9), dtype=bool)

        changed, count = mark_dynamic_obstacles(
            dynamic, static_occupied, np.array([4]), np.array([4]),
            inflation_cells=2)

        self.assertTrue(changed)
        self.assertEqual(count, 1)
        self.assertTrue(dynamic[4, 6])
        self.assertTrue(dynamic[6, 4])
        self.assertFalse(dynamic[6, 6])
        self.assertFalse(dynamic[2, 2])

    def test_compact_dynamic_obstacle_keeps_corridor_passable(self):
        static = np.zeros((7, 9), dtype=np.int16)
        static[0, :] = static[6, :] = 100
        dynamic = np.zeros_like(static, dtype=bool)
        static_occupied = static == 100

        mark_dynamic_obstacles(
            dynamic, static_occupied, np.array([3]), np.array([4]),
            inflation_cells=1)
        merged = apply_dynamic_obstacles(static, dynamic.astype(np.int16) * 100)
        core = GridPlannerCore.from_occupancy(
            merged, 1.0, 0.0, 0.0, robot_radius=0.0,
            clearance_weight=0.0, clearance_max=0.0,
            allow_unknown=False,
        )
        path = core.plan_cells((3, 1), (3, 7), simplify=False)

        self.assertIsNotNone(path)
        self.assertNotIn((3, 4), path)

    def test_nearby_lidar_cluster_stays_compact(self):
        static_occupied = np.zeros((11, 11), dtype=bool)
        dynamic = np.zeros((11, 11), dtype=bool)

        changed, count = mark_dynamic_obstacles(
            dynamic, static_occupied, np.array([5, 5, 5, 5, 5]),
            np.array([3, 4, 5, 6, 7]), inflation_cells=1)

        self.assertTrue(changed)
        self.assertEqual(count, 5)
        self.assertTrue(dynamic[5, 5])
        self.assertTrue(dynamic[5, 4])
        self.assertTrue(dynamic[5, 6])
        self.assertFalse(dynamic[5, 3])
        self.assertFalse(dynamic[5, 7])

    def test_known_walls_are_not_added_as_dynamic_obstacles(self):
        static_occupied = np.zeros((5, 5), dtype=bool)
        static_occupied[2, 2] = True
        dynamic = np.zeros((5, 5), dtype=bool)

        changed, count = mark_dynamic_obstacles(
            dynamic, static_occupied, np.array([2]), np.array([2]),
            inflation_cells=1)

        self.assertFalse(changed)
        self.assertEqual(count, 0)
        self.assertFalse(dynamic.any())

    def test_identical_static_map_republish_is_detected(self):
        first = np.zeros((4, 4), dtype=bool)
        second = first.copy()
        changed = first.copy()
        changed[2, 2] = True

        self.assertTrue(same_static_occupancy(first, second))
        self.assertFalse(same_static_occupancy(first, changed))


if __name__ == '__main__':
    unittest.main()
