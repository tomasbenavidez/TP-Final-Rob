#!/usr/bin/env python3
import math
import sys
import unittest
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from tp_b_navigation.landmark_visibility import (  # noqa: E402
    camera_point_from_base,
    visibility_reason,
)


class LandmarkVisibilityTest(unittest.TestCase):
    def setUp(self):
        self.ranges = np.full(360, np.inf)
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / len(self.ranges)

    def reason(self, base_point, camera_point=None, measured=None):
        if camera_point is None:
            camera_point = base_point
        if measured is not None:
            bearing = math.atan2(base_point[1], base_point[0])
            index = round(
                (bearing - self.angle_min) / self.angle_increment
            ) % len(self.ranges)
            self.ranges[index] = measured
        return visibility_reason(
            base_point=base_point,
            camera_point=camera_point,
            ranges=self.ranges,
            angle_min=self.angle_min,
            angle_increment=self.angle_increment,
            range_min=0.12,
            range_max=3.5,
            camera_fov=1.05,
            camera_max_range=3.0,
            occlusion_tol=0.08,
        )

    def test_front_landmark_on_wall_is_visible(self):
        self.assertEqual(self.reason((2.0, 0.0), measured=1.95), 'visible')

    def test_landmark_behind_camera_is_outside_fov(self):
        self.assertEqual(self.reason((-1.0, 0.0), measured=1.0), 'outside_fov')

    def test_landmark_outside_camera_range_is_rejected(self):
        self.assertEqual(self.reason((3.2, 0.0), measured=3.2), 'outside_range')

    def test_wall_ten_centimetres_before_landmark_occludes_it(self):
        self.assertEqual(self.reason((2.0, 0.0), measured=1.90), 'occluded')

    def test_adjacent_clear_ray_does_not_see_around_corner(self):
        center = round((0.0 - self.angle_min) / self.angle_increment)
        self.ranges[center] = 1.0
        self.ranges[center + 1] = 2.0
        self.assertEqual(self.reason((2.0, 0.0)), 'occluded')

    def test_invalid_central_scan_reading_is_rejected(self):
        self.assertEqual(self.reason((2.0, 0.0)), 'invalid_scan')

    def test_camera_translation_is_applied_before_fov(self):
        point = camera_point_from_base((-0.02, 0.0), camera_tx=-0.06,
                                       camera_ty=0.0, camera_yaw=0.0)
        self.assertAlmostEqual(point[0], 0.04)
        self.assertAlmostEqual(point[1], 0.0)


if __name__ == '__main__':
    unittest.main()
