from pathlib import Path
import sys
import unittest

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from tp_c_mission.cone_perception import (  # noqa: E402
    ConeTracker,
    detect_red_regions,
    estimate_range,
)


class ConePerceptionTest(unittest.TestCase):
    def test_red_trapezoid_is_detected_and_colored_distractors_are_ignored(self):
        image = np.zeros((100, 140, 3), dtype=np.uint8)
        for row in range(25, 85):
            half_width = 5 + (row - 25) // 5
            image[row, 40 - half_width:40 + half_width + 1] = (0, 0, 255)
        image[25:85, 85:115] = (0, 255, 0)
        image[25:85, 115:135] = (255, 0, 0)

        detections, mask = detect_red_regions(image, min_area=100)

        self.assertEqual(len(detections), 1)
        self.assertGreater(mask[:, :70].sum(), 0)
        self.assertEqual(mask[:, 80:].sum(), 0)
        self.assertLess(abs(detections[0].center_u - 40), 3)

    def test_temporal_tracker_confirms_three_of_five_consistent_frames(self):
        tracker = ConeTracker(required_hits=3, window_size=5, max_center_distance_px=12)

        self.assertIsNone(tracker.update((40, 60)))
        self.assertIsNone(tracker.update(None))
        self.assertIsNone(tracker.update((43, 59)))
        confirmed = tracker.update((41, 61))

        self.assertIsNotNone(confirmed)
        self.assertAlmostEqual(confirmed[0], 41.3, places=1)

    def test_depth_median_is_primary_and_monocular_is_fallback(self):
        depth = np.array([0.0, 1.9, 2.0, 2.1, np.nan, 8.0])

        value, source = estimate_range(
            depth, pixel_height=50, focal_y=400, cone_height_m=0.30,
            min_depth=0.2, max_depth=4.0,
        )
        fallback, fallback_source = estimate_range(
            np.array([0.0, np.nan]), pixel_height=50, focal_y=400,
            cone_height_m=0.30, min_depth=0.2, max_depth=4.0,
        )

        self.assertAlmostEqual(value, 2.0)
        self.assertEqual(source, 'depth')
        self.assertAlmostEqual(fallback, 2.4)
        self.assertEqual(fallback_source, 'monocular')


if __name__ == '__main__':
    unittest.main()
