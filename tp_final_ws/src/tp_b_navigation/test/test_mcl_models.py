import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


def _pose_observation(range_m, bearing):
    return SimpleNamespace(
        position=SimpleNamespace(x=range_m, y=0.0, z=bearing),
    )


class MclModelsTest(unittest.TestCase):
    def test_legacy_pose_observations_keep_index_ids(self):
        from tp_b_navigation.mcl_models import legacy_pose_observations_to_measurements

        measurements = legacy_pose_observations_to_measurements([
            _pose_observation(0.0, 0.0),
            _pose_observation(1.2, -0.3),
            _pose_observation(0.8, 0.4),
        ])

        self.assertEqual([m.landmark_id for m in measurements], [1, 2])
        self.assertEqual(measurements[0].range_m, 1.2)
        self.assertEqual(measurements[0].bearing_rad, -0.3)
        self.assertEqual(measurements[0].source, 'legacy_pose_array')

    def test_landmark_likelihood_favors_consistent_particle(self):
        from tp_b_navigation.mcl_models import (
            LandmarkMeasurement,
            landmark_log_likelihood,
        )

        particles = np.array([
            [0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0],
        ])
        log_w, used = landmark_log_likelihood(
            particles,
            [LandmarkMeasurement(7, 1.0, 0.0, 'test')],
            {7: (1.0, 0.0)},
            sigma_range=0.2,
            sigma_bearing=0.15,
        )

        self.assertEqual(used, 1)
        self.assertGreater(log_w[0], log_w[1])

    def test_likelihood_field_favors_scan_endpoint_near_wall(self):
        from tp_b_navigation.mcl_models import (
            LikelihoodField,
            laser_scan_log_likelihood,
        )

        data = np.zeros((5, 5), dtype=np.int16)
        data[:, 3] = 100
        field = LikelihoodField.from_occupancy(
            data.flatten().tolist(),
            width=5,
            height=5,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            max_distance=3.0,
        )
        particles = np.array([
            [1.5, 2.5, 0.0],
            [0.5, 2.5, 0.0],
        ])
        log_w, used = laser_scan_log_likelihood(
            particles,
            ranges=[1.5],
            angle_min=0.0,
            angle_increment=1.0,
            range_min=0.1,
            range_max=5.0,
            sensor_pose=(0.0, 0.0, 0.0),
            field=field,
            max_beams=10,
            sigma_hit=0.4,
            occupied_pose_penalty=20.0,
        )

        self.assertEqual(used, 1)
        self.assertGreater(log_w[0], log_w[1])

    def test_laser_likelihood_penalizes_particle_inside_occupied_cell(self):
        from tp_b_navigation.mcl_models import (
            LikelihoodField,
            laser_scan_log_likelihood,
        )

        data = np.zeros((3, 3), dtype=np.int16)
        data[1, 1] = 100
        field = LikelihoodField.from_occupancy(
            data.flatten().tolist(), 3, 3, 1.0, 0.0, 0.0, 2.0)
        particles = np.array([
            [0.5, 0.5, 0.0],
            [1.5, 1.5, 0.0],
        ])
        log_w, used = laser_scan_log_likelihood(
            particles,
            ranges=[1.0],
            angle_min=math.pi,
            angle_increment=1.0,
            range_min=0.1,
            range_max=5.0,
            sensor_pose=(0.0, 0.0, 0.0),
            field=field,
            max_beams=10,
            sigma_hit=0.5,
            occupied_pose_penalty=20.0,
        )

        self.assertEqual(used, 1)
        self.assertLess(log_w[1], log_w[0])


if __name__ == '__main__':
    unittest.main()
