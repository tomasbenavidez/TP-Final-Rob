import json
from pathlib import Path
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from tp_b_navigation.landmark_io import load_landmark_map  # noqa: E402


class LandmarkIoTest(unittest.TestCase):
    def test_loads_ids_and_positions_from_part_a_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trajectory.json'
            path.write_text(json.dumps({
                'trajectory': [],
                'landmarks': {'7': {'x': 1.2, 'y': -0.4}, '2': {'x': 0, 'y': 3}},
            }))

            landmarks = load_landmark_map(path)

        self.assertEqual(landmarks, {2: (0.0, 3.0), 7: (1.2, -0.4)})

    def test_rejects_missing_landmark_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.json'
            path.write_text(json.dumps({'trajectory': []}))
            with self.assertRaisesRegex(ValueError, 'landmarks'):
                load_landmark_map(path)


if __name__ == '__main__':
    unittest.main()
