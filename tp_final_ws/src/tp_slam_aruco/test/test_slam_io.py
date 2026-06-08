import json

from tp_slam_aruco.slam_io import write_trajectory_json


def test_write_trajectory_json_creates_missing_parent_directories(tmp_path):
    output_path = tmp_path / 'nested' / 'runs' / 'trayectoria.json'

    write_trajectory_json(
        path=output_path,
        trajectory=[
            {'i': 0, 'x': 0.0, 'y': 0.0, 'theta': 0.0, 'stamp': 1.25},
        ],
        landmarks={
            '19': {'x': 1.0, 'y': 2.0},
        },
    )

    assert output_path.exists()
    assert json.loads(output_path.read_text()) == {
        'trajectory': [
            {'i': 0, 'x': 0.0, 'y': 0.0, 'theta': 0.0, 'stamp': 1.25},
        ],
        'landmarks': {
            '19': {'x': 1.0, 'y': 2.0},
        },
    }
