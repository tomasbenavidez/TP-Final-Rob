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


def test_write_trajectory_json_persists_stats_block(tmp_path):
    output_path = tmp_path / 'trayectoria.json'

    write_trajectory_json(
        path=output_path,
        trajectory=[],
        landmarks={},
        stats={
            'visual_observability': {
                'summary': {
                    'frame_count': 2,
                    'max_valid_unique_ever': 2,
                },
                'frames': [
                    {'stamp': 1.0, 'raw_count': 2, 'valid_unique_count': 2},
                    {'stamp': 2.0, 'raw_count': 1, 'valid_unique_count': 0},
                ],
            }
        },
    )

    payload = json.loads(output_path.read_text())
    assert payload['stats']['visual_observability']['summary']['frame_count'] == 2
    assert payload['stats']['visual_observability']['frames'][0]['valid_unique_count'] == 2
