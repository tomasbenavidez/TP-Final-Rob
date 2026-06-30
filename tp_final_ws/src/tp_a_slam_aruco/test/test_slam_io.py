import json

from tp_a_slam_aruco.slam_io import read_trajectory_json, write_trajectory_json


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


def test_read_trajectory_json_sorts_poses_by_stamp(tmp_path):
    input_path = tmp_path / 'trajectory.json'
    input_path.write_text(json.dumps({
        'trajectory': [
            {'i': 1, 'x': 1.0, 'y': 0.0, 'theta': 0.0, 'stamp': 2.0},
            {'i': 0, 'x': 0.0, 'y': 0.0, 'theta': 0.0, 'stamp': 1.0},
        ],
        'landmarks': {'7': {'x': 2.0, 'y': 3.0}},
    }))

    trajectory, landmarks = read_trajectory_json(input_path)

    assert [pose['i'] for pose in trajectory] == [0, 1]
    assert landmarks == {'7': {'x': 2.0, 'y': 3.0}}


def test_read_trajectory_json_rejects_missing_pose_fields(tmp_path):
    input_path = tmp_path / 'trajectory.json'
    input_path.write_text(json.dumps({
        'trajectory': [{'i': 0, 'x': 0.0, 'y': 0.0, 'stamp': 1.0}],
        'landmarks': {},
    }))

    try:
        read_trajectory_json(input_path)
    except ValueError as exc:
        assert 'theta' in str(exc)
    else:
        raise AssertionError('expected missing theta to be rejected')
