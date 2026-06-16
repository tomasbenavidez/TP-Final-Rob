import json
from pathlib import Path


def ensure_parent_dir(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def write_trajectory_json(path, trajectory, landmarks):
    output_path = ensure_parent_dir(path)
    output_path.write_text(
        json.dumps(
            {
                'trajectory': trajectory,
                'landmarks': landmarks,
            },
            indent=2,
        )
    )
    return output_path


def read_trajectory_json(path):
    input_path = Path(path)
    data = json.loads(input_path.read_text())
    if not isinstance(data, dict):
        raise ValueError('trajectory JSON must be an object')

    trajectory = data.get('trajectory')
    landmarks = data.get('landmarks', {})
    if not isinstance(trajectory, list):
        raise ValueError('trajectory JSON must contain a trajectory list')
    if not isinstance(landmarks, dict):
        raise ValueError('trajectory JSON landmarks must be an object')

    required_pose_fields = {'i', 'x', 'y', 'theta', 'stamp'}
    normalized = []
    for index, pose in enumerate(trajectory):
        if not isinstance(pose, dict):
            raise ValueError(f'trajectory pose #{index} must be an object')
        missing = sorted(required_pose_fields - set(pose))
        if missing:
            raise ValueError(
                f'trajectory pose #{index} missing fields: {", ".join(missing)}'
            )
        normalized.append({
            'i': int(pose['i']),
            'x': float(pose['x']),
            'y': float(pose['y']),
            'theta': float(pose['theta']),
            'stamp': float(pose['stamp']),
        })

    normalized.sort(key=lambda pose: pose['stamp'])
    return normalized, landmarks
