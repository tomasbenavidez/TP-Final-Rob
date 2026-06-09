import json
from pathlib import Path


def ensure_parent_dir(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def write_trajectory_json(path, trajectory, landmarks, stats=None):
    output_path = ensure_parent_dir(path)
    payload = {
        'trajectory': trajectory,
        'landmarks': landmarks,
    }
    if stats is not None:
        payload['stats'] = stats
    output_path.write_text(
        json.dumps(payload, indent=2)
    )
    return output_path
