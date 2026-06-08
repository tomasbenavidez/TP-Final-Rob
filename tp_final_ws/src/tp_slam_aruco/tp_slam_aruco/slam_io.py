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
