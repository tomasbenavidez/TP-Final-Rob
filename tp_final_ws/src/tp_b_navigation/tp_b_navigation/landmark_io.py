"""Carga portable del mapa de landmarks producido por Parte A."""

import json
from pathlib import Path


def load_landmark_map(path):
    payload = json.loads(Path(path).read_text(encoding='utf-8'))
    landmarks = payload.get('landmarks')
    if not isinstance(landmarks, dict):
        raise ValueError('landmark map must contain a landmarks object')
    result = {}
    for key, value in landmarks.items():
        if not isinstance(value, dict) or not {'x', 'y'} <= set(value):
            raise ValueError(f'invalid landmark entry {key}')
        result[int(key)] = (float(value['x']), float(value['y']))
    return dict(sorted(result.items()))
