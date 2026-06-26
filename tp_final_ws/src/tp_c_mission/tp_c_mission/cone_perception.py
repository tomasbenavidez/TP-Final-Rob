"""Núcleo independiente de ROS para segmentación y confirmación de conos."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class RedRegion:
    center_u: float
    center_v: float
    area: int
    x: int
    y: int
    width: int
    height: int
    pixels: tuple


def _bgr_to_hsv_opencv(image):
    rgb = np.asarray(image, dtype=np.float32)[..., ::-1] / 255.0
    maximum = rgb.max(axis=2)
    minimum = rgb.min(axis=2)
    delta = maximum - minimum
    hue = np.zeros_like(maximum)
    nonzero = delta > 1e-8
    red = nonzero & (maximum == rgb[..., 0])
    green = nonzero & (maximum == rgb[..., 1])
    blue = nonzero & (maximum == rgb[..., 2])
    hue[red] = ((rgb[..., 1][red] - rgb[..., 2][red]) / delta[red]) % 6.0
    hue[green] = (rgb[..., 2][green] - rgb[..., 0][green]) / delta[green] + 2.0
    hue[blue] = (rgb[..., 0][blue] - rgb[..., 1][blue]) / delta[blue] + 4.0
    hue *= 30.0
    saturation = np.zeros_like(maximum)
    nz_value = maximum > 1e-8
    saturation[nz_value] = delta[nz_value] / maximum[nz_value]
    return hue, saturation * 255.0, maximum * 255.0


def red_mask(image, hue_low=10, hue_high=170, min_saturation=100, min_value=70):
    hue, saturation, value = _bgr_to_hsv_opencv(image)
    red = (hue <= hue_low) | (hue >= hue_high)
    return red & (saturation >= min_saturation) & (value >= min_value)


def _binary_filter(mask, radius, operation):
    if radius <= 0:
        return mask
    padded = np.pad(mask, radius, mode='constant', constant_values=False)
    windows = [
        padded[dr:dr + mask.shape[0], dc:dc + mask.shape[1]]
        for dr in range(2 * radius + 1)
        for dc in range(2 * radius + 1)
    ]
    return operation(np.stack(windows, axis=0), axis=0)


def clean_mask(mask, radius=1):
    opened = _binary_filter(_binary_filter(mask, radius, np.all), radius, np.any)
    return _binary_filter(_binary_filter(opened, radius, np.any), radius, np.all)


def _components(mask):
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    for start_row, start_col in zip(*np.where(mask)):
        if visited[start_row, start_col]:
            continue
        queue = deque([(int(start_row), int(start_col))])
        visited[start_row, start_col] = True
        pixels = []
        while queue:
            row, col = queue.popleft()
            pixels.append((row, col))
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = row + dr, col + dc
                if (0 <= nr < height and 0 <= nc < width and mask[nr, nc]
                        and not visited[nr, nc]):
                    visited[nr, nc] = True
                    queue.append((nr, nc))
        yield pixels


def detect_red_regions(
    image, min_area=150, min_height=10, min_fill=0.20,
    hue_low=10, hue_high=170, min_saturation=100, min_value=70,
    morphology_radius=1,
):
    """Devuelve regiones rojas con geometría compatible con un cono vertical."""
    mask = clean_mask(
        red_mask(image, hue_low, hue_high, min_saturation, min_value),
        morphology_radius)
    detections = []
    for pixels in _components(mask):
        if len(pixels) < min_area:
            continue
        rows = np.fromiter((p[0] for p in pixels), dtype=int)
        cols = np.fromiter((p[1] for p in pixels), dtype=int)
        y0, y1 = int(rows.min()), int(rows.max())
        x0, x1 = int(cols.min()), int(cols.max())
        height, width = y1 - y0 + 1, x1 - x0 + 1
        fill = len(pixels) / float(height * width)
        if height < min_height or fill < min_fill:
            continue
        third = max(1, height // 3)
        top = cols[rows < y0 + third]
        bottom = cols[rows > y1 - third]
        if top.size and bottom.size:
            top_width = int(top.max() - top.min() + 1)
            bottom_width = int(bottom.max() - bottom.min() + 1)
            if bottom_width < 0.8 * top_width:
                continue
        detections.append(RedRegion(
            center_u=float(cols.mean()), center_v=float(rows.mean()), area=len(pixels),
            x=x0, y=y0, width=width, height=height, pixels=tuple(pixels),
        ))
    detections.sort(key=lambda item: item.area, reverse=True)
    return detections, mask


def estimate_range(
    depth_values, pixel_height, focal_y, cone_height_m,
    min_depth=0.2, max_depth=5.0,
):
    values = np.asarray(depth_values, dtype=float)
    values = values[np.isfinite(values) & (values >= min_depth) & (values <= max_depth)]
    if values.size:
        return float(np.median(values)), 'depth'
    if pixel_height <= 0 or focal_y <= 0 or cone_height_m <= 0:
        return None, 'unavailable'
    return float(focal_y * cone_height_m / pixel_height), 'monocular'


def pixel_to_camera(center_u, center_v, distance, fx, fy, cx, cy):
    """Proyección óptica ROS: x derecha, y abajo, z hacia adelante."""
    return (
        (float(center_u) - float(cx)) * distance / float(fx),
        (float(center_v) - float(cy)) * distance / float(fy),
        float(distance),
    )


class ConeTracker:
    def __init__(self, required_hits=3, window_size=5, max_center_distance_px=20.0):
        self.required_hits = int(required_hits)
        self.window_size = int(window_size)
        self.max_distance = float(max_center_distance_px)
        self.history = deque(maxlen=self.window_size)

    def update(self, center):
        if center is not None and self.history:
            previous = [item for item in self.history if item is not None]
            if previous:
                mean = np.mean(previous, axis=0)
                if math.hypot(center[0] - mean[0], center[1] - mean[1]) > self.max_distance:
                    self.history.clear()
        self.history.append(center)
        hits = [item for item in self.history if item is not None]
        if len(hits) < self.required_hits:
            return None
        mean = np.mean(hits, axis=0)
        return float(mean[0]), float(mean[1])

    def reset(self):
        self.history.clear()
