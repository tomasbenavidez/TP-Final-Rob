"""Modelos puros de medición para MCL.

Este módulo no depende de ROS: recibe datos ya extraídos de mensajes y devuelve
log-verosimilitudes por partícula. `mcl_localization` se ocupa de suscripciones,
TF y publicación.
"""

from dataclasses import dataclass
import math

import numpy as np

from tp_b_navigation.planner_core import GridPlannerCore


@dataclass(frozen=True)
class LandmarkMeasurement:
    landmark_id: int
    range_m: float
    bearing_rad: float
    source: str


def legacy_pose_observations_to_measurements(poses, source='legacy_pose_array'):
    """Convierte el contrato legacy PoseArray a mediciones identificadas por índice."""
    measurements = []
    for index, pose in enumerate(poses):
        range_m = float(pose.position.x)
        bearing = float(pose.position.z)
        if range_m == 0.0 and bearing == 0.0:
            continue
        if range_m <= 0.0:
            continue
        measurements.append(
            LandmarkMeasurement(index, range_m, bearing, source))
    return measurements


def landmark_log_likelihood(
    particles,
    measurements,
    landmarks_by_id,
    sigma_range,
    sigma_bearing,
    log_weight=1.0,
):
    """Devuelve log-pesos por range/bearing a landmarks conocidos."""
    particles = np.asarray(particles, dtype=float)
    log_w = np.zeros(len(particles), dtype=float)
    if len(particles) == 0:
        return log_w, 0

    px = particles[:, 0]
    py = particles[:, 1]
    pth = particles[:, 2]
    inv_2sr2 = 1.0 / (2.0 * float(sigma_range) ** 2)
    inv_2sb2 = 1.0 / (2.0 * float(sigma_bearing) ** 2)

    used = 0
    for measurement in measurements:
        if isinstance(measurement, tuple):
            landmark_id, range_m, bearing_rad = measurement[:3]
        else:
            landmark_id = measurement.landmark_id
            range_m = measurement.range_m
            bearing_rad = measurement.bearing_rad
        landmark_id = int(landmark_id)
        if landmark_id not in landmarks_by_id:
            continue
        used += 1
        lmx, lmy = landmarks_by_id[landmark_id]
        dx = float(lmx) - px
        dy = float(lmy) - py
        r_hat = np.hypot(dx, dy)
        phi_hat = np.arctan2(dy, dx) - pth
        dr = float(range_m) - r_hat
        dphi = np.arctan2(
            np.sin(float(bearing_rad) - phi_hat),
            np.cos(float(bearing_rad) - phi_hat),
        )
        log_w += -(dr * dr) * inv_2sr2 - (dphi * dphi) * inv_2sb2
    return log_w * float(log_weight), used


@dataclass(frozen=True)
class LikelihoodField:
    distances_m: np.ndarray
    occupied: np.ndarray
    known: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    max_distance: float

    @classmethod
    def from_occupancy(
        cls,
        data,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        max_distance,
    ):
        values = np.asarray(data, dtype=np.int16).reshape(int(height), int(width))
        occupied = values == 100
        known = values != -1
        distances = GridPlannerCore.distance_transform(occupied) * float(resolution)
        distances = np.minimum(distances, float(max_distance))
        return cls(
            distances_m=distances,
            occupied=occupied,
            known=known,
            resolution=float(resolution),
            origin_x=float(origin_x),
            origin_y=float(origin_y),
            width=int(width),
            height=int(height),
            max_distance=float(max_distance),
        )

    def world_to_cell(self, x, y):
        rows = ((np.asarray(y, dtype=float) - self.origin_y) / self.resolution).astype(int)
        cols = ((np.asarray(x, dtype=float) - self.origin_x) / self.resolution).astype(int)
        return rows, cols

    def inside_mask(self, rows, cols):
        return (
            (rows >= 0) & (rows < self.height) &
            (cols >= 0) & (cols < self.width)
        )

    def endpoint_distances(self, x, y):
        rows, cols = self.world_to_cell(x, y)
        inside = self.inside_mask(rows, cols)
        distances = np.full(np.shape(rows), self.max_distance, dtype=float)
        distances[inside] = self.distances_m[rows[inside], cols[inside]]
        return distances, inside

    def invalid_pose_mask(self, particles):
        rows, cols = self.world_to_cell(particles[:, 0], particles[:, 1])
        inside = self.inside_mask(rows, cols)
        invalid = ~inside
        if np.any(inside):
            invalid_inside = (
                self.occupied[rows[inside], cols[inside]] |
                ~self.known[rows[inside], cols[inside]]
            )
            invalid[inside] = invalid_inside
        return invalid


def _sample_beam_indices(ranges, max_beams):
    count = len(ranges)
    if count == 0:
        return np.array([], dtype=int)
    max_beams = max(1, int(max_beams))
    if count <= max_beams:
        return np.arange(count, dtype=int)
    return np.linspace(0, count - 1, max_beams, dtype=int)


def laser_scan_log_likelihood(
    particles,
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    sensor_pose,
    field,
    max_beams,
    sigma_hit,
    occupied_pose_penalty,
    log_weight=1.0,
):
    """Likelihood field simple para endpoints de LIDAR contra mapa."""
    particles = np.asarray(particles, dtype=float)
    log_w = np.zeros(len(particles), dtype=float)
    ranges_array = np.asarray(ranges, dtype=float)
    if len(particles) == 0 or ranges_array.size == 0:
        return log_w, 0

    indices = _sample_beam_indices(ranges_array, max_beams)
    valid_ranges = []
    valid_angles = []
    for index in indices:
        value = float(ranges_array[index])
        if not math.isfinite(value):
            continue
        if float(range_min) <= value <= float(range_max):
            valid_ranges.append(value)
            valid_angles.append(float(angle_min) + int(index) * float(angle_increment))
    if not valid_ranges:
        return log_w, 0

    sx, sy, syaw = sensor_pose
    sensor_yaw = particles[:, 2] + float(syaw)
    sensor_x = particles[:, 0] + float(sx) * np.cos(particles[:, 2]) - float(sy) * np.sin(particles[:, 2])
    sensor_y = particles[:, 1] + float(sx) * np.sin(particles[:, 2]) + float(sy) * np.cos(particles[:, 2])

    inv_2sigma2 = 1.0 / (2.0 * float(sigma_hit) ** 2)
    for range_m, angle in zip(valid_ranges, valid_angles):
        beam_angle = sensor_yaw + angle
        end_x = sensor_x + range_m * np.cos(beam_angle)
        end_y = sensor_y + range_m * np.sin(beam_angle)
        distances, _inside = field.endpoint_distances(end_x, end_y)
        log_w += -(distances * distances) * inv_2sigma2

    log_w /= float(len(valid_ranges))
    invalid_pose = field.invalid_pose_mask(particles)
    log_w[invalid_pose] -= float(occupied_pose_penalty)
    return log_w * float(log_weight), len(valid_ranges)
