"""Política de exploración: utilidad esperada menos costo de la acción."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CandidateAction:
    pose: tuple
    coverage_gain: float
    localization_gain: float
    path_cost: float
    risk: float
    repetition: float = 0.0


class InformationPolicy:
    def __init__(
        self, coverage_weight=0.55, localization_weight=0.20,
        path_weight=0.20, risk_weight=0.05,
    ):
        self.coverage_weight = float(coverage_weight)
        self.localization_weight = float(localization_weight)
        self.path_weight = float(path_weight)
        self.risk_weight = float(risk_weight)

    def utility(self, candidate, covariance_scale=1.0):
        localization = min(1.0, candidate.localization_gain) * max(0.0, covariance_scale)
        return (
            self.coverage_weight * candidate.coverage_gain
            + self.localization_weight * localization
            - self.path_weight * candidate.path_cost
            - self.risk_weight * (candidate.risk + candidate.repetition)
        )

    def select(self, candidates, covariance_scale=1.0):
        if not candidates:
            return None
        return max(candidates, key=lambda item: self.utility(item, covariance_scale))


class CoverageBelief:
    """Máscara binaria: una celda libre no observada conserva entropía unitaria."""

    def __init__(self, planner):
        self.planner = planner
        self.observed = np.asarray(planner.blocked, dtype=bool).copy()

    def observe(self, pose, fov, max_range):
        cells = self.planner.visible_cells(pose, fov, max_range)
        for cell in cells:
            self.observed[cell] = True
        return cells

    def gain(self, pose, fov, max_range):
        cells = self.planner.visible_cells(pose, fov, max_range)
        unseen = sum(not self.observed[cell] for cell in cells)
        return unseen, cells

    def coverage_fraction(self):
        free = ~self.planner.blocked
        count = int(free.sum())
        return 1.0 if count == 0 else float((self.observed & free).sum() / count)


def expected_landmark_fraction(pose, landmarks, planner, fov, max_range):
    if not landmarks:
        return 0.0
    x, y, yaw = pose
    visible = 0
    for lx, ly in landmarks:
        distance = math.hypot(lx - x, ly - y)
        bearing = math.atan2(ly - y, lx - x)
        error = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
        if distance <= max_range and abs(error) <= fov / 2.0:
            start = planner.world_to_cell(x, y)
            end = planner.world_to_cell(lx, ly)
            # Los landmarks suelen estar sobre paredes: sólo exigimos libre hasta
            # la celda anterior al landmark.
            ray = list(planner.line_cells(start, end))
            if all(planner.inside(cell) and not planner.obstacles[cell] for cell in ray[:-1]):
                visible += 1
    return visible / float(len(landmarks))


def sample_candidate_poses(planner, spacing=0.50, yaw_samples=8):
    stride = max(1, int(round(spacing / planner.resolution)))
    for row in range(0, planner.height, stride):
        for col in range(0, planner.width, stride):
            if planner.blocked[row, col]:
                continue
            x, y = planner.cell_to_world((row, col))
            for index in range(yaw_samples):
                yield x, y, 2.0 * math.pi * index / yaw_samples


def select_approach_pose(planner, robot_xy, cone_xy, standoff=0.55, samples=16):
    """Elige una pose libre y alcanzable, orientada al cono, sobre un anillo."""
    best = None
    for index in range(max(4, int(samples))):
        angle = 2.0 * math.pi * index / max(4, int(samples))
        x = cone_xy[0] + standoff * math.cos(angle)
        y = cone_xy[1] + standoff * math.sin(angle)
        cell = planner.world_to_cell(x, y)
        cone_cell = planner.world_to_cell(*cone_xy)
        if not planner.inside(cell) or planner.blocked[cell]:
            continue
        if not planner.inside(cone_cell):
            continue
        # La aproximación final debe ver el cono sin atravesar una pared conocida.
        ray = list(planner.line_cells(cell, cone_cell))
        if any(planner.obstacles[item] for item in ray[:-1]):
            continue
        start = planner.nearest_free(planner.world_to_cell(*robot_xy))
        if start is None:
            return None
        path = planner.plan_cells(start, cell, simplify=False)
        if path is None:
            continue
        pose = (x, y, math.atan2(cone_xy[1] - y, cone_xy[0] - x))
        score = planner.path_length(path)
        if best is None or score < best[0]:
            best = score, pose, path
    return None if best is None else (best[1], best[2])
