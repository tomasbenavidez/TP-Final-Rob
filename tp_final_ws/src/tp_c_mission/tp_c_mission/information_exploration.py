"""Política de exploración: utilidad esperada menos costo de la acción."""

from dataclasses import dataclass
import hashlib
import math

import numpy as np


_NEIGHBORS_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))


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
        # Cache de visibilidad por pose candidata. El ray-casting de una vista
        # depende sólo de (celda, yaw) y del mapa estático; sólo cambia cuántas
        # de esas celdas siguen sin observar. Precalcularlo una vez evita repetir
        # el ray-casting de miles de poses en cada tick del supervisor.
        self._visibility = None       # dict: pose -> índices planos visibles
        self._cache_params = None     # (fov, max_range) con que se precalculó

    def precompute_visibility(self, poses, fov, max_range):
        """Precalcula las celdas visibles de cada pose como índices planos."""
        width = self.planner.width
        cache = {}
        for pose in poses:
            cells = self.planner.visible_cells(pose, fov, max_range)
            cache[pose] = np.fromiter(
                (row * width + col for row, col in cells),
                dtype=np.intp, count=len(cells))
        self._visibility = cache
        self._cache_params = (float(fov), float(max_range))
        return cache

    def _cached_indices(self, pose, fov, max_range):
        if (self._visibility is None
                or self._cache_params != (float(fov), float(max_range))):
            return None
        return self._visibility.get(pose)

    def observe(self, pose, fov, max_range):
        cells = self.planner.visible_cells(pose, fov, max_range)
        for cell in cells:
            self.observed[cell] = True
        return cells

    def unseen_count(self, pose, fov, max_range):
        """Sólo el número de celdas nuevas; ruta caliente del supervisor."""
        indices = self._cached_indices(pose, fov, max_range)
        if indices is not None:
            return int(np.count_nonzero(~self.observed.reshape(-1)[indices]))
        cells = self.planner.visible_cells(pose, fov, max_range)
        return sum(not self.observed[cell] for cell in cells)

    def gain(self, pose, fov, max_range):
        indices = self._cached_indices(pose, fov, max_range)
        if indices is not None:
            width = self.planner.width
            cells = {(int(idx) // width, int(idx) % width) for idx in indices}
        else:
            cells = self.planner.visible_cells(pose, fov, max_range)
        unseen = sum(not self.observed[cell] for cell in cells)
        return unseen, cells

    def coverage_fraction(self):
        free = ~self.planner.blocked
        count = int(free.sum())
        return 1.0 if count == 0 else float((self.observed & free).sum() / count)


def map_signature(data, resolution, origin_x, origin_y):
    values = np.asarray(data, dtype=np.int16)
    digest = hashlib.blake2b(values.tobytes(), digest_size=16).hexdigest()
    return (
        int(values.shape[0]), int(values.shape[1]),
        round(float(resolution), 9),
        round(float(origin_x), 9), round(float(origin_y), 9),
        digest,
    )


def observed_free_points(planner, coverage):
    rows, cols = np.where(coverage.observed & ~planner.blocked)
    return [
        planner.cell_to_world((row, col))
        for row, col in zip(rows.tolist(), cols.tolist())
    ]


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


def frontier_cells(planner, observed):
    """Celdas libres no observadas que limitan con espacio libre ya observado."""
    observed = np.asarray(observed, dtype=bool)
    free = ~planner.blocked
    frontiers = set()
    rows, cols = np.where(free & ~observed)
    for row, col in zip(rows.tolist(), cols.tolist()):
        for dr, dc in _NEIGHBORS_4:
            neighbor = (row + dr, col + dc)
            if planner.inside(neighbor) and free[neighbor] and observed[neighbor]:
                frontiers.add((row, col))
                break
    return frontiers


def select_frontier_action(
    planner, coverage, robot_xy, candidate_poses, fov, max_range,
    exhausted_cells=None, max_debug=12,
):
    """Elige una vista alcanzable que mire hacia la frontera observado/no observado."""
    frontiers = frontier_cells(planner, coverage.observed)
    if not frontiers:
        return None, []
    exhausted_cells = set() if exhausted_cells is None else set(exhausted_cells)
    diagonal = math.hypot(planner.width, planner.height) * planner.resolution
    max_visible_cells = max(
        1.0,
        float(fov) * float(max_range) ** 2 / (2.0 * planner.resolution ** 2),
    )
    candidates = []
    debug = []
    for pose in candidate_poses:
        cell = planner.world_to_cell(*pose[:2])
        if (not planner.inside(cell) or planner.blocked[cell] or
                cell in exhausted_cells):
            continue
        path = planner.plan_world(robot_xy, pose[:2], simplify=False)
        if path is None:
            continue
        unseen, visible = coverage.gain(pose, fov, max_range)
        frontier_seen = len(frontiers.intersection(visible))
        if frontier_seen == 0 and unseen == 0:
            continue
        nearest_frontier = min(
            math.hypot(cell[0] - item[0], cell[1] - item[1])
            for item in frontiers
        )
        frontier_proximity = 1.0 / (1.0 + nearest_frontier)
        path_cost = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                        for a, b in zip(path, path[1:])) / max(diagonal, 1e-6)
        max_risk = max(float(planner.cost.max()), 1e-6)
        risk = min(1.0, float(planner.cost[cell]) / max_risk)
        coverage_gain = min(
            1.0,
            (float(unseen) + 2.0 * float(frontier_seen)) / max_visible_cells,
        )
        openness = min(1.0, float(len(visible)) / max_visible_cells)
        action = CandidateAction(
            pose, coverage_gain=coverage_gain, localization_gain=0.0,
            path_cost=path_cost, risk=risk, repetition=0.0,
        )
        score = (
            coverage_gain + 0.20 * openness + 0.30 * frontier_proximity
            - 0.20 * path_cost - 0.05 * risk
        )
        candidates.append((score, action))
        debug.append((score, action, frontier_seen, unseen))
    candidates.sort(reverse=True, key=lambda item: item[0])
    debug.sort(reverse=True, key=lambda item: item[0])
    return (
        None if not candidates else candidates[0][1],
        debug[:max(0, int(max_debug))],
    )
