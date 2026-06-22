"""Algoritmos puros de grilla compartidos por navegación y exploración."""

from collections import deque
import heapq
import math

import numpy as np


class GridPlannerCore:
    """A* 8-conexo, inflado, simplificación y visibilidad sobre una grilla."""

    def __init__(self, blocked, cost, resolution, origin_x, origin_y, obstacles=None):
        self.blocked = np.asarray(blocked, dtype=bool)
        self.obstacles = np.asarray(
            self.blocked if obstacles is None else obstacles, dtype=bool)
        self.cost = np.asarray(cost, dtype=float)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.height, self.width = self.blocked.shape

    @classmethod
    def from_occupancy(
        cls, data, resolution, origin_x, origin_y, robot_radius=0.18,
        clearance_weight=0.6, clearance_max=0.5, allow_unknown=False,
    ):
        values = np.asarray(data, dtype=np.int16)
        obstacles = values == 100
        if not allow_unknown:
            obstacles |= values == -1
        return cls.from_blocked(
            obstacles, resolution, origin_x, origin_y, robot_radius,
            clearance_weight, clearance_max,
        )

    @classmethod
    def from_blocked(
        cls, blocked, resolution, origin_x, origin_y, robot_radius=0.18,
        clearance_weight=0.6, clearance_max=0.5,
    ):
        obstacles = np.asarray(blocked, dtype=bool)
        distances = cls.distance_transform(obstacles)
        distance_m = distances * float(resolution)
        inflated = obstacles | (distances < (float(robot_radius) / float(resolution)))
        clearance = np.clip(float(clearance_max) - distance_m, 0.0, float(clearance_max))
        return cls(
            inflated,
            float(clearance_weight) * clearance,
            resolution,
            origin_x,
            origin_y,
            obstacles=obstacles,
        )

    @staticmethod
    def distance_transform(obstacles):
        height, width = obstacles.shape
        distances = np.full((height, width), float('inf'), dtype=float)
        queue = deque()
        rows, cols = np.where(obstacles)
        for row, col in zip(rows.tolist(), cols.tolist()):
            distances[row, col] = 0.0
            queue.append((row, col))
        while queue:
            row, col = queue.popleft()
            next_distance = distances[row, col] + 1.0
            for dr, dc in _NEIGHBORS_8:
                nr, nc = row + dr, col + dc
                if (0 <= nr < height and 0 <= nc < width and
                        distances[nr, nc] > next_distance):
                    distances[nr, nc] = next_distance
                    queue.append((nr, nc))
        return distances

    def world_to_cell(self, x, y):
        return (
            int((float(y) - self.origin_y) / self.resolution),
            int((float(x) - self.origin_x) / self.resolution),
        )

    def cell_to_world(self, cell):
        row, col = cell
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def inside(self, cell):
        row, col = cell
        return 0 <= row < self.height and 0 <= col < self.width

    def nearest_free(self, cell, max_radius=12):
        if not self.inside(cell):
            return None
        if not self.blocked[cell]:
            return cell
        queue = deque([(cell[0], cell[1], 0)])
        seen = {cell}
        while queue:
            row, col, distance = queue.popleft()
            if distance >= max_radius:
                continue
            for dr, dc in _NEIGHBORS_4:
                nxt = (row + dr, col + dc)
                if nxt in seen or not self.inside(nxt):
                    continue
                if not self.blocked[nxt]:
                    return nxt
                seen.add(nxt)
                queue.append((nxt[0], nxt[1], distance + 1))
        return None

    def plan_world(self, start_xy, goal_xy, simplify=True):
        start = self.nearest_free(self.world_to_cell(*start_xy))
        goal = self.nearest_free(self.world_to_cell(*goal_xy))
        if start is None or goal is None:
            return None
        cells = self.plan_cells(start, goal, simplify=simplify)
        return None if cells is None else [self.cell_to_world(cell) for cell in cells]

    def plan_cells(self, start, goal, simplify=True):
        if not self.inside(start) or not self.inside(goal):
            return None
        if self.blocked[start] or self.blocked[goal]:
            return None
        heap = [(self._heuristic(start, goal), 0.0, start)]
        costs = {start: 0.0}
        came_from = {}
        while heap:
            _, current_cost, current = heapq.heappop(heap)
            if current == goal:
                break
            if current_cost > costs.get(current, float('inf')):
                continue
            row, col = current
            for dr, dc, step in _MOVES_8:
                nxt = (row + dr, col + dc)
                if not self.inside(nxt) or self.blocked[nxt]:
                    continue
                if dr and dc and (self.blocked[row + dr, col] or self.blocked[row, col + dc]):
                    continue
                value = current_cost + step * self.resolution + self.cost[nxt] * step
                if value < costs.get(nxt, float('inf')):
                    costs[nxt] = value
                    came_from[nxt] = current
                    heapq.heappush(heap, (value + self._heuristic(nxt, goal), value, nxt))
        if goal != start and goal not in came_from:
            return None
        path = [goal]
        while path[-1] != start:
            path.append(came_from[path[-1]])
        path.reverse()
        return self.simplify(path) if simplify else path

    def _heuristic(self, first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1]) * self.resolution

    def line_cells(self, first, second):
        row0, col0 = first
        row1, col1 = second
        dr, dc = abs(row1 - row0), abs(col1 - col0)
        sr, sc = (1 if row0 < row1 else -1), (1 if col0 < col1 else -1)
        error = dr - dc
        row, col = row0, col0
        while True:
            yield row, col
            if (row, col) == (row1, col1):
                break
            twice = 2 * error
            if twice > -dc:
                error -= dc
                row += sr
            if twice < dr:
                error += dr
                col += sc

    def line_clear(self, first, second):
        return all(self.inside(cell) and not self.blocked[cell]
                   for cell in self.line_cells(first, second))

    def simplify(self, path):
        if len(path) <= 2:
            return path
        result = [path[0]]
        index = 0
        while index < len(path) - 1:
            target = len(path) - 1
            while target > index + 1 and not self.line_clear(path[index], path[target]):
                target -= 1
            result.append(path[target])
            index = target
        return result

    def path_length(self, path):
        if not path:
            return float('inf')
        return sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) * self.resolution
            for a, b in zip(path, path[1:])
        )

    def visible_cells(self, pose, fov, max_range, angular_step=None):
        x, y, yaw = pose
        origin = self.world_to_cell(x, y)
        if not self.inside(origin):
            return set()
        step = angular_step or max(self.resolution / max(max_range, self.resolution), 0.02)
        count = max(2, int(math.ceil(float(fov) / step)))
        visible = {origin}
        for angle in np.linspace(yaw - fov / 2.0, yaw + fov / 2.0, count + 1):
            endpoint = self.world_to_cell(
                x + max_range * math.cos(angle),
                y + max_range * math.sin(angle),
            )
            for cell in self.line_cells(origin, endpoint):
                if not self.inside(cell) or self.obstacles[cell]:
                    break
                visible.add(cell)
        return visible


_NEIGHBORS_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))
_NEIGHBORS_8 = _NEIGHBORS_4 + ((-1, -1), (-1, 1), (1, -1), (1, 1))
_MOVES_8 = tuple((dr, dc, math.sqrt(2.0) if dr and dc else 1.0)
                 for dr, dc in _NEIGHBORS_8)
