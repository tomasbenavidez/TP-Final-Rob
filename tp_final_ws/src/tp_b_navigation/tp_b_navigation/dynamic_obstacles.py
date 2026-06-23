"""Helpers puros para mantener una capa de obstáculos dinámicos."""

import numpy as np


def apply_dynamic_obstacles(static_data, dynamic_data):
    static = np.asarray(static_data, dtype=np.int16)
    if dynamic_data is None:
        return static.copy()
    dynamic = np.asarray(dynamic_data, dtype=np.int16)
    if dynamic.shape != static.shape:
        return static.copy()
    merged = static.copy()
    merged[dynamic == 100] = 100
    return merged


def same_static_occupancy(first, second):
    if first is None or second is None:
        return False
    first = np.asarray(first, dtype=bool)
    second = np.asarray(second, dtype=bool)
    return first.shape == second.shape and np.array_equal(first, second)


def mark_dynamic_obstacles(dynamic, static_occupied, rows, cols, inflation_cells=1):
    dynamic = np.asarray(dynamic, dtype=bool)
    static_occupied = np.asarray(static_occupied, dtype=bool)
    height, width = dynamic.shape
    radius = max(0, int(inflation_cells))
    changed = False
    new_points = 0
    seeds = []
    for row, col in zip(np.asarray(rows, dtype=int), np.asarray(cols, dtype=int)):
        if not (0 <= row < height and 0 <= col < width):
            continue
        r0, r1 = max(0, row - 1), min(height, row + 2)
        c0, c1 = max(0, col - 1), min(width, col + 2)
        if static_occupied[r0:r1, c0:c1].any():
            continue
        new_points += 1
        seeds.append((row, col))

    for cluster in _nearby_clusters(seeds, max(1, radius)):
        row, col = _cluster_center(cluster)
        for rr in range(max(0, row - radius), min(height, row + radius + 1)):
            for cc in range(max(0, col - radius), min(width, col + radius + 1)):
                if (rr - row) ** 2 + (cc - col) ** 2 > radius ** 2:
                    continue
                if not dynamic[rr, cc]:
                    dynamic[rr, cc] = True
                    changed = True
    return changed, new_points


def _nearby_clusters(cells, max_gap):
    pending = set(cells)
    clusters = []
    while pending:
        root = pending.pop()
        cluster = [root]
        stack = [root]
        while stack:
            row, col = stack.pop()
            neighbors = [
                cell for cell in pending
                if abs(cell[0] - row) <= max_gap and abs(cell[1] - col) <= max_gap
            ]
            for cell in neighbors:
                pending.remove(cell)
                cluster.append(cell)
                stack.append(cell)
        clusters.append(cluster)
    return clusters


def _cluster_center(cluster):
    rows = np.array([cell[0] for cell in cluster], dtype=float)
    cols = np.array([cell[1] for cell in cluster], dtype=float)
    center = (rows.mean(), cols.mean())
    return min(
        cluster,
        key=lambda cell: (cell[0] - center[0]) ** 2 + (cell[1] - center[1]) ** 2)
