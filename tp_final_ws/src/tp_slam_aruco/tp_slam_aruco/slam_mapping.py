import bisect
import math


def world_to_grid(wx, wy, origin_x, origin_y, resolution):
    gx = math.floor((wx - origin_x) / resolution + 1e-9)
    gy = math.floor((wy - origin_y) / resolution + 1e-9)
    return gx, gy


def bresenham_cells(x0, y0, x1, y1):
    """Return grid cells crossed by a line, including both endpoints."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    x, y = x0, y0
    if dx > dy:
        err = dx // 2
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    cells.append((x1, y1))
    return cells


def interpolate_pose(trajectory, stamp):
    """Interpolate (x, y, theta) from trajectory samples with wrapped yaw."""
    if not trajectory:
        return None

    stamps = [pose['stamp'] for pose in trajectory]
    if stamp <= stamps[0]:
        p = trajectory[0]
        return p['x'], p['y'], p['theta']
    if stamp >= stamps[-1]:
        p = trajectory[-1]
        return p['x'], p['y'], p['theta']

    hi = bisect.bisect_right(stamps, stamp)
    lo = hi - 1
    t0, t1 = stamps[lo], stamps[hi]
    alpha = (stamp - t0) / (t1 - t0) if t1 != t0 else 0.0

    p0, p1 = trajectory[lo], trajectory[hi]
    x = p0['x'] + alpha * (p1['x'] - p0['x'])
    y = p0['y'] + alpha * (p1['y'] - p0['y'])
    dth = math.atan2(
        math.sin(p1['theta'] - p0['theta']),
        math.cos(p1['theta'] - p0['theta']),
    )
    theta = p0['theta'] + alpha * dth
    return x, y, theta


def log_odds_to_occupancy(log_odds, occupied_threshold=0.6, free_threshold=0.4):
    """Convert log-odds values to ROS OccupancyGrid values: 0, 100, or -1."""
    occupancy = []
    for value in log_odds:
        prob = 1.0 / (1.0 + math.exp(-float(value)))
        if prob >= occupied_threshold:
            occupancy.append(100)
        elif prob <= free_threshold:
            occupancy.append(0)
        else:
            occupancy.append(-1)
    return occupancy
