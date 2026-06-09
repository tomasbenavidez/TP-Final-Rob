import math


def _point_segment_distance(point, start, end):
    px, py = point
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def summarize_landmark_clearance(trajectory, landmarks, min_landmark_clearance_m):
    per_landmark = {}
    crossed_landmark_count = 0
    segments = list(zip(trajectory, trajectory[1:]))
    for landmark_id, point in landmarks.items():
        coords = (float(point['x']), float(point['y']))
        if segments:
            min_distance = min(
                _point_segment_distance(
                    coords,
                    (float(start['x']), float(start['y'])),
                    (float(end['x']), float(end['y'])),
                )
                for start, end in segments
            )
        else:
            min_distance = float('inf')
        crossing = min_distance < float(min_landmark_clearance_m)
        crossed_landmark_count += int(crossing)
        per_landmark[str(landmark_id)] = {
            'min_distance_to_trajectory': min_distance,
            'trajectory_crossing_flag': crossing,
        }
    return {
        'crossed_landmark_count': crossed_landmark_count,
        'per_landmark': per_landmark,
    }

