import math

from tp_slam_aruco.slam_mapping import (
    bresenham_cells,
    interpolate_pose,
    log_odds_to_occupancy,
    world_to_grid,
)


def test_interpolate_pose_wraps_heading_across_pi_boundary():
    trajectory = [
        {'stamp': 10.0, 'x': 0.0, 'y': 0.0, 'theta': math.radians(170.0)},
        {'stamp': 20.0, 'x': 2.0, 'y': 4.0, 'theta': math.radians(-170.0)},
    ]

    pose = interpolate_pose(trajectory, 15.0)

    assert pose[0] == 1.0
    assert pose[1] == 2.0
    assert math.isclose(abs(pose[2]), math.pi)


def test_world_to_grid_uses_metric_origin_and_resolution():
    assert world_to_grid(-4.0, -1.5, origin_x=-4.0, origin_y=-1.5, resolution=0.05) == (0, 0)
    assert world_to_grid(-3.90, -1.35, origin_x=-4.0, origin_y=-1.5, resolution=0.05) == (2, 3)


def test_bresenham_cells_returns_line_endpoints_inclusive():
    assert bresenham_cells(0, 0, 3, 1) == [(0, 0), (1, 0), (2, 1), (3, 1)]


def test_log_odds_to_occupancy_preserves_unknown_band():
    assert log_odds_to_occupancy([-3.0, 0.0, 3.0]) == [0, -1, 100]
