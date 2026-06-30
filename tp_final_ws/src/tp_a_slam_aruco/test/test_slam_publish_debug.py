from builtin_interfaces.msg import Time

from tp_a_slam_aruco.slam_publish import build_base_debug_markers


def test_build_base_debug_markers_uses_cyan_points_in_base_link():
    msg = build_base_debug_markers(
        base_frame='base_link',
        stamp=Time(sec=3),
        observations=[
            {'id': 10, 'x_base': 0.7, 'y_base': -0.2},
        ],
    )

    assert len(msg.markers) == 1
    marker = msg.markers[0]
    assert marker.header.frame_id == 'base_link'
    assert marker.header.stamp.sec == 3
    assert marker.ns == 'aruco_base_debug'
    assert marker.id == 10
    assert marker.pose.position.x == 0.7
    assert marker.pose.position.y == -0.2
    assert marker.color.r == 0.0
    assert marker.color.g == 1.0
    assert marker.color.b == 1.0
