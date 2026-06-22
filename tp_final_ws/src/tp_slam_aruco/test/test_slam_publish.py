from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from tp_slam_aruco.slam_publish import build_accepted_landmark_markers
from tp_slam_aruco.slam_publish import build_raw_landmark_markers


def test_build_raw_landmark_markers_clears_previous_visualization():
    stamp = Time(sec=12, nanosec=340)
    detections = [{
        'id': 7,
        'tvec': (0.12, 0.01, 1.40),
        'rvec': (0.0, 0.0, 0.0),
    }]

    marker_array = build_raw_landmark_markers(
        frame_id='oakd_rgb_camera_optical_frame',
        stamp=stamp,
        detections=detections,
    )

    assert marker_array.markers[0].action == Marker.DELETEALL
    assert marker_array.markers[1].ns == 'aruco_raw'
    assert marker_array.markers[1].id == 7


def test_build_raw_landmark_markers_uses_all_raw_detections():
    stamp = Time(sec=20, nanosec=1)
    detections = [
        {
            'id': 4,
            'tvec': (0.10, 0.02, 1.00),
            'rvec': (0.0, 0.0, 0.0),
        },
        {
            'id': 9,
            'tvec': (0.30, -0.01, 2.10),
            'rvec': (0.0, 0.0, 0.0),
        },
    ]

    marker_array = build_raw_landmark_markers(
        frame_id='camera_frame',
        stamp=stamp,
        detections=detections,
    )

    assert len(marker_array.markers) == 3
    assert [marker.id for marker in marker_array.markers[1:]] == [4, 9]
    assert all(marker.ns == 'aruco_raw' for marker in marker_array.markers[1:])


def test_build_accepted_landmark_markers_uses_distinct_namespace():
    stamp = Time(sec=22, nanosec=5)
    detections = [{
        'id': 3,
        'tvec': (0.20, 0.00, 1.20),
        'rvec': (0.0, 0.0, 0.0),
    }]

    marker_array = build_accepted_landmark_markers(
        frame_id='camera_frame',
        stamp=stamp,
        detections=detections,
    )

    assert marker_array.markers[0].action == Marker.DELETEALL
    assert marker_array.markers[1].ns == 'aruco_accepted'
    assert marker_array.markers[1].id == 3
