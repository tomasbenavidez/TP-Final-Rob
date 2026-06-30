from pathlib import Path


class _DummyLogger:
    def info(self, _message):
        pass


def test_slam_launch_uses_raw_aruco_detections_for_graph_input():
    launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'parte_a_slam.launch.py'
    text = launch_path.read_text()

    assert "'detections_topic': '/aruco_detections'" in text
    assert "'landmarks_topic': '/aruco_detections'" in text
    assert "'optimized_landmarks_topic': '/landmarks'" in text
    assert "'base_debug_topic': '/aruco_base_debug'" in text
    assert "'geometry_debug_file': '/tmp/aruco_geometry_debug.csv'" in text
    assert "'min_marker_depth': 0.15" in text
    assert "'max_marker_depth': ParameterValue(max_marker_depth, value_type=float)" in text
    assert "'min_marker_area_px': ParameterValue(min_marker_area_px, value_type=float)" in text
    assert "'max_reprojection_error_px': ParameterValue(" in text
    assert "'diagnostics_file': '/tmp/aruco_detections.csv'" in text
    assert "'min_landmark_observations': ParameterValue(" in text
    assert "DeclareLaunchArgument(\n        'max_landmark_position_jump'" in text
    assert "'max_landmark_position_jump': ParameterValue(" in text


def test_mapping_launch_exposes_scan_tf_and_fallback_controls():
    launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'parte_a_mapa.launch.py'
    text = launch_path.read_text()

    assert "DeclareLaunchArgument(\n        'scan_topic'" in text
    assert "DeclareLaunchArgument(\n        'base_frame'" in text
    assert "DeclareLaunchArgument(\n        'lidar_fallback_enabled'" in text
    assert "DeclareLaunchArgument(\n        'use_bag_tf'" in text
    assert "DeclareLaunchArgument(\n        'bag_tf_topic'" in text
    assert "DeclareLaunchArgument(\n        'bag_tf_static_topic'" in text
    assert "executable='tf_bridge'" in text
    assert "'scan_topic':      scan_topic" in text
    assert "'base_frame':      base_frame" in text


def test_occupancy_grid_reports_lidar_tf_and_fallback_sources():
    node_path = (
        Path(__file__).resolve().parents[1] / 'tp_a_slam_aruco'
        / 'occupancy_grid_node.py')
    text = node_path.read_text()

    assert 'lookup_transform(\n            self.base_frame,' in text
    assert 'scan.header.frame_id' in text
    assert 'scans_integrated_with_tf' in text
    assert 'scans_integrated_with_fallback' in text
    assert 'scans_skipped_no_lidar_tf' in text


def test_readme_documents_two_pass_outputs_and_raw_detection_topic():
    readme_path = Path(__file__).resolve().parents[4] / 'README.md'
    text = readme_path.read_text()

    assert '/aruco_detections' in text
    assert '/landmarks' in text
    assert '/aruco_base_debug' in text
    assert '/tmp/aruco_detections.csv' in text
    assert '/tmp/aruco_geometry_debug.csv' in text
    assert '/tmp/trayectoria.json' in text
    assert '/tmp/mapa.yaml' in text


def test_aruco_diagnostics_writer_creates_parent_directory(tmp_path):
    from tp_a_slam_aruco.aruco_detector_node import ArucoDetectorNode

    diagnostics_path = tmp_path / 'logs_runs' / 'aruco_detections.csv'
    node = ArucoDetectorNode.__new__(ArucoDetectorNode)
    node.diagnostics_file = str(diagnostics_path)
    node._diag_writer = None
    node._diag_handle = None
    node.get_logger = lambda: _DummyLogger()

    writer = node._ensure_diagnostics_writer()
    node.close_diagnostics()

    assert writer is not None
    assert diagnostics_path.exists()


def test_geometry_debug_writer_creates_parent_directory(tmp_path):
    from tp_a_slam_aruco.graph_slam_node import GraphSlamNode

    debug_path = tmp_path / 'logs_runs' / 'aruco_geometry_debug.csv'
    node = GraphSlamNode.__new__(GraphSlamNode)
    node.geometry_debug_file = str(debug_path)
    node.geometry_observations = []
    node._values_for_geometry_debug = lambda: object()

    node._flush_geometry_debug()

    assert debug_path.exists()
