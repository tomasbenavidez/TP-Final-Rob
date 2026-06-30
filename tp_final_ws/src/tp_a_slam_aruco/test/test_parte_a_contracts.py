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
