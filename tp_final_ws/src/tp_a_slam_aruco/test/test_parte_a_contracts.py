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
    assert "'geometry_debug_file': geometry_debug_file" in text
    assert "'min_marker_depth': 0.15" in text
    assert "'max_marker_depth': ParameterValue(max_marker_depth, value_type=float)" in text
    assert "'min_marker_area_px': ParameterValue(min_marker_area_px, value_type=float)" in text
    assert "'max_reprojection_error_px': ParameterValue(" in text
    assert "'diagnostics_file': diagnostics_file" in text
    assert "'min_landmark_observations': ParameterValue(" in text
    assert "DeclareLaunchArgument('max_landmark_position_jump'" in text
    assert "'max_landmark_position_jump': ParameterValue(" in text


def test_mapping_launch_exposes_scan_tf_and_fallback_controls():
    launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'parte_a_mapa.launch.py'
    text = launch_path.read_text()

    assert "'scan_topic'," in text
    assert "DeclareLaunchArgument('base_frame'" in text
    assert "DeclareLaunchArgument('lidar_fallback_enabled'" in text
    assert "DeclareLaunchArgument('use_bag_tf'" in text
    assert "'bag_tf_topic'," in text
    assert "'bag_tf_static_topic'," in text
    assert "executable='tf_bridge'" in text
    assert "'scan_topic': scan_topic" in text
    assert "'base_frame': LaunchConfiguration('base_frame')" in text


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


def test_occupancy_grid_queues_scans_until_dense_odom_brackets_them():
    node_path = (
        Path(__file__).resolve().parents[1] / 'tp_a_slam_aruco'
        / 'occupancy_grid_node.py')
    text = node_path.read_text()

    assert 'ScanOdomBuffer' in text
    assert 'self.scan_odom_buffer.add_scan(msg, t)' in text
    assert 'self.scan_odom_buffer.add_odom(t, odom_pose)' in text
    assert 'for ready in self.scan_odom_buffer.pop_ready()' in text
    assert 'self.scan_odom_buffer.finalize()' in text

    launch = (
        Path(__file__).resolve().parents[1] / 'launch'
        / 'parte_a_mapa.launch.py').read_text()
    for argument in (
        'max_odom_buffer_samples',
        'max_pending_scans',
        'max_scan_wait_seconds',
    ):
        assert f"DeclareLaunchArgument('{argument}'" in launch
        assert f"'{argument}': LaunchConfiguration('{argument}')" in launch


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


def test_part_a_nodes_handle_ros_launch_external_shutdown():
    package = Path(__file__).resolve().parents[1] / 'tp_a_slam_aruco'
    for filename in (
        'tf_bridge_node.py',
        'aruco_detector_node.py',
        'graph_slam_node.py',
    ):
        text = (package / filename).read_text()
        assert 'ExternalShutdownException' in text, filename
        assert 'except (KeyboardInterrupt, ExternalShutdownException):' in text, filename


def test_occupancy_grid_flushes_buffer_on_external_shutdown():
    source = (
        Path(__file__).resolve().parents[1] / 'tp_a_slam_aruco'
        / 'occupancy_grid_node.py').read_text()

    assert 'ExternalShutdownException' in source
    assert 'except (KeyboardInterrupt, ExternalShutdownException):' in source
    assert 'node.finalize_scan_buffer()' in source
    assert 'if rclpy.ok():' in source
    assert 'except (KeyboardInterrupt, Exception):' in source
