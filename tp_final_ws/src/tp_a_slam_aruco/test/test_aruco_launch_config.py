from pathlib import Path


def test_parte_a_launch_uses_stable_marker_filter_defaults():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'parte_a_slam.launch.py'
    )
    text = launch_path.read_text()

    assert "DeclareLaunchArgument('min_marker_area_px', default_value='250.0')" in text
    assert "DeclareLaunchArgument('max_marker_depth', default_value='3.0')" in text
    assert "DeclareLaunchArgument('max_reprojection_error_px', default_value='4.0')" in text
    assert "ParameterValue(" in text
    assert "value_type=float" in text


def test_parte_a_launch_defaults_to_tb4_rgb_optical_frame():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'parte_a_slam.launch.py'
    )
    text = launch_path.read_text()

    assert "'camera_frame'," in text
    assert "default_value='oakd_rgb_camera_optical_frame'" in text


def test_parte_a_launch_uses_stable_landmark_gating_defaults():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'parte_a_slam.launch.py'
    )
    text = launch_path.read_text()

    assert "DeclareLaunchArgument('min_landmark_observations', default_value='3')" in text
    assert "DeclareLaunchArgument('max_landmark_position_jump', default_value='0.75')" in text


def test_part_a_outputs_default_to_tmp_and_rviz_uses_selected_topics():
    package = Path(__file__).resolve().parents[1]
    slam = (package / 'launch' / 'parte_a_slam.launch.py').read_text()
    mapping = (package / 'launch' / 'parte_a_mapa.launch.py').read_text()
    rviz = (package / 'config' / 'rviz_config.rviz').read_text()

    assert "default_value='/tmp/tp_final_rob/trajectory.json'" in slam
    assert "default_value='/tmp/tp_final_rob/mapa'" in mapping
    assert "stage='parte-a-slam'" in slam
    assert "stage='parte-a-mapa'" in mapping
    assert "DeclareLaunchArgument('run_id'" in slam
    assert "DeclareLaunchArgument('run_id'" in mapping
    assert "('/scan', scan_topic)" in slam
    assert "('/odom', odom_topic)" in slam
    assert 'Value: /tb4_0/scan' not in rviz
    assert 'Value: /tb4_0/odom' not in rviz
