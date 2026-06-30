from pathlib import Path


def test_parte_a_launch_uses_double_literal_for_min_marker_area():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'parte_a_slam.launch.py'
    )
    text = launch_path.read_text()

    assert "default_value='120.0'" in text
    assert "ParameterValue(" in text
    assert "value_type=float" in text


def test_parte_a_launch_defaults_camera_frame_to_message_frame():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'parte_a_slam.launch.py'
    )
    text = launch_path.read_text()

    assert "default_value=''" in text
    assert 'Por defecto se usa image.header.frame_id.' in text


def test_parte_a_launch_uses_more_permissive_landmark_recall_defaults():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'parte_a_slam.launch.py'
    )
    text = launch_path.read_text()

    assert "default_value='0.12'" in text
    assert "default_value='4.0'" in text
    assert "default_value='0.08'" in text
    assert "default_value='0.20'" in text
    assert "default_value='0.25'" in text
