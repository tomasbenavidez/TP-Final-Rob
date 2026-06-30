import pytest
import rclpy
from builtin_interfaces.msg import Time
from types import SimpleNamespace


BASE = {
    'rgb_fresh': True,
    'camera_info_valid': True,
    'rgb_shape': (480, 640),
    'camera_info_shape': (480, 640),
    'depth_shape': (480, 640),
    'rgb_stamp': 10.0,
    'depth_stamp': 9.95,
    'max_depth_age': 0.20,
    'tf_available': True,
    'require_aligned_depth': True,
    'range_source': 'depth',
    'scan_fresh': False,
}


def test_real_rgbd_contract_is_ready_when_every_input_matches():
    from tp_c_mission.cone_detector_node import evaluate_vision_readiness

    assert evaluate_vision_readiness(**BASE) is None


@pytest.mark.parametrize(
    ('override', 'reason'),
    [
        ({'rgb_fresh': False}, 'rgb_stale'),
        ({'camera_info_valid': False}, 'camera_info_invalid'),
        ({'camera_info_shape': (240, 320)}, 'camera_info_misaligned'),
        ({'depth_shape': None}, 'depth_missing'),
        ({'depth_shape': (240, 320)}, 'depth_misaligned'),
        ({'depth_stamp': None}, 'depth_missing'),
        ({'depth_stamp': 9.0}, 'depth_stale'),
        ({'tf_available': False}, 'tf_unavailable'),
    ],
)
def test_real_rgbd_contract_rejects_each_missing_condition(override, reason):
    from tp_c_mission.cone_detector_node import evaluate_vision_readiness

    values = {**BASE, **override}
    assert evaluate_vision_readiness(**values) == reason


def test_monocular_policy_does_not_require_depth():
    from tp_c_mission.cone_detector_node import evaluate_vision_readiness

    values = {
        **BASE,
        'range_source': 'monocular',
        'require_aligned_depth': False,
        'depth_shape': None,
        'depth_stamp': None,
    }
    assert evaluate_vision_readiness(**values) is None


def test_lidar_policy_requires_scan_but_not_depth():
    from tp_c_mission.cone_detector_node import evaluate_vision_readiness

    values = {
        **BASE,
        'range_source': 'lidar',
        'require_aligned_depth': False,
        'depth_shape': None,
        'depth_stamp': None,
        'scan_fresh': False,
    }
    assert evaluate_vision_readiness(**values) == 'scan_stale'
    assert evaluate_vision_readiness(**{**values, 'scan_fresh': True}) is None


def test_real_launch_uses_lidar_range_and_disables_latest_tf_fallback():
    from pathlib import Path

    launch = (
        Path(__file__).resolve().parents[1] / 'launch'
        / 'parte_c_real.launch.py').read_text()

    assert "range_source = _override(context, 'range_source', 'lidar')" in launch
    assert "'range_source': range_source" in launch
    assert "'require_aligned_depth': False" in launch
    assert "'allow_latest_tf_fallback': False" in launch
    assert "'scan_topic': scan_topic" in launch


def test_sim_and_bag_launches_keep_monocular_range_source():
    from pathlib import Path

    root = Path(__file__).resolve().parents[1] / 'launch'

    assert "'range_source': 'monocular'" in (root / 'parte_c_sim.launch.py').read_text()
    assert "'range_source': 'monocular'" in (root / 'parte_c_bag.launch.py').read_text()


def test_false_readiness_invalidates_mission_vision_immediately():
    from pathlib import Path

    manager = (
        Path(__file__).resolve().parents[1] / 'tp_c_mission'
        / 'mission_manager_node.py').read_text()

    assert 'else:\n            self.last_vision_at = None' in manager


def test_cone_transform_lookup_uses_rgb_timestamp():
    from tp_c_mission.cone_detector_node import ConeDetector

    requested = []
    detector = ConeDetector.__new__(ConeDetector)
    detector.global_frame = 'map'
    detector.allow_latest_tf_fallback = False
    detector.tf_buffer = SimpleNamespace(
        lookup_transform=lambda target, source, stamp: requested.append(stamp),
    )
    stamp = Time(sec=21, nanosec=400_000_000)
    measurement_time = rclpy.time.Time.from_msg(stamp)

    detector._lookup_transform('camera', measurement_time)

    assert requested == [measurement_time]
