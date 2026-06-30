import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


def test_fresh_low_covariance_pose_allows_safety_gate():
    from tp_b_navigation.safety_gates import SafetyGateConfig, localization_gate

    reason = localization_gate(
        now_s=10.0,
        last_pose_stamp_s=9.5,
        covariance=[0.01, 0.02, 0.03],
        config=SafetyGateConfig(
            enabled=True,
            max_mcl_pose_age=1.0,
            max_position_covariance=0.25,
            max_yaw_covariance=0.5,
        ),
    )

    assert reason is None


def test_stale_pose_blocks_safety_gate():
    from tp_b_navigation.safety_gates import SafetyGateConfig, localization_gate

    reason = localization_gate(
        now_s=10.0,
        last_pose_stamp_s=8.9,
        covariance=[0.01, 0.02, 0.03],
        config=SafetyGateConfig(enabled=True, max_mcl_pose_age=1.0),
    )

    assert reason == 'mcl_pose_stale'


def test_high_position_covariance_blocks_safety_gate():
    from tp_b_navigation.safety_gates import SafetyGateConfig, localization_gate

    reason = localization_gate(
        now_s=10.0,
        last_pose_stamp_s=9.9,
        covariance=[0.30, 0.02, 0.03],
        config=SafetyGateConfig(
            enabled=True,
            max_position_covariance=0.25,
            max_yaw_covariance=0.5,
        ),
    )

    assert reason == 'mcl_pose_covariance_high'


def test_high_yaw_covariance_blocks_safety_gate():
    from tp_b_navigation.safety_gates import SafetyGateConfig, localization_gate

    reason = localization_gate(
        now_s=10.0,
        last_pose_stamp_s=9.9,
        covariance=[0.01, 0.02, 0.6],
        config=SafetyGateConfig(
            enabled=True,
            max_position_covariance=0.25,
            max_yaw_covariance=0.5,
        ),
    )

    assert reason == 'mcl_pose_covariance_high'


def test_missing_pose_blocks_safety_gate():
    from tp_b_navigation.safety_gates import SafetyGateConfig, localization_gate

    reason = localization_gate(
        now_s=10.0,
        last_pose_stamp_s=None,
        covariance=[0.01, 0.02, 0.03],
        config=SafetyGateConfig(enabled=True),
    )

    assert reason == 'mcl_pose_missing'


def test_disabled_localization_gate_allows_unknown_pose():
    from tp_b_navigation.safety_gates import SafetyGateConfig, localization_gate

    reason = localization_gate(
        now_s=10.0,
        last_pose_stamp_s=None,
        covariance=[math.inf, math.inf, math.inf],
        config=SafetyGateConfig(enabled=False),
    )

    assert reason is None


def test_stale_scan_blocks_obstacle_insertion():
    from tp_b_navigation.safety_gates import SafetyGateConfig, scan_gate

    reason = scan_gate(
        now_s=10.0,
        scan_stamp_s=8.0,
        config=SafetyGateConfig(enabled=True, max_scan_age=1.0),
    )

    assert reason == 'scan_stale'


def test_missing_scan_stamp_blocks_obstacle_insertion():
    from tp_b_navigation.safety_gates import SafetyGateConfig, scan_gate

    reason = scan_gate(
        now_s=10.0,
        scan_stamp_s=None,
        config=SafetyGateConfig(enabled=True),
    )

    assert reason == 'scan_stamp_missing'


def test_stale_monitor_heartbeat_blocks_navigation():
    from tp_b_navigation.safety_gates import SafetyGateConfig, monitor_gate

    reason = monitor_gate(
        now_s=10.0,
        last_monitor_stamp_s=8.0,
        monitor_healthy=True,
        config=SafetyGateConfig(enabled=True, max_monitor_age=1.0),
    )

    assert reason == 'obstacle_monitor_stale'


def test_missing_monitor_heartbeat_is_not_safe():
    from tp_b_navigation.safety_gates import SafetyGateConfig, monitor_gate

    reason = monitor_gate(
        now_s=10.0,
        last_monitor_stamp_s=None,
        monitor_healthy=None,
        config=SafetyGateConfig(enabled=True),
    )

    assert reason == 'obstacle_monitor_missing'


def test_unhealthy_monitor_blocks_navigation():
    from tp_b_navigation.safety_gates import SafetyGateConfig, monitor_gate

    reason = monitor_gate(
        now_s=10.0,
        last_monitor_stamp_s=9.9,
        monitor_healthy=False,
        config=SafetyGateConfig(enabled=True),
    )

    assert reason == 'obstacle_monitor_unhealthy'


def test_disabled_monitor_gate_preserves_simulation_behavior():
    from tp_b_navigation.safety_gates import SafetyGateConfig, monitor_gate

    reason = monitor_gate(
        now_s=10.0,
        last_monitor_stamp_s=None,
        monitor_healthy=None,
        config=SafetyGateConfig(enabled=False),
    )

    assert reason is None
