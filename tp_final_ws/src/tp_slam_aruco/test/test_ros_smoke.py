import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest


def _kill_process_group(process):
    if process is None or process.poll() is not None:
        return

    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


@pytest.mark.skipif(
    os.environ.get('RUN_ROS_SMOKE') != '1',
    reason='Set RUN_ROS_SMOKE=1 to run ROS bag smoke tests.',
)
def test_parte_a_launch_smoke(tmp_path):
    workspace = Path(__file__).resolve().parents[3]
    package_dir = workspace / 'src' / 'tp_slam_aruco'
    bag_dir = workspace / 'bags' / 'aruco_estimation'
    trajectory_file = tmp_path / 'trajectory.json'
    bag_log = tmp_path / 'bag.log'
    launch_log = tmp_path / 'launch.log'
    ros_log_dir = tmp_path / 'ros_logs'
    ros_log_dir.mkdir()

    env = os.environ.copy()
    env['ROS_LOG_DIR'] = str(ros_log_dir)
    env['ROS_LOCALHOST_ONLY'] = '1'

    bag_command = (
        f'source "{workspace / "install" / "setup.bash"}" >/dev/null 2>&1 && '
        f'ros2 bag play "{bag_dir}" --clock'
    )
    launch_command = (
        f'source "{workspace / "install" / "setup.bash"}" >/dev/null 2>&1 && '
        'ros2 launch tp_slam_aruco parte_a_slam.launch.py '
        f'calibration_file:="{package_dir / "config" / "camera_tb4_0.yaml"}" '
        f'trajectory_file:="{trajectory_file}" '
        'use_bag_tf:=true '
        'launch_rviz:=false'
    )

    bag_process = None
    launch_process = None
    with bag_log.open('w') as bag_stream, launch_log.open('w') as launch_stream:
        try:
            bag_process = subprocess.Popen(
                ['bash', '-lc', bag_command],
                stdout=bag_stream,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
            time.sleep(1.0)
            launch_process = subprocess.Popen(
                ['bash', '-lc', launch_command],
                stdout=launch_stream,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )

            bag_process.wait(timeout=60)
            time.sleep(2.0)
        finally:
            _kill_process_group(launch_process)
            _kill_process_group(bag_process)

    assert trajectory_file.exists()

    data = json.loads(trajectory_file.read_text())
    assert data['trajectory']

    launch_output = launch_log.read_text()
    assert 'tf_bridge_node republishing' in launch_output
    assert 'Traceback' not in launch_output
