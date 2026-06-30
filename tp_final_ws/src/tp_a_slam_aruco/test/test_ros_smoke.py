import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest


def _kill_process_group(process, managed_launch=False):
    if process is None or process.poll() is not None:
        return
    if managed_launch:
        os.kill(process.pid, signal.SIGINT)
    else:
        os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def _bag_path(workspace):
    configured = os.environ.get('TP_TB4_TEST_BAG')
    candidate = Path(configured) if configured else workspace / 'bags' / 'laberinto'
    if not candidate.exists():
        pytest.skip('Set TP_TB4_TEST_BAG to an existing TurtleBot4 rosbag.')
    return candidate


def _run_launch_with_bag(
    *,
    workspace,
    env,
    launch_command,
    bag_path,
    launch_log,
    bag_log,
):
    launch_process = bag_process = None
    with launch_log.open('w') as launch_stream, bag_log.open('w') as bag_stream:
        try:
            launch_process = subprocess.Popen(
                ['bash', '-lc', launch_command],
                stdout=launch_stream,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
            time.sleep(3.0)
            bag_command = (
                f'source "{workspace / "install" / "setup.bash"}" && '
                f'ros2 bag play "{bag_path}" --clock --rate 4.0 '
                '--disable-keyboard-controls'
            )
            bag_process = subprocess.Popen(
                ['bash', '-lc', bag_command],
                stdout=bag_stream,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
            time.sleep(15.0)
        finally:
            _kill_process_group(launch_process, managed_launch=True)
            _kill_process_group(bag_process)


@pytest.mark.skipif(
    os.environ.get('RUN_ROS_SMOKE') != '1',
    reason='Set RUN_ROS_SMOKE=1 to run ROS bag smoke tests.',
)
def test_parte_a_two_pass_runtime_contract(tmp_path):
    workspace = Path(__file__).resolve().parents[3]
    package_dir = workspace / 'src' / 'tp_a_slam_aruco'
    bag_dir = _bag_path(workspace)
    trajectory_file = tmp_path / 'trajectory.json'
    map_prefix = tmp_path / 'map'
    env = os.environ.copy()
    env.update({
        'ROS_LOG_DIR': str(tmp_path / 'ros_logs'),
        'ROS_LOCALHOST_ONLY': '1',
        'ROS_DOMAIN_ID': str(40 + os.getpid() % 180),
    })
    Path(env['ROS_LOG_DIR']).mkdir()
    setup = workspace / 'install' / 'setup.bash'

    slam_command = (
        f'source "{setup}" && '
        'ros2 launch tp_a_slam_aruco parte_a_slam.launch.py '
        f'calibration_file:="{package_dir / "config" / "camera_tb4_0.yaml"}" '
        f'trajectory_file:="{trajectory_file}" use_bag_tf:=true '
        'launch_rviz:=false run_id:=smoke'
    )
    _run_launch_with_bag(
        workspace=workspace,
        env=env,
        launch_command=slam_command,
        bag_path=bag_dir,
        launch_log=tmp_path / 'slam.log',
        bag_log=tmp_path / 'bag-slam.log',
    )

    data = json.loads(trajectory_file.read_text())
    assert data['trajectory']

    map_command = (
        f'source "{setup}" && '
        'ros2 launch tp_a_slam_aruco parte_a_mapa.launch.py '
        f'trajectory_file:="{trajectory_file}" map_output:="{map_prefix}" '
        'use_bag_tf:=true run_id:=smoke'
    )
    _run_launch_with_bag(
        workspace=workspace,
        env=env,
        launch_command=map_command,
        bag_path=bag_dir,
        launch_log=tmp_path / 'map.log',
        bag_log=tmp_path / 'bag-map.log',
    )

    assert (tmp_path / 'map.pgm').stat().st_size > 0
    assert (tmp_path / 'map.yaml').stat().st_size > 0
    output = (tmp_path / 'slam.log').read_text() + (tmp_path / 'map.log').read_text()
    assert 'tf_bridge_node republishing' in output
    assert 'aruco_detector_node escuchando imágenes' in output
    assert 'Calibración recibida desde camera_info' in output
    assert ('fuente_lidar=tf' in output or 'fuente_lidar=fallback' in output)
    assert 'Traceback' not in output
