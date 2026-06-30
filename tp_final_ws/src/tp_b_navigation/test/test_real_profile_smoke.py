import os
import signal
import subprocess
import time
from pathlib import Path

import pytest


def _stop(process, managed_launch=False):
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


def _shell(workspace, command, env, **kwargs):
    setup = workspace / 'install' / 'setup.bash'
    return subprocess.run(
        ['bash', '-lc', f'source "{setup}" && {command}'],
        env=env,
        text=True,
        capture_output=True,
        check=True,
        **kwargs,
    )


def _observe_for(workspace, topic, env, seconds=5):
    setup = workspace / 'install' / 'setup.bash'
    process = subprocess.Popen(
        ['bash', '-lc', f'source "{setup}" && ros2 topic echo {topic}'],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        time.sleep(seconds)
    finally:
        _stop(process)
    return process.stdout.read()


@pytest.mark.skipif(
    os.environ.get('RUN_ROS_SMOKE') != '1',
    reason='Set RUN_ROS_SMOKE=1 to run ROS bag smoke tests.',
)
def test_bag_profile_localizes_without_hardware_velocity(tmp_path):
    workspace = Path(__file__).resolve().parents[3]
    bag = Path(os.environ.get(
        'TP_TB4_TEST_BAG',
        workspace / 'bags' / 'laberinto',
    ))
    if not bag.exists():
        pytest.skip('Set TP_TB4_TEST_BAG to an existing TurtleBot4 rosbag.')
    env = os.environ.copy()
    env.update({
        'ROS_LOG_DIR': str(tmp_path / 'ros_logs'),
        'ROS_LOCALHOST_ONLY': '1',
        'ROS_DOMAIN_ID': str(40 + os.getpid() % 180),
    })
    Path(env['ROS_LOG_DIR']).mkdir()
    setup = workspace / 'install' / 'setup.bash'
    launch_log = (tmp_path / 'launch.log').open('w')
    bag_log = (tmp_path / 'bag.log').open('w')
    launch = bag_process = None
    try:
        launch = subprocess.Popen(
            ['bash', '-lc',
             f'source "{setup}" && ros2 launch tp_b_navigation parte_b.launch.py '
             'profile:=bag_tb4 launch_rviz:=false enable_safety_gates:=false '
             'cmd_vel_topic:=/test/cmd_vel run_id:=smoke'],
            env=env,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        time.sleep(3)
        bag_process = subprocess.Popen(
            ['bash', '-lc',
             f'source "{setup}" && ros2 bag play "{bag}" --clock '
             '--rate 2.0 --disable-keyboard-controls'],
            env=env,
            stdout=bag_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        time.sleep(3)
        initialpose = (
            "ros2 topic pub --once /initialpose "
            "geometry_msgs/msg/PoseWithCovarianceStamped "
            "\"{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}\""
        )
        _shell(workspace, initialpose, env, timeout=10)
        first = _shell(
            workspace,
            'ros2 topic echo /mcl_pose --once --field header.stamp',
            env,
            timeout=10,
        ).stdout
        time.sleep(2)
        second = _shell(
            workspace,
            'ros2 topic echo /mcl_pose --once --field header.stamp',
            env,
            timeout=10,
        ).stdout
        nodes = _shell(workspace, 'ros2 node list', env, timeout=10).stdout
        health = _observe_for(
            workspace, '/obstacle_monitor_healthy', env, seconds=5)
        transforms = _observe_for(
            workspace, '/tb4_0/tf', env, seconds=3)
        velocity_info = _shell(
            workspace,
            'ros2 node info /state_machine',
            env,
            timeout=10,
        ).stdout
    finally:
        _stop(launch, managed_launch=True)
        _stop(bag_process)
        launch_log.close()
        bag_log.close()

    assert first != second
    assert 'landmark_publisher' not in nodes
    assert 'landmark_sensor' not in nodes
    assert 'data: true' in health.lower()
    assert 'frame_id: map' in transforms
    assert 'child_frame_id: odom' in transforms
    assert '/test/cmd_vel' in velocity_info
    assert '/tb4_0/cmd_vel' not in velocity_info
