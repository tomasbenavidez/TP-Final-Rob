import os
import signal
import subprocess
from pathlib import Path

import pytest


@pytest.mark.skipif(
    os.environ.get('RUN_ROS_SMOKE') != '1',
    reason='Set RUN_ROS_SMOKE=1 to run ROS runtime smoke tests.',
)
def test_calculated_odometry_publishes_while_stationary():
    rclpy = pytest.importorskip('rclpy')
    odometry_type = pytest.importorskip('nav_msgs.msg').Odometry

    workspace = Path(__file__).resolve().parents[3]
    executable = (
        workspace / 'install' / 'turtlebot3_custom_simulation' / 'lib'
        / 'turtlebot3_custom_simulation' / 'turtlebot3_custom_simulation')
    assert executable.is_file(), (
        'Build turtlebot3_custom_simulation before running the ROS smoke test.')

    process = subprocess.Popen(
        [
            str(executable),
            '--ros-args',
            '-p', 'use_sim_time:=false',
            '-p', 'joint_states_frame:=base_footprint',
            '-p', 'odom_frame:=calc_odom',
            '-p', 'base_frame:=base_footprint',
            '-p', 'wheels.separation:=0.160',
            '-p', 'wheels.radius:=0.033',
            '-p', 'publish_tf:=false',
        ],
        env={**os.environ, 'ROS_LOG_DIR': '/tmp/tp_final_ros_smoke_logs'},
        start_new_session=True,
    )

    message = None
    rclpy.init()
    node = rclpy.create_node('calc_odom_smoke_test')

    def odom_callback(msg):
        nonlocal message
        message = msg

    node.create_subscription(odometry_type, '/calc_odom', odom_callback, 10)
    try:
        timeout = node.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
        while message is None and node.get_clock().now() < timeout:
            assert process.poll() is None, (
                f'calculated_odometry exited with code {process.returncode}')
            rclpy.spin_once(node, timeout_sec=0.1)
        assert message is not None, 'No /calc_odom message received within 5 seconds.'
        assert message.header.frame_id == 'calc_odom'
        assert message.child_frame_id == 'base_footprint'
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)
