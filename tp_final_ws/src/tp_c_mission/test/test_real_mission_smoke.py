import os
import signal
import subprocess
import time
from pathlib import Path

import pytest


def _stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def _capture_readiness(workspace, env, bag, depth_topic, tmp_path):
    setup = workspace / 'install' / 'setup.bash'
    detector_log = (tmp_path / f'detector-{depth_topic.rsplit("/", 1)[-1]}.log').open('w')
    processes = []
    try:
        bridge = subprocess.Popen(
            ['bash', '-lc',
             f'source "{setup}" && ros2 run tp_a_slam_aruco tf_bridge '
             '--ros-args -p use_sim_time:=true '
             '-p bag_tf_topic:=/tb4_0/tf '
             '-p bag_tf_static_topic:=/tb4_0/tf_static'],
            env=env,
            stdout=detector_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        processes.append(bridge)
        detector = subprocess.Popen(
            ['bash', '-lc',
             f'source "{setup}" && ros2 run tp_c_mission red_cone_detector '
             '--ros-args -p use_sim_time:=true -p global_frame:=base_link '
             '-p rgb_topic:=/tb4_0/oakd/rgb/preview/image_raw '
             '-p camera_info_topic:=/tb4_0/oakd/rgb/preview/camera_info '
             f'-p depth_topic:={depth_topic} -p require_aligned_depth:=true'],
            env=env,
            stdout=detector_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        processes.append(detector)
        time.sleep(2)
        player = subprocess.Popen(
            ['bash', '-lc',
             f'source "{setup}" && ros2 bag play "{bag}" --clock '
             '--rate 4.0 --disable-keyboard-controls'],
            env=env,
            stdout=detector_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        processes.append(player)
        time.sleep(4)
        result = subprocess.run(
            ['bash', '-lc',
             f'source "{setup}" && '
             'ros2 topic echo /red_cone/vision_ready --once'],
            env=env,
            text=True,
            capture_output=True,
            timeout=10,
            check=True,
        )
        return result.stdout
    finally:
        for process in reversed(processes):
            _stop(process)
        detector_log.close()


@pytest.mark.skipif(
    os.environ.get('RUN_ROS_SMOKE') != '1',
    reason='Set RUN_ROS_SMOKE=1 to run ROS bag smoke tests.',
)
def test_cone_bag_without_depth_fails_rgbd_readiness_closed(tmp_path):
    workspace = Path(__file__).resolve().parents[3]
    bag = Path(os.environ.get(
        'TP_TB4_CONE_TEST_BAG',
        workspace / 'bags' / 'laberinto_conos',
    ))
    if not bag.exists():
        pytest.skip('Set TP_TB4_CONE_TEST_BAG to an existing RGB-D cone bag.')
    env = os.environ.copy()
    env.update({
        'ROS_LOG_DIR': str(tmp_path / 'ros_logs'),
        'ROS_LOCALHOST_ONLY': '1',
        'ROS_DOMAIN_ID': str(40 + os.getpid() % 180),
    })
    Path(env['ROS_LOG_DIR']).mkdir()

    ready = _capture_readiness(
        workspace,
        env,
        bag,
        '/tb4_0/oakd/stereo/image_raw',
        tmp_path,
    )
    missing = _capture_readiness(
        workspace,
        env,
        bag,
        '/test/missing_depth',
        tmp_path,
    )

    setup = workspace / 'install' / 'setup.bash'
    bag_info = subprocess.run(
        ['bash', '-lc', f'source "{setup}" && ros2 bag info "{bag}"'],
        env=env,
        text=True,
        capture_output=True,
        timeout=10,
        check=True,
    ).stdout

    assert '/tb4_0/oakd/stereo/image_raw' not in bag_info
    assert 'data: false' in ready.lower()
    assert 'data: false' in missing.lower()
