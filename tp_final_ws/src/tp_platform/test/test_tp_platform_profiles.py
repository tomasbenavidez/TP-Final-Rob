import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


def test_normalizes_supported_tb4_namespaces():
    from tp_platform.platform_profiles import normalize_robot_namespace

    assert normalize_robot_namespace('tb4_0') == '/tb4_0'
    assert normalize_robot_namespace('/tb4_0') == '/tb4_0'
    assert normalize_robot_namespace('tb4_1') == '/tb4_1'
    assert normalize_robot_namespace('/tb4_1') == '/tb4_1'


def test_rejects_unknown_tb4_namespace():
    from tp_platform.platform_profiles import normalize_robot_namespace

    with pytest.raises(ValueError, match='tb4_0, tb4_1'):
        normalize_robot_namespace('tb4_2')


def test_derives_tb4_topic_set_from_namespace():
    from tp_platform.platform_profiles import tb4_topics

    topics = tb4_topics('tb4_1')

    assert topics.robot_namespace == '/tb4_1'
    assert topics.odom_topic == '/tb4_1/odom'
    assert topics.scan_topic == '/tb4_1/scan'
    assert topics.tf_topic == '/tb4_1/tf'
    assert topics.tf_static_topic == '/tb4_1/tf_static'
    assert topics.cmd_vel_topic == '/tb4_1/cmd_vel'
    assert topics.rgb_topic == '/tb4_1/oakd/rgb/preview/image_raw'
    assert topics.depth_topic == '/tb4_1/oakd/stereo/image_raw'
    assert topics.camera_info_topic == '/tb4_1/oakd/rgb/preview/camera_info'


def test_validates_critical_topics_do_not_point_to_other_tb4():
    from tp_platform.platform_profiles import validate_topic_namespace

    validate_topic_namespace('/tb4_1/scan', 'tb4_1', 'scan_topic')

    with pytest.raises(ValueError, match='scan_topic'):
        validate_topic_namespace('/tb4_0/scan', 'tb4_1', 'scan_topic')


def test_resolves_tb4_profiles_with_selected_namespace():
    from tp_platform.platform_profiles import resolve_profile

    bag_profile = resolve_profile('bag_tb4', robot_namespace='tb4_1')
    real_profile = resolve_profile('real_tb4', robot_namespace='/tb4_1')

    assert bag_profile.robot_namespace == '/tb4_1'
    assert bag_profile.odom_topic == '/tb4_1/odom'
    assert bag_profile.use_sim_time is True
    assert bag_profile.launch_virtual_landmarks is False
    assert real_profile.robot_namespace == '/tb4_1'
    assert real_profile.cmd_vel_topic == '/tb4_1/cmd_vel'
    assert real_profile.use_sim_time is False


def test_simulation_profile_ignores_robot_namespace_and_preserves_defaults():
    from tp_platform.platform_profiles import resolve_profile

    profile = resolve_profile('simulation_tb3', robot_namespace='tb4_1')

    assert profile.robot_namespace == ''
    assert profile.odom_topic == '/calc_odom'
    assert profile.scan_topic == '/scan'
    assert profile.cmd_vel_topic == '/cmd_vel'
    assert profile.launch_virtual_landmarks is True


def test_writes_resolved_platform_artifact(tmp_path):
    from tp_platform.platform_profiles import resolve_profile, write_resolved_platform

    profile = resolve_profile('real_tb4', robot_namespace='tb4_1')
    output = write_resolved_platform(
        tmp_path,
        profile,
        stage='parte-b',
        run_id='run-42',
        topics={'scan_topic': profile.scan_topic},
        frames={'base_frame': profile.base_frame},
        artifacts={'trajectory_file': '/tmp/run/trajectory.json'},
    )

    assert output == tmp_path / 'config' / 'platform-parte-b.yaml'
    text = output.read_text()
    assert 'run_id: run-42' in text
    assert 'profile: real_tb4' in text
    assert 'robot_namespace: /tb4_1' in text
    assert 'scan_topic: /tb4_1/scan' in text
    assert 'base_frame: base_link' in text
    assert 'trajectory_file: /tmp/run/trajectory.json' in text


def test_stage_artifacts_do_not_overwrite_each_other(tmp_path):
    from tp_platform.platform_profiles import resolve_profile, write_resolved_platform

    profile = resolve_profile('bag_tb4')
    slam = write_resolved_platform(
        tmp_path, profile, stage='parte-a-slam', run_id='run-42')
    slam_bytes = slam.read_bytes()
    mapping = write_resolved_platform(
        tmp_path, profile, stage='parte-a-mapa', run_id='run-42')
    navigation = write_resolved_platform(
        tmp_path, profile, stage='parte-b', run_id='run-42')

    assert slam.read_bytes() == slam_bytes
    assert len({slam, mapping, navigation}) == 3
    for output in (slam, mapping, navigation):
        text = output.read_text()
        assert 'run_id: run-42' in text
        assert 'profile: bag_tb4' in text
        assert 'robot_namespace: /tb4_0' in text
        assert 'topics:' in text
        assert 'frames:' in text
        assert 'artifacts:' in text


@pytest.mark.parametrize('stage', ['', '..', '../parte-b', 'parte/b'])
def test_rejects_empty_or_path_containing_stage(stage, tmp_path):
    from tp_platform.platform_profiles import resolve_profile, write_resolved_platform

    with pytest.raises(ValueError, match='stage'):
        write_resolved_platform(
            tmp_path,
            resolve_profile('simulation_tb3'),
            stage=stage,
            run_id='run-42',
        )
