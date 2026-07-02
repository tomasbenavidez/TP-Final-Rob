#!/usr/bin/env python3
"""Localizacion pasiva de Parte B/C sobre rosbag TurtleBot4.

Levanta solo mapa, MCL hibrido, detector ArUco, adaptador ArUco->MCL y RViz.
No inicia planner, obstacle_monitor, state_machine ni ningun productor de /cmd_vel.
El rosbag se reproduce aparte con `ros2 bag play ... --clock`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tp_platform.platform_profiles import (
    resolve_profile,
    supported_profiles,
    validate_tb4_topics,
    write_resolved_platform,
)


_UNSET = ''


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _override(context, name, default):
    value = _arg(context, name)
    return default if value == _UNSET else value


def _bool_override(context, name, default):
    value = _arg(context, name)
    if value == _UNSET:
        return default
    return value.lower() in ('1', 'true', 'yes', 'on')


def _launch_nodes(context, pkg_share):
    profile = resolve_profile(
        _arg(context, 'profile'),
        robot_namespace=_arg(context, 'robot_namespace'))
    map_yaml = _override(
        context, 'map_yaml', os.path.join(pkg_share, 'maps', profile.map_yaml))
    robot_frame = _override(context, 'base_frame', profile.base_frame)
    odom_frame = _override(context, 'odom_frame', profile.odom_frame)
    odom_topic = _override(context, 'odom_topic', profile.odom_topic)
    reference_odom_topic = _override(
        context, 'reference_odom_topic', profile.reference_odom_topic)
    scan_topic = _override(context, 'scan_topic', profile.scan_topic)
    rgb_topic = _override(context, 'rgb_topic', profile.rgb_topic)
    tf_topic = _override(context, 'tf_topic', profile.tf_topic)
    tf_static_topic = _override(
        context, 'tf_static_topic', profile.tf_static_topic)
    camera_frame = _override(context, 'camera_frame', profile.camera_frame)
    use_sim_time = _bool_override(context, 'use_sim_time', profile.use_sim_time)
    landmark_map_file = _arg(context, 'landmark_map_file')
    use_landmark_likelihood = _bool_override(
        context, 'use_landmark_likelihood', True)
    use_laser_likelihood = _bool_override(context, 'use_laser_likelihood', True)
    laser_max_beams = int(_arg(context, 'laser_max_beams'))
    laser_sigma_hit = float(_arg(context, 'laser_sigma_hit'))
    laser_max_distance = float(_arg(context, 'laser_max_distance'))
    laser_log_weight = float(_arg(context, 'laser_log_weight'))
    landmark_log_weight = float(_arg(context, 'landmark_log_weight'))
    occupied_pose_penalty = float(_arg(context, 'occupied_pose_penalty'))
    max_landmark_measurement_age = float(_arg(
        context, 'max_landmark_measurement_age'))
    use_oos_landmark_updates = _bool_override(
        context, 'use_oos_landmark_updates', False)
    oos_max_observation_age = float(_arg(context, 'oos_max_observation_age'))
    oos_history_duration = float(_arg(context, 'oos_history_duration'))
    oos_max_snapshot_gap = float(_arg(context, 'oos_max_snapshot_gap'))
    diagnostics_csv = _arg(context, 'diagnostics_csv')
    compensate_delayed_observations = _bool_override(
        context, 'compensate_delayed_observations', True)
    if use_oos_landmark_updates:
        compensate_delayed_observations = False
    max_compensation_age = float(_arg(context, 'max_compensation_age'))
    compensation_diagnostics_csv = _arg(context, 'compensation_diagnostics_csv')

    if profile.robot_namespace:
        validate_tb4_topics(
            profile.robot_namespace,
            odom_topic=odom_topic,
            reference_odom_topic=reference_odom_topic,
            scan_topic=scan_topic,
            rgb_topic=rgb_topic,
            tf_topic=tf_topic,
            tf_static_topic=tf_static_topic,
        )

    write_resolved_platform(
        _arg(context, 'artifact_dir') or '/tmp/tp_final_rob',
        profile,
        stage='parte-b-bag-localization',
        run_id=_arg(context, 'run_id'),
        topics={
            'odom_topic': odom_topic,
            'reference_odom_topic': reference_odom_topic,
            'scan_topic': scan_topic,
            'rgb_topic': rgb_topic,
            'tf_topic': tf_topic,
            'tf_static_topic': tf_static_topic,
        },
        frames={
            'base_frame': robot_frame,
            'odom_frame': odom_frame,
            'camera_frame': camera_frame,
        },
        artifacts={
            'map_yaml': map_yaml,
            'landmark_map_file': landmark_map_file,
        },
    )

    common = {'use_sim_time': use_sim_time}
    tf_remaps = [
        ('/tf', tf_topic),
        ('/tf_static', tf_static_topic),
    ]
    sensor_remaps = [
        ('/odom', odom_topic),
        ('/scan', scan_topic),
        *tf_remaps,
    ]
    mcl_likelihood_params = {
        'use_landmark_likelihood': use_landmark_likelihood,
        'use_laser_likelihood': use_laser_likelihood,
        'laser_max_beams': laser_max_beams,
        'laser_sigma_hit': laser_sigma_hit,
        'laser_max_distance': laser_max_distance,
        'laser_log_weight': laser_log_weight,
        'landmark_log_weight': landmark_log_weight,
        'occupied_pose_penalty': occupied_pose_penalty,
        'max_landmark_measurement_age': max_landmark_measurement_age,
        'use_oos_landmark_updates': use_oos_landmark_updates,
        'oos_max_observation_age': oos_max_observation_age,
        'oos_history_duration': oos_history_duration,
        'oos_max_snapshot_gap': oos_max_snapshot_gap,
        'diagnostics_csv': diagnostics_csv,
    }
    aruco_compensation_params = {
        'odom_frame': odom_frame,
        'compensate_delayed_observations': compensate_delayed_observations,
        'max_compensation_age': max_compensation_age,
        'compensation_diagnostics_csv': compensation_diagnostics_csv,
    }

    warnings = []
    default_map = os.path.join(pkg_share, 'maps', profile.map_yaml)
    if map_yaml == default_map:
        warnings.append(LogInfo(
            msg='WARNING: bag localization is using the profile default map_yaml; '
                'pass the Parte A map.yaml for lab bag localization.'))
    if not landmark_map_file:
        warnings.append(LogInfo(
            msg='WARNING: empty landmark_map_file; MCL will use laser likelihood '
                'but will not correct with ArUco IDs.'))

    map_loader = Node(
        package='tp_b_navigation', executable='map_loader', name='map_loader',
        output='screen', parameters=[{'map_yaml': map_yaml}, common])
    mcl = Node(
        package='tp_b_navigation', executable='mcl_localization',
        name='mcl_localization', output='screen',
        parameters=[{
            'base_frame': robot_frame,
            'odom_frame': odom_frame,
            'motion_odom_topic': odom_topic,
            'reference_odom_topic': reference_odom_topic,
            'landmark_map_file': landmark_map_file,
        }, mcl_likelihood_params, common], remappings=sensor_remaps)
    aruco_detector = Node(
        package='tp_a_slam_aruco', executable='aruco_detector',
        name='aruco_detector', output='screen',
        parameters=[{
            'image_topic': rgb_topic,
            'camera_frame': camera_frame,
        }, common])
    aruco_adapter = Node(
        package='tp_b_navigation', executable='aruco_mcl_adapter',
        name='aruco_mcl_adapter', output='screen',
        parameters=[{'base_frame': robot_frame}, aruco_compensation_params, common],
        remappings=tf_remaps)
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[common],
        remappings=sensor_remaps,
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen')

    return [*warnings, map_loader, mcl, aruco_detector, aruco_adapter, rviz]


def generate_launch_description():
    pkg_share = get_package_share_directory('tp_b_navigation')
    default_rviz_config = os.path.join(pkg_share, 'config', 'parte_b.rviz')
    args = [
        DeclareLaunchArgument(
            'profile', default_value='bag_tb4',
            choices=list(supported_profiles())),
        DeclareLaunchArgument(
            'robot_namespace', default_value='tb4_0',
            description='Namespace del TurtleBot4 grabado en el bag.'),
        DeclareLaunchArgument(
            'artifact_dir', default_value='/tmp/tp_final_rob',
            description='Directorio de artefactos platform-resolved.yaml.'),
        DeclareLaunchArgument('run_id', default_value='bag-localization'),
        DeclareLaunchArgument('map_yaml', default_value=_UNSET),
        DeclareLaunchArgument('base_frame', default_value=_UNSET),
        DeclareLaunchArgument('odom_frame', default_value=_UNSET),
        DeclareLaunchArgument('odom_topic', default_value=_UNSET),
        DeclareLaunchArgument('reference_odom_topic', default_value=_UNSET),
        DeclareLaunchArgument('scan_topic', default_value=_UNSET),
        DeclareLaunchArgument('rgb_topic', default_value=_UNSET),
        DeclareLaunchArgument('tf_topic', default_value=_UNSET),
        DeclareLaunchArgument('tf_static_topic', default_value=_UNSET),
        DeclareLaunchArgument('camera_frame', default_value=_UNSET),
        DeclareLaunchArgument(
            'landmark_map_file', default_value='',
            description='JSON de trayectoria+landmarks producido por Parte A.'),
        DeclareLaunchArgument('use_landmark_likelihood', default_value='true'),
        DeclareLaunchArgument('use_laser_likelihood', default_value='true'),
        DeclareLaunchArgument('laser_max_beams', default_value='60'),
        DeclareLaunchArgument('laser_sigma_hit', default_value='0.18'),
        DeclareLaunchArgument('laser_max_distance', default_value='0.8'),
        DeclareLaunchArgument('laser_log_weight', default_value='1.0'),
        DeclareLaunchArgument('landmark_log_weight', default_value='1.0'),
        DeclareLaunchArgument('occupied_pose_penalty', default_value='20.0'),
        DeclareLaunchArgument('max_landmark_measurement_age', default_value='0.5'),
        DeclareLaunchArgument('use_oos_landmark_updates', default_value='false'),
        DeclareLaunchArgument('oos_max_observation_age', default_value='4.0'),
        DeclareLaunchArgument('oos_history_duration', default_value='6.0'),
        DeclareLaunchArgument('oos_max_snapshot_gap', default_value='0.15'),
        DeclareLaunchArgument('compensate_delayed_observations', default_value='true'),
        DeclareLaunchArgument('max_compensation_age', default_value='4.0'),
        DeclareLaunchArgument(
            'compensation_diagnostics_csv',
            default_value='/tmp/aruco_mcl_compensation.csv'),
        DeclareLaunchArgument(
            'diagnostics_csv', default_value='',
            description='CSV opcional para diagnosticar correcciones MCL.'),
        DeclareLaunchArgument('use_sim_time', default_value=_UNSET),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
    ]
    return LaunchDescription(args + [
        OpaqueFunction(function=_launch_nodes, args=[pkg_share]),
    ])
