#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tp_platform.platform_profiles import (
    BAG_TB4,
    REAL_TB4,
    resolve_profile,
    validate_tb4_topics,
    write_resolved_platform,
)


_UNSET = ''


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _override(context, name, default):
    value = _arg(context, name)
    return default if value == _UNSET else value


def _bool_arg(context, name):
    return _arg(context, name).lower() in ('1', 'true', 'yes', 'on')


def _launch_nodes(context, mission_share, config):
    profile = resolve_profile(
        _arg(context, 'profile'),
        robot_namespace=_arg(context, 'robot_namespace'))
    use_sim_time = profile.use_sim_time
    map_yaml = LaunchConfiguration('map_yaml')
    landmark_map = LaunchConfiguration('landmark_map_file')
    odom_topic = _override(context, 'odom_topic', profile.odom_topic)
    scan_topic = _override(context, 'scan_topic', profile.scan_topic)
    cmd_vel_topic = _override(context, 'cmd_vel_topic', profile.cmd_vel_topic)
    rgb = _override(context, 'rgb_topic', profile.rgb_topic)
    depth = _override(context, 'depth_topic', profile.depth_topic)
    info = _override(context, 'camera_info_topic', profile.camera_info_topic)
    tf_topic = _override(context, 'tf_topic', profile.tf_topic)
    tf_static_topic = _override(
        context, 'tf_static_topic', profile.tf_static_topic)
    base_frame = _override(context, 'base_frame', profile.base_frame)
    odom_frame = _override(context, 'odom_frame', profile.odom_frame)
    localization_safety_params = {
        'enable_safety_gates': _bool_arg(context, 'enable_safety_gates'),
        'max_mcl_pose_age': float(_arg(context, 'max_mcl_pose_age')),
        'max_position_covariance': float(_arg(context, 'max_position_covariance')),
        'max_yaw_covariance': float(_arg(context, 'max_yaw_covariance')),
    }
    monitor_safety_params = {
        **localization_safety_params,
        'max_scan_age': float(_arg(context, 'max_scan_age')),
    }
    state_machine_safety_params = {
        **monitor_safety_params,
        'max_monitor_age': float(_arg(context, 'max_monitor_age')),
    }
    validate_tb4_topics(
        profile.robot_namespace,
        odom_topic=odom_topic,
        scan_topic=scan_topic,
        cmd_vel_topic=cmd_vel_topic,
        rgb_topic=rgb,
        depth_topic=depth,
        camera_info_topic=info,
        tf_topic=tf_topic,
        tf_static_topic=tf_static_topic,
    )
    write_resolved_platform(
        _arg(context, 'artifact_dir') or '/tmp/tp_final_rob',
        profile,
        topics={
            'odom_topic': odom_topic,
            'scan_topic': scan_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'rgb_topic': rgb,
            'depth_topic': depth,
            'camera_info_topic': info,
            'tf_topic': tf_topic,
            'tf_static_topic': tf_static_topic,
        },
        frames={'base_frame': base_frame, 'odom_frame': odom_frame},
        artifacts={
            'map_yaml': _arg(context, 'map_yaml'),
            'landmark_map_file': _arg(context, 'landmark_map_file'),
        },
    )

    tf_remaps = [
        ('/tf', tf_topic),
        ('/tf_static', tf_static_topic),
    ]
    common_remaps = [
        ('/odom', odom_topic),
        ('/scan', scan_topic),
        ('/cmd_vel', cmd_vel_topic),
        *tf_remaps,
    ]
    return [
        Node(package='tp_b_navigation', executable='map_loader', output='screen',
             parameters=[{'map_yaml': map_yaml, 'use_sim_time': use_sim_time}]),
        Node(package='tp_b_navigation', executable='mcl_localization', output='screen',
             parameters=[{'landmark_map_file': landmark_map, 'base_frame': base_frame,
                          'odom_frame': odom_frame,
                          'motion_odom_topic': odom_topic,
                          'reference_odom_topic': odom_topic,
                          'use_sim_time': use_sim_time}], remappings=common_remaps),
        Node(package='tp_b_navigation', executable='global_planner', output='screen',
             parameters=[{'base_frame': base_frame,
                          'use_sim_time': use_sim_time}],
             remappings=tf_remaps),
        Node(package='tp_b_navigation', executable='obstacle_monitor', output='screen',
             parameters=[{'base_frame': base_frame,
                          'use_sim_time': use_sim_time}, monitor_safety_params],
             remappings=common_remaps),
        Node(package='tp_b_navigation', executable='state_machine', output='screen',
             parameters=[{'base_frame': base_frame,
                          'use_sim_time': use_sim_time},
                         state_machine_safety_params],
             remappings=common_remaps),
        Node(package='tp_a_slam_aruco', executable='aruco_detector', output='screen',
             parameters=[{'image_topic': rgb, 'camera_frame': '',
                          'use_sim_time': use_sim_time}]),
        Node(package='tp_b_navigation', executable='aruco_mcl_adapter', output='screen',
             parameters=[{'base_frame': base_frame,
                          'use_sim_time': use_sim_time}],
             remappings=tf_remaps),
        Node(package='tp_c_mission', executable='red_cone_detector', output='screen',
             parameters=[config, {'rgb_topic': rgb, 'depth_topic': depth,
                                  'camera_info_topic': info,
                                  'use_sim_time': use_sim_time}],
             remappings=tf_remaps),
        Node(package='tp_c_mission', executable='mission_manager', output='screen',
             parameters=[config, {'landmark_map_file': landmark_map,
                                  'use_sim_time': use_sim_time},
                         localization_safety_params]),
    ]


def generate_launch_description():
    mission_share = get_package_share_directory('tp_c_mission')
    config = os.path.join(mission_share, 'config', 'parte_c.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='real_tb4',
                              choices=[REAL_TB4, BAG_TB4]),
        DeclareLaunchArgument('robot_namespace', default_value='tb4_0'),
        DeclareLaunchArgument('artifact_dir', default_value='/tmp/tp_final_rob'),
        DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('landmark_map_file'),
        DeclareLaunchArgument('odom_topic', default_value=_UNSET),
        DeclareLaunchArgument('scan_topic', default_value=_UNSET),
        DeclareLaunchArgument('cmd_vel_topic', default_value=_UNSET),
        DeclareLaunchArgument('tf_topic', default_value=_UNSET),
        DeclareLaunchArgument('tf_static_topic', default_value=_UNSET),
        DeclareLaunchArgument('base_frame', default_value=_UNSET),
        DeclareLaunchArgument('odom_frame', default_value=_UNSET),
        DeclareLaunchArgument('enable_safety_gates', default_value='true'),
        DeclareLaunchArgument('max_mcl_pose_age', default_value='1.0'),
        DeclareLaunchArgument('max_scan_age', default_value='1.0'),
        DeclareLaunchArgument('max_monitor_age', default_value='1.0'),
        DeclareLaunchArgument('max_position_covariance', default_value='0.25'),
        DeclareLaunchArgument('max_yaw_covariance', default_value='0.5'),
        DeclareLaunchArgument('rgb_topic',
                              default_value=_UNSET),
        DeclareLaunchArgument('depth_topic',
                              default_value=_UNSET),
        DeclareLaunchArgument('camera_info_topic',
                              default_value=_UNSET),
        OpaqueFunction(function=_launch_nodes, args=[mission_share, config]),
    ])
