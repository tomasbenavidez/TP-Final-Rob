#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tp_b_navigation.platform_profiles import BAG_TB4, REAL_TB4, resolve_profile


_UNSET = ''


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _override(context, name, default):
    value = _arg(context, name)
    return default if value == _UNSET else value


def _launch_nodes(context, mission_share, config):
    profile = resolve_profile(_arg(context, 'profile'))
    use_sim_time = profile.use_sim_time
    map_yaml = LaunchConfiguration('map_yaml')
    landmark_map = LaunchConfiguration('landmark_map_file')
    odom_topic = _override(context, 'odom_topic', profile.odom_topic)
    scan_topic = _override(context, 'scan_topic', profile.scan_topic)
    cmd_vel_topic = _override(context, 'cmd_vel_topic', profile.cmd_vel_topic)
    rgb = _override(context, 'rgb_topic', profile.rgb_topic)
    depth = _override(context, 'depth_topic', profile.depth_topic)
    info = _override(context, 'camera_info_topic', profile.camera_info_topic)
    base_frame = _override(context, 'base_frame', profile.base_frame)
    odom_frame = _override(context, 'odom_frame', profile.odom_frame)

    common_remaps = [('/odom', odom_topic),
                     ('/scan', scan_topic),
                     ('/cmd_vel', cmd_vel_topic)]
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
                          'use_sim_time': use_sim_time}]),
        Node(package='tp_b_navigation', executable='obstacle_monitor', output='screen',
             parameters=[{'base_frame': base_frame,
                          'use_sim_time': use_sim_time}],
             remappings=common_remaps),
        Node(package='tp_b_navigation', executable='state_machine', output='screen',
             parameters=[{'base_frame': base_frame,
                          'use_sim_time': use_sim_time}],
             remappings=common_remaps),
        Node(package='tp_a_slam_aruco', executable='aruco_detector', output='screen',
             parameters=[{'image_topic': rgb, 'camera_frame': '',
                          'use_sim_time': use_sim_time}]),
        Node(package='tp_c_mission', executable='aruco_mcl_adapter', output='screen',
             parameters=[{'base_frame': base_frame,
                          'use_sim_time': use_sim_time}]),
        Node(package='tp_c_mission', executable='red_cone_detector', output='screen',
             parameters=[config, {'rgb_topic': rgb, 'depth_topic': depth,
                                  'camera_info_topic': info,
                                  'use_sim_time': use_sim_time}]),
        Node(package='tp_c_mission', executable='mission_manager', output='screen',
             parameters=[config, {'landmark_map_file': landmark_map,
                                  'use_sim_time': use_sim_time}]),
    ]


def generate_launch_description():
    mission_share = get_package_share_directory('tp_c_mission')
    config = os.path.join(mission_share, 'config', 'parte_c.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='real_tb4',
                              choices=[REAL_TB4, BAG_TB4]),
        DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('landmark_map_file'),
        DeclareLaunchArgument('odom_topic', default_value=_UNSET),
        DeclareLaunchArgument('scan_topic', default_value=_UNSET),
        DeclareLaunchArgument('cmd_vel_topic', default_value=_UNSET),
        DeclareLaunchArgument('base_frame', default_value=_UNSET),
        DeclareLaunchArgument('odom_frame', default_value=_UNSET),
        DeclareLaunchArgument('rgb_topic',
                              default_value=_UNSET),
        DeclareLaunchArgument('depth_topic',
                              default_value=_UNSET),
        DeclareLaunchArgument('camera_info_topic',
                              default_value=_UNSET),
        OpaqueFunction(function=_launch_nodes, args=[mission_share, config]),
    ])
