#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_share = get_package_share_directory('tp_c_mission')
    config = os.path.join(mission_share, 'config', 'parte_c.yaml')
    map_yaml = LaunchConfiguration('map_yaml')
    landmark_map = LaunchConfiguration('landmark_map_file')
    rgb = LaunchConfiguration('rgb_topic')
    depth = LaunchConfiguration('depth_topic')
    info = LaunchConfiguration('camera_info_topic')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    common_remaps = [('/odom', LaunchConfiguration('odom_topic')),
                     ('/scan', LaunchConfiguration('scan_topic')),
                     ('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))]
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('landmark_map_file'),
        DeclareLaunchArgument('odom_topic', default_value='/tb4_0/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/tb4_0/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/tb4_0/cmd_vel'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('rgb_topic',
                              default_value='/tb4_0/oakd/rgb/preview/image_raw'),
        DeclareLaunchArgument('depth_topic',
                              default_value='/tb4_0/oakd/stereo/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/tb4_0/oakd/rgb/preview/camera_info'),
        Node(package='tp_b_navigation', executable='map_loader', output='screen',
             parameters=[{'map_yaml': map_yaml, 'use_sim_time': False}]),
        Node(package='tp_b_navigation', executable='mcl_localization', output='screen',
             parameters=[{'landmark_map_file': landmark_map, 'base_frame': base_frame,
                          'odom_frame': odom_frame,
                          'motion_odom_topic': LaunchConfiguration('odom_topic'),
                          'reference_odom_topic': LaunchConfiguration('odom_topic'),
                          'use_sim_time': False}], remappings=common_remaps),
        Node(package='tp_b_navigation', executable='global_planner', output='screen',
             parameters=[{'base_frame': base_frame}]),
        Node(package='tp_b_navigation', executable='obstacle_monitor', output='screen',
             parameters=[{'base_frame': base_frame}], remappings=common_remaps),
        Node(package='tp_b_navigation', executable='state_machine', output='screen',
             parameters=[{'base_frame': base_frame}], remappings=common_remaps),
        Node(package='tp_slam_aruco', executable='aruco_detector', output='screen',
             parameters=[{'image_topic': rgb, 'camera_frame': ''}]),
        Node(package='tp_c_mission', executable='aruco_mcl_adapter', output='screen',
             parameters=[{'base_frame': base_frame}]),
        Node(package='tp_c_mission', executable='red_cone_detector', output='screen',
             parameters=[config, {'rgb_topic': rgb, 'depth_topic': depth,
                                  'camera_info_topic': info}]),
        Node(package='tp_c_mission', executable='mission_manager', output='screen',
             parameters=[config, {'landmark_map_file': landmark_map}]),
    ])
