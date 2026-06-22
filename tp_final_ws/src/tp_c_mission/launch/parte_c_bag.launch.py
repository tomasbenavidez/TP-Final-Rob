#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tp_c_mission')
    config = os.path.join(share, 'config', 'parte_c.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic',
                              default_value='/tb4_0/oakd/rgb/preview/image_raw'),
        DeclareLaunchArgument('depth_topic',
                              default_value='/tb4_0/oakd/stereo/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/tb4_0/oakd/rgb/preview/camera_info'),
        Node(
            package='tp_c_mission', executable='red_cone_detector', output='screen',
            parameters=[config, {
                'use_sim_time': True,
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            }]),
        Node(
            package='tp_slam_aruco', executable='tf_bridge', output='screen',
            parameters=[{'use_sim_time': True}]),
    ])
