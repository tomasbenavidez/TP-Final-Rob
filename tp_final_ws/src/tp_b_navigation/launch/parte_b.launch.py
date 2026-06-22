#!/usr/bin/env python3
"""Launch completo de Parte B (navegación autónoma — Sistema 3).

Levanta toda la pila sobre la simulación de Gazebo (que se lanza aparte):
  ros2 launch turtlebot3_custom_simulation custom_casa.launch.py

Pila:
  map_loader -> landmark_publisher -> landmark_sensor -> mcl_localization
             -> global_planner -> obstacle_monitor -> state_machine -> rviz2

Uso en RViz: "2D Pose Estimate" (initialpose) y luego "2D Goal Pose" (goal_pose).
Todos los nodos usan use_sim_time:=true para alinearse con el /clock de Gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tp_b_navigation')
    default_map = os.path.join(pkg_share, 'maps', 'map.yaml')
    landmarks_yaml = os.path.join(pkg_share, 'config', 'landmarks.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'parte_b.rviz')

    map_yaml = LaunchConfiguration('map_yaml')
    robot_frame = LaunchConfiguration('robot_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    args = [
        DeclareLaunchArgument('map_yaml', default_value=default_map),
        DeclareLaunchArgument('robot_frame', default_value='base_footprint'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
    ]

    common = {'use_sim_time': use_sim_time}

    map_loader = Node(
        package='tp_b_navigation', executable='map_loader', name='map_loader',
        output='screen', parameters=[{'map_yaml': map_yaml}, common])

    landmark_publisher = Node(
        package='tp_b_navigation', executable='landmark_publisher',
        name='landmark_publisher', output='screen',
        parameters=[landmarks_yaml, common])

    landmark_sensor = Node(
        package='tp_b_navigation', executable='landmark_sensor',
        name='landmark_sensor', output='screen',
        parameters=[{'robot_frame': robot_frame}, common])

    mcl = Node(
        package='tp_b_navigation', executable='mcl_localization',
        name='mcl_localization', output='screen',
        parameters=[{'base_frame': robot_frame}, common])

    planner = Node(
        package='tp_b_navigation', executable='global_planner',
        name='global_planner', output='screen',
        parameters=[{'base_frame': robot_frame}, common])

    monitor = Node(
        package='tp_b_navigation', executable='obstacle_monitor',
        name='obstacle_monitor', output='screen',
        parameters=[{'base_frame': robot_frame}, common])

    sm = Node(
        package='tp_b_navigation', executable='state_machine',
        name='state_machine', output='screen',
        parameters=[{'base_frame': robot_frame}, common])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[common],
        condition=IfCondition(launch_rviz), output='screen')

    return LaunchDescription(args + [
        map_loader, landmark_publisher, landmark_sensor, mcl,
        planner, monitor, sm, rviz,
    ])
