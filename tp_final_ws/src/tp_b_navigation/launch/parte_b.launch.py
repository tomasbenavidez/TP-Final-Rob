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
    # Default: mapa del entorno SIMULADO generado con sim_mapper (ver 05_remapeo_sim.md).
    # Para volver al mapa del profe: map_yaml:=.../mapas/map.yaml
    default_map = os.path.join(pkg_share, 'maps', 'map_sim.yaml')
    landmarks_yaml = os.path.join(pkg_share, 'config', 'landmarks.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'parte_b.rviz')

    map_yaml = LaunchConfiguration('map_yaml')
    robot_frame = LaunchConfiguration('robot_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    odom_topic = LaunchConfiguration('odom_topic')
    truth_odom_topic = LaunchConfiguration('truth_odom_topic')
    truth_odom_frame = LaunchConfiguration('truth_odom_frame')
    camera_frame = LaunchConfiguration('camera_frame')

    args = [
        DeclareLaunchArgument('map_yaml', default_value=default_map),
        DeclareLaunchArgument('robot_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_topic', default_value='/calc_odom'),
        DeclareLaunchArgument('truth_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('truth_odom_frame', default_value='odom'),
        DeclareLaunchArgument('camera_frame', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
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
        parameters=[{
            'robot_frame': robot_frame,
            'truth_frame': truth_odom_frame,
            'camera_frame': camera_frame,
        }, common])

    mcl = Node(
        package='tp_b_navigation', executable='mcl_localization',
        name='mcl_localization', output='screen',
        parameters=[{
            'base_frame': robot_frame,
            'motion_odom_topic': odom_topic,
            'reference_odom_topic': truth_odom_topic,
        }, common])

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
