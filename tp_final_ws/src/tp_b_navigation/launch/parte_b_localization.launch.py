#!/usr/bin/env python3
"""Launch de Parte B (incremento 1: mapa + landmarks + sensor virtual).

Levanta:
  - map_loader        -> publica /map (carga mapas/map.yaml)
  - landmark_publisher-> publica /landmarks (landmarks virtuales fijos)
  - landmark_sensor   -> publica /observed_landmarks (FOV + oclusión + ruido)
  - rviz2 (opcional)

La simulación de Gazebo se lanza aparte, en otra terminal:
  ros2 launch turtlebot3_custom_simulation custom_casa.launch.py

Nota: el sensor necesita la TF map->robot para producir observaciones; esa cadena la
cerrará el nodo MCL (incremento siguiente). Sin MCL, /map y /landmarks ya se visualizan
en RViz con fixed frame 'map'.
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

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml', default_value=default_map,
        description='Ruta al .yaml del mapa (formato map_server).')
    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame', default_value='base_footprint',
        description='Frame del robot donde se expresan las observaciones.')
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='Abrir RViz2 con la config de Parte B.')

    map_loader = Node(
        package='tp_b_navigation',
        executable='map_loader',
        name='map_loader',
        output='screen',
        parameters=[{'map_yaml': LaunchConfiguration('map_yaml')}],
    )

    landmark_publisher = Node(
        package='tp_b_navigation',
        executable='landmark_publisher',
        name='landmark_publisher',
        output='screen',
        parameters=[landmarks_yaml],
    )

    landmark_sensor = Node(
        package='tp_b_navigation',
        executable='landmark_sensor',
        name='landmark_sensor',
        output='screen',
        parameters=[{'robot_frame': LaunchConfiguration('robot_frame')}],
    )

    mcl = Node(
        package='tp_b_navigation',
        executable='mcl_localization',
        name='mcl_localization',
        output='screen',
        parameters=[{'base_frame': LaunchConfiguration('robot_frame')}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen',
    )

    return LaunchDescription([
        map_yaml_arg,
        robot_frame_arg,
        launch_rviz_arg,
        map_loader,
        landmark_publisher,
        landmark_sensor,
        mcl,
        rviz,
    ])
