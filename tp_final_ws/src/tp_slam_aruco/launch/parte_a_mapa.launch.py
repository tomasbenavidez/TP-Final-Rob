#!/usr/bin/env python3
"""
parte_a_mapa.launch.py
======================
Segunda pasada del pipeline SLAM (Opción 3): genera el mapa de grilla de
ocupación proyectando los scans LIDAR sobre la trayectoria ya corregida.

PREREQUISITO: haber corrido parte_a_slam.launch.py y tener el JSON de salida.

USO:
  # Terminal 1: reproducir el bag
  ros2 bag play <carpeta_bag> --clock

  # Terminal 2: lanzar la segunda pasada
  ros2 launch tp_slam_aruco parte_a_mapa.launch.py \\
      trajectory_file:=/ruta/trayectoria.json \\
      map_output:=/ruta/mapa

  Ctrl+C al terminar el bag → exporta mapa.pgm + mapa.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg = get_package_share_directory('tp_slam_aruco')

    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=os.path.join(pkg, 'runs', 'trayectoria.json'),
        description='JSON con la trayectoria optimizada (salida de parte_a_slam)',
    )
    map_output_arg = DeclareLaunchArgument(
        'map_output',
        default_value=os.path.join(pkg, 'runs', 'mapa'),
        description='Prefijo de salida del mapa (sin extensión). Genera .pgm y .yaml.',
    )
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolución de la grilla en metros/celda',
    )
    enable_motion_filter_arg = DeclareLaunchArgument(
        'enable_motion_filter',
        default_value='true',
        description='Integrar scans solo en tramos quietos o casi rectos.',
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed_rad_s',
        default_value='0.10',
        description='Velocidad angular máxima para integrar scans.',
    )
    max_curvature_arg = DeclareLaunchArgument(
        'max_path_curvature_rad_m',
        default_value='0.35',
        description='Curvatura máxima permitida para integrar scans.',
    )
    max_lateral_change_arg = DeclareLaunchArgument(
        'max_lateral_pose_change_m',
        default_value='0.03',
        description='Desvío lateral máximo permitido en la ventana cinemática.',
    )
    still_speed_arg = DeclareLaunchArgument(
        'still_linear_speed_m_s',
        default_value='0.03',
        description='Velocidad lineal por debajo de la cual el robot se considera quieto.',
    )

    occupancy_node = Node(
        package='tp_slam_aruco',
        executable='occupancy_grid',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'trajectory_file': LaunchConfiguration('trajectory_file'),
            'map_output':      LaunchConfiguration('map_output'),
            'resolution':      LaunchConfiguration('resolution'),
            'scan_topic':      'tb4_0/scan',
            'publish_every':   50,
            'base_frame':      'base_link',
            'scan_frame':      'rplidar_link',
            'tf_static_topic': '/tb4_0/tf_static',
            'use_tf_static_extrinsics': True,
            'lidar_tx':       -0.04,
            'lidar_ty':        0.0,
            'lidar_yaw':       1.57079632679,
            'enable_motion_filter': LaunchConfiguration('enable_motion_filter'),
            'max_angular_speed_rad_s': LaunchConfiguration('max_angular_speed_rad_s'),
            'max_path_curvature_rad_m': LaunchConfiguration('max_path_curvature_rad_m'),
            'max_lateral_pose_change_m': LaunchConfiguration('max_lateral_pose_change_m'),
            'still_linear_speed_m_s': LaunchConfiguration('still_linear_speed_m_s'),
        }],
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        trajectory_file_arg,
        map_output_arg,
        resolution_arg,
        enable_motion_filter_arg,
        max_angular_speed_arg,
        max_curvature_arg,
        max_lateral_change_arg,
        still_speed_arg,
        occupancy_node,
    ])
