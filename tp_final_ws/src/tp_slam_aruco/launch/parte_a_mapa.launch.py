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

import math
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
    max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='0.0',
        description='No integrar scans con |omega| mayor a este valor (rad/s). '
                    '0 desactiva el gate de rotación.',
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
            'odom_topic':      'tb4_0/odom',
            'publish_every':   50,
            # Extrinsecos LIDAR del TF estatico del bag:
            #   base_link -> shell_link (yaw 0) -> rplidar_link (tx=-0.04, yaw=+90deg)
            'lidar_tx':       -0.04,
            'lidar_ty':        0.0,
            'lidar_yaw':       math.pi / 2,   # +90deg: el RPLIDAR esta rotado en base_link
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
        }],
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        trajectory_file_arg,
        map_output_arg,
        resolution_arg,
        max_angular_velocity_arg,
        occupancy_node,
    ])
