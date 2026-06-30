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
  ros2 launch tp_a_slam_aruco parte_a_mapa.launch.py \\
      trajectory_file:=/ruta/trayectoria.json \\
      map_output:=/ruta/mapa

  Ctrl+C al terminar el bag → exporta mapa.pgm + mapa.yaml
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('tp_a_slam_aruco')

    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=os.path.join(pkg, 'runs', 'trayectoria.json'),
        description='JSON con la trayectoria optimizada (salida de parte_a_slam)',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/tb4_0/odom',
        description='Odometría física del TurtleBot4 o del rosbag.',
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/tb4_0/scan',
        description='Tópico LaserScan físico del TurtleBot4 o del rosbag.',
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Frame base usado para resolver TF base<-LIDAR.',
    )
    lidar_fallback_enabled_arg = DeclareLaunchArgument(
        'lidar_fallback_enabled',
        default_value='true',
        description='Usar extrínsecos TB4 fallback si TF base<-LIDAR no está disponible.',
    )
    use_bag_tf_arg = DeclareLaunchArgument(
        'use_bag_tf',
        default_value='true',
        description='Repubicar TF del bag a /tf y /tf_static.',
    )
    bag_tf_topic_arg = DeclareLaunchArgument(
        'bag_tf_topic',
        default_value='/tb4_0/tf',
        description='Tópico TF dinámico grabado en el rosbag.',
    )
    bag_tf_static_topic_arg = DeclareLaunchArgument(
        'bag_tf_static_topic',
        default_value='/tb4_0/tf_static',
        description='Tópico TF estático grabado en el rosbag.',
    )
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    lidar_fallback_enabled = LaunchConfiguration('lidar_fallback_enabled')
    use_bag_tf = LaunchConfiguration('use_bag_tf')
    bag_tf_topic = LaunchConfiguration('bag_tf_topic')
    bag_tf_static_topic = LaunchConfiguration('bag_tf_static_topic')
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
        package='tp_a_slam_aruco',
        executable='occupancy_grid',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'trajectory_file': LaunchConfiguration('trajectory_file'),
            'map_output':      LaunchConfiguration('map_output'),
            'resolution':      LaunchConfiguration('resolution'),
            'scan_topic':      scan_topic,
            'odom_topic':      odom_topic,
            'base_frame':      base_frame,
            'publish_every':   50,
            # Extrinsecos LIDAR del TF estatico del bag:
            #   base_link -> shell_link (yaw 0) -> rplidar_link (tx=-0.04, yaw=+90deg)
            'lidar_tx':       -0.04,
            'lidar_ty':        0.0,
            'lidar_yaw':       math.pi / 2,   # +90deg: el RPLIDAR esta rotado en base_link
            'lidar_fallback_enabled': ParameterValue(
                lidar_fallback_enabled, value_type=bool),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
        }],
    )

    tf_bridge_node = Node(
        package='tp_a_slam_aruco',
        executable='tf_bridge',
        name='tf_bridge_node',
        output='screen',
        condition=IfCondition(use_bag_tf),
        parameters=[{
            'bag_tf_topic': bag_tf_topic,
            'bag_tf_static_topic': bag_tf_static_topic,
            'tf_topic': '/tf',
            'tf_static_topic': '/tf_static',
        }],
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        trajectory_file_arg,
        odom_topic_arg,
        scan_topic_arg,
        base_frame_arg,
        lidar_fallback_enabled_arg,
        use_bag_tf_arg,
        bag_tf_topic_arg,
        bag_tf_static_topic_arg,
        map_output_arg,
        resolution_arg,
        max_angular_velocity_arg,
        tf_bridge_node,
        occupancy_node,
    ])
