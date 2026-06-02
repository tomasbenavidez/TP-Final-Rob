#!/usr/bin/env python3
"""
parte_a_slam.launch.py
======================
Lanza el pipeline de SLAM de la Parte A (Opción 3):
  - aruco_detector_node : detecta los landmarks ArUco
  - odometry_node       : calcula los deltas de movimiento
  - graph_slam_node     : construye y optimiza el grafo (F4, en progreso)

USO:
  # Terminal 1: reproducir el bag del laberinto
  ros2 bag play <carpeta_del_bag>
  # Terminal 2: lanzar el pipeline
  ros2 launch tp_slam_aruco parte_a_slam.launch.py \\
      calibration_file:=<ruta_al_yaml_de_calibracion>

Recordá pasar la ruta a la calibración de la cámara (K + coeficientes),
provista por la cátedra para el TB4 #0.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Argumento de launch para la ruta de calibración (sin default: hay que pasarlo).
    calib_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='',
        description='Ruta al YAML con K y coeficientes de distorsión (TB4 #0)',
    )
    calibration_file = LaunchConfiguration('calibration_file')

    aruco_node = Node(
        package='tp_slam_aruco',
        executable='aruco_detector',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'calibration_file': calibration_file,
            'image_topic': 'tb4_0/oakd/rgb/image_raw',
            'marker_length': 0.15,
            'aruco_dict': 'DICT_4X4_50',
        }],
    )

    odometry_node = Node(
        package='tp_slam_aruco',
        executable='odometry',
        name='odometry_node',
        output='screen',
        parameters=[{'odom_topic': 'tb4_0/odom'}],
    )

    graph_slam_node = Node(
        package='tp_slam_aruco',
        executable='graph_slam',
        name='graph_slam_node',
        output='screen',
        parameters=[{'odom_topic': 'tb4_0/odom'}],
    )

    return LaunchDescription([
        calib_arg,
        aruco_node,
        odometry_node,
        graph_slam_node,
    ])
