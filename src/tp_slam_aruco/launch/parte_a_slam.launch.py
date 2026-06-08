#!/usr/bin/env python3
"""
parte_a_slam.launch.py
======================
Lanza el pipeline de SLAM de la Parte A (Opción 3):
  - aruco_detector_node : detecta los landmarks ArUco
  - odometry_node       : calcula los deltas de movimiento
  - graph_slam_node     : construye y optimiza el grafo

USO:
  # Terminal 1: reproducir el bag del laberinto
  ros2 bag play <carpeta_del_bag>
  # Terminal 2: lanzar el pipeline
  ros2 launch tp_slam_aruco parte_a_slam.launch.py \\
      calibration_file:=<ruta_al_yaml_de_calibracion> \\
      trajectory_file:=/tmp/trayectoria.json

Parar con Ctrl+C guarda automáticamente la trayectoria optimizada
(si trajectory_file está configurado), lista para la segunda pasada
de mapeo LIDAR (Fase 5).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    calib_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='',
        description='Ruta al YAML con K y coeficientes de distorsión (TB4 #0)',
    )
    kf_dist_arg = DeclareLaunchArgument(
        'kf_dist',
        default_value='0.15',
        description='Distancia mínima para crear un keyframe nuevo (metros)',
    )
    kf_angle_max_arg = DeclareLaunchArgument(
        'kf_angle_max',
        default_value='0.60',
        description='Giro mínimo para crear un keyframe por rotación (rad)',
    )
    reobs_min_parallax_arg = DeclareLaunchArgument(
        'reobs_min_parallax',
        default_value='0.20',
        description='Parallax mínima para aceptar una reobservación (metros)',
    )
    optimize_every_arg = DeclareLaunchArgument(
        'optimize_every',
        default_value='1',
        description='Cada cuántos keyframes optimizar el grafo',
    )
    camera_tx_arg = DeclareLaunchArgument(
        'camera_tx',
        default_value='0.0',
        description='Traslación x de cámara respecto de base_link (metros)',
    )
    camera_ty_arg = DeclareLaunchArgument(
        'camera_ty',
        default_value='0.0',
        description='Traslación y de cámara respecto de base_link (metros)',
    )
    camera_yaw_arg = DeclareLaunchArgument(
        'camera_yaw',
        default_value='0.0',
        description='Yaw de cámara respecto de base_link (rad)',
    )
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=os.path.join(
            get_package_share_directory('tp_slam_aruco'), 'runs', 'trayectoria.json'),
        description='Ruta JSON donde guardar la trayectoria optimizada al finalizar (para Fase 5)',
    )
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='oakd_rgb_camera_optical_frame',
        description='frame_id de la cámara. graph_slam_node usará el TF '
                    'de este frame a base_link para transformar observaciones.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar reloj de simulación (necesario al reproducir un bag con --clock)',
    )

    calibration_file = LaunchConfiguration('calibration_file')
    camera_frame = LaunchConfiguration('camera_frame')
    kf_dist = LaunchConfiguration('kf_dist')
    kf_angle_max = LaunchConfiguration('kf_angle_max')
    reobs_min_parallax = LaunchConfiguration('reobs_min_parallax')
    optimize_every = LaunchConfiguration('optimize_every')
    camera_tx = LaunchConfiguration('camera_tx')
    camera_ty = LaunchConfiguration('camera_ty')
    camera_yaw = LaunchConfiguration('camera_yaw')
    trajectory_file = LaunchConfiguration('trajectory_file')

    aruco_node = Node(
        package='tp_slam_aruco',
        executable='aruco_detector',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'calibration_file': calibration_file,
            'image_topic': 'tb4_0/oakd/rgb/preview/image_raw',
            'marker_length': 0.0889,
            'aruco_dict': 'DICT_4X4_50',
            'camera_frame': camera_frame,
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
        parameters=[{
            'odom_topic': 'tb4_0/odom',
            'kf_dist': kf_dist,
            'kf_angle_max': kf_angle_max,
            'reobs_min_parallax': reobs_min_parallax,
            'optimize_every': optimize_every,
            'camera_tx': camera_tx,
            'camera_ty': camera_ty,
            'camera_yaw': camera_yaw,
            'trajectory_file': trajectory_file,
        }],
    )

    return LaunchDescription([
        calib_arg,
        camera_frame_arg,
        kf_dist_arg,
        kf_angle_max_arg,
        reobs_min_parallax_arg,
        optimize_every_arg,
        camera_tx_arg,
        camera_ty_arg,
        camera_yaw_arg,
        trajectory_file_arg,
        use_sim_time_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        aruco_node,
        odometry_node,
        graph_slam_node,
    ])
