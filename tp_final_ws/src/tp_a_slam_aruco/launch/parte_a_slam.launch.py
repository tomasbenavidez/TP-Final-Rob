#!/usr/bin/env python3
"""
parte_a_slam.launch.py
======================
Lanza el pipeline de SLAM de la Parte A (Opción 3):
  - tf_bridge_node      : repubica el TF del bag a /tf y /tf_static
  - aruco_detector_node : detecta los landmarks ArUco
  - graph_slam_node     : construye y optimiza el grafo
  - rviz2               : vista en tiempo simulado con trayectoria, TF e imagen

USO:
  # Terminal 1: reproducir el bag del laberinto
  ros2 bag play <carpeta_del_bag> --clock
  # Terminal 2: lanzar el pipeline
  ros2 launch tp_a_slam_aruco parte_a_slam.launch.py \\
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_default_config = os.path.join(
        get_package_share_directory('tp_a_slam_aruco'),
        'config',
        'rviz_config.rviz',
    )

    calib_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='',
        description='Ruta al YAML con K y coeficientes de distorsión (TB4 #0)',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/tb4_0/odom',
        description='Odometría física del TurtleBot4 o del rosbag.',
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
        default_value='-0.0596',
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
    use_bag_tf_arg = DeclareLaunchArgument(
        'use_bag_tf',
        default_value='true',
        description='Repubicar el TF del bag a /tf y /tf_static para tf2/RViz.',
    )
    bag_tf_topic_arg = DeclareLaunchArgument(
        'bag_tf_topic',
        default_value='/tb4_0/tf',
        description='Topico TF dinamico grabado en el rosbag.',
    )
    bag_tf_static_topic_arg = DeclareLaunchArgument(
        'bag_tf_static_topic',
        default_value='/tb4_0/tf_static',
        description='Topico TF estatico grabado en el rosbag.',
    )
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=os.path.join(
            get_package_share_directory('tp_a_slam_aruco'), 'runs', 'trayectoria.json'),
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
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Lanzar RViz sincronizado con /clock como parte del pipeline.',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_default_config,
        description='Ruta al archivo .rviz para la visualizacion de Parte A.',
    )
    min_marker_area_px_arg = DeclareLaunchArgument(
        'min_marker_area_px',
        default_value='250.0',
        description='Area minima del marcador detectado en pixeles.',
    )
    max_marker_depth_arg = DeclareLaunchArgument(
        'max_marker_depth',
        default_value='3.0',
        description='Profundidad maxima aceptada para ArUco en metros.',
    )
    max_reprojection_error_px_arg = DeclareLaunchArgument(
        'max_reprojection_error_px',
        default_value='4.0',
        description='Error maximo de reproyeccion para aceptar una pose ArUco.',
    )
    allowed_marker_ids_arg = DeclareLaunchArgument(
        'allowed_marker_ids',
        default_value='',
        description='Whitelist opcional de IDs ArUco, separados por coma.',
    )
    min_landmark_observations_arg = DeclareLaunchArgument(
        'min_landmark_observations',
        default_value='3',
        description='Cantidad de keyframes donde debe aparecer un ID nuevo.',
    )
    max_landmark_position_jump_arg = DeclareLaunchArgument(
        'max_landmark_position_jump',
        default_value='0.75',
        description='Salto maximo en metros para aceptar una reobservacion ArUco.',
    )

    calibration_file = LaunchConfiguration('calibration_file')
    odom_topic = LaunchConfiguration('odom_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    kf_dist = LaunchConfiguration('kf_dist')
    kf_angle_max = LaunchConfiguration('kf_angle_max')
    reobs_min_parallax = LaunchConfiguration('reobs_min_parallax')
    optimize_every = LaunchConfiguration('optimize_every')
    use_bag_tf = LaunchConfiguration('use_bag_tf')
    bag_tf_topic = LaunchConfiguration('bag_tf_topic')
    bag_tf_static_topic = LaunchConfiguration('bag_tf_static_topic')
    camera_tx = LaunchConfiguration('camera_tx')
    camera_ty = LaunchConfiguration('camera_ty')
    camera_yaw = LaunchConfiguration('camera_yaw')
    trajectory_file = LaunchConfiguration('trajectory_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    min_marker_area_px = LaunchConfiguration('min_marker_area_px')
    max_marker_depth = LaunchConfiguration('max_marker_depth')
    max_reprojection_error_px = LaunchConfiguration('max_reprojection_error_px')
    allowed_marker_ids = LaunchConfiguration('allowed_marker_ids')
    min_landmark_observations = LaunchConfiguration('min_landmark_observations')
    max_landmark_position_jump = LaunchConfiguration('max_landmark_position_jump')

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

    aruco_node = Node(
        package='tp_a_slam_aruco',
        executable='aruco_detector',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'calibration_file': calibration_file,
            'image_topic': 'tb4_0/oakd/rgb/preview/image_raw',
            'marker_length': 0.0889,
            'aruco_dict': 'DICT_4X4_50',
            'camera_frame': camera_frame,
            'detections_topic': '/aruco_detections',
            'debug_image_topic': '/aruco/debug_image',
            'publish_debug_image': True,
            'min_marker_area_px': ParameterValue(min_marker_area_px, value_type=float),
            'min_marker_depth': 0.15,
            'max_marker_depth': ParameterValue(max_marker_depth, value_type=float),
            'max_reprojection_error_px': ParameterValue(
                max_reprojection_error_px, value_type=float
            ),
            'allowed_marker_ids': allowed_marker_ids,
            'diagnostics_file': '/tmp/aruco_detections.csv',
        }],
    )

    graph_slam_node = Node(
        package='tp_a_slam_aruco',
        executable='graph_slam',
        name='graph_slam_node',
        output='screen',
        parameters=[{
            'odom_topic': odom_topic,
            'kf_dist': kf_dist,
            'kf_angle_max': kf_angle_max,
            'reobs_min_parallax': reobs_min_parallax,
            'optimize_every': optimize_every,
            'landmarks_topic': '/aruco_detections',
            'optimized_landmarks_topic': '/landmarks',
            'legacy_landmarks_topic': '/landmarks_opt',
            'base_debug_topic': '/aruco_base_debug',
            'min_marker_depth': 0.15,
            'max_marker_depth': ParameterValue(max_marker_depth, value_type=float),
            'min_landmark_observations': ParameterValue(
                min_landmark_observations, value_type=int
            ),
            'max_landmark_position_jump': ParameterValue(
                max_landmark_position_jump, value_type=float
            ),
            'camera_tx': camera_tx,
            'camera_ty': camera_ty,
            'camera_yaw': camera_yaw,
            'trajectory_file': trajectory_file,
            'geometry_debug_file': '/tmp/aruco_geometry_debug.csv',
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        calib_arg,
        odom_topic_arg,
        camera_frame_arg,
        kf_dist_arg,
        kf_angle_max_arg,
        reobs_min_parallax_arg,
        optimize_every_arg,
        use_bag_tf_arg,
        bag_tf_topic_arg,
        bag_tf_static_topic_arg,
        camera_tx_arg,
        camera_ty_arg,
        camera_yaw_arg,
        trajectory_file_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        rviz_config_arg,
        min_marker_area_px_arg,
        max_marker_depth_arg,
        max_reprojection_error_px_arg,
        allowed_marker_ids_arg,
        min_landmark_observations_arg,
        max_landmark_position_jump_arg,
        SetParameter(name='use_sim_time', value=use_sim_time),
        tf_bridge_node,
        aruco_node,
        graph_slam_node,
        rviz_node,
    ])
