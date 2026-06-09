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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_default_config = os.path.join(
        get_package_share_directory('tp_slam_aruco'),
        'config',
        'rviz_config.rviz',
    )

    calib_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='',
        description='Ruta al YAML con K y coeficientes de distorsión (TB4 #0)',
    )
    allow_yaml_fallback_arg = DeclareLaunchArgument(
        'allow_yaml_fallback',
        default_value='false',
        description='Permitir usar el YAML si todavía no llegó camera_info.',
    )
    allow_fallback_tf_arg = DeclareLaunchArgument(
        'allow_fallback_tf',
        default_value='false',
        description='Permitir extrínsecos numéricos si no está disponible TF cámara->base.',
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
        default_value='0.12',
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
            get_package_share_directory('tp_slam_aruco'), 'runs', 'trayectoria.json'),
        description='Ruta JSON donde guardar la trayectoria optimizada al finalizar (para Fase 5)',
    )
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='',
        description='Override opcional del frame_id de cámara. '
                    'Por defecto se usa image.header.frame_id.',
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
    diagnostics_window_frames_arg = DeclareLaunchArgument(
        'diagnostics_window_frames',
        default_value='50',
        description='Cantidad de frames usada para resumir detecciones ArUco.',
    )
    diagnostics_topic_arg = DeclareLaunchArgument(
        'diagnostics_topic',
        default_value='/landmark_detection_stats',
        description='Tópico diagnóstico con [raw, valid, valid_unique] por frame.',
    )
    min_visual_landmarks_arg = DeclareLaunchArgument(
        'min_visual_landmarks',
        default_value='2',
        description='Cantidad mínima de landmarks válidos para crear keyframes visuales.',
    )
    min_candidate_observations_arg = DeclareLaunchArgument(
        'min_candidate_observations',
        default_value='2',
        description='Observaciones necesarias para confirmar un landmark nuevo.',
    )
    max_candidate_reprojection_arg = DeclareLaunchArgument(
        'max_candidate_reprojection_error_px',
        default_value='4.0',
        description='Error de reproyección máximo promedio para confirmar un landmark.',
    )
    min_marker_area_arg = DeclareLaunchArgument(
        'min_marker_area_px2',
        default_value='120.0',
        description='Área mínima del tag en píxeles cuadrados para aceptarlo.',
    )
    max_candidate_buffer_arg = DeclareLaunchArgument(
        'max_candidate_buffer_observations',
        default_value='10',
        description='Máximo de observaciones acumuladas antes de resetear un candidato.',
    )
    candidate_reset_arg = DeclareLaunchArgument(
        'candidate_reset_on_bad_observation',
        default_value='true',
        description='Resetear candidatos si entra una observación mala.',
    )
    min_candidate_pose_sep_arg = DeclareLaunchArgument(
        'min_candidate_pose_separation_m',
        default_value='0.08',
        description='Separación mínima entre poses para confirmar un candidato.',
    )
    max_obs_age_new_arg = DeclareLaunchArgument(
        'max_observation_age_s_for_new_landmark',
        default_value='0.20',
        description='Edad máxima de observación para sembrar un landmark nuevo.',
    )
    max_obs_age_reobs_arg = DeclareLaunchArgument(
        'max_observation_age_s_for_reobservation',
        default_value='0.25',
        description='Edad máxima de observación para reobservaciones.',
    )
    min_landmark_clearance_arg = DeclareLaunchArgument(
        'min_landmark_clearance_m',
        default_value='0.20',
        description='Distancia mínima razonable entre trayectoria y landmark.',
    )

    calibration_file = LaunchConfiguration('calibration_file')
    allow_yaml_fallback = LaunchConfiguration('allow_yaml_fallback')
    allow_fallback_tf = LaunchConfiguration('allow_fallback_tf')
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
    diagnostics_window_frames = LaunchConfiguration('diagnostics_window_frames')
    diagnostics_topic = LaunchConfiguration('diagnostics_topic')
    min_visual_landmarks = LaunchConfiguration('min_visual_landmarks')
    min_candidate_observations = LaunchConfiguration('min_candidate_observations')
    max_candidate_reprojection = LaunchConfiguration('max_candidate_reprojection_error_px')
    min_marker_area_px2 = LaunchConfiguration('min_marker_area_px2')
    max_candidate_buffer_observations = LaunchConfiguration('max_candidate_buffer_observations')
    candidate_reset_on_bad_observation = LaunchConfiguration('candidate_reset_on_bad_observation')
    min_candidate_pose_separation_m = LaunchConfiguration('min_candidate_pose_separation_m')
    max_observation_age_s_for_new_landmark = LaunchConfiguration('max_observation_age_s_for_new_landmark')
    max_observation_age_s_for_reobservation = LaunchConfiguration('max_observation_age_s_for_reobservation')
    min_landmark_clearance_m = LaunchConfiguration('min_landmark_clearance_m')

    tf_bridge_node = Node(
        package='tp_slam_aruco',
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
        package='tp_slam_aruco',
        executable='aruco_detector',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'calibration_file': calibration_file,
            'allow_yaml_fallback': allow_yaml_fallback,
            'allow_fallback_tf': allow_fallback_tf,
            'image_topic': 'tb4_0/oakd/rgb/preview/image_raw',
            'marker_length': 0.0889,
            'aruco_dict': 'DICT_4X4_50',
            'camera_frame': camera_frame,
            'base_frame': 'base_link',
            'raw_landmarks_topic': '/landmarks_raw',
            'accepted_landmarks_topic': '/landmarks_accepted',
            'observations_topic': '/landmark_observations',
            'diagnostics_window_frames': diagnostics_window_frames,
            'diagnostics_topic': diagnostics_topic,
            'min_marker_area_px2': ParameterValue(
                min_marker_area_px2, value_type=float
            ),
            'camera_tx': camera_tx,
            'camera_ty': camera_ty,
            'camera_yaw': camera_yaw,
        }],
    )

    graph_slam_node = Node(
        package='tp_slam_aruco',
        executable='graph_slam',
        name='graph_slam_node',
        output='screen',
        parameters=[{
            'odom_topic': 'tb4_0/odom',
            'observations_topic': '/landmark_observations',
            'kf_dist': kf_dist,
            'kf_angle_max': kf_angle_max,
            'reobs_min_parallax': reobs_min_parallax,
            'optimize_every': optimize_every,
            'camera_tx': camera_tx,
            'camera_ty': camera_ty,
            'camera_yaw': camera_yaw,
            'trajectory_file': trajectory_file,
            'visual_diagnostics_topic': diagnostics_topic,
            'min_visual_landmarks': min_visual_landmarks,
            'min_candidate_observations': min_candidate_observations,
            'max_candidate_reprojection_error_px': max_candidate_reprojection,
            'max_candidate_buffer_observations': max_candidate_buffer_observations,
            'candidate_reset_on_bad_observation': candidate_reset_on_bad_observation,
            'min_candidate_pose_separation_m': min_candidate_pose_separation_m,
            'max_observation_age_s_for_new_landmark': max_observation_age_s_for_new_landmark,
            'max_observation_age_s_for_reobservation': max_observation_age_s_for_reobservation,
            'min_landmark_clearance_m': min_landmark_clearance_m,
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
        allow_yaml_fallback_arg,
        allow_fallback_tf_arg,
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
        diagnostics_window_frames_arg,
        diagnostics_topic_arg,
        min_visual_landmarks_arg,
        min_candidate_observations_arg,
        max_candidate_reprojection_arg,
        min_marker_area_arg,
        max_candidate_buffer_arg,
        candidate_reset_arg,
        min_candidate_pose_sep_arg,
        max_obs_age_new_arg,
        max_obs_age_reobs_arg,
        min_landmark_clearance_arg,
        SetParameter(name='use_sim_time', value=use_sim_time),
        tf_bridge_node,
        aruco_node,
        graph_slam_node,
        rviz_node,
    ])
