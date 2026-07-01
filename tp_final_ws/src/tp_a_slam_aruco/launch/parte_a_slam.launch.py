#!/usr/bin/env python3
"""
parte_a_slam.launch.py
======================
Lanza el pipeline de SLAM de la Parte A (Opción 3):
  - tf_bridge_node      : repubica el TF del bag a /tf y /tf_static
  - aruco_detector_node : detecta los landmarks ArUco
  - graph_slam_node     : construye y optimiza el grafo
  - rviz2               : vista en tiempo simulado con trayectoria, TF e imagen
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from tp_platform.platform_profiles import (
    BAG_TB4,
    resolve_profile,
    validate_tb4_topics,
    write_resolved_platform,
)


_UNSET = ''


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _override(context, name, default):
    value = _arg(context, name)
    return default if value == _UNSET else value


def _launch_nodes(context, pkg):
    profile = resolve_profile(BAG_TB4, robot_namespace=_arg(context, 'robot_namespace'))
    odom_topic = _override(context, 'odom_topic', profile.odom_topic)
    scan_topic = _override(context, 'scan_topic', profile.scan_topic)
    image_topic = _override(context, 'image_topic', profile.rgb_topic)
    bag_tf_topic = _override(context, 'bag_tf_topic', profile.tf_topic)
    bag_tf_static_topic = _override(
        context, 'bag_tf_static_topic', profile.tf_static_topic)
    validate_tb4_topics(
        profile.robot_namespace,
        odom_topic=odom_topic,
        scan_topic=scan_topic,
        image_topic=image_topic,
        bag_tf_topic=bag_tf_topic,
        bag_tf_static_topic=bag_tf_static_topic,
    )

    calibration_file = LaunchConfiguration('calibration_file')
    prefer_camera_info = LaunchConfiguration('prefer_camera_info')
    camera_frame = LaunchConfiguration('camera_frame')
    kf_dist = LaunchConfiguration('kf_dist')
    kf_angle_max = LaunchConfiguration('kf_angle_max')
    reobs_min_parallax = LaunchConfiguration('reobs_min_parallax')
    optimize_every = LaunchConfiguration('optimize_every')
    use_bag_tf = LaunchConfiguration('use_bag_tf')
    camera_tx = LaunchConfiguration('camera_tx')
    camera_ty = LaunchConfiguration('camera_ty')
    camera_yaw = LaunchConfiguration('camera_yaw')
    trajectory_file = LaunchConfiguration('trajectory_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_log_level = LaunchConfiguration('rviz_log_level')
    min_marker_area_px = LaunchConfiguration('min_marker_area_px')
    min_marker_depth = LaunchConfiguration('min_marker_depth')
    max_marker_depth = LaunchConfiguration('max_marker_depth')
    max_reprojection_error_px = LaunchConfiguration('max_reprojection_error_px')
    allowed_marker_ids = LaunchConfiguration('allowed_marker_ids')
    min_landmark_observations = LaunchConfiguration('min_landmark_observations')
    max_landmark_position_jump = LaunchConfiguration('max_landmark_position_jump')
    detection_keyframe_tolerance = LaunchConfiguration('detection_keyframe_tolerance')
    max_pending_detections = LaunchConfiguration('max_pending_detections')
    max_detection_wait_seconds = LaunchConfiguration('max_detection_wait_seconds')
    max_odom_buffer_samples = LaunchConfiguration('max_odom_buffer_samples')
    maha_threshold = LaunchConfiguration('maha_threshold')
    cauchy_k = LaunchConfiguration('cauchy_k')
    diagnostics_file = _override(
        context, 'diagnostics_file', '/tmp/aruco_detections.csv')
    geometry_debug_file = _override(
        context, 'geometry_debug_file', '/tmp/aruco_geometry_debug.csv')

    write_resolved_platform(
        _arg(context, 'artifact_dir') or '/tmp/tp_final_rob',
        profile,
        stage='parte-a-slam',
        run_id=_arg(context, 'run_id'),
        topics={
            'odom_topic': odom_topic,
            'scan_topic': scan_topic,
            'image_topic': image_topic,
            'bag_tf_topic': bag_tf_topic,
            'bag_tf_static_topic': bag_tf_static_topic,
        },
        frames={'camera_frame': _arg(context, 'camera_frame')},
        artifacts={
            'trajectory_file': _arg(context, 'trajectory_file'),
            'diagnostics_file': diagnostics_file,
            'geometry_debug_file': geometry_debug_file,
        },
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

    aruco_node = Node(
        package='tp_a_slam_aruco',
        executable='aruco_detector',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'calibration_file': calibration_file,
            'prefer_camera_info': ParameterValue(prefer_camera_info, value_type=bool),
            'image_topic': image_topic,
            'marker_length': 0.0889,
            'aruco_dict': 'DICT_4X4_50',
            'camera_frame': camera_frame,
            'detections_topic': '/aruco_detections',
            'debug_image_topic': '/aruco/debug_image',
            'publish_debug_image': True,
            'min_marker_area_px': ParameterValue(min_marker_area_px, value_type=float),
            'min_marker_depth': ParameterValue(min_marker_depth, value_type=float),
            'max_marker_depth': ParameterValue(max_marker_depth, value_type=float),
            'max_reprojection_error_px': ParameterValue(
                max_reprojection_error_px, value_type=float
            ),
            'allowed_marker_ids': allowed_marker_ids,
            'diagnostics_file': diagnostics_file,
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
            'min_marker_depth': ParameterValue(min_marker_depth, value_type=float),
            'max_marker_depth': ParameterValue(max_marker_depth, value_type=float),
            'min_landmark_observations': ParameterValue(
                min_landmark_observations, value_type=int
            ),
            'max_landmark_position_jump': ParameterValue(
                max_landmark_position_jump, value_type=float
            ),
            'detection_keyframe_tolerance': ParameterValue(
                detection_keyframe_tolerance, value_type=float
            ),
            'max_pending_detections': ParameterValue(
                max_pending_detections, value_type=int
            ),
            'max_detection_wait_seconds': ParameterValue(
                max_detection_wait_seconds, value_type=float
            ),
            'max_odom_buffer_samples': ParameterValue(
                max_odom_buffer_samples, value_type=int
            ),
            'maha_threshold': ParameterValue(maha_threshold, value_type=float),
            'cauchy_k': ParameterValue(cauchy_k, value_type=float),
            'camera_tx': camera_tx,
            'camera_ty': camera_ty,
            'camera_yaw': camera_yaw,
            'trajectory_file': trajectory_file,
            'geometry_debug_file': geometry_debug_file,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(launch_rviz),
        arguments=[
            '-d', rviz_config,
            '--ros-args', '--log-level', rviz_log_level,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/scan', scan_topic),
            ('/odom', odom_topic),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    )

    return [tf_bridge_node, aruco_node, graph_slam_node, rviz_node]


def generate_launch_description():
    pkg = get_package_share_directory('tp_a_slam_aruco')
    rviz_default_config = os.path.join(pkg, 'config', 'rviz_config.rviz')
    calibration_default = os.path.join(pkg, 'config', 'camera_tb4_0.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='tb4_0',
            description='Namespace del TurtleBot4: tb4_0 o tb4_1.',
        ),
        DeclareLaunchArgument(
            'artifact_dir',
            default_value='/tmp/tp_final_rob',
            description='Directorio de artefactos donde registrar platform-resolved.yaml.',
        ),
        DeclareLaunchArgument('run_id', default_value='manual'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=calibration_default,
            description='Ruta al YAML con K y coeficientes de distorsión.',
        ),
        DeclareLaunchArgument('prefer_camera_info', default_value='false'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value=_UNSET,
            description='Override de odometría; default: <robot_namespace>/odom.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value=_UNSET,
            description='Override de RGB; default: <robot_namespace>/oakd/rgb/preview/image_raw.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value=_UNSET,
            description='Override de scan; default: <robot_namespace>/scan.',
        ),
        DeclareLaunchArgument('kf_dist', default_value='0.15'),
        DeclareLaunchArgument('kf_angle_max', default_value='0.60'),
        DeclareLaunchArgument('reobs_min_parallax', default_value='0.20'),
        DeclareLaunchArgument('optimize_every', default_value='1'),
        DeclareLaunchArgument('camera_tx', default_value='-0.0596'),
        DeclareLaunchArgument('camera_ty', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_bag_tf', default_value='true'),
        DeclareLaunchArgument(
            'bag_tf_topic',
            default_value=_UNSET,
            description='Override de TF dinámico; default: <robot_namespace>/tf.',
        ),
        DeclareLaunchArgument(
            'bag_tf_static_topic',
            default_value=_UNSET,
            description='Override de TF estático; default: <robot_namespace>/tf_static.',
        ),
        DeclareLaunchArgument(
            'trajectory_file',
            default_value='/tmp/tp_final_rob/trajectory.json',
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='oakd_rgb_camera_optical_frame',
            description='frame_id de cámara usado para transformar observaciones.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_default_config),
        DeclareLaunchArgument('rviz_log_level', default_value='warn'),
        DeclareLaunchArgument('min_marker_area_px', default_value='250.0'),
        DeclareLaunchArgument('min_marker_depth', default_value='0.05'),
        DeclareLaunchArgument('max_marker_depth', default_value='3.0'),
        DeclareLaunchArgument('max_reprojection_error_px', default_value='4.0'),
        DeclareLaunchArgument('allowed_marker_ids', default_value=''),
        DeclareLaunchArgument('min_landmark_observations', default_value='3'),
        DeclareLaunchArgument('max_landmark_position_jump', default_value='0.75'),
        DeclareLaunchArgument('detection_keyframe_tolerance', default_value='0.10'),
        DeclareLaunchArgument('max_pending_detections', default_value='500'),
        DeclareLaunchArgument('max_detection_wait_seconds', default_value='1.0'),
        DeclareLaunchArgument('max_odom_buffer_samples', default_value='4000'),
        DeclareLaunchArgument('maha_threshold', default_value='5.99'),
        DeclareLaunchArgument('cauchy_k', default_value='1.0'),
        DeclareLaunchArgument('diagnostics_file', default_value=_UNSET),
        DeclareLaunchArgument('geometry_debug_file', default_value=_UNSET),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        OpaqueFunction(function=_launch_nodes, args=[pkg]),
    ])
