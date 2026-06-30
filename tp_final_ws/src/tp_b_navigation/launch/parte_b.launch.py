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
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tp_platform.platform_profiles import (
    resolve_profile,
    supported_profiles,
    validate_tb4_topics,
    write_resolved_platform,
)


_UNSET = ''


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _override(context, name, default):
    value = _arg(context, name)
    return default if value == _UNSET else value


def _first_override(context, names, default):
    for name in names:
        value = _arg(context, name)
        if value != _UNSET:
            return value
    return default


def _bool_override(context, name, default):
    value = _arg(context, name)
    if value == _UNSET:
        return default
    return value.lower() in ('1', 'true', 'yes', 'on')


def _launch_nodes(context, pkg_share, landmarks_yaml):
    profile = resolve_profile(
        _arg(context, 'profile'),
        robot_namespace=_arg(context, 'robot_namespace'))
    map_yaml = _override(
        context, 'map_yaml', os.path.join(pkg_share, 'maps', profile.map_yaml))
    robot_frame = _first_override(
        context, ('base_frame', 'robot_frame'), profile.base_frame)
    odom_frame = _override(context, 'odom_frame', profile.odom_frame)
    odom_topic = _override(context, 'odom_topic', profile.odom_topic)
    truth_odom_topic = _first_override(
        context, ('reference_odom_topic', 'truth_odom_topic'),
        profile.reference_odom_topic)
    truth_odom_frame = _override(context, 'truth_odom_frame', odom_frame)
    scan_topic = _override(context, 'scan_topic', profile.scan_topic)
    cmd_vel_topic = _override(context, 'cmd_vel_topic', profile.cmd_vel_topic)
    rgb_topic = _override(context, 'rgb_topic', profile.rgb_topic)
    camera_frame = _override(context, 'camera_frame', profile.camera_frame)
    use_sim_time = _bool_override(context, 'use_sim_time', profile.use_sim_time)
    landmark_map_file = _arg(context, 'landmark_map_file')
    enable_safety_gates = _bool_override(
        context, 'enable_safety_gates', profile.profile == 'real_tb4')
    max_mcl_pose_age = float(_arg(context, 'max_mcl_pose_age'))
    max_scan_age = float(_arg(context, 'max_scan_age'))
    max_monitor_age = float(_arg(context, 'max_monitor_age'))
    max_position_covariance = float(_arg(context, 'max_position_covariance'))
    max_yaw_covariance = float(_arg(context, 'max_yaw_covariance'))
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    if profile.robot_namespace:
        validate_tb4_topics(
            profile.robot_namespace,
            odom_topic=odom_topic,
            reference_odom_topic=truth_odom_topic,
            scan_topic=scan_topic,
            cmd_vel_topic=cmd_vel_topic,
            rgb_topic=rgb_topic,
        )

    write_resolved_platform(
        _arg(context, 'artifact_dir') or '/tmp/tp_final_rob',
        profile,
        topics={
            'odom_topic': odom_topic,
            'reference_odom_topic': truth_odom_topic,
            'scan_topic': scan_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'rgb_topic': rgb_topic,
        },
        frames={
            'base_frame': robot_frame,
            'odom_frame': odom_frame,
            'truth_odom_frame': truth_odom_frame,
            'camera_frame': camera_frame,
        },
        artifacts={
            'map_yaml': map_yaml,
            'landmark_map_file': landmark_map_file,
        },
    )

    common = {'use_sim_time': use_sim_time}
    monitor_safety_params = {
        'enable_safety_gates': enable_safety_gates,
        'max_mcl_pose_age': max_mcl_pose_age,
        'max_scan_age': max_scan_age,
        'max_position_covariance': max_position_covariance,
        'max_yaw_covariance': max_yaw_covariance,
    }
    state_machine_safety_params = {
        **monitor_safety_params,
        'max_monitor_age': max_monitor_age,
    }
    common_remaps = [
        ('/odom', odom_topic),
        ('/scan', scan_topic),
        ('/cmd_vel', cmd_vel_topic),
    ]

    map_loader = Node(
        package='tp_b_navigation', executable='map_loader', name='map_loader',
        output='screen', parameters=[{'map_yaml': map_yaml}, common])

    virtual_landmark_nodes = []
    if profile.launch_virtual_landmarks:
        virtual_landmark_nodes = [
            Node(
                package='tp_b_navigation', executable='landmark_publisher',
                name='landmark_publisher', output='screen',
                parameters=[landmarks_yaml, common]),
            Node(
                package='tp_b_navigation', executable='landmark_sensor',
                name='landmark_sensor', output='screen',
                parameters=[{
                    'robot_frame': robot_frame,
                    'truth_frame': truth_odom_frame,
                    'camera_frame': camera_frame,
                }, common], remappings=common_remaps),
        ]

    mcl = Node(
        package='tp_b_navigation', executable='mcl_localization',
        name='mcl_localization', output='screen',
        parameters=[{
            'base_frame': robot_frame,
            'odom_frame': odom_frame,
            'motion_odom_topic': odom_topic,
            'reference_odom_topic': truth_odom_topic,
            'landmark_map_file': landmark_map_file,
        }, common], remappings=common_remaps)

    planner = Node(
        package='tp_b_navigation', executable='global_planner',
        name='global_planner', output='screen',
        parameters=[{'base_frame': robot_frame}, common])

    monitor = Node(
        package='tp_b_navigation', executable='obstacle_monitor',
        name='obstacle_monitor', output='screen',
        parameters=[{'base_frame': robot_frame}, monitor_safety_params, common],
        remappings=common_remaps)

    sm = Node(
        package='tp_b_navigation', executable='state_machine',
        name='state_machine', output='screen',
        parameters=[{'base_frame': robot_frame}, state_machine_safety_params, common],
        remappings=common_remaps)

    aruco_nodes = []
    if profile.landmark_source == 'aruco':
        aruco_nodes = [
            Node(
                package='tp_a_slam_aruco', executable='aruco_detector',
                name='aruco_detector', output='screen',
                parameters=[{
                    'image_topic': rgb_topic,
                    'camera_frame': camera_frame,
                }, common]),
            Node(
                package='tp_b_navigation', executable='aruco_mcl_adapter',
                name='aruco_mcl_adapter', output='screen',
                parameters=[{'base_frame': robot_frame}, common]),
        ]

    warnings = []
    default_map = os.path.join(pkg_share, 'maps', profile.map_yaml)
    if profile.profile == 'real_tb4' and map_yaml == default_map:
        warnings.append(LogInfo(
            msg='WARNING: profile real_tb4 is using the default map_yaml; '
                'pass the Parte A map.yaml for lab navigation.'))
    if profile.profile == 'real_tb4' and not landmark_map_file:
        warnings.append(LogInfo(
            msg='WARNING: profile real_tb4 has empty landmark_map_file; '
                'landmark_source=aruco will not correct MCL by ArUco IDs.'))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[common],
        condition=IfCondition(launch_rviz), output='screen')

    return [
        *warnings, map_loader, *virtual_landmark_nodes, mcl, planner,
        monitor, sm, *aruco_nodes, rviz,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('tp_b_navigation')
    # Default: mapa del entorno SIMULADO generado con sim_mapper (ver 05_remapeo_sim.md).
    # Para volver al mapa del profe: map_yaml:=.../mapas/map.yaml
    default_map = os.path.join(pkg_share, 'maps', 'map_sim.yaml')
    landmarks_yaml = os.path.join(pkg_share, 'config', 'landmarks.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'parte_b.rviz')

    args = [
        DeclareLaunchArgument('profile', default_value='simulation_tb3',
                              choices=list(supported_profiles())),
        DeclareLaunchArgument(
            'robot_namespace', default_value='tb4_0',
            description='Namespace del TurtleBot4 para perfiles bag_tb4/real_tb4.'),
        DeclareLaunchArgument(
            'artifact_dir', default_value='/tmp/tp_final_rob',
            description='Directorio de artefactos donde registrar platform-resolved.yaml.'),
        DeclareLaunchArgument('map_yaml', default_value=_UNSET,
                              description=(
                                  'Override del mapa; default de perfil: '
                                  f'{default_map}')),
        DeclareLaunchArgument('robot_frame', default_value=_UNSET),
        DeclareLaunchArgument('base_frame', default_value=_UNSET),
        DeclareLaunchArgument('odom_frame', default_value=_UNSET),
        DeclareLaunchArgument('odom_topic', default_value=_UNSET),
        DeclareLaunchArgument('truth_odom_topic', default_value=_UNSET),
        DeclareLaunchArgument('reference_odom_topic', default_value=_UNSET),
        DeclareLaunchArgument('truth_odom_frame', default_value=_UNSET),
        DeclareLaunchArgument('scan_topic', default_value=_UNSET),
        DeclareLaunchArgument('cmd_vel_topic', default_value=_UNSET),
        DeclareLaunchArgument('rgb_topic', default_value=_UNSET),
        DeclareLaunchArgument('camera_frame', default_value=_UNSET),
        DeclareLaunchArgument('landmark_map_file', default_value='',
                              description='JSON de trayectoria+landmarks producido por Parte A.'),
        DeclareLaunchArgument('enable_safety_gates', default_value=_UNSET,
                              description='Default true en real_tb4, false en otros perfiles.'),
        DeclareLaunchArgument('max_mcl_pose_age', default_value='1.0'),
        DeclareLaunchArgument('max_scan_age', default_value='1.0'),
        DeclareLaunchArgument('max_monitor_age', default_value='1.0'),
        DeclareLaunchArgument('max_position_covariance', default_value='0.25'),
        DeclareLaunchArgument('max_yaw_covariance', default_value='0.5'),
        DeclareLaunchArgument('use_sim_time', default_value=_UNSET),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
    ]

    return LaunchDescription(args + [
        OpaqueFunction(function=_launch_nodes,
                       args=[pkg_share, landmarks_yaml]),
    ])
