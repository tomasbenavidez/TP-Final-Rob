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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tp_b_navigation.platform_profiles import resolve_profile, supported_profiles


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
    profile = resolve_profile(_arg(context, 'profile'))
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
    camera_frame = _override(context, 'camera_frame', profile.camera_frame)
    use_sim_time = _bool_override(context, 'use_sim_time', profile.use_sim_time)
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    common = {'use_sim_time': use_sim_time}
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
        }, common], remappings=common_remaps)

    planner = Node(
        package='tp_b_navigation', executable='global_planner',
        name='global_planner', output='screen',
        parameters=[{'base_frame': robot_frame}, common])

    monitor = Node(
        package='tp_b_navigation', executable='obstacle_monitor',
        name='obstacle_monitor', output='screen',
        parameters=[{'base_frame': robot_frame}, common],
        remappings=common_remaps)

    sm = Node(
        package='tp_b_navigation', executable='state_machine',
        name='state_machine', output='screen',
        parameters=[{'base_frame': robot_frame}, common],
        remappings=common_remaps)

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[common],
        condition=IfCondition(launch_rviz), output='screen')

    return [
        map_loader, *virtual_landmark_nodes, mcl, planner, monitor, sm, rviz,
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
        DeclareLaunchArgument('camera_frame', default_value=_UNSET),
        DeclareLaunchArgument('use_sim_time', default_value=_UNSET),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
    ]

    return LaunchDescription(args + [
        OpaqueFunction(function=_launch_nodes,
                       args=[pkg_share, landmarks_yaml]),
    ])
