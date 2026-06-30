#!/usr/bin/env python3
"""
parte_a_mapa.launch.py
======================
Segunda pasada del pipeline SLAM (Opción 3): genera el mapa de grilla de
ocupación proyectando los scans LIDAR sobre la trayectoria ya corregida.
"""

import math
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


def _launch_nodes(context):
    profile = resolve_profile(BAG_TB4, robot_namespace=_arg(context, 'robot_namespace'))
    odom_topic = _override(context, 'odom_topic', profile.odom_topic)
    scan_topic = _override(context, 'scan_topic', profile.scan_topic)
    bag_tf_topic = _override(context, 'bag_tf_topic', profile.tf_topic)
    bag_tf_static_topic = _override(
        context, 'bag_tf_static_topic', profile.tf_static_topic)
    validate_tb4_topics(
        profile.robot_namespace,
        odom_topic=odom_topic,
        scan_topic=scan_topic,
        bag_tf_topic=bag_tf_topic,
        bag_tf_static_topic=bag_tf_static_topic,
    )

    write_resolved_platform(
        _arg(context, 'artifact_dir') or '/tmp/tp_final_rob',
        profile,
        topics={
            'odom_topic': odom_topic,
            'scan_topic': scan_topic,
            'bag_tf_topic': bag_tf_topic,
            'bag_tf_static_topic': bag_tf_static_topic,
        },
        frames={'base_frame': _arg(context, 'base_frame')},
        artifacts={
            'trajectory_file': _arg(context, 'trajectory_file'),
            'map_output': _arg(context, 'map_output'),
        },
    )

    occupancy_node = Node(
        package='tp_a_slam_aruco',
        executable='occupancy_grid',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'trajectory_file': LaunchConfiguration('trajectory_file'),
            'map_output': LaunchConfiguration('map_output'),
            'resolution': LaunchConfiguration('resolution'),
            'scan_topic': scan_topic,
            'odom_topic': odom_topic,
            'base_frame': LaunchConfiguration('base_frame'),
            'publish_every': 50,
            # Extrinsecos LIDAR del TF estatico del bag:
            #   base_link -> shell_link (yaw 0) -> rplidar_link (tx=-0.04, yaw=+90deg)
            'lidar_tx': -0.04,
            'lidar_ty': 0.0,
            'lidar_yaw': math.pi / 2,
            'lidar_fallback_enabled': ParameterValue(
                LaunchConfiguration('lidar_fallback_enabled'), value_type=bool),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'max_odom_buffer_samples': LaunchConfiguration('max_odom_buffer_samples'),
            'max_pending_scans': LaunchConfiguration('max_pending_scans'),
            'max_scan_wait_seconds': LaunchConfiguration('max_scan_wait_seconds'),
        }],
    )

    tf_bridge_node = Node(
        package='tp_a_slam_aruco',
        executable='tf_bridge',
        name='tf_bridge_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_bag_tf')),
        parameters=[{
            'bag_tf_topic': bag_tf_topic,
            'bag_tf_static_topic': bag_tf_static_topic,
            'tf_topic': '/tf',
            'tf_static_topic': '/tf_static',
        }],
    )

    return [tf_bridge_node, occupancy_node]


def generate_launch_description():
    pkg = get_package_share_directory('tp_a_slam_aruco')

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
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
        DeclareLaunchArgument(
            'trajectory_file',
            default_value=os.path.join(pkg, 'runs', 'trayectoria.json'),
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value=_UNSET,
            description='Override de odometría; default: <robot_namespace>/odom.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value=_UNSET,
            description='Override de scan; default: <robot_namespace>/scan.',
        ),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_fallback_enabled', default_value='true'),
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
            'map_output',
            default_value=os.path.join(pkg, 'runs', 'mapa'),
        ),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('max_angular_velocity', default_value='0.0'),
        DeclareLaunchArgument('max_odom_buffer_samples', default_value='4000'),
        DeclareLaunchArgument('max_pending_scans', default_value='500'),
        DeclareLaunchArgument('max_scan_wait_seconds', default_value='1.0'),
        OpaqueFunction(function=_launch_nodes),
    ])
