#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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


def _launch_nodes(context, config, rviz_config):
    profile = resolve_profile(BAG_TB4, robot_namespace=_arg(context, 'robot_namespace'))
    rgb = _override(context, 'rgb_topic', profile.rgb_topic)
    depth = _override(context, 'depth_topic', profile.depth_topic)
    info = _override(context, 'camera_info_topic', profile.camera_info_topic)
    bag_tf_topic = _override(context, 'bag_tf_topic', profile.tf_topic)
    bag_tf_static_topic = _override(
        context, 'bag_tf_static_topic', profile.tf_static_topic)
    validate_tb4_topics(
        profile.robot_namespace,
        rgb_topic=rgb,
        depth_topic=depth,
        camera_info_topic=info,
        bag_tf_topic=bag_tf_topic,
        bag_tf_static_topic=bag_tf_static_topic,
    )
    write_resolved_platform(
        _arg(context, 'artifact_dir') or '/tmp/tp_final_rob',
        profile,
        topics={
            'rgb_topic': rgb,
            'depth_topic': depth,
            'camera_info_topic': info,
            'bag_tf_topic': bag_tf_topic,
            'bag_tf_static_topic': bag_tf_static_topic,
        },
        artifacts={'bag_path': _arg(context, 'bag_path')},
    )

    return [
        Node(
            package='tp_c_mission', executable='red_cone_detector', output='screen',
            parameters=[config, {
                'use_sim_time': True,
                'rgb_topic': rgb,
                'depth_topic': depth,
                'camera_info_topic': info,
            }]),
        Node(
            package='tp_a_slam_aruco', executable='tf_bridge', output='screen',
            parameters=[{
                'use_sim_time': True,
                'bag_tf_topic': bag_tf_topic,
                'bag_tf_static_topic': bag_tf_static_topic,
            }]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('tp_c_mission')
    config = os.path.join(share, 'config', 'parte_c.yaml')
    rviz_config = os.path.join(share, 'config', 'parte_c_bag.rviz')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='tb4_0',
            description='Namespace del TurtleBot4: tb4_0 o tb4_1.'),
        DeclareLaunchArgument('artifact_dir', default_value='/tmp/tp_final_rob'),
        DeclareLaunchArgument('bag_path',
                              default_value='tp_final_ws/bags/laberinto_conos',
                              description='Ruta al rosbag2 de conos'),
        DeclareLaunchArgument('play_bag', default_value='true',
                              description='Reproducir el rosbag desde este launch'),
        DeclareLaunchArgument('rgb_topic', default_value=_UNSET),
        DeclareLaunchArgument('depth_topic', default_value=_UNSET),
        DeclareLaunchArgument('camera_info_topic', default_value=_UNSET),
        DeclareLaunchArgument('bag_tf_topic', default_value=_UNSET),
        DeclareLaunchArgument('bag_tf_static_topic', default_value=_UNSET),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='Abrir RViz para robot, LIDAR, odometría y detector'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--clock'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('play_bag'))),
        OpaqueFunction(function=_launch_nodes, args=[config, rviz_config]),
    ])
