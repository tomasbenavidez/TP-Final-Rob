#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('turtlebot3_custom_simulation')
    nav_share = get_package_share_directory('tp_b_navigation')
    mission_share = get_package_share_directory('tp_c_mission')
    world = LaunchConfiguration('world')
    auto_start = LaunchConfiguration('auto_start')
    launch_rviz = LaunchConfiguration('launch_rviz')
    config = os.path.join(mission_share, 'config', 'parte_c.yaml')
    rviz_config = os.path.join(mission_share, 'config', 'parte_c_sim.rviz')

    def environment(name, launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', launch_file)),
            launch_arguments={'spawn_robot': 'false'}.items(),
            condition=IfCondition(PythonExpression(["'", world, "' == '", name, "'"])),
        )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'parte_b.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_rviz': launch_rviz,
            'rviz_config': rviz_config,
        }.items())

    model = os.path.join(sim_share, 'worlds', 'TurtleBot3BurgerRGBD', 'model.sdf')
    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'burger_rgbd', '-file', model, '-x', '0', '-y', '0', '-z', '0.01'])

    static_camera_tf = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.06', '0', '0.20', '0', '0', '0', 'base_link', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963',
                        'camera_link', 'camera_optical_frame']),
    ]

    # El modelo TurtleBot3BurgerRGBD incluye el burger como sub-modelo anidado 'base_robot',
    # por lo que Gazebo publica sus frames SCOPEADOS (base_robot::base_footprint,
    # base_robot::base_scan) mientras robot_state_publisher usa nombres PLANOS. Sin estos
    # puentes el árbol del robot nunca conecta con map -> la cámara/LIDAR no se transforman a
    # map y la misión no arranca. Identidades que reconectan ambos árboles:
    #   - diff_drive publica odom->base_robot::base_footprint  =>  puenteamos a base_footprint
    #   - el /scan llega con frame_id base_robot::base_scan     =>  lo colgamos de base_scan
    tf_frame_bridges = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0',
                        'base_robot::base_footprint', 'base_footprint']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0',
                        'base_scan', 'base_robot::base_scan']),
    ]

    # NOTA: (1.65, 1.50) caía en una celda DESCONOCIDA del mapa y detrás de una pared de
    # custom_casa -> el cono quedaba ocluido y en zona no navegable (planner allow_unknown=False),
    # así que la misión nunca lo veía (NOT_FOUND). Reubicado a un punto LIBRE y mapeado del cuarto
    # principal, en la dirección +x donde el robot arranca mirando (línea de visión directa).
    cone_specs = [
        # ('red_cone', 'ConeRed', '1.50', '0.0'),
        ('red_cone', 'ConeRed', '-1.70', '-2.20'),
        ('yellow_cone', 'ConeYellow', '-1.70', '1.45'),
        ('blue_cone', 'ConeBlue', '1.70', '-2.20'),
    ]
    cone_nodes = []
    for entity, folder, x, y in cone_specs:
        cone_nodes.append(Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-entity', entity, '-file',
                       os.path.join(sim_share, 'worlds', folder, 'model.sdf'),
                       '-x', x, '-y', y, '-z', '0']))

    detector = Node(
        package='tp_c_mission', executable='red_cone_detector', output='screen',
        parameters=[config, {'use_sim_time': True}])
    manager = Node(
        package='tp_c_mission', executable='mission_manager', output='screen',
        parameters=[config, {'use_sim_time': True, 'auto_start': auto_start}])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='casa',
                              description='casa o casa_obs'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        environment('casa', 'custom_casa.launch.py'),
        environment('casa_obs', 'custom_casa_obs.launch.py'),
        TimerAction(period=2.0, actions=[spawn_robot] + cone_nodes),
        *static_camera_tf,
        *tf_frame_bridges,
        navigation,
        detector,
        manager,
    ])
