#!/usr/bin/env python3
"""sim_mapping.launch.py — Re-mapeo del entorno SIMULADO (Sistema 3, consigna).

Levanta el nodo `sim_mapper`, que reusa la lógica de mapeo de Parte A
(occupancy_grid_node: modelo de sensor inverso + log-odds) sobre el /scan + TF de
Gazebo, para producir el mapa del entorno simulado que después usa Parte B.

FLUJO (3 terminales, todas con el entorno de Parte B sourceado):
  T1: ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
  T2: ros2 launch tp_b_navigation sim_mapping.launch.py
  T3: ros2 run teleop_twist_keyboard teleop_twist_keyboard
      -> manejar el TB3 por TODA la casa (incl. bajar por el corredor ESTE x≈2.4
         al sur). Se puede ver el mapa crecer en RViz (display Map sobre /map).
  Al terminar de recorrer: Ctrl+C en T2 -> exporta <map_output>.pgm + .yaml

El mapa se publica en /map (latcheado) para inspección en vivo. Por defecto se
exporta a mapas/map_sim.{pgm,yaml} del repo (no pisa el map.yaml del profe).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_out = os.path.join(
        os.path.expanduser('~'),
        'Documents', 'GitHub', 'TP-Final-Rob', 'mapas', 'map_sim')

    map_output = LaunchConfiguration('map_output')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mapping_frame = LaunchConfiguration('mapping_frame')

    args = [
        DeclareLaunchArgument('map_output', default_value=default_out,
                              description='Prefijo de salida (.pgm + .yaml).'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mapping_frame', default_value='odom',
                              description='Frame de mapeo (odom ≈ ground-truth en Gazebo).'),
    ]

    sim_mapper = Node(
        package='tp_b_navigation', executable='sim_mapper', name='sim_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_output': map_output,
            'mapping_frame': mapping_frame,
            'publish_period_sec': 1.0,   # publica /map para ver el mapeo en vivo (RViz)
            'log_every': 300,
        }])

    return LaunchDescription(args + [sim_mapper])
