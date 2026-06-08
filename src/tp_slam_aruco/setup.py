from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tp_slam_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Índice de paquetes de ament (obligatorio para que ROS encuentre el paquete)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # El manifiesto
        ('share/' + package_name, ['package.xml']),
        # Launch files: copiamos todos los *.launch.py
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Archivos de configuración (yaml, rviz, calibración de cámara)
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        # Carpeta de salida de corridas (el nodo escribe aquí al terminar)
        (os.path.join('share', package_name, 'runs'),
            glob('runs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo TP Final',
    maintainer_email='',
    description='Graph SLAM con ArUco + LIDAR sobre TurtleBot4 (Opción 3)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Cada línea registra un nodo ejecutable: 'nombre = paquete.modulo:main'
            # Se ejecutan con: ros2 run tp_slam_aruco <nombre>
            'aruco_detector = tp_slam_aruco.aruco_detector_node:main',
            'odometry = tp_slam_aruco.odometry_node:main',
            'graph_slam = tp_slam_aruco.graph_slam_node:main',
            'occupancy_grid = tp_slam_aruco.occupancy_grid_node:main',
        ],
    },
)
