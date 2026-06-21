from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tp_b_navigation'

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
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Configuración: params yaml, rviz
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo TP Final',
    maintainer_email='equipo@udesa.edu.ar',
    description='Parte B: navegación autónoma (Sistema 3, grilla + landmarks de cámara virtuales)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 'nombre = paquete.modulo:main'  ->  ros2 run tp_b_navigation <nombre>
            'map_loader = tp_b_navigation.map_loader:main',
            'landmark_publisher = tp_b_navigation.landmark_publisher:main',
            'landmark_sensor = tp_b_navigation.landmark_sensor:main',
            'mcl_localization = tp_b_navigation.mcl_localization:main',
            'global_planner = tp_b_navigation.global_planner:main',
            'obstacle_monitor = tp_b_navigation.obstacle_monitor:main',
            'state_machine = tp_b_navigation.state_machine:main',
        ],
    },
)
