from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'tp_c_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo TP Final',
    maintainer_email='equipo@udesa.edu.ar',
    description='Parte C: exploración informativa y búsqueda de conos rojos',
    license='MIT',
    entry_points={'console_scripts': [
        'red_cone_detector = tp_c_mission.cone_detector_node:main',
        'mission_manager = tp_c_mission.mission_manager_node:main',
    ]},
)
