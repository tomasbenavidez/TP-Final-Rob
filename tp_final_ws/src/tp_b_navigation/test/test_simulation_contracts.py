#!/usr/bin/env python3
import ast
import re
import sys
import unittest
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_b_navigation'
sys.path.insert(0, str(PACKAGE_ROOT))


class SimulationContractsTest(unittest.TestCase):
    def test_virtual_map_contains_sixty_unique_landmarks(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / 'config' / 'landmarks.yaml').read_text())
        flat = config['landmark_publisher']['ros__parameters']['landmarks']
        points = list(zip(flat[::2], flat[1::2]))
        self.assertEqual(len(points), 60)
        self.assertEqual(len(set(points)), 60)

    def test_part_b_defaults_to_calculated_odom_and_ground_truth_sensor(self):
        source = (PACKAGE_ROOT / 'launch' / 'parte_b.launch.py').read_text()
        self.assertIn("DeclareLaunchArgument('odom_topic', default_value='/calc_odom')",
                      source)
        self.assertIn(
            "DeclareLaunchArgument('truth_odom_topic', default_value='/odom')",
            source)
        self.assertIn("'motion_odom_topic': odom_topic", source)
        self.assertIn("'truth_frame': truth_odom_frame", source)

    def test_c_real_keeps_tb4_odometry_default(self):
        source = (
            REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_c_mission' / 'launch'
            / 'parte_c_real.launch.py').read_text()
        self.assertIn(
            "DeclareLaunchArgument('odom_topic', default_value='/tb4_0/odom')",
            source)

    def test_part_a_exposes_tb4_odometry_default(self):
        launch_dir = (
            REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_a_slam_aruco' / 'launch')
        for filename in ('parte_a_slam.launch.py', 'parte_a_mapa.launch.py'):
            source = (launch_dir / filename).read_text()
            self.assertIn("default_value='/tb4_0/odom'", source, filename)
            self.assertRegex(
                source, re.compile(r"'odom_topic':\s+odom_topic"), filename)

    def test_c_sim_explicitly_selects_calculated_odom(self):
        source = (
            REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_c_mission' / 'launch'
            / 'parte_c_sim.launch.py').read_text()
        self.assertIn(
            "DeclareLaunchArgument('odom_topic', default_value='/calc_odom')",
            source)
        self.assertIn(
            "DeclareLaunchArgument('truth_odom_topic', default_value='/odom')",
            source)
        self.assertIn("'odom_topic': odom_topic", source)
        self.assertIn("'camera_frame': 'camera_link'", source)

    def test_casa_launches_start_calculated_odometry(self):
        launch_dir = (
            REPO_ROOT / 'tp_final_ws' / 'src'
            / 'turtlebot3_custom_simulation' / 'launch')
        for filename in ('custom_casa.launch.py', 'custom_casa_obs.launch.py'):
            source = (launch_dir / filename).read_text()
            ast.parse(source)
            self.assertIn(
                "executable='turtlebot3_custom_simulation'", source,
                filename)
            self.assertIn("'publish_tf': False", source, filename)


if __name__ == '__main__':
    unittest.main()
