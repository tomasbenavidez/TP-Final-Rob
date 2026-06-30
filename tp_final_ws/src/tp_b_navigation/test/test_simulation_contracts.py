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
        from tp_b_navigation.platform_profiles import resolve_profile

        profile = resolve_profile('simulation_tb3')
        source = (PACKAGE_ROOT / 'launch' / 'parte_b.launch.py').read_text()
        self.assertIn(
            "DeclareLaunchArgument('profile', default_value='simulation_tb3'",
            source)
        self.assertEqual(profile.odom_topic, '/calc_odom')
        self.assertEqual(profile.reference_odom_topic, '/odom')
        self.assertEqual(profile.base_frame, 'base_footprint')
        self.assertTrue(profile.launch_virtual_landmarks)
        self.assertIn("DeclareLaunchArgument('base_frame'", source)
        self.assertIn("DeclareLaunchArgument('reference_odom_topic'", source)
        self.assertIn("'motion_odom_topic': odom_topic", source)
        self.assertIn("'truth_frame': truth_odom_frame", source)
        self.assertIn("profile.launch_virtual_landmarks", source)

    def test_part_b_real_profile_uses_aruco_observations_without_virtual_sensor(self):
        source = (PACKAGE_ROOT / 'launch' / 'parte_b.launch.py').read_text()

        self.assertIn("DeclareLaunchArgument('landmark_map_file'", source)
        self.assertIn("'landmark_map_file': landmark_map_file", source)
        self.assertIn("profile.landmark_source == 'aruco'", source)
        self.assertIn("executable='aruco_detector'", source)
        self.assertIn("executable='aruco_mcl_adapter'", source)
        self.assertIn('landmark_source=aruco', source)
        self.assertNotIn("profile.landmark_source == 'aruco' and profile.launch_virtual_landmarks", source)

    def test_part_b_exposes_real_safety_gate_parameters(self):
        source = (PACKAGE_ROOT / 'launch' / 'parte_b.launch.py').read_text()

        for argument in (
            'enable_safety_gates',
            'max_mcl_pose_age',
            'max_scan_age',
            'max_monitor_age',
            'max_position_covariance',
            'max_yaw_covariance',
        ):
            self.assertIn(f"DeclareLaunchArgument('{argument}'", source)
            self.assertIn(f"'{argument}':", source)

    def test_part_b_declares_runtime_dependencies_for_real_aruco_profile(self):
        package_xml = (PACKAGE_ROOT / 'package.xml').read_text()

        self.assertIn('<exec_depend>tp_a_slam_aruco</exec_depend>', package_xml)
        self.assertNotIn('<exec_depend>tp_c_mission</exec_depend>', package_xml)

    def test_part_b_owns_aruco_mcl_adapter(self):
        setup = (PACKAGE_ROOT / 'setup.py').read_text()
        adapter = PACKAGE_ROOT / 'tp_b_navigation' / 'aruco_mcl_adapter.py'
        launch = (PACKAGE_ROOT / 'launch' / 'parte_b.launch.py').read_text()

        self.assertTrue(adapter.is_file())
        self.assertIn(
            'aruco_mcl_adapter = tp_b_navigation.aruco_mcl_adapter:main', setup)
        self.assertIn(
            "package='tp_b_navigation', executable='aruco_mcl_adapter'", launch)

    def test_state_machine_subscribes_to_scan_and_monitor_health(self):
        source = (PACKAGE_ROOT / 'tp_b_navigation' / 'state_machine.py').read_text()

        self.assertIn("LaserScan, '/scan'", source)
        self.assertIn("Bool, '/obstacle_monitor_healthy'", source)
        self.assertIn('scan_gate(', source)
        self.assertIn('monitor_gate(', source)

    def test_new_plan_request_invalidates_previous_plan_before_safety_check(self):
        source = (PACKAGE_ROOT / 'tp_b_navigation' / 'state_machine.py').read_text()
        request_plan = source[
            source.index('    def request_plan(self):'):
            source.index('    # --------------------------------------------------------------- bucle FSM')
        ]

        self.assertLess(
            request_plan.index('self.plan_ok = None'),
            request_plan.index('if not self.navigation_safe():'))
        self.assertLess(
            request_plan.index('self.path = None'),
            request_plan.index('if not self.navigation_safe():'))

    def test_c_real_keeps_tb4_odometry_default(self):
        from tp_b_navigation.platform_profiles import resolve_profile

        profile = resolve_profile('real_tb4')
        source = (
            REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_c_mission' / 'launch'
            / 'parte_c_real.launch.py').read_text()
        self.assertIn(
            "DeclareLaunchArgument('profile', default_value='real_tb4'",
            source)
        self.assertEqual(profile.odom_topic, '/tb4_0/odom')
        self.assertEqual(profile.reference_odom_topic, '/tb4_0/odom')
        self.assertIn("DeclareLaunchArgument('robot_namespace'", source)
        self.assertIn('robot_namespace=_arg(context, ', source)

    def test_c_real_resolves_tb4_1_odometry(self):
        from tp_b_navigation.platform_profiles import resolve_profile

        profile = resolve_profile('real_tb4', robot_namespace='tb4_1')

        self.assertEqual(profile.odom_topic, '/tb4_1/odom')
        self.assertEqual(profile.reference_odom_topic, '/tb4_1/odom')

    def test_part_a_exposes_tb4_odometry_default(self):
        launch_dir = (
            REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_a_slam_aruco' / 'launch')
        for filename in ('parte_a_slam.launch.py', 'parte_a_mapa.launch.py'):
            source = (launch_dir / filename).read_text()
            self.assertIn("DeclareLaunchArgument(\n            'robot_namespace'", source, filename)
            self.assertIn("default_value='tb4_0'", source, filename)
            self.assertRegex(
                source, re.compile(r"'odom_topic':\s+odom_topic"), filename)

    def test_part_a_mapping_exposes_tb4_scan_default(self):
        source = (
            REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_a_slam_aruco' / 'launch'
            / 'parte_a_mapa.launch.py').read_text()
        self.assertIn("default: <robot_namespace>/scan", source)
        self.assertRegex(source, re.compile(r"'scan_topic':\s+scan_topic"))

    def test_obstacle_monitor_projects_scan_frame_through_tf(self):
        source = (PACKAGE_ROOT / 'tp_b_navigation' / 'obstacle_monitor.py').read_text()
        self.assertIn('self.base_frame, sensor_frame, stamp', source)
        self.assertIn('self.global_frame, sensor_frame, stamp', source)
        self.assertIn('transform_scan_points(', source)
        self.assertIn('scans_skipped_no_tf', source)
        self.assertIn('points_rejected_invalid_ranges', source)

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
