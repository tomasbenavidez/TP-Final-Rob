from pathlib import Path
import re
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]


class ParteCContractsTest(unittest.TestCase):
    def test_package_exposes_required_nodes_and_launch_profiles(self):
        setup = (PACKAGE_ROOT / 'setup.py').read_text()
        for executable in ('red_cone_detector', 'mission_manager'):
            self.assertIn(executable, setup)
        self.assertNotIn('aruco_mcl_adapter', setup)
        for profile in ('parte_c_sim.launch.py', 'parte_c_bag.launch.py',
                        'parte_c_real.launch.py'):
            self.assertTrue((PACKAGE_ROOT / 'launch' / profile).is_file())

    def test_state_machine_remains_only_navigation_cmd_vel_owner(self):
        package = REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_b_navigation' / 'tp_b_navigation'
        producers = []
        for path in package.glob('*.py'):
            if "create_publisher(Twist, '/cmd_vel'" in path.read_text():
                producers.append(path.name)
        self.assertEqual(producers, ['state_machine.py'])

    def test_mission_package_never_publishes_cmd_vel(self):
        for path in (PACKAGE_ROOT / 'tp_c_mission').glob('*.py'):
            with self.subTest(path=path.name):
                self.assertNotIn("create_publisher(Twist, '/cmd_vel'",
                                 path.read_text())

    def test_mission_navigation_interfaces_are_explicit(self):
        source = (REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_b_navigation' /
                  'tp_b_navigation' / 'state_machine.py').read_text()
        for contract in ('/mission_goal', '/mission_cancel', '/navigation_result',
                         'REACHED', 'PLAN_FAILED', 'TIMEOUT', 'PREEMPTED'):
            self.assertIn(contract, source)

    def test_mission_manager_ignores_republished_identical_maps(self):
        source = (PACKAGE_ROOT / 'tp_c_mission' / 'mission_manager_node.py').read_text()
        same_map_guard = source.index('if signature == self.map_signature:')
        planner_rebuild = source.index('self.planner = GridPlannerCore.from_occupancy')
        visited_reset = source.index('self.visited.clear()')
        self.assertLess(same_map_guard, planner_rebuild)
        self.assertLess(same_map_guard, visited_reset)

    def test_real_profile_uses_identified_aruco_and_not_virtual_sensor(self):
        source = (PACKAGE_ROOT / 'launch' / 'parte_c_real.launch.py').read_text()
        self.assertIn("DeclareLaunchArgument('profile', default_value='real_tb4'",
                      source)
        self.assertIn("DeclareLaunchArgument('robot_namespace'", source)
        self.assertIn('validate_tb4_topics', source)
        self.assertIn('aruco_mcl_adapter', source)
        self.assertIn(
            "package='tp_b_navigation', executable='aruco_mcl_adapter'", source)
        self.assertIn('landmark_map_file', source)
        self.assertNotIn("executable='landmark_sensor'", source)

    def test_real_profile_routes_namespaced_tf_to_all_tf_consumers(self):
        source = (PACKAGE_ROOT / 'launch' / 'parte_c_real.launch.py').read_text()

        self.assertIn("tf_topic = _override(context, 'tf_topic', profile.tf_topic)", source)
        self.assertIn(
            "tf_static_topic = _override(\n"
            "        context, 'tf_static_topic', profile.tf_static_topic)",
            source)
        self.assertIn("('/tf', tf_topic)", source)
        self.assertIn("('/tf_static', tf_static_topic)", source)
        for argument in ('tf_topic', 'tf_static_topic'):
            self.assertIn(f"DeclareLaunchArgument('{argument}'", source)

        for executable in (
            'mcl_localization',
            'global_planner',
            'obstacle_monitor',
            'state_machine',
            'aruco_mcl_adapter',
            'red_cone_detector',
        ):
            start = source.index(f"executable='{executable}'")
            end = source.find('),', start) + 2
            self.assertIn('remappings=', source[start:end], executable)

    def test_real_profile_wires_mission_safety_gates(self):
        launch = (PACKAGE_ROOT / 'launch' / 'parte_c_real.launch.py').read_text()
        manager = (PACKAGE_ROOT / 'tp_c_mission' / 'mission_manager_node.py').read_text()
        config = (PACKAGE_ROOT / 'config' / 'parte_c.yaml').read_text()

        for argument in (
            'enable_safety_gates',
            'max_mcl_pose_age',
            'max_position_covariance',
            'max_yaw_covariance',
        ):
            self.assertIn(f"DeclareLaunchArgument('{argument}'", launch)
            self.assertIn(f"'{argument}':", launch)
            self.assertIn(f"'{argument}':", manager)
            self.assertIn(f'{argument}:', config)
        self.assertIn("default_value='true'", launch)
        self.assertIn('localization_gate(', manager)
        self.assertIn('last_mcl_pose_stamp', manager)
        self.assertIn('last_mcl_covariance', manager)

    def test_mission_failure_paths_cancel_navigation_and_publish_failed(self):
        source = (PACKAGE_ROOT / 'tp_c_mission' / 'mission_manager_node.py').read_text()

        self.assertIn("self._fail_mission('vision_stale')", source)
        self.assertIn("self._fail_mission(reason)", source)
        self.assertIn('self.cancel_pub.publish(Empty())', source)
        self.assertIn("self._set_status('FAILED')", source)
        self.assertIn('Faltan /map, /mcl_pose o cámara/TF disponible.', source)

    def test_bag_profile_opens_navigation_rviz_for_detector_calibration(self):
        launch = (PACKAGE_ROOT / 'launch' / 'parte_c_bag.launch.py').read_text()
        setup = (PACKAGE_ROOT / 'setup.py').read_text()
        rviz = PACKAGE_ROOT / 'config' / 'parte_c_bag.rviz'
        self.assertIn("DeclareLaunchArgument('launch_rviz'", launch)
        self.assertIn("DeclareLaunchArgument('bag_path'", launch)
        self.assertIn("DeclareLaunchArgument('play_bag'", launch)
        self.assertIn("DeclareLaunchArgument(\n            'robot_namespace'", launch)
        self.assertIn('resolve_profile(BAG_TB4', launch)
        self.assertIn("default_value='true'", launch)
        self.assertIn("ros2', 'bag', 'play'", launch)
        self.assertIn("'--clock'", launch)
        self.assertIn("package='rviz2'", launch)
        self.assertIn('parte_c_bag.rviz', launch)
        self.assertIn("glob('config/*.rviz')", setup)
        self.assertTrue(rviz.is_file())
        rviz_text = rviz.read_text()
        for topic in ('/tb4_0/scan', '/tb4_0/odom', '/plan', '/map',
                      '/red_cone/debug_image', '/red_cone/mask', '/red_cone_pose'):
            self.assertIn(topic, rviz_text)
        self.assertIn('Fixed Frame: odom', rviz_text)
        self.assertIn('TopDownOrtho', rviz_text)

    def test_sim_profile_opens_mission_rviz_with_exploration_overlays(self):
        launch = (PACKAGE_ROOT / 'launch' / 'parte_c_sim.launch.py').read_text()
        setup = (PACKAGE_ROOT / 'setup.py').read_text()
        rviz = PACKAGE_ROOT / 'config' / 'parte_c_sim.rviz'
        self.assertIn('parte_c_sim.rviz', launch)
        self.assertIn('rviz_config', launch)
        self.assertIn("glob('config/*.rviz')", setup)
        self.assertTrue(rviz.is_file())
        rviz_text = rviz.read_text()
        for topic in ('/plan', '/dynamic_obstacles', '/mission_goal',
                      '/mission/coverage_markers', '/mission/frontier_markers',
                      '/red_cone_pose'):
            self.assertIn(topic, rviz_text)
        self.assertIn('Fixed Frame: map', rviz_text)

    def test_navigation_launch_keeps_default_rviz_but_accepts_override(self):
        launch = (REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_b_navigation' /
                  'launch' / 'parte_b.launch.py').read_text()
        self.assertIn("DeclareLaunchArgument('rviz_config'", launch)
        self.assertIn('parte_b.rviz', launch)
        self.assertIn("arguments=['-d', rviz_config]", launch)

    def test_navigation_uses_dynamic_obstacle_layer_for_replanning(self):
        package = REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_b_navigation' / 'tp_b_navigation'
        planner = (package / 'global_planner.py').read_text()
        monitor = (package / 'obstacle_monitor.py').read_text()
        self.assertIn("'/dynamic_obstacles'", planner)
        self.assertIn('apply_dynamic_obstacles', planner)
        self.assertIn("'/dynamic_obstacles'", monitor)
        self.assertIn('mark_dynamic_obstacles', monitor)

    def test_supported_worlds_exclude_optional_obs2(self):
        source = (PACKAGE_ROOT / 'launch' / 'parte_c_sim.launch.py').read_text()
        self.assertIn('custom_casa.launch.py', source)
        self.assertIn('custom_casa_obs.launch.py', source)
        self.assertNotIn('obs2', source)

    def test_runtime_files_have_no_personal_paths(self):
        personal = re.compile(r'/Users/|~/Documents')
        runtime_files = list((PACKAGE_ROOT / 'tp_c_mission').glob('*.py'))
        runtime_files += list((PACKAGE_ROOT / 'launch').glob('*.py'))
        runtime_files += list((PACKAGE_ROOT / 'config').glob('*.yaml'))
        for path in runtime_files:
            with self.subTest(path=path):
                self.assertIsNone(personal.search(path.read_text()))


if __name__ == '__main__':
    unittest.main()
