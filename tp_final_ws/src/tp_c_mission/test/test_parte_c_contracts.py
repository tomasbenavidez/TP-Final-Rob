from pathlib import Path
import re
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]


class ParteCContractsTest(unittest.TestCase):
    def test_package_exposes_required_nodes_and_launch_profiles(self):
        setup = (PACKAGE_ROOT / 'setup.py').read_text()
        for executable in ('red_cone_detector', 'mission_manager', 'aruco_mcl_adapter'):
            self.assertIn(executable, setup)
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

    def test_mission_navigation_interfaces_are_explicit(self):
        source = (REPO_ROOT / 'tp_final_ws' / 'src' / 'tp_b_navigation' /
                  'tp_b_navigation' / 'state_machine.py').read_text()
        for contract in ('/mission_goal', '/mission_cancel', '/navigation_result',
                         'REACHED', 'PLAN_FAILED', 'TIMEOUT', 'PREEMPTED'):
            self.assertIn(contract, source)

    def test_real_profile_uses_identified_aruco_and_not_virtual_sensor(self):
        source = (PACKAGE_ROOT / 'launch' / 'parte_c_real.launch.py').read_text()
        self.assertIn('aruco_mcl_adapter', source)
        self.assertIn('landmark_map_file', source)
        self.assertNotIn("executable='landmark_sensor'", source)

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
