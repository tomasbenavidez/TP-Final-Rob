#!/usr/bin/env python3
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]


class BuildWarningContractsTest(unittest.TestCase):
    def test_custom_simulation_uses_const_subscription_callback(self):
        simulation_root = (
            REPO_ROOT / 'tp_final_ws' / 'src'
            / 'turtlebot3_custom_simulation')
        for relative_path in (
                'include/turtlebot3_custom_simulation/'
                'turtlebot3_custom_simulation.hpp',
                'src/turtlebot3_custom_simulation.cpp'):
            source = (simulation_root / relative_path).read_text()
            self.assertIn(
                'geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_msg',
                source,
                relative_path)
            self.assertNotIn(
                'geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg',
                source,
                relative_path)

    def test_slam_interfaces_selects_legacy_python_policy_locally(self):
        cmake_source = (
            REPO_ROOT / 'tp_final_ws' / 'src'
            / 'tp_slam_interfaces' / 'CMakeLists.txt').read_text()
        self.assertIn('if(POLICY CMP0148)', cmake_source)
        self.assertIn('cmake_policy(SET CMP0148 OLD)', cmake_source)
        self.assertNotIn('-Wno-dev', cmake_source)


if __name__ == '__main__':
    unittest.main()
