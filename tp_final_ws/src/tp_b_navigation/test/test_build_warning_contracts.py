#!/usr/bin/env python3
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]


class BuildWarningContractsTest(unittest.TestCase):
    def test_custom_simulation_uses_portable_cpp_link_targets(self):
        cmake_source = (
            REPO_ROOT / 'tp_final_ws' / 'src'
            / 'turtlebot3_custom_simulation' / 'CMakeLists.txt').read_text()

        self.assertNotIn('CONDA_PREFIX', cmake_source)
        self.assertNotIn('INSTALL_RPATH "', cmake_source)
        self.assertIn('INSTALL_RPATH_USE_LINK_PATH TRUE', cmake_source)
        self.assertNotIn('ament_target_dependencies', cmake_source)
        for target in (
                'rclcpp::rclcpp',
                'tf2::tf2',
                'geometry_msgs::geometry_msgs__rosidl_typesupport_cpp',
                'nav_msgs::nav_msgs__rosidl_typesupport_cpp',
                'sensor_msgs::sensor_msgs__rosidl_typesupport_cpp',
                'tf2_msgs::tf2_msgs__rosidl_typesupport_cpp'):
            self.assertIn(target, cmake_source)
        self.assertIn('target_compile_features(${EXEC_NAME} PRIVATE cxx_std_17)',
                      cmake_source)
        self.assertIn('LINKER:-dead_strip_dylibs', cmake_source)

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
