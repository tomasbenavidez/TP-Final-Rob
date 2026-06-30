#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


class PlatformProfilesTest(unittest.TestCase):
    def test_simulation_tb3_preserves_current_navigation_defaults(self):
        from tp_b_navigation.platform_profiles import resolve_profile

        profile = resolve_profile('simulation_tb3')

        self.assertEqual(profile.profile, 'simulation_tb3')
        self.assertEqual(profile.robot_namespace, '')
        self.assertEqual(profile.odom_topic, '/calc_odom')
        self.assertEqual(profile.reference_odom_topic, '/odom')
        self.assertEqual(profile.scan_topic, '/scan')
        self.assertEqual(profile.cmd_vel_topic, '/cmd_vel')
        self.assertEqual(profile.tf_topic, '/tf')
        self.assertEqual(profile.tf_static_topic, '/tf_static')
        self.assertEqual(profile.base_frame, 'base_footprint')
        self.assertEqual(profile.odom_frame, 'odom')
        self.assertEqual(profile.camera_frame, '')
        self.assertTrue(profile.use_sim_time)
        self.assertEqual(profile.landmark_source, 'virtual')
        self.assertTrue(profile.launch_virtual_landmarks)

    def test_bag_tb4_uses_bag_clock_and_real_robot_topics_without_virtual_landmarks(self):
        from tp_b_navigation.platform_profiles import resolve_profile

        profile = resolve_profile('bag_tb4')

        self.assertEqual(profile.profile, 'bag_tb4')
        self.assertEqual(profile.robot_namespace, '/tb4_0')
        self.assertEqual(profile.odom_topic, '/tb4_0/odom')
        self.assertEqual(profile.reference_odom_topic, '/tb4_0/odom')
        self.assertEqual(profile.scan_topic, '/tb4_0/scan')
        self.assertEqual(profile.cmd_vel_topic, '/tb4_0/cmd_vel')
        self.assertEqual(profile.tf_topic, '/tb4_0/tf')
        self.assertEqual(profile.tf_static_topic, '/tb4_0/tf_static')
        self.assertEqual(profile.rgb_topic, '/tb4_0/oakd/rgb/preview/image_raw')
        self.assertEqual(profile.depth_topic, '/tb4_0/oakd/stereo/image_raw')
        self.assertEqual(profile.camera_info_topic,
                         '/tb4_0/oakd/rgb/preview/camera_info')
        self.assertEqual(profile.base_frame, 'base_link')
        self.assertEqual(profile.odom_frame, 'odom')
        self.assertTrue(profile.use_sim_time)
        self.assertEqual(profile.landmark_source, 'aruco')
        self.assertFalse(profile.launch_virtual_landmarks)

    def test_bag_tb4_can_resolve_tb4_1_topics(self):
        from tp_b_navigation.platform_profiles import resolve_profile

        profile = resolve_profile('bag_tb4', robot_namespace='tb4_1')

        self.assertEqual(profile.robot_namespace, '/tb4_1')
        self.assertEqual(profile.odom_topic, '/tb4_1/odom')
        self.assertEqual(profile.scan_topic, '/tb4_1/scan')
        self.assertEqual(profile.tf_topic, '/tb4_1/tf')
        self.assertEqual(profile.tf_static_topic, '/tb4_1/tf_static')

    def test_real_tb4_uses_wall_clock_and_real_robot_topics_without_virtual_landmarks(self):
        from tp_b_navigation.platform_profiles import resolve_profile

        profile = resolve_profile('real_tb4')

        self.assertEqual(profile.profile, 'real_tb4')
        self.assertEqual(profile.robot_namespace, '/tb4_0')
        self.assertEqual(profile.odom_topic, '/tb4_0/odom')
        self.assertEqual(profile.reference_odom_topic, '/tb4_0/odom')
        self.assertEqual(profile.scan_topic, '/tb4_0/scan')
        self.assertEqual(profile.cmd_vel_topic, '/tb4_0/cmd_vel')
        self.assertEqual(profile.tf_topic, '/tb4_0/tf')
        self.assertEqual(profile.tf_static_topic, '/tb4_0/tf_static')
        self.assertFalse(profile.use_sim_time)
        self.assertEqual(profile.landmark_source, 'aruco')
        self.assertFalse(profile.launch_virtual_landmarks)

    def test_real_tb4_can_resolve_tb4_1_topics(self):
        from tp_b_navigation.platform_profiles import resolve_profile

        profile = resolve_profile('real_tb4', robot_namespace='/tb4_1')

        self.assertEqual(profile.robot_namespace, '/tb4_1')
        self.assertEqual(profile.odom_topic, '/tb4_1/odom')
        self.assertEqual(profile.cmd_vel_topic, '/tb4_1/cmd_vel')
        self.assertFalse(profile.use_sim_time)

    def test_unknown_profile_lists_supported_profiles(self):
        from tp_b_navigation.platform_profiles import resolve_profile

        with self.assertRaisesRegex(ValueError, 'simulation_tb3, bag_tb4, real_tb4'):
            resolve_profile('not_a_profile')


if __name__ == '__main__':
    unittest.main()
