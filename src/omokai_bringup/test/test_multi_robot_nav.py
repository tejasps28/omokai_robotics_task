import unittest
from pathlib import Path

import yaml

from omokai_bringup.multi_robot_nav import namespace_nav_parameters


PARAMETERS = {
    'amcl': {
        'ros__parameters': {
            'base_frame_id': 'base_footprint',
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'scan_topic': 'scan',
        }
    },
    'bt_navigator': {
        'ros__parameters': {
            'global_frame': 'map',
            'local_frame': 'odom',
            'robot_base_frame': 'base_link',
            'odom_topic': '/odom',
        }
    },
    'controller_server': {
        'ros__parameters': {
            'general_goal_checker': {
                'xy_goal_tolerance': 0.25,
                'yaw_goal_tolerance': 0.25,
            }
        }
    },
    'local_costmap': {
        'local_costmap': {
            'ros__parameters': {
                'global_frame': 'odom',
                'robot_base_frame': 'base_link',
                'scan': {'topic': '<robot_namespace>/scan'},
            }
        }
    },
    'global_costmap': {
        'global_costmap': {
            'ros__parameters': {
                'plugins': [
                    'static_layer',
                    'obstacle_layer',
                    'inflation_layer',
                ],
                'obstacle_layer': {'enabled': True},
            }
        }
    },
    'collision_monitor': {
        'ros__parameters': {
            'footprint_topic': '/local_costmap/published_footprint',
        }
    },
}


class MultiRobotNavParametersTest(unittest.TestCase):
    def test_gazebo_bridge_accepts_nav2_twist_commands(self) -> None:
        bridge_path = (
            Path(__file__).resolve().parents[1]
            / 'config'
            / 'multi_robot_bridge.yaml'
        )
        entries = yaml.safe_load(bridge_path.read_text(encoding='utf-8'))
        command_entries = [
            entry
            for entry in entries
            if entry['ros_topic_name'].endswith('/cmd_vel')
        ]

        self.assertEqual(3, len(command_entries))
        self.assertTrue(
            all(
                entry['ros_type_name'] == 'geometry_msgs/msg/Twist'
                for entry in command_entries
            )
        )

    def test_clock_bridge_uses_the_named_fleet_world(self) -> None:
        bridge_path = (
            Path(__file__).resolve().parents[1]
            / 'config'
            / 'multi_robot_bridge.yaml'
        )
        entries = yaml.safe_load(bridge_path.read_text(encoding='utf-8'))
        clock = next(
            entry
            for entry in entries
            if entry['ros_topic_name'] == '/clock'
        )

        self.assertEqual(
            '/world/multi_robot_test_zone/clock',
            clock['gz_topic_name'],
        )

    def test_rewrites_frames_and_absolute_topics(self) -> None:
        result = namespace_nav_parameters(PARAMETERS, 'robot3')

        self.assertEqual(
            'robot3/base_footprint',
            result['amcl']['ros__parameters']['base_frame_id'],
        )
        self.assertEqual(
            'robot3/odom',
            result['local_costmap']['local_costmap']['ros__parameters'][
                'global_frame'
            ],
        )
        self.assertEqual(
            'robot3/base_link',
            result['bt_navigator']['ros__parameters']['robot_base_frame'],
        )
        self.assertEqual(
            'robot3/odom',
            result['bt_navigator']['ros__parameters']['local_frame'],
        )
        self.assertEqual(
            '/robot3/odom',
            result['bt_navigator']['ros__parameters']['odom_topic'],
        )
        self.assertEqual(
            '/robot3/scan',
            result['local_costmap']['local_costmap']['ros__parameters'][
                'scan'
            ]['topic'],
        )
        self.assertEqual(
            '/robot3/local_costmap/published_footprint',
            result['collision_monitor']['ros__parameters'][
                'footprint_topic'
            ],
        )
        self.assertEqual(
            'cmd_vel_safe',
            result['collision_monitor']['ros__parameters'][
                'cmd_vel_out_topic'
            ],
        )

    def test_preserves_shared_map_and_relative_topics(self) -> None:
        result = namespace_nav_parameters(PARAMETERS, 'robot1')

        self.assertEqual(
            'map',
            result['amcl']['ros__parameters']['global_frame_id'],
        )
        self.assertEqual(
            'scan',
            result['amcl']['ros__parameters']['scan_topic'],
        )
        self.assertTrue(result['amcl']['ros__parameters']['do_beamskip'])
        self.assertEqual(
            3.5,
            result['amcl']['ros__parameters']['laser_max_range'],
        )

    def test_does_not_mutate_source(self) -> None:
        namespace_nav_parameters(PARAMETERS, 'robot2')

        self.assertEqual(
            'base_footprint',
            PARAMETERS['amcl']['ros__parameters']['base_frame_id'],
        )

    def test_global_planner_uses_static_map_for_fleet_coordination(self) -> None:
        result = namespace_nav_parameters(PARAMETERS, 'robot2')
        parameters = result['global_costmap']['global_costmap'][
            'ros__parameters'
        ]

        self.assertEqual(
            ['static_layer', 'inflation_layer'],
            parameters['plugins'],
        )
        self.assertFalse(parameters['obstacle_layer']['enabled'])

    def test_patrol_goal_tolerance_avoids_heading_only_deadlock(self) -> None:
        result = namespace_nav_parameters(PARAMETERS, 'robot1')
        checker = result['controller_server']['ros__parameters'][
            'general_goal_checker'
        ]

        self.assertEqual(0.30, checker['xy_goal_tolerance'])
        self.assertEqual(1.00, checker['yaw_goal_tolerance'])


if __name__ == '__main__':
    unittest.main()
