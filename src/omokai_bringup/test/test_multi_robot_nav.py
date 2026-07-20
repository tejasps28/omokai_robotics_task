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
    'local_costmap': {
        'local_costmap': {
            'ros__parameters': {
                'global_frame': 'odom',
                'robot_base_frame': 'base_link',
                'scan': {'topic': '<robot_namespace>/scan'},
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

    def test_does_not_mutate_source(self) -> None:
        namespace_nav_parameters(PARAMETERS, 'robot2')

        self.assertEqual(
            'base_footprint',
            PARAMETERS['amcl']['ros__parameters']['base_frame_id'],
        )


if __name__ == '__main__':
    unittest.main()
