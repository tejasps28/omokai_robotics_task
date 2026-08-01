from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

from omokai_bringup.vision_scene import ACTOR_WAYPOINTS
from omokai_bringup.vision_scene import OFFICE_X_BOUNDS_M
from omokai_bringup.vision_scene import OFFICE_Y_BOUNDS_M
from omokai_bringup.vision_scene import ROBOT_START
from omokai_bringup.vision_scene import STATIONARY_ACTOR_YAW_RAD
from omokai_bringup.vision_scene import actor_world_waypoints
from omokai_bringup.vision_scene import initial_actor_outside_camera_fov
from omokai_bringup.vision_scene import path_length_m


PACKAGE = Path(__file__).resolve().parents[1]


class VisionSceneTest(unittest.TestCase):
    def test_actor_path_is_closed_nontrivial_and_inside_office(self) -> None:
        self.assertEqual(ACTOR_WAYPOINTS[0], ACTOR_WAYPOINTS[-1])
        self.assertGreater(path_length_m(), 2.0)
        for x, y in actor_world_waypoints():
            self.assertGreater(x, OFFICE_X_BOUNDS_M[0])
            self.assertLess(x, OFFICE_X_BOUNDS_M[1])
            self.assertGreater(y, OFFICE_Y_BOUNDS_M[0])
            self.assertLess(y, OFFICE_Y_BOUNDS_M[1])

        # The configured camera initially looks west; the patrol remains in
        # the south strip outside its 90-degree view until active search.
        self.assertTrue(initial_actor_outside_camera_fov())
        self.assertLess(actor_world_waypoints()[0][1], ROBOT_START[1])

    def test_world_contains_minimal_office_and_translating_actor_plugin(self) -> None:
        root = ET.parse(
            PACKAGE / 'worlds' / 'vision_rgbd.world'
        ).getroot()
        world = root.find('world')
        models = {
            model.get('name'): model for model in world.findall('model')
        }

        self.assertEqual({'office_small_minimal'}, set(models))
        shell = models['office_small_minimal'].find("link[@name='shell']")
        self.assertIsNotNone(shell)
        self.assertEqual(6, len(shell.findall('collision')))
        self.assertEqual(6, len(shell.findall('visual')))

        self.assertIsNone(world.find("include[name='vision_target']"))

        moving_actor = ET.parse(
            PACKAGE / 'models' / 'DoctorFemaleWalkScenario' / 'moving.sdf'
        ).getroot().find('actor')
        plugin = moving_actor.find('plugin')
        self.assertEqual('path', plugin.findtext('follow_mode'))
        self.assertEqual(
            '/vision_target/cmd_path', plugin.findtext('path_topic')
        )
        self.assertGreater(float(plugin.findtext('linear_velocity')), 0.0)
        self.assertGreaterEqual(
            float(plugin.findtext('animation_factor')), 4.0
        )

        stationary_actor = ET.parse(
            PACKAGE / 'models' / 'DoctorFemaleWalkScenario' / 'stationary.sdf'
        ).getroot().find('actor')
        self.assertIsNone(stationary_actor.find('animation'))
        self.assertIsNone(stationary_actor.find('plugin'))

    def test_only_office_source_and_license_records_remain(self) -> None:
        root = PACKAGE / 'models' / 'office_small_collection'
        self.assertTrue((root / 'SOURCE.md').is_file())
        self.assertTrue((root / 'LICENSE.GPL-3.0').is_file())
        self.assertFalse(any(path.is_dir() for path in root.iterdir()))

    def test_red_actor_is_a_packaged_portable_model(self) -> None:
        root = PACKAGE / 'models' / 'DoctorFemaleWalkRed'

        self.assertTrue((root / 'SOURCE.md').is_file())
        self.assertTrue((root / 'model.sdf').is_file())
        self.assertTrue(
            (root / 'meshes' / 'DoctorFemaleWalkRed.dae').is_file()
        )
        self.assertTrue(
            (root / 'meshes' / 'DoctorFemaleWalkRed_Diffuse.png').is_file()
        )

    def test_actor_path_has_ros_to_gazebo_bridge(self) -> None:
        entries = yaml.safe_load(
            (PACKAGE / 'config' / 'vision_bridge.yaml').read_text(
                encoding='utf-8'
            )
        )
        path_entry = next(
            entry
            for entry in entries
            if entry['ros_topic_name'] == '/vision_target/cmd_path'
        )

        self.assertEqual('ROS_TO_GZ', path_entry['direction'])
        self.assertEqual('geometry_msgs/msg/PoseArray', path_entry['ros_type_name'])
        self.assertEqual('gz.msgs.Pose_V', path_entry['gz_type_name'])

        clock_entry = next(
            entry
            for entry in entries
            if entry['ros_topic_name'] == '/clock'
        )
        self.assertEqual(
            '/world/vision_rgbd/clock', clock_entry['gz_topic_name']
        )


if __name__ == '__main__':
    unittest.main()
