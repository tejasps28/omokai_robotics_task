import unittest
import xml.etree.ElementTree as ET

from omokai_bringup.multi_robot_model import render_namespaced_sdf


SOURCE = """\
<sdf version="1.8">
  <model name="upstream">
    <link name="base_scan">
      <sensor name="lidar" type="gpu_lidar">
        <topic>scan</topic>
        <gz_frame_id>base_scan</gz_frame_id>
      </sensor>
      <sensor name="camera" type="camera">
        <topic>camera/image_raw</topic>
        <gz_frame_id>camera_rgb_frame</gz_frame_id>
        <camera>
          <camera_info_topic>camera/camera_info</camera_info_topic>
        </camera>
      </sensor>
    </link>
    <plugin filename="diff" name="gz::sim::systems::DiffDrive">
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
      <tf_topic>/tf</tf_topic>
    </plugin>
    <plugin filename="joints" name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
    </plugin>
  </model>
</sdf>
"""


class MultiRobotModelTest(unittest.TestCase):
    def test_rewrites_model_topics_and_frames(self) -> None:
        root = ET.fromstring(render_namespaced_sdf(SOURCE, 'robot2'))
        model = root.find('model')

        self.assertEqual('robot2', model.get('name'))
        self.assertEqual(
            {
                '/robot2/scan',
                '/robot2/camera/image_raw',
                '/robot2/cmd_vel',
                '/robot2/joint_states',
            },
            {element.text for element in model.iter('topic')},
        )
        self.assertEqual(
            {'/robot2/odom'},
            {element.text for element in model.iter('odom_topic')},
        )
        self.assertEqual(
            {'/robot2/tf'},
            {element.text for element in model.iter('tf_topic')},
        )
        self.assertEqual(
            {'robot2/base_scan', 'robot2/camera_rgb_frame'},
            {element.text for element in model.iter('gz_frame_id')},
        )
        self.assertEqual(
            {'robot2/odom'},
            {element.text for element in model.iter('frame_id')},
        )
        self.assertEqual(
            {'robot2/base_footprint'},
            {element.text for element in model.iter('child_frame_id')},
        )
        self.assertEqual(
            {'/robot2/camera/camera_info'},
            {element.text for element in model.iter('camera_info_topic')},
        )

    def test_rejects_unsafe_robot_name(self) -> None:
        with self.assertRaises(ValueError):
            render_namespaced_sdf(SOURCE, '../robot')

    def test_requires_a_model(self) -> None:
        with self.assertRaises(ValueError):
            render_namespaced_sdf('<sdf version="1.8"/>', 'robot1')


if __name__ == '__main__':
    unittest.main()
