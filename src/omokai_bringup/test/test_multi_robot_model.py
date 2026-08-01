import unittest
import xml.etree.ElementTree as ET

from omokai_bringup.multi_robot_model import render_namespaced_sdf
from omokai_bringup.multi_robot_model import render_vision_urdf


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
    <link name="camera_rgb_frame">
      <pose>0.076 0 0.093 0 0 0</pose>
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

    def test_configures_one_aligned_rgbd_sensor(self) -> None:
        root = ET.fromstring(
            render_namespaced_sdf(SOURCE, 'robot1', rgbd_camera=True)
        )
        sensor = next(
            item for item in root.iter('sensor')
            if item.get('name') == 'rgbd_camera'
        )

        self.assertEqual('rgbd_camera', sensor.get('type'))
        self.assertEqual('/robot1/camera', sensor.findtext('topic'))
        self.assertEqual(
            'robot1/camera_rgb_optical_frame',
            sensor.findtext('gz_frame_id'),
        )
        self.assertEqual(
            '/robot1/camera/camera_info',
            sensor.findtext('camera/camera_info_topic'),
        )
        self.assertEqual(
            'robot1/camera_rgb_optical_frame',
            sensor.findtext('camera/optical_frame_id'),
        )
        self.assertEqual(
            '1.5708',
            sensor.findtext('camera/horizontal_fov'),
        )
        self.assertEqual('15', sensor.findtext('update_rate'))
        camera_frame = next(
            item for item in root.findall('model/link')
            if item.get('name') == 'camera_rgb_frame'
        )
        self.assertEqual(
            '-0.22',
            camera_frame.findtext('pose').split()[4],
        )

    def test_vision_urdf_matches_camera_pitch(self) -> None:
        source = """
        <robot name="robot">
          <joint name="camera_rgb_joint" type="fixed">
            <origin xyz="0 0 0" rpy="0 0 0"/>
          </joint>
        </robot>
        """
        root = ET.fromstring(render_vision_urdf(source))
        self.assertEqual(
            '0 -0.22 0',
            root.find("joint[@name='camera_rgb_joint']/origin").get('rpy'),
        )

    def test_requires_a_model(self) -> None:
        with self.assertRaises(ValueError):
            render_namespaced_sdf('<sdf version="1.8"/>', 'robot1')


if __name__ == '__main__':
    unittest.main()
