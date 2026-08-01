import xml.etree.ElementTree as ET

import pytest

from omokai_bringup.vision_robot_model import render_vision_sdf
from omokai_bringup.vision_robot_model import render_vision_urdf


SDF = """
<sdf version="1.9">
  <model name="upstream">
    <link name="camera_rgb_frame">
      <pose>0.076 0 0.093 0 0 0</pose>
      <sensor name="camera" type="camera">
        <topic>camera/image_raw</topic>
        <gz_frame_id>camera_rgb_frame</gz_frame_id>
        <camera><clip><near>0.1</near><far>100</far></clip></camera>
      </sensor>
    </link>
    <plugin filename="diff" name="gz::sim::systems::DiffDrive">
      <topic>cmd_vel</topic><odom_topic>odom</odom_topic>
      <frame_id>odom</frame_id><child_frame_id>base_footprint</child_frame_id>
      <tf_topic>tf</tf_topic>
    </plugin>
  </model>
</sdf>
"""


def test_configures_namespaced_aligned_rgbd_sensor() -> None:
    root = ET.fromstring(render_vision_sdf(SDF, 'robot1'))
    sensor = root.find(".//sensor[@name='rgbd_camera']")

    assert sensor is not None
    assert sensor.get('type') == 'rgbd_camera'
    assert sensor.findtext('topic') == '/robot1/camera'
    assert sensor.findtext('gz_frame_id') == 'robot1/camera_rgb_optical_frame'
    assert sensor.findtext('camera/horizontal_fov') == '1.5708'
    assert sensor.findtext('camera/clip/near') == '0.12'
    assert sensor.findtext('camera/clip/far') == '8.0'


def test_urdf_camera_pitch_matches_sensor_mount() -> None:
    source = """
    <robot name="robot"><joint name="camera_rgb_joint" type="fixed">
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint></robot>
    """
    root = ET.fromstring(render_vision_urdf(source))

    assert root.find("joint[@name='camera_rgb_joint']/origin").get('rpy') == (
        '0 -0.22 0'
    )


def test_rejects_unsafe_robot_name() -> None:
    with pytest.raises(ValueError):
        render_vision_sdf(SDF, '../robot')
