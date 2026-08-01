"""Create isolated TurtleBot3 SDF instances from the installed upstream model."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET


ROBOT_NAME_PATTERN = re.compile(r'^[a-z][a-z0-9_]{0,31}$')
# The stock ~62 degree camera cannot contain a standing adult at the required
# 1.2 m stand-off. Use a common 90 degree RGB-D field of view and a modest
# upward pitch so the full target remains detectable throughout approach.
VISION_CAMERA_PITCH_RAD = -0.22
VISION_CAMERA_HORIZONTAL_FOV_RAD = 1.5708


def _scoped(robot_name: str, value: str) -> str:
    return f'/{robot_name}/{value.lstrip("/")}'


def _prefixed(robot_name: str, value: str) -> str:
    return f'{robot_name}/{value.lstrip("/")}'


def _configure_rgbd_camera(model: ET.Element) -> None:
    """Upgrade the stock RGB camera to one aligned RGB-D sensor."""
    camera_frame = next(
        (
            link
            for link in model.findall('link')
            if link.get('name') == 'camera_rgb_frame'
        ),
        None,
    )
    if camera_frame is None:
        raise ValueError('TurtleBot SDF does not contain camera_rgb_frame')
    pose = camera_frame.find('pose')
    if pose is None or not pose.text:
        raise ValueError('camera_rgb_frame does not contain a pose')
    pose_values = pose.text.split()
    if len(pose_values) != 6:
        raise ValueError('camera_rgb_frame pose must contain six values')
    pose_values[4] = str(VISION_CAMERA_PITCH_RAD)
    pose.text = ' '.join(pose_values)

    camera_sensor = next(
        (
            sensor
            for sensor in model.iter('sensor')
            if sensor.get('name') == 'camera'
        ),
        None,
    )
    if camera_sensor is None:
        raise ValueError('TurtleBot SDF does not contain the camera sensor')

    camera_sensor.set('name', 'rgbd_camera')
    camera_sensor.set('type', 'rgbd_camera')
    def set_sensor_value(tag: str, value: str) -> None:
        element = camera_sensor.find(tag)
        if element is None:
            element = ET.SubElement(camera_sensor, tag)
        element.text = value

    set_sensor_value('always_on', 'true')
    set_sensor_value('update_rate', '15')
    set_sensor_value('visualize', 'true')
    set_sensor_value('topic', 'camera')
    set_sensor_value('gz_frame_id', 'camera_rgb_optical_frame')

    camera = camera_sensor.find('camera')
    if camera is None:
        raise ValueError('camera sensor does not contain camera geometry')
    camera.set('name', 'aligned_rgbd')
    horizontal_fov = camera.find('horizontal_fov')
    if horizontal_fov is None:
        horizontal_fov = ET.SubElement(camera, 'horizontal_fov')
    horizontal_fov.text = str(VISION_CAMERA_HORIZONTAL_FOV_RAD)
    camera_info = camera.find('camera_info_topic')
    if camera_info is None:
        camera_info = ET.SubElement(camera, 'camera_info_topic')
    camera_info.text = 'camera/camera_info'
    optical_frame = camera.find('optical_frame_id')
    if optical_frame is None:
        optical_frame = ET.SubElement(camera, 'optical_frame_id')
    optical_frame.text = 'camera_rgb_optical_frame'

    clip = camera.find('clip')
    if clip is not None:
        clip.find('near').text = '0.12'
        clip.find('far').text = '8.0'


def render_vision_urdf(source: str) -> str:
    """Return the robot description with TF matching the tilted RGB-D sensor."""
    root = ET.fromstring(source)
    camera_joint = next(
        (
            joint
            for joint in root.findall('joint')
            if joint.get('name') == 'camera_rgb_joint'
        ),
        None,
    )
    if camera_joint is None:
        raise ValueError('TurtleBot URDF does not contain camera_rgb_joint')
    origin = camera_joint.find('origin')
    if origin is None:
        raise ValueError('camera_rgb_joint does not contain an origin')
    origin.set('rpy', f'0 {VISION_CAMERA_PITCH_RAD} 0')
    return ET.tostring(root, encoding='unicode')


def render_namespaced_sdf(
    source: str,
    robot_name: str,
    *,
    rgbd_camera: bool = False,
) -> str:
    """Return an upstream TurtleBot SDF with isolated topics and frame IDs."""
    if not ROBOT_NAME_PATTERN.fullmatch(robot_name):
        raise ValueError('robot_name must be a lowercase ROS-safe identifier')

    root = ET.fromstring(source)
    model = root.find('model')
    if model is None:
        raise ValueError('SDF must contain one top-level model')
    model.set('name', robot_name)
    if rgbd_camera:
        _configure_rgbd_camera(model)

    for sensor in model.iter('sensor'):
        topic = sensor.find('topic')
        if topic is not None and topic.text:
            topic.text = _scoped(robot_name, topic.text)

        frame_id = sensor.find('gz_frame_id')
        if frame_id is not None and frame_id.text:
            frame_id.text = _prefixed(robot_name, frame_id.text)

        for camera_info in sensor.iter('camera_info_topic'):
            if camera_info.text:
                camera_info.text = _scoped(robot_name, camera_info.text)

        for optical_frame in sensor.iter('optical_frame_id'):
            if optical_frame.text:
                optical_frame.text = _prefixed(robot_name, optical_frame.text)

    for plugin in model.iter('plugin'):
        plugin_name = plugin.get('name', '')
        if plugin_name.endswith('DiffDrive'):
            for tag in ('topic', 'odom_topic'):
                element = plugin.find(tag)
                if element is not None and element.text:
                    element.text = _scoped(robot_name, element.text)
            for tag in ('frame_id', 'child_frame_id'):
                element = plugin.find(tag)
                if element is not None and element.text:
                    element.text = _prefixed(robot_name, element.text)
            tf_topic = plugin.find('tf_topic')
            if tf_topic is not None:
                tf_topic.text = _scoped(robot_name, 'tf')

        if plugin_name.endswith('JointStatePublisher'):
            topic = plugin.find('topic')
            if topic is not None and topic.text:
                topic.text = _scoped(robot_name, topic.text)

    return ET.tostring(root, encoding='unicode')
