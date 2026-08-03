"""Configure the standalone vision robot and its aligned RGB-D sensor."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET


ROBOT_NAME_PATTERN = re.compile(r'^[a-z][a-z0-9_]{0,31}$')
VISION_CAMERA_PITCH_RAD = -0.22
VISION_CAMERA_HORIZONTAL_FOV_RAD = 1.5708


def _scoped(robot_name: str, value: str) -> str:
    return f'/{robot_name}/{value.lstrip("/")}'


def _prefixed(robot_name: str, value: str) -> str:
    return f'{robot_name}/{value.lstrip("/")}'


def _configure_rgbd_camera(model: ET.Element) -> None:
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

    sensor = next(
        (
            item
            for item in model.iter('sensor')
            if item.get('name') == 'camera'
        ),
        None,
    )
    if sensor is None:
        raise ValueError('TurtleBot SDF does not contain the camera sensor')
    sensor.set('name', 'rgbd_camera')
    sensor.set('type', 'rgbd_camera')

    def set_sensor_value(tag: str, value: str) -> None:
        element = sensor.find(tag)
        if element is None:
            element = ET.SubElement(sensor, tag)
        element.text = value

    set_sensor_value('always_on', 'true')
    set_sensor_value('update_rate', '15')
    set_sensor_value('visualize', 'true')
    set_sensor_value('topic', 'camera')
    set_sensor_value('gz_frame_id', 'camera_rgb_optical_frame')

    camera = sensor.find('camera')
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
        near = clip.find('near')
        far = clip.find('far')
        if near is not None:
            near.text = '0.12'
        if far is not None:
            far.text = '8.0'


def render_vision_urdf(source: str) -> str:
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


def render_vision_sdf(source: str, robot_name: str) -> str:
    if not ROBOT_NAME_PATTERN.fullmatch(robot_name):
        raise ValueError('robot_name must be a lowercase ROS-safe identifier')

    root = ET.fromstring(source)
    model = root.find('model')
    if model is None:
        raise ValueError('SDF must contain one top-level model')
    model.set('name', robot_name)
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
