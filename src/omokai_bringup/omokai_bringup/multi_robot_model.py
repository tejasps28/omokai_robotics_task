"""Create isolated TurtleBot3 SDF instances from the installed upstream model."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET


ROBOT_NAME_PATTERN = re.compile(r'^[a-z][a-z0-9_]{0,31}$')


def _scoped(robot_name: str, value: str) -> str:
    return f'/{robot_name}/{value.lstrip("/")}'


def _prefixed(robot_name: str, value: str) -> str:
    return f'{robot_name}/{value.lstrip("/")}'


def render_namespaced_sdf(source: str, robot_name: str) -> str:
    """Return an upstream TurtleBot SDF with isolated topics and frame IDs."""
    if not ROBOT_NAME_PATTERN.fullmatch(robot_name):
        raise ValueError('robot_name must be a lowercase ROS-safe identifier')

    root = ET.fromstring(source)
    model = root.find('model')
    if model is None:
        raise ValueError('SDF must contain one top-level model')
    model.set('name', robot_name)

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
