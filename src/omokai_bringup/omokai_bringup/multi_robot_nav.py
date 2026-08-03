"""Generate robot-specific Nav2 parameters from the installed Nav2 template."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping

from omokai_bringup.multi_robot_model import ROBOT_NAME_PATTERN


FRAME_KEYS = {
    'base_frame',
    'base_frame_id',
    'global_frame',
    'global_frame_id',
    'local_frame',
    'odom_frame_id',
    'robot_base_frame',
}
PREFIXED_FRAMES = {'base_footprint', 'base_link', 'odom'}


def namespace_nav_parameters(
    parameters: Mapping[str, Any],
    robot_name: str,
) -> dict[str, Any]:
    """Return a deep robot-specific copy of generic Nav2 parameters."""
    if not ROBOT_NAME_PATTERN.fullmatch(robot_name):
        raise ValueError('robot_name must be a lowercase ROS-safe identifier')

    def rewrite(value: Any, key: str = '') -> Any:
        if isinstance(value, dict):
            return {
                child_key: rewrite(child_value, child_key)
                for child_key, child_value in value.items()
            }
        if isinstance(value, list):
            return [rewrite(item, key) for item in value]
        if not isinstance(value, str):
            return value

        result = value.replace('<robot_namespace>', f'/{robot_name}')
        if key in FRAME_KEYS and result in PREFIXED_FRAMES:
            return f'{robot_name}/{result}'
        if key == 'odom_topic' and result == '/odom':
            return f'/{robot_name}/odom'
        if result.startswith('/local_costmap/'):
            return f'/{robot_name}{result}'
        if result.startswith('/global_costmap/'):
            return f'/{robot_name}{result}'
        return result

    result = rewrite(deepcopy(dict(parameters)))
    amcl = result.get('amcl', {}).get('ros__parameters', {})
    amcl.update(
        {
            # Other robots are valid dynamic lidar returns, not map landmarks.
            'do_beamskip': True,
            'beam_skip_distance': 0.5,
            'beam_skip_threshold': 0.3,
            'beam_skip_error_threshold': 0.9,
            'laser_max_range': 3.5,
        }
    )
    bt_navigator = result.get('bt_navigator', {}).get(
        'ros__parameters',
        {},
    )
    # Three composed Nav2 stacks plus Gazebo/RViz can briefly delay an action
    # acknowledgement. The upstream 20 ms default is too tight under that
    # load and makes the BT abort an otherwise healthy navigation goal.
    bt_navigator['default_server_timeout'] = 2000
    global_costmap = (
        result.get('global_costmap', {})
        .get('global_costmap', {})
        .get('ros__parameters', {})
    )
    plugins = global_costmap.get('plugins')
    if isinstance(plugins, list):
        global_costmap['plugins'] = [
            plugin for plugin in plugins if plugin != 'obstacle_layer'
        ]
    global_costmap.get('obstacle_layer', {})['enabled'] = False
    collision_monitor = result.get('collision_monitor', {}).get(
        'ros__parameters',
        {},
    )
    collision_monitor['cmd_vel_out_topic'] = 'cmd_vel_safe'
    controller = result.get('controller_server', {}).get(
        'ros__parameters',
        {},
    )
    goal_checker = controller.setdefault('general_goal_checker', {})
    # Patrol headings are advisory. A generous final-yaw tolerance avoids
    # spending an entire batch rotating at a waypoint while peers wait.
    goal_checker['xy_goal_tolerance'] = 0.30
    goal_checker['yaw_goal_tolerance'] = 1.00
    return result
