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

    return rewrite(deepcopy(dict(parameters)))
