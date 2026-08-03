"""Small conversion and Nav2 adapter layer for exploration."""

from __future__ import annotations

import math
from typing import Any, Optional

from omokai_executor import ExecutionGoal, GoalPose

from .frontier import FrontierCandidate
from .grid import GridMetadata, OccupancyGrid


def grid_from_message(message: Any) -> OccupancyGrid:
    """Convert a ROS OccupancyGrid-like value into the pure domain model."""

    if message.header.frame_id != 'map':
        raise ValueError("occupancy grid must use the 'map' frame")
    orientation = message.info.origin.orientation
    origin_yaw = math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )
    if not math.isfinite(origin_yaw) or abs(origin_yaw) > 1e-6:
        raise ValueError('rotated occupancy-grid origins are not supported')
    return OccupancyGrid(
        GridMetadata(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
        ),
        message.data,
    )


def robot_xy_from_transform(transform: Any) -> tuple[float, float]:
    """Extract and validate planar robot coordinates from a TF-like value."""

    x = transform.transform.translation.x
    y = transform.transform.translation.y
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError('robot transform coordinates must be finite')
    return x, y


class ExplorationNav2Adapter:
    """Translate frontier candidates for the existing Nav2 action adapter."""

    def __init__(
        self,
        node: Any,
        *,
        speed_mps: float = 0.15,
        delegate: Optional[Any] = None,
    ) -> None:
        if (
            isinstance(speed_mps, bool)
            or not isinstance(speed_mps, (int, float))
            or not math.isfinite(speed_mps)
            or speed_mps <= 0.0
        ):
            raise ValueError('speed_mps must be a positive finite number')
        if delegate is None:
            from omokai_executor.nav2_adapter import Nav2NavigationAdapter

            delegate = Nav2NavigationAdapter(node)
        self._delegate = delegate
        self._speed_mps = speed_mps
        self._next_goal = 1

    def bind_outcomes(self, outcomes: Any) -> None:
        self._delegate.bind_outcomes(outcomes)

    def server_is_ready(self) -> bool:
        return self._delegate.server_is_ready()

    def dispatch(self, candidate: FrontierCandidate) -> str:
        goal = ExecutionGoal(
            goal_id=f'frontier-{self._next_goal:06d}',
            pose=GoalPose(
                x=candidate.goal_x,
                y=candidate.goal_y,
                yaw=0.0,
                frame_id='map',
            ),
        )
        self._next_goal += 1
        return self._delegate.dispatch(goal, self._speed_mps)

    def cancel(self, goal_handle: str) -> None:
        self._delegate.cancel(goal_handle)
