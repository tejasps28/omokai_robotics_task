"""ROS-independent right-of-way policy for nearby moving robots."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite
from typing import Mapping

from omokai_fleet.model import ROBOT_IDS, Pose2D, RobotId


TRAFFIC_CLEARANCE_M = 0.50
PREDICTION_HORIZON_SEC = 3.0
PRIORITY_WINDOW_SEC = 5.0


@dataclass(frozen=True)
class PlanarVelocity:
    x: float
    y: float

    def __post_init__(self) -> None:
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and isfinite(value)
            for value in (self.x, self.y)
        ):
            raise ValueError('velocity components must be finite numbers')

    @property
    def speed(self) -> float:
        return hypot(self.x, self.y)


@dataclass(frozen=True)
class RightOfWayDecision:
    allowed: tuple[RobotId, ...]
    yielding: tuple[RobotId, ...]
    conflicts: tuple[tuple[RobotId, RobotId], ...]


def decide_right_of_way(
    poses: Mapping[RobotId, Pose2D],
    velocities: Mapping[RobotId, PlanarVelocity],
    now_sec: float,
    *,
    clearance_m: float = TRAFFIC_CLEARANCE_M,
    horizon_sec: float = PREDICTION_HORIZON_SEC,
    priority_window_sec: float = PRIORITY_WINDOW_SEC,
) -> RightOfWayDecision:
    """Yield lower-priority members of each predicted conflict component."""
    if tuple(poses) != ROBOT_IDS or tuple(velocities) != ROBOT_IDS:
        raise ValueError('poses and velocities must use stable robot order')
    if not all(
        isinstance(poses[robot_id], Pose2D)
        and isinstance(velocities[robot_id], PlanarVelocity)
        for robot_id in ROBOT_IDS
    ):
        raise ValueError('invalid pose or velocity value')
    if not all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and isfinite(value)
        and value > 0
        for value in (clearance_m, horizon_sec, priority_window_sec)
    ):
        raise ValueError('policy limits must be finite and positive')
    if not isinstance(now_sec, (int, float)) or not isfinite(now_sec):
        raise ValueError('now_sec must be finite')

    conflicts = tuple(
        (first, second)
        for index, first in enumerate(ROBOT_IDS)
        for second in ROBOT_IDS[index + 1 :]
        if _will_conflict(
            poses[first],
            velocities[first],
            poses[second],
            velocities[second],
            clearance_m,
            horizon_sec,
        )
    )
    if not conflicts:
        return RightOfWayDecision(ROBOT_IDS, (), ())

    priority_offset = int(now_sec // priority_window_sec) % len(ROBOT_IDS)
    priority = ROBOT_IDS[priority_offset:] + ROBOT_IDS[:priority_offset]
    rank = {robot_id: index for index, robot_id in enumerate(priority)}
    yielding: set[RobotId] = set()
    for component in _conflict_components(conflicts):
        winner = min(component, key=rank.__getitem__)
        yielding.update(component - {winner})
    return RightOfWayDecision(
        tuple(robot_id for robot_id in ROBOT_IDS if robot_id not in yielding),
        tuple(robot_id for robot_id in ROBOT_IDS if robot_id in yielding),
        conflicts,
    )


def predicted_separation(
    first_pose: Pose2D,
    first_velocity: PlanarVelocity,
    second_pose: Pose2D,
    second_velocity: PlanarVelocity,
    horizon_sec: float = PREDICTION_HORIZON_SEC,
) -> float:
    """Return closest constant-velocity separation within the horizon."""
    relative_x = second_pose.x - first_pose.x
    relative_y = second_pose.y - first_pose.y
    velocity_x = second_velocity.x - first_velocity.x
    velocity_y = second_velocity.y - first_velocity.y
    velocity_squared = velocity_x * velocity_x + velocity_y * velocity_y
    if velocity_squared <= 1e-12:
        return hypot(relative_x, relative_y)
    closest_time = -(
        relative_x * velocity_x + relative_y * velocity_y
    ) / velocity_squared
    closest_time = max(0.0, min(horizon_sec, closest_time))
    return hypot(
        relative_x + velocity_x * closest_time,
        relative_y + velocity_y * closest_time,
    )


def _will_conflict(
    first_pose: Pose2D,
    first_velocity: PlanarVelocity,
    second_pose: Pose2D,
    second_velocity: PlanarVelocity,
    clearance_m: float,
    horizon_sec: float,
) -> bool:
    if first_velocity.speed < 0.01 or second_velocity.speed < 0.01:
        return False
    return predicted_separation(
        first_pose,
        first_velocity,
        second_pose,
        second_velocity,
        horizon_sec,
    ) < clearance_m


def _conflict_components(
    conflicts: tuple[tuple[RobotId, RobotId], ...],
) -> tuple[set[RobotId], ...]:
    components: list[set[RobotId]] = []
    for first, second in conflicts:
        matching = [
            component
            for component in components
            if first in component or second in component
        ]
        if not matching:
            components.append({first, second})
            continue
        merged = {first, second}
        for component in matching:
            merged.update(component)
            components.remove(component)
        components.append(merged)
    return tuple(components)
