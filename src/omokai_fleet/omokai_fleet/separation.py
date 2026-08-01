"""Observed pairwise separation checks for live squad poses."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from math import hypot, isfinite

from omokai_fleet.model import ROBOT_IDS, Pose2D, RobotId


EMERGENCY_MINIMUM_DISTANCE_M = 0.38


@dataclass(frozen=True)
class ObservedRobotPose:
    robot_id: RobotId
    pose: Pose2D

    def __post_init__(self) -> None:
        if not isinstance(self.robot_id, RobotId):
            raise ValueError('robot_id must be a known RobotId')
        if not isinstance(self.pose, Pose2D):
            raise ValueError('pose must be a Pose2D')


@dataclass(frozen=True)
class ObservedSeparation:
    first: RobotId
    second: RobotId
    distance_m: float


def observed_separations(
    poses: tuple[ObservedRobotPose, ...],
) -> tuple[ObservedSeparation, ...]:
    if not isinstance(poses, tuple) or len(poses) != 3:
        raise ValueError('poses must contain exactly three robot poses')
    if tuple(item.robot_id for item in poses) != ROBOT_IDS:
        raise ValueError('poses must use stable robot order')
    return tuple(
        ObservedSeparation(
            first.robot_id,
            second.robot_id,
            hypot(
                first.pose.x - second.pose.x,
                first.pose.y - second.pose.y,
            ),
        )
        for first, second in combinations(poses, 2)
    )


def emergency_separation_reason(
    poses: tuple[ObservedRobotPose, ...],
    minimum_m: float = EMERGENCY_MINIMUM_DISTANCE_M,
) -> str | None:
    if (
        not isinstance(minimum_m, (int, float))
        or isinstance(minimum_m, bool)
        or not isfinite(minimum_m)
        or minimum_m <= 0
    ):
        raise ValueError('minimum_m must be finite and positive')
    unsafe = tuple(
        item
        for item in observed_separations(poses)
        if item.distance_m < minimum_m
    )
    if not unsafe:
        return None
    details = ', '.join(
        f'{item.first.value}/{item.second.value}={item.distance_m:.3f}m'
        for item in unsafe
    )
    return (
        f'emergency separation below {minimum_m:.2f} m: {details}'
    )
