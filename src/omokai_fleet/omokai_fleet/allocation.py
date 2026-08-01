"""Deterministic route allocation and separated regroup planning."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import permutations
from math import hypot

from omokai_fleet.formation import (
    formation_goals,
    validate_goal_separation,
)
from omokai_fleet.model import (
    ROBOT_IDS,
    Formation,
    Pose2D,
    RobotGoal,
    RobotId,
)


@dataclass(frozen=True)
class RoutePoint:
    point_id: str
    pose: Pose2D

    def __post_init__(self) -> None:
        if not isinstance(self.point_id, str) or not self.point_id.strip():
            raise ValueError('point_id must not be blank')
        if not isinstance(self.pose, Pose2D):
            raise ValueError('pose must be a Pose2D')


@dataclass(frozen=True)
class RouteAssignment:
    robot_id: RobotId
    goals: tuple[RobotGoal, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.robot_id, RobotId):
            raise ValueError('robot_id must be a known RobotId')
        if not isinstance(self.goals, tuple) or not self.goals:
            raise ValueError('goals must be a non-empty immutable tuple')
        if any(goal.robot_id is not self.robot_id for goal in self.goals):
            raise ValueError('every assigned goal must belong to robot_id')


def partition_route(
    points: tuple[RoutePoint, ...],
    *,
    phase_id: str = 'split_route',
    current_poses: dict[RobotId, Pose2D] | None = None,
) -> tuple[RouteAssignment, ...]:
    """Split into contiguous sections and assign them by approach distance."""
    if not isinstance(points, tuple) or len(points) < len(ROBOT_IDS):
        raise ValueError(
            'points must be an immutable tuple with at least one point per robot'
        )
    if not isinstance(phase_id, str) or not phase_id.strip():
        raise ValueError('phase_id must not be blank')
    if not all(isinstance(point, RoutePoint) for point in points):
        raise ValueError('every route item must be a RoutePoint')

    point_ids = tuple(point.point_id for point in points)
    if len(set(point_ids)) != len(point_ids):
        raise ValueError('route point IDs must be unique')
    if current_poses is not None and (
        not isinstance(current_poses, dict)
        or set(current_poses) != set(ROBOT_IDS)
        or not all(isinstance(pose, Pose2D) for pose in current_poses.values())
    ):
        raise ValueError('current_poses must contain one Pose2D per robot')

    base_count, remainder = divmod(len(points), len(ROBOT_IDS))
    sections = []
    start = 0
    for index in range(len(ROBOT_IDS)):
        count = base_count + (1 if index < remainder else 0)
        sections.append(points[start : start + count])
        start += count

    section_order = tuple(range(len(ROBOT_IDS)))
    if current_poses is not None:
        section_order = min(
            permutations(range(len(ROBOT_IDS))),
            key=lambda order: (
                sum(
                    hypot(
                        current_poses[robot_id].x - sections[section_index][0].pose.x,
                        current_poses[robot_id].y - sections[section_index][0].pose.y,
                    )
                    for robot_id, section_index in zip(ROBOT_IDS, order)
                ),
                order,
            ),
        )

    assignments = []
    for robot_id, section_index in zip(ROBOT_IDS, section_order):
        section = sections[section_index]
        goals = tuple(
            RobotGoal(
                phase_id=phase_id,
                goal_id=f'{phase_id}/{robot_id.value}/{point.point_id}',
                robot_id=robot_id,
                pose=point.pose,
            )
            for point in section
        )
        assignments.append(RouteAssignment(robot_id, goals))

    return tuple(assignments)


def regroup_goals(
    reference: Pose2D,
    formation: Formation,
    spacing_m: float,
    *,
    phase_id: str = 'regroup',
) -> tuple[RobotGoal, ...]:
    """Create three distinct formation goals around a rendezvous reference."""
    goals = formation_goals(
        reference,
        formation,
        spacing_m,
        phase_id=phase_id,
        step_index=1,
    )
    validate_goal_separation(goals)
    return goals
