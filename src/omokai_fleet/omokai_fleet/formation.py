"""Deterministic formation geometry and planned-separation checks."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from math import cos, hypot, isfinite, sin

from omokai_fleet.model import (
    ROBOT_IDS,
    Formation,
    Pose2D,
    RobotGoal,
    RobotId,
)


PLANNED_MINIMUM_DISTANCE_M = 0.50


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and isfinite(value)
    )


@dataclass(frozen=True)
class FormationOffset:
    robot_id: RobotId
    forward_m: float
    left_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.robot_id, RobotId):
            raise ValueError('robot_id must be a known RobotId')
        if not all(
            _is_finite_number(value)
            for value in (self.forward_m, self.left_m)
        ):
            raise ValueError('formation offsets must be finite numbers')


@dataclass(frozen=True)
class GoalSeparation:
    first: RobotId
    second: RobotId
    distance_m: float


def formation_offsets(
    formation: Formation,
    spacing_m: float,
) -> tuple[FormationOffset, ...]:
    """Return stable robot-local offsets for a supported formation."""
    if not isinstance(formation, Formation):
        raise ValueError('formation must be a supported Formation')
    if (
        not _is_finite_number(spacing_m)
        or not 0.60 <= spacing_m <= 1.20
    ):
        raise ValueError('spacing_m must be between 0.60 and 1.20')

    if formation is Formation.LINE:
        coordinates = (
            (0.0, 0.0),
            (-spacing_m, 0.0),
            (-2.0 * spacing_m, 0.0),
        )
    else:
        coordinates = (
            (0.0, 0.0),
            (-spacing_m, spacing_m),
            (-spacing_m, -spacing_m),
        )
    return tuple(
        FormationOffset(robot_id, forward_m, left_m)
        for robot_id, (forward_m, left_m) in zip(
            ROBOT_IDS,
            coordinates,
        )
    )


def transform_offset(
    reference: Pose2D,
    offset: FormationOffset,
) -> Pose2D:
    """Rotate a robot-local formation offset into the shared map frame."""
    if not isinstance(reference, Pose2D):
        raise ValueError('reference must be a Pose2D')
    if not isinstance(offset, FormationOffset):
        raise ValueError('offset must be a FormationOffset')

    return Pose2D(
        x=(
            reference.x
            + offset.forward_m * cos(reference.yaw)
            - offset.left_m * sin(reference.yaw)
        ),
        y=(
            reference.y
            + offset.forward_m * sin(reference.yaw)
            + offset.left_m * cos(reference.yaw)
        ),
        yaw=reference.yaw,
        frame_id=reference.frame_id,
    )


def formation_goals(
    reference: Pose2D,
    formation: Formation,
    spacing_m: float,
    *,
    phase_id: str,
    step_index: int,
) -> tuple[RobotGoal, ...]:
    """Create one stable, correlated goal per robot."""
    if not isinstance(step_index, int) or isinstance(step_index, bool):
        raise ValueError('step_index must be a positive integer')
    if step_index < 1:
        raise ValueError('step_index must be a positive integer')

    return tuple(
        RobotGoal(
            phase_id=phase_id,
            goal_id=(
                f'{phase_id}/step{step_index}/{offset.robot_id.value}'
            ),
            robot_id=offset.robot_id,
            pose=transform_offset(reference, offset),
        )
        for offset in formation_offsets(formation, spacing_m)
    )


def goal_separations(
    goals: tuple[RobotGoal, ...],
) -> tuple[GoalSeparation, ...]:
    """Return all unordered pairwise distances in stable robot order."""
    if not isinstance(goals, tuple) or len(goals) < 2:
        raise ValueError('goals must be an immutable tuple with two or more goals')
    robot_ids = tuple(goal.robot_id for goal in goals)
    if len(set(robot_ids)) != len(robot_ids):
        raise ValueError('goals must not contain duplicate robot IDs')

    return tuple(
        GoalSeparation(
            first.robot_id,
            second.robot_id,
            hypot(
                first.pose.x - second.pose.x,
                first.pose.y - second.pose.y,
            ),
        )
        for first, second in combinations(goals, 2)
    )


def validate_goal_separation(
    goals: tuple[RobotGoal, ...],
    minimum_m: float = PLANNED_MINIMUM_DISTANCE_M,
) -> tuple[GoalSeparation, ...]:
    """Return distances or reject a planned separation violation."""
    if not _is_finite_number(minimum_m) or minimum_m <= 0:
        raise ValueError('minimum_m must be finite and positive')

    separations = goal_separations(goals)
    unsafe = tuple(
        separation
        for separation in separations
        if separation.distance_m < minimum_m
    )
    if unsafe:
        pairs = ', '.join(
            f'{item.first.value}/{item.second.value}'
            for item in unsafe
        )
        raise ValueError(
            f'planned robot separation below {minimum_m:.2f} m: {pairs}'
        )
    return separations
