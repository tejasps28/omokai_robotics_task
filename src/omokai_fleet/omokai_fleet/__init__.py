"""Deterministic multi-robot fleet planning and coordination."""

from omokai_fleet.formation import (
    PLANNED_MINIMUM_DISTANCE_M,
    FormationOffset,
    GoalSeparation,
    formation_goals,
    formation_offsets,
    goal_separations,
    transform_offset,
    validate_goal_separation,
)
from omokai_fleet.model import (
    ROBOT_IDS,
    Formation,
    Pose2D,
    Robot,
    RobotGoal,
    RobotId,
    SquadAction,
    SquadPlan,
)

__all__ = [
    'PLANNED_MINIMUM_DISTANCE_M',
    'ROBOT_IDS',
    'Formation',
    'FormationOffset',
    'GoalSeparation',
    'Pose2D',
    'Robot',
    'RobotGoal',
    'RobotId',
    'SquadAction',
    'SquadPlan',
    'formation_goals',
    'formation_offsets',
    'goal_separations',
    'transform_offset',
    'validate_goal_separation',
]
