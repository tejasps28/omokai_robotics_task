"""Deterministic multi-robot fleet planning and coordination."""

from omokai_fleet.allocation import (
    RouteAssignment,
    RoutePoint,
    partition_route,
    regroup_goals,
)
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
from omokai_fleet.lifecycle import (
    LifecycleError,
    PhaseRecord,
    RobotOutcome,
    RobotPhaseResult,
    SquadLifecycle,
    SquadSnapshot,
    SquadState,
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
    'LifecycleError',
    'PhaseRecord',
    'Pose2D',
    'RouteAssignment',
    'RoutePoint',
    'Robot',
    'RobotGoal',
    'RobotId',
    'RobotOutcome',
    'RobotPhaseResult',
    'SquadAction',
    'SquadLifecycle',
    'SquadPlan',
    'SquadSnapshot',
    'SquadState',
    'formation_goals',
    'formation_offsets',
    'goal_separations',
    'partition_route',
    'regroup_goals',
    'transform_offset',
    'validate_goal_separation',
]
