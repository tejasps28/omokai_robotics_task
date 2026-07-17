"""Explicit translation between compiler output and executor input."""

from omokai_executor import (
    ExecutionGoal,
    ExecutionPlan,
    GoalKind,
    GoalPose,
)
from omokai_interfaces import ValidatedMission
from omokai_mission import CompiledRoute


def to_execution_plan(
    compiled: CompiledRoute,
    validated: ValidatedMission,
) -> ExecutionPlan:
    """Translate an immutable compiled route into the executor contract."""

    if compiled.mission_id != validated.mission_id:
        raise ValueError('compiled and validated mission IDs do not match')
    if compiled.route_id != validated.mission.route_id:
        raise ValueError('compiled route ID does not match the validated mission')

    execution_goals = []
    for index, goal in enumerate(compiled.goals):
        is_home = goal.label == 'home'
        if is_home and index != len(compiled.goals) - 1:
            raise ValueError('home goal may appear only at the end of a plan')
        execution_goals.append(
            ExecutionGoal(
                goal_id=goal.label,
                pose=GoalPose(
                    x=goal.pose.x,
                    y=goal.pose.y,
                    yaw=goal.pose.yaw,
                    frame_id=goal.frame_id,
                ),
                kind=GoalKind.HOME if is_home else GoalKind.ROUTE,
            )
        )

    return ExecutionPlan(
        mission_id=compiled.mission_id,
        goals=tuple(execution_goals),
        speed_mps=validated.mission.speed_mps,
    )
