"""Deterministic multi-phase fleet mission construction and execution."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin
from typing import Protocol

from omokai_fleet.allocation import (
    RoutePoint,
    partition_route,
    regroup_goals,
)
from omokai_fleet.formation import (
    formation_goals,
    validate_goal_separation,
)
from omokai_fleet.lifecycle import SquadLifecycle, SquadState
from omokai_fleet.model import (
    ROBOT_IDS,
    Formation,
    Pose2D,
    RobotGoal,
    SquadPlan,
)
from omokai_fleet.navigation import (
    NavigationBatch,
    apply_navigation_batch,
)


class FleetNavigation(Protocol):
    def execute(
        self,
        goals: tuple[RobotGoal, ...],
        *,
        speed_mps: float,
        timeout_sec: float,
    ) -> NavigationBatch:
        ...


@dataclass(frozen=True)
class SquadMission:
    plan: SquadPlan
    forming: tuple[RobotGoal, ...]
    formation_movement: tuple[tuple[RobotGoal, ...], ...]
    split_execution: tuple[tuple[RobotGoal, ...], ...]
    regrouping: tuple[RobotGoal, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.plan, SquadPlan):
            raise ValueError('plan must be a SquadPlan')
        _validate_batch(self.forming)
        if (
            not isinstance(self.formation_movement, tuple)
            or not self.formation_movement
        ):
            raise ValueError('formation_movement must contain at least one batch')
        for batch in self.formation_movement:
            _validate_batch(batch)
        if self.plan.split_route:
            if not isinstance(self.split_execution, tuple) or not self.split_execution:
                raise ValueError('split_execution is required by the plan')
            for batch in self.split_execution:
                _validate_batch(batch)
        elif self.split_execution:
            raise ValueError('split_execution must be empty when disabled')
        if self.plan.regroup:
            _validate_batch(self.regrouping)
        elif self.regrouping:
            raise ValueError('regrouping must be empty when disabled')


def build_squad_mission(
    plan: SquadPlan,
    *,
    formation_references: tuple[Pose2D, ...],
    route_points: tuple[RoutePoint, ...],
    home: Pose2D,
) -> SquadMission:
    """Compile trusted fleet intent and known poses into synchronized batches."""
    if not isinstance(plan, SquadPlan):
        raise ValueError('plan must be a SquadPlan')
    if (
        not isinstance(formation_references, tuple)
        or len(formation_references) < 2
    ):
        raise ValueError(
            'formation_references must contain forming and movement poses'
        )
    if not all(
        isinstance(reference, Pose2D)
        for reference in formation_references
    ):
        raise ValueError('every formation reference must be a Pose2D')
    if not isinstance(home, Pose2D):
        raise ValueError('home must be a Pose2D')

    forming = formation_goals(
        formation_references[0],
        plan.formation,
        plan.spacing_m,
        phase_id='forming',
        step_index=1,
    )
    movement = tuple(
        formation_goals(
            reference,
            plan.formation,
            plan.spacing_m,
            phase_id='formation_moving',
            step_index=index,
        )
        for index, reference in enumerate(
            formation_references[1:],
            start=1,
        )
    )
    split_batches = (
        _split_batches(route_points) if plan.split_route else ()
    )
    regrouping = (
        regroup_goals(
            _safe_regroup_reference(
                home,
                plan.formation,
                plan.spacing_m,
            ),
            plan.formation,
            plan.spacing_m,
        )
        if plan.regroup
        else ()
    )
    return SquadMission(
        plan=plan,
        forming=forming,
        formation_movement=movement,
        split_execution=split_batches,
        regrouping=regrouping,
    )


def execute_squad_mission(
    navigation: FleetNavigation,
    mission: SquadMission,
) -> SquadLifecycle:
    """Execute all enabled phases and return the terminal/current lifecycle."""
    if not isinstance(mission, SquadMission):
        raise ValueError('mission must be a SquadMission')
    lifecycle = SquadLifecycle()
    lifecycle.accept(mission.plan)

    if not _execute_phase(
        navigation,
        lifecycle,
        SquadState.FORMING,
        (mission.forming,),
        mission.plan,
    ):
        return lifecycle
    if not _execute_phase(
        navigation,
        lifecycle,
        SquadState.FORMATION_MOVING,
        mission.formation_movement,
        mission.plan,
    ):
        return lifecycle

    if mission.plan.split_route:
        lifecycle.prepare_split()
        if not _execute_phase(
            navigation,
            lifecycle,
            SquadState.EXECUTING_SPLIT,
            mission.split_execution,
            mission.plan,
        ):
            return lifecycle

    if mission.plan.regroup:
        if not _execute_phase(
            navigation,
            lifecycle,
            SquadState.REGROUPING,
            (mission.regrouping,),
            mission.plan,
        ):
            return lifecycle

    lifecycle.finish_success()
    return lifecycle


def _execute_phase(
    navigation: FleetNavigation,
    lifecycle: SquadLifecycle,
    phase: SquadState,
    batches: tuple[tuple[RobotGoal, ...], ...],
    plan: SquadPlan,
) -> bool:
    lifecycle.start_phase(phase)
    for index, goals in enumerate(batches):
        result = navigation.execute(
            goals,
            speed_mps=plan.speed_mps,
            timeout_sec=plan.goal_timeout_sec,
        )
        if not result.succeeded or index == len(batches) - 1:
            apply_navigation_batch(lifecycle, result)
        if not result.succeeded:
            return False
    return True


def _split_batches(
    route_points: tuple[RoutePoint, ...],
) -> tuple[tuple[RobotGoal, ...], ...]:
    assignments = partition_route(
        route_points,
        phase_id='executing_split',
    )
    batch_count = max(len(item.goals) for item in assignments)
    batches = []
    for batch_index in range(batch_count):
        batch = []
        for assignment in assignments:
            if batch_index < len(assignment.goals):
                batch.append(assignment.goals[batch_index])
                continue
            hold_pose = assignment.goals[-1].pose
            batch.append(
                RobotGoal(
                    phase_id='executing_split',
                    goal_id=(
                        'executing_split/'
                        f'{assignment.robot_id.value}/hold{batch_index + 1}'
                    ),
                    robot_id=assignment.robot_id,
                    pose=hold_pose,
                )
            )
        batch_tuple = tuple(batch)
        validate_goal_separation(batch_tuple)
        batches.append(batch_tuple)
    return tuple(batches)


def _safe_regroup_reference(
    home: Pose2D,
    formation: Formation,
    spacing_m: float,
) -> Pose2D:
    forward_shift = (
        2.0 * spacing_m
        if formation is Formation.LINE
        else spacing_m
    )
    return Pose2D(
        x=home.x + forward_shift * cos(home.yaw),
        y=home.y + forward_shift * sin(home.yaw),
        yaw=home.yaw,
    )


def _validate_batch(goals: tuple[RobotGoal, ...]) -> None:
    if not isinstance(goals, tuple) or len(goals) != 3:
        raise ValueError('every mission batch must contain three robot goals')
    if tuple(goal.robot_id for goal in goals) != ROBOT_IDS:
        raise ValueError('mission batch must use stable robot order')
    validate_goal_separation(goals)
