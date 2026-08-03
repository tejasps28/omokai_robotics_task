"""Deterministic multi-phase fleet mission construction and execution."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin
from time import monotonic
from typing import Callable, Protocol

from omokai_fleet.allocation import (
    RoutePoint,
    partition_route,
    regroup_goals,
)
from omokai_fleet.formation import (
    formation_goals,
    validate_goal_separation,
)
from omokai_fleet.lifecycle import RobotOutcome, SquadLifecycle, SquadState
from omokai_fleet.model import (
    ROBOT_IDS,
    Formation,
    Pose2D,
    RobotGoal,
    SquadPlan,
)
from omokai_fleet.navigation import (
    NavigationBatch,
    NavigationResult,
    NavigationStatus,
    apply_navigation_batch,
)
from omokai_fleet.split_execution import (
    SplitExecutionResult,
    SplitExecutionPlan,
    SplitReservationScheduler,
    compile_split_execution,
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

    def execute_formation(
        self,
        batches: tuple[tuple[RobotGoal, ...], ...],
        *,
        formation: Formation,
        spacing_m: float,
        speed_mps: float,
        timeout_sec: float,
    ) -> NavigationBatch:
        """Move the leader path while followers track continuously."""
        ...

    def execute_split(
        self,
        plan: SplitExecutionPlan,
        *,
        speed_mps: float,
        timeout_sec: float,
    ) -> SplitExecutionResult:
        ...


@dataclass(frozen=True)
class SquadMission:
    plan: SquadPlan
    forming: tuple[RobotGoal, ...]
    formation_movement: tuple[tuple[RobotGoal, ...], ...]
    split_execution: tuple[tuple[RobotGoal, ...], ...]
    regrouping: tuple[RobotGoal, ...]

    @property
    def split_plan(self) -> SplitExecutionPlan | None:
        return (
            compile_split_execution(self.split_execution)
            if self.split_execution
            else None
        )

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
            compile_split_execution(self.split_execution)
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
    *,
    clock: Callable[[], float] = monotonic,
) -> SquadLifecycle:
    """Execute all enabled phases and return the terminal/current lifecycle."""
    if not isinstance(mission, SquadMission):
        raise ValueError('mission must be a SquadMission')
    lifecycle = SquadLifecycle()
    lifecycle.accept(mission.plan)
    mission_deadline = clock() + mission.plan.mission_timeout_sec

    if not _execute_phase(
        navigation,
        lifecycle,
        SquadState.FORMING,
        (mission.forming,),
        mission.plan,
        mission_deadline,
        clock,
    ):
        return lifecycle
    if not _execute_continuous_formation(
        navigation,
        lifecycle,
        mission.formation_movement,
        mission.plan,
        mission_deadline,
        clock,
    ):
        return lifecycle

    if mission.plan.split_route:
        lifecycle.prepare_split()
        if not _execute_split_phase(
            navigation,
            lifecycle,
            mission.split_plan,
            mission.plan,
            mission_deadline,
            clock,
        ):
            return lifecycle

    if mission.plan.regroup:
        if not _execute_phase(
            navigation,
            lifecycle,
            SquadState.REGROUPING,
            (mission.regrouping,),
            mission.plan,
            mission_deadline,
            clock,
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
    mission_deadline: float,
    clock: Callable[[], float],
) -> bool:
    lifecycle.start_phase(phase)
    for index, goals in enumerate(batches):
        remaining_sec = mission_deadline - clock()
        if remaining_sec <= 0:
            _finish_undispatched_timeout(lifecycle)
            return False
        result = navigation.execute(
            goals,
            speed_mps=plan.speed_mps,
            timeout_sec=min(plan.goal_timeout_sec, remaining_sec),
        )
        if not result.succeeded or index == len(batches) - 1:
            apply_navigation_batch(lifecycle, result)
        if not result.succeeded:
            return False
    return True


def _execute_split_phase(
    navigation: FleetNavigation,
    lifecycle: SquadLifecycle,
    split_plan: SplitExecutionPlan | None,
    plan: SquadPlan,
    mission_deadline: float,
    clock: Callable[[], float],
) -> bool:
    if split_plan is None:
        raise ValueError('split execution requires a compiled plan')
    lifecycle.start_phase(SquadState.EXECUTING_SPLIT)
    execute_split = getattr(navigation, 'execute_split', None)
    if execute_split is not None:
        remaining_sec = mission_deadline - clock()
        if remaining_sec <= 0:
            _finish_undispatched_timeout(lifecycle)
            return False
        result = execute_split(
            split_plan,
            speed_mps=plan.speed_mps,
            timeout_sec=remaining_sec,
        )
        if result.succeeded:
            for robot_id in ROBOT_IDS:
                lifecycle.record_success(robot_id)
            return True
        outcomes = {}
        for item in result.results:
            if item.status is NavigationStatus.SUCCEEDED:
                outcomes[item.robot_id] = (RobotOutcome.SUCCEEDED, '')
            elif item.status is NavigationStatus.TIMED_OUT:
                outcomes[item.robot_id] = (RobotOutcome.TIMED_OUT, item.reason)
            elif item.status is NavigationStatus.CANCELLED:
                outcomes[item.robot_id] = (RobotOutcome.CANCELLED, item.reason)
            else:
                outcomes[item.robot_id] = (RobotOutcome.FAILED, item.reason)
        lifecycle.finish_partial_split(outcomes)
        return False

    # Compatibility path for transports that only implement synchronized
    # batches. Production Nav2 uses execute_split above.
    scheduler = SplitReservationScheduler(split_plan)
    last_pose = {
        item.robot_id: item.goals[0].pose
        for item in split_plan.work
    }
    while not scheduler.complete:
        ready = scheduler.ready_goals()
        if not ready:
            raise RuntimeError('split reservation scheduler deadlocked')
        scheduler.claim(ready)
        by_robot = {goal.robot_id: goal for goal in ready}
        goals = tuple(
            by_robot.get(robot_id)
            or RobotGoal(
                'executing_split',
                f'executing_split/runtime/{robot_id.value}/hold',
                robot_id,
                last_pose[robot_id],
            )
            for robot_id in ROBOT_IDS
        )
        remaining_sec = mission_deadline - clock()
        if remaining_sec <= 0:
            _finish_undispatched_timeout(lifecycle)
            return False
        result = navigation.execute(
            goals,
            speed_mps=plan.speed_mps,
            timeout_sec=min(plan.goal_timeout_sec, remaining_sec),
        )
        if not result.succeeded:
            apply_navigation_batch(lifecycle, result)
            return False
        for goal in ready:
            last_pose[goal.robot_id] = goal.pose
            scheduler.release(goal.robot_id)
    apply_navigation_batch(
        lifecycle,
        NavigationBatch(
            tuple(
                NavigationResult(robot_id, NavigationStatus.SUCCEEDED)
                for robot_id in ROBOT_IDS
            )
        ),
    )
    return True


def _execute_continuous_formation(
    navigation: FleetNavigation,
    lifecycle: SquadLifecycle,
    batches: tuple[tuple[RobotGoal, ...], ...],
    plan: SquadPlan,
    mission_deadline: float,
    clock: Callable[[], float],
) -> bool:
    """Run one leader path with continuous followers and one final barrier."""
    lifecycle.start_phase(SquadState.FORMATION_MOVING)
    remaining_sec = mission_deadline - clock()
    if remaining_sec <= 0:
        _finish_undispatched_timeout(lifecycle)
        return False
    result = navigation.execute_formation(
        batches,
        formation=plan.formation,
        spacing_m=plan.spacing_m,
        speed_mps=plan.speed_mps,
        timeout_sec=min(
            plan.goal_timeout_sec * len(batches),
            remaining_sec,
        ),
    )
    apply_navigation_batch(lifecycle, result)
    return result.succeeded


def _finish_undispatched_timeout(lifecycle: SquadLifecycle) -> None:
    reason = 'squad mission timed out before the next batch was dispatched'
    lifecycle.request_timeout(reason)
    for robot_id in ROBOT_IDS:
        lifecycle.record_cancelled(robot_id, reason)
    lifecycle.finish_cancellation()


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
