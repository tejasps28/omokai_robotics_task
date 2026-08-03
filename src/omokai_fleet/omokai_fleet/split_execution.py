"""Pure split-work and shared-reservation planning."""

from __future__ import annotations

from dataclasses import dataclass

from omokai_fleet.model import ROBOT_IDS, RobotGoal, RobotId
from omokai_fleet.navigation import NavigationStatus, is_hold_goal_id


@dataclass(frozen=True)
class RobotWorkSequence:
    robot_id: RobotId
    goals: tuple[RobotGoal, ...]

    def __post_init__(self) -> None:
        if self.robot_id not in ROBOT_IDS or not self.goals:
            raise ValueError('robot work requires a known robot and goals')
        if any(goal.robot_id is not self.robot_id for goal in self.goals):
            raise ValueError('robot work contains cross-robot ownership')


@dataclass(frozen=True)
class ReservationStep:
    step_index: int
    robot_ids: tuple[RobotId, ...]
    reservation_id: str = 'dock_corridor'
    goal_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class SplitExecutionPlan:
    work: tuple[RobotWorkSequence, ...]
    reservations: tuple[ReservationStep, ...]


@dataclass(frozen=True)
class SplitRobotResult:
    robot_id: RobotId
    status: NavigationStatus
    completed_goals: int
    skipped_goals: int = 0
    reason: str = ''


@dataclass(frozen=True)
class SplitExecutionResult:
    results: tuple[SplitRobotResult, ...]

    def __post_init__(self) -> None:
        if tuple(result.robot_id for result in self.results) != ROBOT_IDS:
            raise ValueError('split results must use stable robot order')

    @property
    def succeeded(self) -> bool:
        return all(
            result.status is NavigationStatus.SUCCEEDED
            for result in self.results
        )


class SplitReservationScheduler:
    """Release the next goal per robot when its reservation is available."""

    def __init__(self, plan: SplitExecutionPlan) -> None:
        if not isinstance(plan, SplitExecutionPlan):
            raise ValueError('plan must be a SplitExecutionPlan')
        self._pending = {
            item.robot_id: list(item.goals)
            for item in plan.work
        }
        self._reservation_by_goal = {
            goal_id: step.reservation_id
            for step in plan.reservations
            for goal_id in step.goal_ids
        }
        self._reservation_queues: dict[str, list[str]] = {}
        for step in plan.reservations:
            queue = self._reservation_queues.setdefault(step.reservation_id, [])
            queue.extend(step.goal_ids)
        self._active: dict[RobotId, str] = {}
        self._cleared: set[RobotId] = set()

    @property
    def complete(self) -> bool:
        return not self._active and all(not goals for goals in self._pending.values())

    @property
    def active_robot_ids(self) -> tuple[RobotId, ...]:
        return tuple(robot_id for robot_id in ROBOT_IDS if robot_id in self._active)

    def remaining_goals(self, robot_id: RobotId) -> tuple[RobotGoal, ...]:
        if robot_id not in self._pending:
            raise ValueError('unknown robot')
        return tuple(self._pending[robot_id])

    def failure_is_isolatable(self, robot_id: RobotId) -> bool:
        """Only a failure on the robot's private route is proven isolated."""
        reservation = self._active.get(robot_id)
        if reservation is None:
            raise ValueError('robot has no active reservation')
        return (
            reservation == f'route/{robot_id.value}'
            or robot_id in self._cleared
        )

    def active_reservation(self, robot_id: RobotId) -> str | None:
        """Return the active reservation, or None when the robot is idle."""
        if robot_id not in ROBOT_IDS:
            raise ValueError('unknown robot')
        return self._active.get(robot_id)

    def clear_active_reservation(self, robot_id: RobotId) -> None:
        """Release a corridor after clearance while its goal stays active."""
        reservation = self._active.get(robot_id)
        if reservation is None:
            raise ValueError('robot has no active reservation')
        if reservation != 'dock_departure':
            raise ValueError('only dock departures support early clearance')
        if robot_id in self._cleared:
            raise ValueError('reservation is already clear')
        goal = self._pending[robot_id][0]
        queue = self._reservation_queues[reservation]
        if not queue or queue[0] != goal.goal_id:
            raise RuntimeError('reservation release order was corrupted')
        queue.pop(0)
        self._cleared.add(robot_id)

    def ready_goals(self) -> tuple[RobotGoal, ...]:
        claimed = {
            reservation
            for robot_id, reservation in self._active.items()
            if robot_id not in self._cleared
        }
        ready = []
        for robot_id in ROBOT_IDS:
            goals = self._pending[robot_id]
            if robot_id in self._active or not goals:
                continue
            goal = goals[0]
            reservation = self._reservation_by_goal[goal.goal_id]
            if reservation in claimed:
                continue
            if self._reservation_queues[reservation][0] != goal.goal_id:
                continue
            claimed.add(reservation)
            ready.append(goal)
        return tuple(ready)

    def claim(self, goals: tuple[RobotGoal, ...]) -> None:
        for goal in goals:
            if not self._pending[goal.robot_id] or self._pending[goal.robot_id][0] != goal:
                raise ValueError('only ready goals may be claimed')
            reservation = self._reservation_by_goal[goal.goal_id]
            if any(
                active_reservation == reservation
                and active_robot not in self._cleared
                for active_robot, active_reservation in self._active.items()
            ):
                raise ValueError('reservation is already active')
            self._active[goal.robot_id] = reservation

    def release(self, robot_id: RobotId) -> None:
        if robot_id not in self._active:
            raise ValueError('robot has no active reservation')
        completed_goal = self._pending[robot_id].pop(0)
        if robot_id not in self._cleared:
            queue = self._reservation_queues[self._active[robot_id]]
            if not queue or queue[0] != completed_goal.goal_id:
                raise RuntimeError('reservation release order was corrupted')
            queue.pop(0)
        self._cleared.discard(robot_id)
        del self._active[robot_id]

    def abandon(self, robot_id: RobotId) -> tuple[RobotGoal, ...]:
        """Drop a failed robot's active and remaining work, returning skipped goals."""
        if robot_id not in self._active:
            raise ValueError('robot has no active reservation')
        skipped = tuple(self._pending[robot_id])
        self._pending[robot_id].clear()
        self._cleared.discard(robot_id)
        del self._active[robot_id]
        return skipped


def compile_split_execution(
    batches: tuple[tuple[RobotGoal, ...], ...],
) -> SplitExecutionPlan:
    """Separate real robot work from batch-level hold reservations."""
    if not isinstance(batches, tuple) or not batches:
        raise ValueError('split batches must be a non-empty tuple')
    work = {robot_id: [] for robot_id in ROBOT_IDS}
    reservations = []
    for step_index, batch in enumerate(batches, start=1):
        if (
            not isinstance(batch, tuple)
            or tuple(goal.robot_id for goal in batch) != ROBOT_IDS
        ):
            raise ValueError('every split batch must use stable robot order')
        moving = tuple(
            goal.robot_id for goal in batch if not is_hold_goal_id(goal.goal_id)
        )
        if not moving:
            raise ValueError('every split batch must contain real work')
        real_goals = tuple(
            goal for goal in batch if not is_hold_goal_id(goal.goal_id)
        )
        for goal in real_goals:
            if '/outbound' in goal.goal_id:
                reservation_id = 'dock_departure'
            elif '/inbound' in goal.goal_id:
                reservation_id = 'dock_arrival'
            else:
                reservation_id = f'route/{goal.robot_id.value}'
            work[goal.robot_id].append(goal)
            reservations.append(
                ReservationStep(
                    step_index,
                    (goal.robot_id,),
                    reservation_id,
                    (goal.goal_id,),
                )
            )
    if any(not work[robot_id] for robot_id in ROBOT_IDS):
        raise ValueError('split work must assign at least one goal to every robot')
    goal_ids = tuple(goal.goal_id for goals in work.values() for goal in goals)
    if len(goal_ids) != len(set(goal_ids)):
        raise ValueError('split work goal IDs must be unique')
    sequences = tuple(
        RobotWorkSequence(robot_id, tuple(work[robot_id]))
        for robot_id in ROBOT_IDS
    )
    return SplitExecutionPlan(sequences, tuple(reservations))
