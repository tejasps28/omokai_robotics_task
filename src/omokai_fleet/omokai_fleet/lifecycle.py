"""Pure synchronized lifecycle for a three-robot squad mission."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from omokai_fleet.model import ROBOT_IDS, RobotId, SquadPlan


class SquadState(str, Enum):
    IDLE = 'idle'
    VALIDATED = 'validated'
    FORMING = 'forming'
    FORMATION_MOVING = 'formation_moving'
    SPLITTING = 'splitting'
    EXECUTING_SPLIT = 'executing_split'
    REGROUPING = 'regrouping'
    CANCELLING = 'cancelling'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    TIMED_OUT = 'timed_out'
    CANCELLED = 'cancelled'

    @property
    def active_phase(self) -> bool:
        return self in {
            SquadState.FORMING,
            SquadState.FORMATION_MOVING,
            SquadState.EXECUTING_SPLIT,
            SquadState.REGROUPING,
        }

    @property
    def terminal(self) -> bool:
        return self in {
            SquadState.SUCCEEDED,
            SquadState.FAILED,
            SquadState.TIMED_OUT,
            SquadState.CANCELLED,
        }


class RobotOutcome(str, Enum):
    ACTIVE = 'active'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    TIMED_OUT = 'timed_out'
    CANCELLED = 'cancelled'


@dataclass(frozen=True)
class RobotPhaseResult:
    phase: SquadState
    robot_id: RobotId
    outcome: RobotOutcome
    reason: str = ''

    def __post_init__(self) -> None:
        if not isinstance(self.phase, SquadState) or not self.phase.active_phase:
            raise ValueError('phase must be an active squad phase')
        if not isinstance(self.robot_id, RobotId):
            raise ValueError('robot_id must be a known RobotId')
        if not isinstance(self.outcome, RobotOutcome):
            raise ValueError('outcome must be a RobotOutcome')
        if not isinstance(self.reason, str):
            raise ValueError('reason must be a string')
        if (
            self.outcome
            in {
                RobotOutcome.FAILED,
                RobotOutcome.TIMED_OUT,
                RobotOutcome.CANCELLED,
            }
            and not self.reason.strip()
        ):
            raise ValueError('unsuccessful outcomes require a reason')


@dataclass(frozen=True)
class PhaseRecord:
    phase: SquadState
    results: tuple[RobotPhaseResult, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.phase, SquadState) or not self.phase.active_phase:
            raise ValueError('phase must be an active squad phase')
        if not isinstance(self.results, tuple) or len(self.results) != 3:
            raise ValueError('phase results must contain exactly three robots')
        if tuple(item.robot_id for item in self.results) != ROBOT_IDS:
            raise ValueError('phase results must use stable robot order')
        if any(item.phase is not self.phase for item in self.results):
            raise ValueError('all results must belong to the phase')


@dataclass(frozen=True)
class SquadSnapshot:
    state: SquadState
    plan_id: str | None
    current_results: tuple[RobotPhaseResult, ...]
    history: tuple[PhaseRecord, ...]
    reason: str = ''


class LifecycleError(RuntimeError):
    """Raised when a caller violates squad lifecycle order."""


class SquadLifecycle:
    """Coordinate pure phase barriers without importing ROS."""

    def __init__(self) -> None:
        self._state = SquadState.IDLE
        self._plan: SquadPlan | None = None
        self._current: tuple[RobotPhaseResult, ...] = ()
        self._history: list[PhaseRecord] = []
        self._reason = ''
        self._terminal_target: SquadState | None = None

    @property
    def state(self) -> SquadState:
        return self._state

    @property
    def barrier_complete(self) -> bool:
        return bool(self._current) and all(
            item.outcome is RobotOutcome.SUCCEEDED
            for item in self._current
        )

    def snapshot(self) -> SquadSnapshot:
        return SquadSnapshot(
            state=self._state,
            plan_id=self._plan.plan_id if self._plan else None,
            current_results=self._current,
            history=tuple(self._history),
            reason=self._reason,
        )

    def accept(self, plan: SquadPlan) -> None:
        if self._state is not SquadState.IDLE:
            raise LifecycleError('a squad plan can only be accepted from idle')
        if not isinstance(plan, SquadPlan):
            raise ValueError('plan must be a SquadPlan')
        self._plan = plan
        self._state = SquadState.VALIDATED

    def start_phase(self, phase: SquadState) -> None:
        if not isinstance(phase, SquadState) or not phase.active_phase:
            raise ValueError('phase must be an active squad phase')
        expected = self._expected_phase()
        if phase is not expected:
            raise LifecycleError(
                f'expected phase {expected.value}, received {phase.value}'
            )
        if self._current:
            if not self.barrier_complete:
                raise LifecycleError(
                    'all robots must succeed before the next phase'
                )
            self._archive_current()
        self._state = phase
        self._current = tuple(
            RobotPhaseResult(phase, robot_id, RobotOutcome.ACTIVE)
            for robot_id in ROBOT_IDS
        )

    def prepare_split(self) -> None:
        self._require_state(SquadState.FORMATION_MOVING)
        self._require_barrier()
        if not self._require_plan().split_route:
            raise LifecycleError('the accepted plan does not request a split')
        self._archive_current()
        self._state = SquadState.SPLITTING

    def record_success(self, robot_id: RobotId) -> None:
        if not self._state.active_phase:
            raise LifecycleError('success requires an active squad phase')
        self._replace_active(robot_id, RobotOutcome.SUCCEEDED)

    def record_failure(
        self,
        robot_id: RobotId,
        reason: str,
        *,
        timed_out: bool = False,
    ) -> None:
        if not self._state.active_phase:
            raise LifecycleError('failure requires an active squad phase')
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError('failure reason must not be blank')
        outcome = (
            RobotOutcome.TIMED_OUT if timed_out else RobotOutcome.FAILED
        )
        self._replace_active(robot_id, outcome, reason)
        self._reason = reason
        self._terminal_target = (
            SquadState.TIMED_OUT if timed_out else SquadState.FAILED
        )
        self._state = SquadState.CANCELLING

    def request_cancel(self, reason: str = 'operator requested cancellation') -> None:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError('cancellation reason must not be blank')
        if (
            self._state.terminal
            or self._state in {SquadState.IDLE, SquadState.CANCELLING}
        ):
            raise LifecycleError('the squad is not cancellable in its current state')
        self._reason = reason
        self._terminal_target = SquadState.CANCELLED
        if not self._current:
            self._state = SquadState.CANCELLED
            return
        self._state = SquadState.CANCELLING

    def request_timeout(self, reason: str = 'squad mission timed out') -> None:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError('timeout reason must not be blank')
        if not self._state.active_phase:
            raise LifecycleError('mission timeout requires an active phase')
        self._reason = reason
        self._terminal_target = SquadState.TIMED_OUT
        self._state = SquadState.CANCELLING

    def record_cancelled(
        self,
        robot_id: RobotId,
        reason: str = 'active goal cancelled',
    ) -> None:
        self._require_state(SquadState.CANCELLING)
        self._replace_active(robot_id, RobotOutcome.CANCELLED, reason)

    def record_cancellation_failed(
        self,
        robot_id: RobotId,
        reason: str,
    ) -> None:
        self._require_state(SquadState.CANCELLING)
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError('cancellation failure reason must not be blank')
        self._replace_active(robot_id, RobotOutcome.FAILED, reason)
        if self._terminal_target is SquadState.CANCELLED:
            self._terminal_target = SquadState.FAILED
            self._reason = reason

    def finish_cancellation(self) -> None:
        self._require_state(SquadState.CANCELLING)
        if any(
            item.outcome is RobotOutcome.ACTIVE
            for item in self._current
        ):
            raise LifecycleError(
                'all active robot goals must resolve before cancellation ends'
            )
        if self._terminal_target is None:
            raise LifecycleError('cancellation has no terminal target')
        self._archive_current()
        self._state = self._terminal_target
        self._terminal_target = None

    def finish_success(self) -> None:
        self._require_barrier()
        plan = self._require_plan()
        valid_terminal_phase = (
            self._state is SquadState.REGROUPING
            or (
                self._state is SquadState.EXECUTING_SPLIT
                and not plan.regroup
            )
            or (
                self._state is SquadState.FORMATION_MOVING
                and not plan.split_route
                and not plan.regroup
            )
        )
        if not valid_terminal_phase:
            raise LifecycleError('required squad phases are not complete')
        self._archive_current()
        self._state = SquadState.SUCCEEDED

    def finish_partial_split(
        self,
        outcomes: dict[RobotId, tuple[RobotOutcome, str]],
    ) -> None:
        """Finish split after isolated failures and completed survivor work."""
        self._require_state(SquadState.EXECUTING_SPLIT)
        if set(outcomes) != set(ROBOT_IDS):
            raise ValueError('split outcomes must cover every robot')
        for robot_id in ROBOT_IDS:
            outcome, reason = outcomes[robot_id]
            self._replace_active(robot_id, outcome, reason)
        failures = tuple(
            item for item in self._current
            if item.outcome in {RobotOutcome.FAILED, RobotOutcome.TIMED_OUT}
        )
        if not failures:
            raise LifecycleError('partial split requires at least one failure')
        self._reason = '; '.join(
            f'{item.robot_id.value}: {item.reason}' for item in failures
        )
        self._archive_current()
        self._state = (
            SquadState.TIMED_OUT
            if any(item.outcome is RobotOutcome.TIMED_OUT for item in failures)
            else SquadState.FAILED
        )

    def _expected_phase(self) -> SquadState:
        plan = self._require_plan()
        if self._state is SquadState.VALIDATED:
            return SquadState.FORMING
        if self._state is SquadState.FORMING:
            return SquadState.FORMATION_MOVING
        if self._state is SquadState.SPLITTING:
            return SquadState.EXECUTING_SPLIT
        if (
            self._state is SquadState.FORMATION_MOVING
            and not plan.split_route
            and plan.regroup
        ):
            return SquadState.REGROUPING
        if self._state is SquadState.EXECUTING_SPLIT and plan.regroup:
            return SquadState.REGROUPING
        raise LifecycleError(
            f'no phase may start from state {self._state.value}'
        )

    def _replace_active(
        self,
        robot_id: RobotId,
        outcome: RobotOutcome,
        reason: str = '',
    ) -> None:
        if not isinstance(robot_id, RobotId):
            raise ValueError('robot_id must be a known RobotId')
        if not self._state.active_phase and self._state is not SquadState.CANCELLING:
            raise LifecycleError('no robot phase is active')
        index = ROBOT_IDS.index(robot_id)
        current = self._current[index]
        if current.outcome is not RobotOutcome.ACTIVE:
            raise LifecycleError(
                f'{robot_id.value} already resolved as {current.outcome.value}'
            )
        updated = list(self._current)
        updated[index] = RobotPhaseResult(
            current.phase,
            robot_id,
            outcome,
            reason,
        )
        self._current = tuple(updated)

    def _archive_current(self) -> None:
        if self._current:
            self._history.append(
                PhaseRecord(self._current[0].phase, self._current)
            )
            self._current = ()

    def _require_barrier(self) -> None:
        if not self._state.active_phase or not self.barrier_complete:
            raise LifecycleError('the current all-robot barrier is incomplete')

    def _require_plan(self) -> SquadPlan:
        if self._plan is None:
            raise LifecycleError('no squad plan has been accepted')
        return self._plan

    def _require_state(self, expected: SquadState) -> None:
        if self._state is not expected:
            raise LifecycleError(
                f'expected state {expected.value}, found {self._state.value}'
            )
