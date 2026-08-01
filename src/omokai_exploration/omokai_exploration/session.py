"""ROS-independent exploration lifecycle and failure policy."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from time import monotonic
from typing import Optional, Protocol

from .frontier import FrontierCandidate, FrontierConfig, rank_frontiers
from .grid import OccupancyGrid


class ExplorationState(str, Enum):
    CREATED = 'created'
    WAITING_FOR_MAP = 'waiting_for_map'
    NAVIGATING = 'navigating'
    CANCELLING = 'cancelling'
    COMPLETED = 'completed'
    CANCELLED = 'cancelled'
    TIMED_OUT = 'timed_out'
    FAILED = 'failed'

    @property
    def terminal(self) -> bool:
        return self in {
            ExplorationState.COMPLETED,
            ExplorationState.CANCELLED,
            ExplorationState.TIMED_OUT,
            ExplorationState.FAILED,
        }


class CancellationCause(str, Enum):
    OPERATOR = 'operator'
    GOAL_TIMEOUT = 'goal_timeout'
    MISSION_TIMEOUT = 'mission_timeout'


@dataclass(frozen=True)
class ExplorationConfig:
    frontier: FrontierConfig = FrontierConfig()
    goal_timeout_sec: float = 120.0
    mission_timeout_sec: float = 900.0
    map_timeout_sec: float = 30.0
    cancel_timeout_sec: float = 10.0
    completion_confirmations: int = 3
    max_failed_goals: int = 8
    max_completed_goals: Optional[int] = None

    def __post_init__(self) -> None:
        if not isinstance(self.frontier, FrontierConfig):
            raise ValueError('frontier must be a FrontierConfig')
        for name, value in (
            ('goal_timeout_sec', self.goal_timeout_sec),
            ('mission_timeout_sec', self.mission_timeout_sec),
            ('map_timeout_sec', self.map_timeout_sec),
            ('cancel_timeout_sec', self.cancel_timeout_sec),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f'{name} must be a positive finite number')
        for name, value in (
            ('completion_confirmations', self.completion_confirmations),
            ('max_failed_goals', self.max_failed_goals),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or value <= 0
            ):
                raise ValueError(f'{name} must be a positive integer')
        if self.max_completed_goals is not None and (
            isinstance(self.max_completed_goals, bool)
            or not isinstance(self.max_completed_goals, int)
            or self.max_completed_goals <= 0
        ):
            raise ValueError(
                'max_completed_goals must be a positive integer or None'
            )


@dataclass(frozen=True)
class ExplorationEvent:
    sequence: int
    event_type: str
    state: ExplorationState
    elapsed_sec: float
    details: tuple[tuple[str, object], ...] = ()


@dataclass(frozen=True)
class ExplorationResult:
    exploration_id: str
    state: ExplorationState
    reason: str
    completed_goals: int
    failed_goals: int
    blacklisted_points: tuple[tuple[float, float], ...]
    visited_points: tuple[tuple[float, float], ...]
    duration_sec: float


@dataclass(frozen=True)
class ExplorationSnapshot:
    exploration_id: str
    state: ExplorationState
    completed_goals: int
    failed_goals: int
    visited_count: int
    blacklist_count: int
    active_goal_handle: Optional[str]


class ExplorationNavigation(Protocol):
    def dispatch(self, candidate: FrontierCandidate) -> str:
        """Submit one frontier candidate and return a unique goal handle."""

    def cancel(self, goal_handle: str) -> None:
        """Request cancellation of the active navigation goal."""


class ExplorationClock(Protocol):
    def monotonic(self) -> float:
        """Return monotonic seconds for timeout calculations."""


class SystemExplorationClock:
    def monotonic(self) -> float:
        return monotonic()


class ExplorationSession:
    """Coordinate frontier selection and navigation through explicit events."""

    def __init__(
        self,
        exploration_id: str,
        navigation: ExplorationNavigation,
        *,
        config: ExplorationConfig = ExplorationConfig(),
        clock: Optional[ExplorationClock] = None,
    ) -> None:
        if not exploration_id.strip():
            raise ValueError('exploration_id must not be blank')
        self._exploration_id = exploration_id
        self._navigation = navigation
        self._config = config
        self._clock = clock or SystemExplorationClock()
        self._state = ExplorationState.CREATED
        self._started_at: Optional[float] = None
        self._ended_at: Optional[float] = None
        self._mission_deadline: Optional[float] = None
        self._map_deadline: Optional[float] = None
        self._goal_deadline: Optional[float] = None
        self._cancel_deadline: Optional[float] = None
        self._active_handle: Optional[str] = None
        self._active_candidate: Optional[FrontierCandidate] = None
        self._cancel_cause: Optional[CancellationCause] = None
        self._completion_count = 0
        self._completed_goals = 0
        self._failed_goals = 0
        self._blacklist: list[tuple[float, float]] = []
        self._visited: list[tuple[float, float]] = []
        self._reason = ''
        self._events: list[ExplorationEvent] = []
        self._emit('session_created')

    @property
    def state(self) -> ExplorationState:
        return self._state

    @property
    def active_goal_handle(self) -> Optional[str]:
        return self._active_handle

    @property
    def active_candidate(self) -> Optional[FrontierCandidate]:
        return self._active_candidate

    @property
    def blacklist(self) -> tuple[tuple[float, float], ...]:
        return tuple(self._blacklist)

    @property
    def visited(self) -> tuple[tuple[float, float], ...]:
        return tuple(self._visited)

    @property
    def events(self) -> tuple[ExplorationEvent, ...]:
        return tuple(self._events)

    @property
    def result(self) -> Optional[ExplorationResult]:
        if not self._state.terminal:
            return None
        assert self._started_at is not None
        assert self._ended_at is not None
        return ExplorationResult(
            exploration_id=self._exploration_id,
            state=self._state,
            reason=self._reason,
            completed_goals=self._completed_goals,
            failed_goals=self._failed_goals,
            blacklisted_points=tuple(self._blacklist),
            visited_points=tuple(self._visited),
            duration_sec=self._ended_at - self._started_at,
        )

    @property
    def snapshot(self) -> ExplorationSnapshot:
        return ExplorationSnapshot(
            exploration_id=self._exploration_id,
            state=self._state,
            completed_goals=self._completed_goals,
            failed_goals=self._failed_goals,
            visited_count=len(self._visited),
            blacklist_count=len(self._blacklist),
            active_goal_handle=self._active_handle,
        )

    def start(self) -> None:
        self._require_state(ExplorationState.CREATED)
        now = self._clock.monotonic()
        self._started_at = now
        self._mission_deadline = now + self._config.mission_timeout_sec
        self._map_deadline = now + self._config.map_timeout_sec
        self._transition(ExplorationState.WAITING_FOR_MAP)

    def observe_map(
        self,
        grid: OccupancyGrid,
        robot_x: float,
        robot_y: float,
    ) -> bool:
        """Evaluate one map and dispatch a frontier; return true if dispatched."""

        self._require_state(ExplorationState.WAITING_FOR_MAP)
        if self._mission_expired():
            self._finish(ExplorationState.TIMED_OUT, 'mission_timeout')
            return False

        now = self._clock.monotonic()
        self._map_deadline = now + self._config.map_timeout_sec
        candidates = rank_frontiers(
            grid,
            robot_x,
            robot_y,
            config=self._config.frontier,
            blacklist=self._blacklist,
            visited=self._visited,
        )
        self._emit(
            'map_evaluated',
            candidate_count=len(candidates),
            blacklist_count=len(self._blacklist),
        )
        if not candidates:
            self._completion_count += 1
            self._emit(
                'no_frontier_observed',
                confirmation=self._completion_count,
                required=self._config.completion_confirmations,
            )
            if self._completion_count >= self._config.completion_confirmations:
                self._finish(ExplorationState.COMPLETED, 'no_frontiers')
            return False

        self._completion_count = 0
        candidate = candidates[0]
        self._emit(
            'frontier_selected',
            goal_x=candidate.goal_x,
            goal_y=candidate.goal_y,
            score=candidate.score,
        )
        try:
            handle = self._navigation.dispatch(candidate)
            if not handle.strip():
                raise ValueError('navigation returned a blank goal handle')
        except Exception as exc:
            self._finish(
                ExplorationState.FAILED,
                'navigation_dispatch_error',
                error=str(exc),
            )
            return False

        self._active_candidate = candidate
        self._active_handle = handle
        self._goal_deadline = now + self._config.goal_timeout_sec
        self._transition(
            ExplorationState.NAVIGATING,
            goal_handle=handle,
            goal_x=candidate.goal_x,
            goal_y=candidate.goal_y,
        )
        return True

    def navigation_succeeded(self, goal_handle: str) -> None:
        self._require_navigation_handle(goal_handle)
        assert self._active_candidate is not None
        self._visited.append(
            (
                self._active_candidate.goal_x,
                self._active_candidate.goal_y,
            )
        )
        self._completed_goals += 1
        self._emit(
            'navigation_succeeded',
            goal_handle=goal_handle,
            visited_count=len(self._visited),
        )
        self._clear_active_goal()
        if (
            self._config.max_completed_goals is not None
            and self._completed_goals >= self._config.max_completed_goals
        ):
            self._finish(ExplorationState.COMPLETED, 'goal_limit')
            return
        self._wait_for_map()

    def navigation_failed(self, goal_handle: str, reason: str) -> None:
        self._require_navigation_handle(goal_handle)
        self._emit(
            'navigation_failed',
            goal_handle=goal_handle,
            reason=reason,
        )
        self._record_active_failure()
        if self._failed_goals >= self._config.max_failed_goals:
            self._finish(ExplorationState.FAILED, 'failure_limit')
            return
        self._wait_for_map()

    def request_cancel(self) -> bool:
        if self._state.terminal or self._state is ExplorationState.CANCELLING:
            return False
        if self._state in {
            ExplorationState.CREATED,
            ExplorationState.WAITING_FOR_MAP,
        }:
            if self._started_at is None:
                self._started_at = self._clock.monotonic()
            self._finish(ExplorationState.CANCELLED, 'operator_cancelled')
            return True
        self._require_state(ExplorationState.NAVIGATING)
        return self._begin_cancellation(CancellationCause.OPERATOR)

    def cancellation_confirmed(self, goal_handle: str) -> None:
        self._require_cancelling_handle(goal_handle)
        cause = self._cancel_cause
        assert cause is not None
        self._emit(
            'cancellation_confirmed',
            goal_handle=goal_handle,
            cause=cause.value,
        )
        if cause is CancellationCause.GOAL_TIMEOUT:
            self._record_active_failure()
            if self._failed_goals >= self._config.max_failed_goals:
                self._finish(ExplorationState.FAILED, 'failure_limit')
                return
            self._wait_for_map()
            return

        self._clear_active_goal()
        if cause is CancellationCause.MISSION_TIMEOUT:
            self._finish(ExplorationState.TIMED_OUT, 'mission_timeout')
        else:
            self._finish(ExplorationState.CANCELLED, 'operator_cancelled')

    def cancellation_failed(self, goal_handle: str, reason: str) -> None:
        self._require_cancelling_handle(goal_handle)
        self._clear_active_goal()
        self._finish(
            ExplorationState.FAILED,
            'navigation_cancel_error',
            error=reason,
        )

    def tick(self) -> bool:
        """Apply timeout policy; return true when the state changes."""

        if self._state.terminal or self._state is ExplorationState.CREATED:
            return False
        now = self._clock.monotonic()

        assert self._mission_deadline is not None
        if now >= self._mission_deadline:
            if self._state is ExplorationState.NAVIGATING:
                return self._begin_cancellation(
                    CancellationCause.MISSION_TIMEOUT
                )
            if self._state is ExplorationState.CANCELLING:
                if self._cancel_cause is not CancellationCause.MISSION_TIMEOUT:
                    self._cancel_cause = CancellationCause.MISSION_TIMEOUT
                    self._emit('mission_timeout_while_cancelling')
            else:
                self._finish(ExplorationState.TIMED_OUT, 'mission_timeout')
                return True

        if self._state is ExplorationState.WAITING_FOR_MAP:
            assert self._map_deadline is not None
            if now >= self._map_deadline:
                self._finish(ExplorationState.FAILED, 'map_timeout')
                return True
            return False

        if self._state is ExplorationState.NAVIGATING:
            assert self._goal_deadline is not None
            if now >= self._goal_deadline:
                return self._begin_cancellation(CancellationCause.GOAL_TIMEOUT)
            return False

        assert self._state is ExplorationState.CANCELLING
        assert self._cancel_deadline is not None
        if now >= self._cancel_deadline:
            self._clear_active_goal()
            self._finish(ExplorationState.FAILED, 'cancellation_timeout')
            return True
        return False

    def _begin_cancellation(self, cause: CancellationCause) -> bool:
        assert self._active_handle is not None
        handle = self._active_handle
        self._cancel_cause = cause
        self._cancel_deadline = (
            self._clock.monotonic() + self._config.cancel_timeout_sec
        )
        self._transition(
            ExplorationState.CANCELLING,
            cause=cause.value,
            goal_handle=handle,
        )
        try:
            self._navigation.cancel(handle)
        except Exception as exc:
            self._clear_active_goal()
            self._finish(
                ExplorationState.FAILED,
                'navigation_cancel_error',
                error=str(exc),
            )
            return False
        return True

    def _record_active_failure(self) -> None:
        assert self._active_candidate is not None
        self._blacklist.append(
            (
                self._active_candidate.goal_x,
                self._active_candidate.goal_y,
            )
        )
        self._failed_goals += 1
        self._emit(
            'frontier_blacklisted',
            failed_goals=self._failed_goals,
            goal_x=self._active_candidate.goal_x,
            goal_y=self._active_candidate.goal_y,
        )
        self._clear_active_goal()

    def _wait_for_map(self) -> None:
        self._clear_active_goal()
        self._map_deadline = (
            self._clock.monotonic() + self._config.map_timeout_sec
        )
        self._transition(ExplorationState.WAITING_FOR_MAP)

    def _clear_active_goal(self) -> None:
        self._active_handle = None
        self._active_candidate = None
        self._goal_deadline = None
        self._cancel_deadline = None
        self._cancel_cause = None

    def _mission_expired(self) -> bool:
        assert self._mission_deadline is not None
        return self._clock.monotonic() >= self._mission_deadline

    def _require_navigation_handle(self, goal_handle: str) -> None:
        self._require_state(ExplorationState.NAVIGATING)
        if goal_handle != self._active_handle:
            raise ValueError('navigation result does not match the active goal')

    def _require_cancelling_handle(self, goal_handle: str) -> None:
        self._require_state(ExplorationState.CANCELLING)
        if goal_handle != self._active_handle:
            raise ValueError('cancellation result does not match the active goal')
        assert self._cancel_cause is not None

    def _require_state(self, expected: ExplorationState) -> None:
        if self._state is not expected:
            raise RuntimeError(
                f'expected state {expected.value}, got {self._state.value}'
            )

    def _transition(
        self,
        state: ExplorationState,
        **details: object,
    ) -> None:
        previous = self._state
        self._state = state
        self._emit(
            'state_transition',
            from_state=previous.value,
            to_state=state.value,
            **details,
        )

    def _finish(
        self,
        state: ExplorationState,
        reason: str,
        **details: object,
    ) -> None:
        if not state.terminal:
            raise ValueError('finish state must be terminal')
        self._reason = reason
        self._ended_at = self._clock.monotonic()
        self._transition(state, reason=reason, **details)

    def _emit(self, event_type: str, **details: object) -> None:
        elapsed = 0.0
        if self._started_at is not None:
            elapsed = self._clock.monotonic() - self._started_at
        self._events.append(
            ExplorationEvent(
                sequence=len(self._events) + 1,
                event_type=event_type,
                state=self._state,
                elapsed_sec=elapsed,
                details=tuple(sorted(details.items())),
            )
        )
