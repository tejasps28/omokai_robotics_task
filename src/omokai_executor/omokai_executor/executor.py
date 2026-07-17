"""Event-driven deterministic mission executor."""

from threading import RLock
from typing import Any, Dict, Optional

from .events import ExecutionEvent
from .model import ExecutionPlan, ExecutorConfig, ExecutorState, GoalKind
from .ports import EventSink, NavigationPort, RuntimeClock, SystemRuntimeClock


_ALLOWED_TRANSITIONS = {
    ExecutorState.ACCEPTED: {
        ExecutorState.COMPILING,
        ExecutorState.CANCELLED,
        ExecutorState.FAILED,
    },
    ExecutorState.COMPILING: {
        ExecutorState.DISPATCHING,
        ExecutorState.CANCELLED,
        ExecutorState.FAILED,
    },
    ExecutorState.DISPATCHING: {
        ExecutorState.EXECUTING,
        ExecutorState.RETURNING_HOME,
        ExecutorState.FAILED,
    },
    ExecutorState.EXECUTING: {
        ExecutorState.DISPATCHING,
        ExecutorState.CANCELLING,
        ExecutorState.SUCCEEDED,
        ExecutorState.FAILED,
        ExecutorState.TIMED_OUT,
    },
    ExecutorState.RETURNING_HOME: {
        ExecutorState.DISPATCHING,
        ExecutorState.CANCELLING,
        ExecutorState.SUCCEEDED,
        ExecutorState.FAILED,
        ExecutorState.TIMED_OUT,
    },
    ExecutorState.CANCELLING: {
        ExecutorState.CANCELLED,
        ExecutorState.FAILED,
        ExecutorState.TIMED_OUT,
    },
}


class MissionExecutor:
    """Drive one compiled plan through a navigation adapter.

    Public methods are safe to call from serialized ROS action callbacks. No
    planner or model is consulted after execution begins.
    """

    def __init__(
        self,
        mission_id: str,
        navigation: NavigationPort,
        event_sink: EventSink,
        *,
        config: ExecutorConfig = ExecutorConfig(),
        clock: Optional[RuntimeClock] = None,
    ) -> None:
        if not mission_id.strip():
            raise ValueError('mission_id must not be blank')
        self._mission_id = mission_id
        self._navigation = navigation
        self._event_sink = event_sink
        self._config = config
        self._clock = clock or SystemRuntimeClock()
        self._state = ExecutorState.ACCEPTED
        self._sequence = 0
        self._plan: Optional[ExecutionPlan] = None
        self._goal_index = 0
        self._goal_handle: Optional[str] = None
        self._deadline: Optional[float] = None
        self._retry_count = 0
        self._lock = RLock()
        self._emit('executor_created')

    @property
    def state(self) -> ExecutorState:
        return self._state

    @property
    def active_goal_handle(self) -> Optional[str]:
        return self._goal_handle

    @property
    def current_goal_index(self) -> int:
        return self._goal_index

    def begin_compilation(self) -> None:
        with self._lock:
            self._transition(ExecutorState.COMPILING)

    def load_plan(self, plan: ExecutionPlan) -> None:
        with self._lock:
            self._require_state(ExecutorState.COMPILING)
            if plan.mission_id != self._mission_id:
                raise ValueError('execution plan mission_id does not match executor')
            self._plan = plan
            self._goal_index = 0
            self._retry_count = 0
            self._transition(ExecutorState.DISPATCHING)
            self._dispatch_current_goal()

    def navigation_succeeded(self, goal_handle: str) -> None:
        with self._lock:
            self._require_active_handle(goal_handle)
            self._emit('goal_succeeded', self._goal_details())
            self._clear_active_goal()
            self._goal_index += 1
            self._retry_count = 0

            assert self._plan is not None
            if self._goal_index == len(self._plan.goals):
                self._transition(ExecutorState.SUCCEEDED)
                return

            self._transition(ExecutorState.DISPATCHING)
            self._dispatch_current_goal()

    def navigation_failed(self, goal_handle: str, reason: str) -> None:
        with self._lock:
            self._require_active_handle(goal_handle)
            details = self._goal_details()
            details['reason'] = reason
            self._emit('goal_failed', details)
            self._clear_active_goal()

            if self._retry_count < self._config.max_retries:
                self._retry_count += 1
                self._transition(
                    ExecutorState.DISPATCHING,
                    {'retry': self._retry_count},
                )
                self._dispatch_current_goal()
                return

            self._transition(ExecutorState.FAILED, {'reason': reason})

    def request_cancel(self) -> bool:
        with self._lock:
            if self._state.terminal:
                return False
            if self._state in {ExecutorState.ACCEPTED, ExecutorState.COMPILING}:
                self._transition(ExecutorState.CANCELLED)
                return True
            if self._state not in {
                ExecutorState.EXECUTING,
                ExecutorState.RETURNING_HOME,
            }:
                raise RuntimeError(f'cannot cancel while state is {self._state.value}')

            assert self._goal_handle is not None
            goal_handle = self._goal_handle
            self._transition(ExecutorState.CANCELLING)
            self._emit('cancellation_requested', {'goal_handle': goal_handle})
            try:
                self._navigation.cancel(goal_handle)
            except Exception as exc:
                self._transition(
                    ExecutorState.FAILED,
                    {'reason': 'navigation_cancel_error', 'error': str(exc)},
                )
                return False
            return True

    def cancellation_confirmed(self, goal_handle: str) -> None:
        with self._lock:
            self._require_state(ExecutorState.CANCELLING)
            self._require_active_handle(goal_handle, allow_cancelling=True)
            self._emit('cancellation_confirmed', {'goal_handle': goal_handle})
            self._clear_active_goal()
            self._transition(ExecutorState.CANCELLED)

    def cancellation_failed(self, goal_handle: str, reason: str) -> None:
        with self._lock:
            self._require_state(ExecutorState.CANCELLING)
            self._require_active_handle(goal_handle, allow_cancelling=True)
            self._emit(
                'cancellation_failed',
                {'goal_handle': goal_handle, 'reason': reason},
            )
            self._clear_active_goal()
            self._transition(ExecutorState.FAILED, {'reason': reason})

    def tick(self) -> bool:
        """Apply deadline policy; return true only when a timeout occurs."""

        with self._lock:
            if self._state not in {
                ExecutorState.EXECUTING,
                ExecutorState.RETURNING_HOME,
                ExecutorState.CANCELLING,
            }:
                return False
            assert self._deadline is not None
            if self._clock.monotonic() < self._deadline:
                return False

            goal_handle = self._goal_handle
            if goal_handle is not None:
                try:
                    self._navigation.cancel(goal_handle)
                except Exception as exc:
                    self._emit('timeout_cancel_failed', {'error': str(exc)})
            self._emit('goal_timed_out', self._goal_details())
            self._clear_active_goal()
            self._transition(ExecutorState.TIMED_OUT)
            return True

    def _dispatch_current_goal(self) -> None:
        assert self._state is ExecutorState.DISPATCHING
        assert self._plan is not None
        goal = self._plan.goals[self._goal_index]
        self._emit(
            'goal_dispatch_requested',
            {
                'goal_id': goal.goal_id,
                'goal_index': self._goal_index,
                'retry': self._retry_count,
            },
        )
        try:
            goal_handle = self._navigation.dispatch(goal, self._plan.speed_mps)
            if not goal_handle.strip():
                raise ValueError('navigation adapter returned a blank goal handle')
        except Exception as exc:
            self._transition(
                ExecutorState.FAILED,
                {'reason': 'navigation_dispatch_error', 'error': str(exc)},
            )
            return

        self._goal_handle = goal_handle
        self._deadline = self._clock.monotonic() + self._config.goal_timeout_sec
        self._emit('goal_dispatched', self._goal_details())
        next_state = (
            ExecutorState.RETURNING_HOME
            if goal.kind is GoalKind.HOME
            else ExecutorState.EXECUTING
        )
        self._transition(next_state)

    def _clear_active_goal(self) -> None:
        self._goal_handle = None
        self._deadline = None

    def _goal_details(self) -> Dict[str, Any]:
        details: Dict[str, Any] = {'goal_index': self._goal_index}
        if self._goal_handle is not None:
            details['goal_handle'] = self._goal_handle
        if self._plan is not None and self._goal_index < len(self._plan.goals):
            details['goal_id'] = self._plan.goals[self._goal_index].goal_id
            details['retry'] = self._retry_count
        return details

    def _require_active_handle(
        self, goal_handle: str, *, allow_cancelling: bool = False
    ) -> None:
        states = {ExecutorState.EXECUTING, ExecutorState.RETURNING_HOME}
        if allow_cancelling:
            states.add(ExecutorState.CANCELLING)
        if self._state not in states:
            raise RuntimeError(
                f'no goal result expected while state is {self._state.value}'
            )
        if goal_handle != self._goal_handle:
            raise ValueError('navigation result does not match the active goal handle')

    def _require_state(self, state: ExecutorState) -> None:
        if self._state is not state:
            raise RuntimeError(
                f'expected executor state {state.value}, got {self._state.value}'
            )

    def _transition(
        self, next_state: ExecutorState, details: Optional[Dict[str, Any]] = None
    ) -> None:
        if self._state.terminal:
            raise RuntimeError(f'terminal state {self._state.value} cannot transition')
        if next_state not in _ALLOWED_TRANSITIONS.get(self._state, set()):
            raise RuntimeError(
                f'invalid executor transition {self._state.value} -> {next_state.value}'
            )
        previous = self._state
        transition_details: Dict[str, Any] = {
            'from_state': previous.value,
            'to_state': next_state.value,
        }
        if details:
            transition_details.update(details)
        self._emit('state_transition', transition_details, state=next_state)
        self._state = next_state

    def _emit(
        self,
        event_type: str,
        details: Optional[Dict[str, Any]] = None,
        *,
        state: Optional[ExecutorState] = None,
    ) -> None:
        event = ExecutionEvent(
            sequence=self._sequence + 1,
            mission_id=self._mission_id,
            event_type=event_type,
            state=state or self._state,
            timestamp_utc=self._clock.utc_now(),
            details=details or {},
        )
        self._event_sink.record(event)
        self._sequence = event.sequence
