"""Three-client Nav2 transport adapter for one synchronized fleet phase."""

from __future__ import annotations

import json
from math import isfinite
from time import monotonic
from typing import Any, Callable
from uuid import uuid4

import rclpy
from nav2_msgs.msg import SpeedLimit
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from omokai_fleet.formation_tracking import is_current_formation_state

from omokai_executor.model import ExecutionGoal, GoalKind, GoalPose
from omokai_executor.nav2_adapter import (
    Nav2NavigationAdapter,
    NavigationOutcomes,
)
from omokai_fleet.model import (
    ROBOT_IDS,
    Formation,
    RobotGoal,
    RobotId,
    maximum_squad_speed_mps,
)
from omokai_fleet.navigation import (
    NavigationBatch,
    NavigationResult,
    NavigationStatus,
    is_hold_goal_id,
)
from omokai_fleet.split_execution import (
    SplitExecutionPlan,
    SplitExecutionResult,
    SplitReservationScheduler,
    SplitRobotResult,
)


class Nav2ServerUnavailable(RuntimeError):
    """Raised before dispatch when one or more robot action servers are absent."""


class FleetStateUnavailable(RuntimeError):
    """Raised before dispatch when fresh poses are not available for the fleet."""


class _RobotOutcomes(NavigationOutcomes):
    def __init__(self, owner: 'FleetNav2Adapter', robot_id: RobotId) -> None:
        self._owner = owner
        self._robot_id = robot_id

    def navigation_succeeded(self, goal_handle: str) -> None:
        self._owner._navigation_succeeded(self._robot_id, goal_handle)

    def navigation_failed(self, goal_handle: str, reason: str) -> None:
        self._owner._navigation_failed(self._robot_id, goal_handle, reason)

    def cancellation_confirmed(self, goal_handle: str) -> None:
        self._owner._cancellation_confirmed(self._robot_id, goal_handle)

    def cancellation_failed(self, goal_handle: str, reason: str) -> None:
        self._owner._cancellation_failed(
            self._robot_id,
            goal_handle,
            reason,
        )


class FleetNav2Adapter:
    """Dispatch and correlate one concurrent goal for each known robot."""

    def __init__(
        self,
        node: Any,
        *,
        adapters: dict[RobotId, Any] | None = None,
        safety_check: Callable[[], str | None] | None = None,
        cancel_check: Callable[[], bool] | None = None,
    ) -> None:
        self._node = node
        self._safety_check = safety_check
        self._cancel_check = cancel_check
        self._adapters: dict[RobotId, Any] = adapters or {}
        if not self._adapters:
            for robot_id in ROBOT_IDS:
                name = robot_id.value
                speed_publisher = node.create_publisher(
                    SpeedLimit,
                    f'/{name}/speed_limit',
                    10,
                )
                adapter = Nav2NavigationAdapter(
                    node,
                    action_name=f'/{name}/navigate_to_pose',
                    speed_limit_publisher=speed_publisher,
                )
                adapter.bind_outcomes(_RobotOutcomes(self, robot_id))
                self._adapters[robot_id] = adapter

        if tuple(self._adapters) != ROBOT_IDS:
            raise ValueError('adapters must use stable robot order')

        self._tokens: dict[RobotId, str] = {}
        self._pending: set[RobotId] = set()
        self._results: dict[RobotId, NavigationResult] = {}
        self._cancel_status: dict[RobotId, NavigationStatus] = {}
        self._cancel_reasons: dict[RobotId, str] = {}
        self._root_failure: RobotId | None = None
        self._split_scheduler: SplitReservationScheduler | None = None
        self._split_completed: dict[RobotId, int] = {}
        self._split_final: dict[RobotId, SplitRobotResult] = {}
        self._formation_state: dict[str, Any] = {}
        self._active_formation_operation_id: str | None = None
        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        command_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._formation_publisher = node.create_publisher(
            String,
            '/omokai_fleet/formation_command',
            command_qos,
        )
        self._formation_subscription = node.create_subscription(
            String,
            '/omokai_fleet/formation_tracking_state',
            self._receive_formation_state,
            10,
        )

    def unavailable_servers(self) -> tuple[RobotId, ...]:
        return tuple(
            robot_id
            for robot_id in ROBOT_IDS
            if not self._adapters[robot_id].server_is_ready()
        )

    def execute(
        self,
        goals: tuple[RobotGoal, ...],
        *,
        speed_mps: float,
        timeout_sec: float,
    ) -> NavigationBatch:
        self._validate_request(goals, speed_mps, timeout_sec)
        server_deadline = monotonic() + 10.0
        unavailable = self.unavailable_servers()
        while unavailable and monotonic() < server_deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            unavailable = self.unavailable_servers()
        if unavailable:
            names = ', '.join(item.value for item in unavailable)
            raise Nav2ServerUnavailable(
                f'Nav2 action server unavailable for: {names}'
            )
        if self._safety_check is not None:
            state_deadline = monotonic() + 10.0
            state_reason = self._safety_check()
            while state_reason and monotonic() < state_deadline:
                rclpy.spin_once(self._node, timeout_sec=0.05)
                state_reason = self._safety_check()
            if state_reason:
                raise FleetStateUnavailable(state_reason)
        self._reset()

        for goal in goals:
            # A hold is a scheduling reservation, not a request to drive back
            # to a compile-time pose. This matters after continuous formation,
            # where Nav2 may finish within its positional tolerance and the
            # live formation can therefore be offset from the nominal goal.
            if is_hold_goal_id(goal.goal_id):
                self._results[goal.robot_id] = NavigationResult(
                    goal.robot_id,
                    NavigationStatus.SUCCEEDED,
                )
                continue
            try:
                token = self._adapters[goal.robot_id].dispatch(
                    self._execution_goal(goal),
                    speed_mps,
                )
            except Exception as exc:
                self._handle_dispatch_error(goal.robot_id, str(exc))
                break
            self._tokens[goal.robot_id] = token
            self._pending.add(goal.robot_id)

        deadline = monotonic() + timeout_sec
        while self._pending and monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if self._safety_check is not None:
                reason = self._safety_check()
                if reason:
                    self._begin_safety_stop(reason)
            if self._cancel_check is not None and self._cancel_check():
                self._begin_operator_cancel()

        if self._pending:
            self._begin_timeout()
            cancel_deadline = monotonic() + 5.0
            while self._pending and monotonic() < cancel_deadline:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            for robot_id in tuple(self._pending):
                self._results[robot_id] = NavigationResult(
                    robot_id,
                    NavigationStatus.CANCEL_FAILED,
                    'Nav2 cancellation confirmation timed out',
                )
                self._pending.remove(robot_id)

        return NavigationBatch(
            tuple(self._results[robot_id] for robot_id in ROBOT_IDS)
        )

    def execute_split(
        self,
        plan: SplitExecutionPlan,
        *,
        speed_mps: float,
        timeout_sec: float,
    ) -> SplitExecutionResult:
        """Advance independent sequences immediately, isolating private failures."""
        if not isinstance(plan, SplitExecutionPlan):
            raise ValueError('plan must be a SplitExecutionPlan')
        self._validate_request(
            tuple(item.goals[0] for item in plan.work),
            speed_mps,
            timeout_sec,
        )
        self._require_ready(ROBOT_IDS)
        self._reset()
        self._split_scheduler = SplitReservationScheduler(plan)
        self._split_completed = {robot_id: 0 for robot_id in ROBOT_IDS}
        self._split_final = {}
        deadline = monotonic() + timeout_sec
        try:
            while not self._split_scheduler.complete and monotonic() < deadline:
                ready = self._split_scheduler.ready_goals()
                self._split_scheduler.claim(ready)
                for goal in ready:
                    try:
                        token = self._adapters[goal.robot_id].dispatch(
                            self._execution_goal(goal),
                            speed_mps,
                        )
                    except Exception as exc:
                        self._split_dispatch_failed(goal.robot_id, str(exc))
                        continue
                    self._tokens[goal.robot_id] = token
                    self._pending.add(goal.robot_id)
                if self._root_failure is not None:
                    break
                rclpy.spin_once(self._node, timeout_sec=0.05)
                if self._safety_check is not None:
                    reason = self._safety_check()
                    if reason:
                        self._begin_safety_stop(reason)
                if self._cancel_check is not None and self._cancel_check():
                    self._begin_operator_cancel()

            if self._pending or not self._split_scheduler.complete:
                if self._root_failure is None:
                    self._begin_timeout()
                cancel_deadline = monotonic() + 5.0
                while self._pending and monotonic() < cancel_deadline:
                    rclpy.spin_once(self._node, timeout_sec=0.05)
            return self._finish_split_result(plan)
        finally:
            self._split_scheduler = None

    def execute_formation(
        self,
        batches: tuple[tuple[RobotGoal, ...], ...],
        *,
        formation: Formation,
        spacing_m: float,
        speed_mps: float,
        timeout_sec: float,
    ) -> NavigationBatch:
        """Execute a leader path while robot2/3 track live formation offsets."""
        if not isinstance(batches, tuple) or not batches:
            raise ValueError('formation batches must be a non-empty tuple')
        for goals in batches:
            self._validate_request(goals, speed_mps, timeout_sec)
        if not isinstance(formation, Formation):
            raise ValueError('formation must be a supported Formation')
        if (
            not isinstance(spacing_m, (int, float))
            or isinstance(spacing_m, bool)
            or not isfinite(spacing_m)
            or not 0.60 <= spacing_m <= 1.20
        ):
            raise ValueError('spacing_m must be between 0.60 and 1.20')

        self._require_ready((RobotId.ROBOT1,))
        deadline = monotonic() + timeout_sec
        self._publish_formation_command(
            True,
            formation,
            spacing_m,
            speed_mps,
        )
        try:
            for goals in batches:
                remaining = deadline - monotonic()
                if remaining <= 0.0:
                    return self._formation_failure_batch(
                        NavigationStatus.TIMED_OUT,
                        'formation path timed out',
                    )
                leader_result = self._execute_leader_goal(
                    goals[0],
                    speed_mps,
                    remaining,
                )
                if leader_result.status is not NavigationStatus.SUCCEEDED:
                    return self._formation_failure_batch(
                        leader_result.status,
                        leader_result.reason,
                    )

            # Followers pass through smoothing and fleet traffic control. Keep
            # a bounded, but realistic, allowance for the final yaw settle.
            settle_deadline = min(deadline, monotonic() + 30.0)
            while monotonic() < settle_deadline:
                if self._followers_settled():
                    return NavigationBatch(
                        tuple(
                            NavigationResult(
                                robot_id,
                                NavigationStatus.SUCCEEDED,
                            )
                            for robot_id in ROBOT_IDS
                        )
                    )
                rclpy.spin_once(self._node, timeout_sec=0.05)
                failure = self._formation_stop_reason()
                if failure is not None:
                    return self._formation_failure_batch(
                        NavigationStatus.FAILED,
                        failure,
                    )
            return self._formation_failure_batch(
                NavigationStatus.TIMED_OUT,
                'followers did not settle into formation',
            )
        finally:
            self._publish_formation_command(
                False,
                formation,
                spacing_m,
                speed_mps,
            )

    def _execute_leader_goal(
        self,
        goal: RobotGoal,
        speed_mps: float,
        timeout_sec: float,
    ) -> NavigationResult:
        self._reset()
        try:
            token = self._adapters[RobotId.ROBOT1].dispatch(
                self._execution_goal(goal),
                speed_mps,
            )
        except Exception as exc:
            return NavigationResult(
                RobotId.ROBOT1,
                NavigationStatus.REJECTED,
                f'leader dispatch failed: {exc}',
            )
        self._tokens[RobotId.ROBOT1] = token
        self._pending.add(RobotId.ROBOT1)
        deadline = monotonic() + timeout_sec
        while self._pending and monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            failure = self._formation_stop_reason()
            if failure is not None:
                self._begin_safety_stop(failure)
            if self._cancel_check is not None and self._cancel_check():
                self._begin_operator_cancel()
        if self._pending:
            self._begin_timeout()
            cancel_deadline = monotonic() + 5.0
            while self._pending and monotonic() < cancel_deadline:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            if self._pending:
                self._pending.remove(RobotId.ROBOT1)
                return NavigationResult(
                    RobotId.ROBOT1,
                    NavigationStatus.CANCEL_FAILED,
                    'leader cancellation confirmation timed out',
                )
        return self._results[RobotId.ROBOT1]

    def _require_ready(self, robot_ids: tuple[RobotId, ...]) -> None:
        server_deadline = monotonic() + 10.0
        unavailable = tuple(
            robot_id
            for robot_id in robot_ids
            if not self._adapters[robot_id].server_is_ready()
        )
        while unavailable and monotonic() < server_deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            unavailable = tuple(
                robot_id
                for robot_id in robot_ids
                if not self._adapters[robot_id].server_is_ready()
            )
        if unavailable:
            names = ', '.join(item.value for item in unavailable)
            raise Nav2ServerUnavailable(
                f'Nav2 action server unavailable for: {names}'
            )
        if self._safety_check is not None:
            state_deadline = monotonic() + 10.0
            reason = self._safety_check()
            while reason and monotonic() < state_deadline:
                rclpy.spin_once(self._node, timeout_sec=0.05)
                reason = self._safety_check()
            if reason:
                raise FleetStateUnavailable(reason)

    def _receive_formation_state(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if is_current_formation_state(
            value,
            self._active_formation_operation_id,
        ):
            self._formation_state = value

    def _followers_settled(self) -> bool:
        followers = self._formation_state.get('followers')
        return isinstance(followers, dict) and all(
            followers.get(robot_id.value, {}).get('target_reached') is True
            for robot_id in (RobotId.ROBOT2, RobotId.ROBOT3)
        )

    def _formation_stop_reason(self) -> str | None:
        blocked = self._formation_state.get('blocked')
        if isinstance(blocked, str) and blocked:
            return f'formation controller blocked: {blocked}'
        if self._safety_check is not None:
            return self._safety_check()
        return None

    def _publish_formation_command(
        self,
        enabled: bool,
        formation: Formation,
        spacing_m: float,
        speed_mps: float,
    ) -> None:
        if enabled and self._active_formation_operation_id is None:
            self._active_formation_operation_id = uuid4().hex
        operation_id = self._active_formation_operation_id or uuid4().hex
        self._formation_publisher.publish(
            String(
                data=json.dumps(
                    {
                        'enabled': enabled,
                        'formation': formation.value,
                        'spacing_m': spacing_m,
                        'max_linear_mps': min(
                            maximum_squad_speed_mps(), speed_mps + 0.04
                        ),
                        'operation_id': operation_id,
                    },
                    sort_keys=True,
                )
            )
        )
        if enabled:
            self._formation_state = {}
        else:
            self._formation_state = {}
            self._active_formation_operation_id = None

    @staticmethod
    def _formation_failure_batch(
        status: NavigationStatus,
        reason: str,
    ) -> NavigationBatch:
        if status is NavigationStatus.SUCCEEDED:
            raise ValueError('formation failure status cannot be succeeded')
        root_status = status
        if status is NavigationStatus.CANCELLED:
            return NavigationBatch(
                tuple(
                    NavigationResult(robot_id, status, reason)
                    for robot_id in ROBOT_IDS
                )
            )
        return NavigationBatch(
            (
                NavigationResult(RobotId.ROBOT1, root_status, reason),
                NavigationResult(
                    RobotId.ROBOT2,
                    NavigationStatus.CANCELLED,
                    'continuous formation tracking stopped',
                ),
                NavigationResult(
                    RobotId.ROBOT3,
                    NavigationStatus.CANCELLED,
                    'continuous formation tracking stopped',
                ),
            )
        )

    def _validate_request(
        self,
        goals: tuple[RobotGoal, ...],
        speed_mps: float,
        timeout_sec: float,
    ) -> None:
        if not isinstance(goals, tuple) or len(goals) != 3:
            raise ValueError('goals must contain exactly three robot goals')
        if tuple(goal.robot_id for goal in goals) != ROBOT_IDS:
            raise ValueError('goals must use stable robot order')
        if (
            not isinstance(speed_mps, (int, float))
            or isinstance(speed_mps, bool)
            or not isfinite(speed_mps)
            or speed_mps <= 0
        ):
            raise ValueError('speed_mps must be finite and positive')
        if (
            not isinstance(timeout_sec, (int, float))
            or isinstance(timeout_sec, bool)
            or not isfinite(timeout_sec)
            or timeout_sec <= 0
        ):
            raise ValueError('timeout_sec must be finite and positive')

    def _reset(self) -> None:
        self._tokens = {}
        self._pending = set()
        self._results = {}
        self._cancel_status = {}
        self._cancel_reasons = {}
        self._root_failure = None
        self._split_scheduler = None
        self._split_completed = {}
        self._split_final = {}

    def _execution_goal(self, goal: RobotGoal) -> ExecutionGoal:
        return ExecutionGoal(
            goal_id=goal.goal_id,
            pose=GoalPose(
                x=goal.pose.x,
                y=goal.pose.y,
                yaw=goal.pose.yaw,
                frame_id=goal.pose.frame_id,
            ),
            kind=GoalKind.ROUTE,
        )

    def _navigation_succeeded(
        self,
        robot_id: RobotId,
        token: str,
    ) -> None:
        if not self._matches_pending(robot_id, token):
            return
        if self._split_scheduler is not None:
            self._split_scheduler.release(robot_id)
            self._split_completed[robot_id] += 1
        self._results[robot_id] = NavigationResult(
            robot_id,
            NavigationStatus.SUCCEEDED,
        )
        self._pending.remove(robot_id)

    def _navigation_failed(
        self,
        robot_id: RobotId,
        token: str,
        reason: str,
    ) -> None:
        if not self._matches_pending(robot_id, token):
            return
        if (
            self._split_scheduler is not None
            and self._split_scheduler.failure_is_isolatable(robot_id)
        ):
            status = (
                NavigationStatus.REJECTED
                if 'rejected' in reason
                else NavigationStatus.FAILED
            )
            skipped = self._split_scheduler.abandon(robot_id)
            self._split_final[robot_id] = SplitRobotResult(
                robot_id,
                status,
                self._split_completed[robot_id],
                len(skipped),
                reason,
            )
            self._results[robot_id] = NavigationResult(robot_id, status, reason)
            self._pending.remove(robot_id)
            return
        if self._root_failure is None:
            self._root_failure = robot_id
            status = (
                NavigationStatus.REJECTED
                if 'rejected' in reason
                else NavigationStatus.FAILED
            )
            self._results[robot_id] = NavigationResult(
                robot_id,
                status,
                reason,
            )
            self._pending.remove(robot_id)
            self._cancel_pending(
                f'cancelled after {robot_id.value} navigation failed'
            )
            return
        self._results[robot_id] = NavigationResult(
            robot_id,
            NavigationStatus.CANCEL_FAILED,
            f'goal failed while squad was stopping: {reason}',
        )
        self._pending.remove(robot_id)

    def _split_dispatch_failed(self, robot_id: RobotId, reason: str) -> None:
        if self._split_scheduler is None:
            raise RuntimeError('split scheduler is not active')
        reason = f'goal dispatch rejected: {reason}'
        if self._split_scheduler.failure_is_isolatable(robot_id):
            skipped = self._split_scheduler.abandon(robot_id)
            self._split_final[robot_id] = SplitRobotResult(
                robot_id,
                NavigationStatus.REJECTED,
                self._split_completed[robot_id],
                len(skipped),
                reason,
            )
            return
        self._root_failure = robot_id
        self._results[robot_id] = NavigationResult(
            robot_id, NavigationStatus.REJECTED, reason
        )
        self._split_scheduler.abandon(robot_id)
        self._cancel_pending(f'cancelled after {robot_id.value} dispatch failed')

    def _finish_split_result(
        self,
        plan: SplitExecutionPlan,
    ) -> SplitExecutionResult:
        planned = {item.robot_id: len(item.goals) for item in plan.work}
        results = []
        for robot_id in ROBOT_IDS:
            if robot_id in self._split_final:
                results.append(self._split_final[robot_id])
                continue
            result = self._results.get(robot_id)
            if result is None:
                if self._root_failure is None:
                    status = NavigationStatus.SUCCEEDED
                    reason = ''
                else:
                    status = NavigationStatus.CANCELLED
                    reason = 'skipped after fleet-stop split failure'
            else:
                status = result.status
                reason = result.reason
            completed = self._split_completed[robot_id]
            results.append(
                SplitRobotResult(
                    robot_id,
                    status,
                    completed,
                    max(0, planned[robot_id] - completed),
                    reason,
                )
            )
        return SplitExecutionResult(tuple(results))

    def _cancellation_confirmed(
        self,
        robot_id: RobotId,
        token: str,
    ) -> None:
        if not self._matches_pending(robot_id, token):
            return
        status = self._cancel_status.get(
            robot_id,
            NavigationStatus.CANCELLED,
        )
        reason = (
            self._cancel_reasons.get(
                robot_id,
                'goal cancellation confirmed',
            )
        )
        self._results[robot_id] = NavigationResult(
            robot_id,
            status,
            reason,
        )
        self._pending.remove(robot_id)

    def _cancellation_failed(
        self,
        robot_id: RobotId,
        token: str,
        reason: str,
    ) -> None:
        if not self._matches_pending(robot_id, token):
            return
        self._results[robot_id] = NavigationResult(
            robot_id,
            NavigationStatus.CANCEL_FAILED,
            reason,
        )
        self._pending.remove(robot_id)

    def _handle_dispatch_error(
        self,
        robot_id: RobotId,
        reason: str,
    ) -> None:
        self._root_failure = robot_id
        self._results[robot_id] = NavigationResult(
            robot_id,
            NavigationStatus.REJECTED,
            f'dispatch failed: {reason}',
        )
        self._cancel_pending(
            f'cancelled after {robot_id.value} dispatch failed'
        )
        for pending_id in ROBOT_IDS:
            if pending_id not in self._tokens and pending_id not in self._results:
                self._results[pending_id] = NavigationResult(
                    pending_id,
                    NavigationStatus.CANCELLED,
                    f'not dispatched after {robot_id.value} failed',
                )

    def _begin_timeout(self) -> None:
        if self._root_failure is None:
            self._root_failure = next(
                robot_id
                for robot_id in ROBOT_IDS
                if robot_id in self._pending
            )
        for robot_id in tuple(self._pending):
            self._cancel_status[robot_id] = (
                NavigationStatus.TIMED_OUT
                if robot_id is self._root_failure
                else NavigationStatus.CANCELLED
            )
            self._cancel_reasons[robot_id] = (
                'goal timed out and cancellation was confirmed'
                if robot_id is self._root_failure
                else f'cancelled after {self._root_failure.value} timed out'
            )
            self._adapters[robot_id].cancel(self._tokens[robot_id])

    def _cancel_pending(self, reason: str) -> None:
        for robot_id in tuple(self._pending):
            self._cancel_status[robot_id] = NavigationStatus.CANCELLED
            self._cancel_reasons[robot_id] = reason
            self._adapters[robot_id].cancel(self._tokens[robot_id])

    def _begin_safety_stop(self, reason: str) -> None:
        if not self._pending or self._cancel_status:
            return
        self._root_failure = next(
            robot_id
            for robot_id in ROBOT_IDS
            if robot_id in self._pending
        )
        for robot_id in tuple(self._pending):
            self._cancel_status[robot_id] = (
                NavigationStatus.FAILED
                if robot_id is self._root_failure
                else NavigationStatus.CANCELLED
            )
            self._cancel_reasons[robot_id] = (
                reason
                if robot_id is self._root_failure
                else f'cancelled after separation violation: {reason}'
            )
            self._adapters[robot_id].cancel(self._tokens[robot_id])

    def _begin_operator_cancel(self) -> None:
        if not self._pending or self._cancel_status:
            return
        for robot_id in tuple(self._pending):
            self._cancel_status[robot_id] = NavigationStatus.CANCELLED
            self._cancel_reasons[robot_id] = 'operator cancellation confirmed'
            self._adapters[robot_id].cancel(self._tokens[robot_id])

    def _matches_pending(self, robot_id: RobotId, token: str) -> bool:
        return (
            robot_id in self._pending
            and self._tokens.get(robot_id) == token
        )
