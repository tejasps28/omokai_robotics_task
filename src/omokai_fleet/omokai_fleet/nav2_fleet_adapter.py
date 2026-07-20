"""Three-client Nav2 transport adapter for one synchronized fleet phase."""

from __future__ import annotations

from math import isfinite
from time import monotonic
from typing import Any, Callable

import rclpy
from nav2_msgs.msg import SpeedLimit

from omokai_executor.model import ExecutionGoal, GoalKind, GoalPose
from omokai_executor.nav2_adapter import (
    Nav2NavigationAdapter,
    NavigationOutcomes,
)
from omokai_fleet.model import ROBOT_IDS, RobotGoal, RobotId
from omokai_fleet.navigation import (
    NavigationBatch,
    NavigationResult,
    NavigationStatus,
)


class Nav2ServerUnavailable(RuntimeError):
    """Raised before dispatch when one or more robot action servers are absent."""


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
        self._reset()

        for goal in goals:
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
