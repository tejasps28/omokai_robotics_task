"""Asynchronous Nav2 NavigateToPose adapter for the execution core."""

from dataclasses import dataclass
from math import cos, isfinite, sin
from threading import RLock
from typing import Any, Dict, Optional, Protocol

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.node import Node

from .model import ExecutionGoal


class NavigationOutcomes(Protocol):
    """Executor callbacks consumed by the asynchronous action adapter."""

    def navigation_succeeded(self, goal_handle: str) -> None:
        ...

    def navigation_failed(self, goal_handle: str, reason: str) -> None:
        ...

    def cancellation_confirmed(self, goal_handle: str) -> None:
        ...

    def cancellation_failed(self, goal_handle: str, reason: str) -> None:
        ...


@dataclass
class _GoalContext:
    token: str
    ros_goal_handle: Optional[Any] = None
    cancel_requested: bool = False


class Nav2NavigationAdapter:
    """Translate execution goals into correlated Nav2 action requests.

    The adapter returns a local deterministic token immediately. Goal acceptance
    and terminal results arrive asynchronously and are correlated back to the
    executor through `NavigationOutcomes`.
    """

    def __init__(
        self,
        node: Node,
        *,
        action_name: str = 'navigate_to_pose',
        action_client: Optional[Any] = None,
        speed_limit_publisher: Optional[Any] = None,
    ) -> None:
        self._node = node
        self._client = action_client or ActionClient(
            node, NavigateToPose, action_name
        )
        self._speed_limit_publisher = speed_limit_publisher or node.create_publisher(
            SpeedLimit, 'speed_limit', 10
        )
        self._outcomes: Optional[NavigationOutcomes] = None
        self._contexts: Dict[str, _GoalContext] = {}
        self._next_token = 1
        self._lock = RLock()

    def bind_outcomes(self, outcomes: NavigationOutcomes) -> None:
        with self._lock:
            if self._contexts:
                raise RuntimeError('cannot bind outcomes while goals are active')
            self._outcomes = outcomes

    def server_is_ready(self) -> bool:
        """Return whether Nav2 can accept a goal without blocking."""

        return bool(self._client.server_is_ready())

    def dispatch(self, goal: ExecutionGoal, speed_mps: float) -> str:
        with self._lock:
            if self._outcomes is None:
                raise RuntimeError('navigation outcomes must be bound before dispatch')
            if not self.server_is_ready():
                raise RuntimeError('Nav2 NavigateToPose action server is unavailable')
            if not isfinite(speed_mps) or speed_mps <= 0:
                raise ValueError('speed_mps must be finite and greater than zero')

            token = f'nav-goal-{self._next_token:06d}'
            self._next_token += 1
            context = _GoalContext(token=token)
            self._contexts[token] = context

            request = NavigateToPose.Goal()
            request.pose.header.frame_id = goal.pose.frame_id
            request.pose.header.stamp = self._node.get_clock().now().to_msg()
            request.pose.pose.position.x = goal.pose.x
            request.pose.pose.position.y = goal.pose.y
            request.pose.pose.orientation.z = sin(goal.pose.yaw / 2.0)
            request.pose.pose.orientation.w = cos(goal.pose.yaw / 2.0)

            speed_limit = SpeedLimit()
            speed_limit.header.frame_id = goal.pose.frame_id
            speed_limit.header.stamp = request.pose.header.stamp
            speed_limit.percentage = False
            speed_limit.speed_limit = speed_mps

            try:
                self._speed_limit_publisher.publish(speed_limit)
                future = self._client.send_goal_async(request)
            except Exception:
                self._contexts.pop(token, None)
                raise
            future.add_done_callback(
                lambda completed, goal_token=token: self._on_goal_response(
                    goal_token, completed
                )
            )
            return token

    def cancel(self, goal_handle: str) -> None:
        with self._lock:
            context = self._contexts.get(goal_handle)
            if context is None:
                raise ValueError('cannot cancel an unknown Nav2 goal token')
            context.cancel_requested = True
            ros_goal_handle = context.ros_goal_handle

        if ros_goal_handle is not None:
            self._send_cancel(goal_handle, ros_goal_handle)

    def _on_goal_response(self, token: str, future: Any) -> None:
        try:
            ros_goal_handle = future.result()
        except Exception as exc:
            context = self._pop_context(token)
            if context is None:
                return
            self._report_goal_error(context, f'goal_response_error: {exc}')
            return

        with self._lock:
            context = self._contexts.get(token)
            if context is None:
                return
            if not ros_goal_handle.accepted:
                self._contexts.pop(token, None)
                rejected_context = context
            else:
                context.ros_goal_handle = ros_goal_handle
                rejected_context = None
                result_future = ros_goal_handle.get_result_async()
                result_future.add_done_callback(
                    lambda completed, goal_token=token: self._on_result(
                        goal_token, completed
                    )
                )
                cancel_requested = context.cancel_requested

        if rejected_context is not None:
            self._report_goal_error(rejected_context, 'goal_rejected')
            return
        if cancel_requested:
            self._send_cancel(token, ros_goal_handle)

    def _send_cancel(self, token: str, ros_goal_handle: Any) -> None:
        try:
            future = ros_goal_handle.cancel_goal_async()
        except Exception as exc:
            context = self._pop_context(token)
            if context is not None:
                self._outcome().cancellation_failed(
                    token, f'cancel_request_error: {exc}'
                )
            return
        future.add_done_callback(
            lambda completed, goal_token=token: self._on_cancel_response(
                goal_token, completed
            )
        )

    def _on_cancel_response(self, token: str, future: Any) -> None:
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
            reason = 'cancel_rejected'
        except Exception as exc:
            accepted = False
            reason = f'cancel_response_error: {exc}'

        if accepted:
            return
        context = self._pop_context(token)
        if context is not None:
            self._outcome().cancellation_failed(token, reason)

    def _on_result(self, token: str, future: Any) -> None:
        context = self._pop_context(token)
        if context is None:
            return
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._report_goal_error(context, f'result_error: {exc}')
            return

        status = wrapped_result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            if context.cancel_requested:
                self._outcome().cancellation_failed(
                    token, 'goal_succeeded_before_cancellation'
                )
            else:
                self._outcome().navigation_succeeded(token)
            return
        if status == GoalStatus.STATUS_CANCELED:
            self._outcome().cancellation_confirmed(token)
            return

        result = wrapped_result.result
        reason = (
            f'nav2_status={status} error_code={result.error_code} '
            f'error_msg={result.error_msg}'
        )
        self._report_goal_error(context, reason)

    def _report_goal_error(self, context: _GoalContext, reason: str) -> None:
        if context.cancel_requested:
            self._outcome().cancellation_failed(context.token, reason)
        else:
            self._outcome().navigation_failed(context.token, reason)

    def _pop_context(self, token: str) -> Optional[_GoalContext]:
        with self._lock:
            return self._contexts.pop(token, None)

    def _outcome(self) -> NavigationOutcomes:
        assert self._outcomes is not None
        return self._outcomes
