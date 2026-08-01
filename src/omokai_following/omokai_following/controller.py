"""Deterministic bounded follow controller and loss/reacquisition policy."""

from __future__ import annotations

import math

from omokai_following.domain import FollowConfig
from omokai_following.domain import FollowDecision
from omokai_following.domain import FollowState
from omokai_following.domain import TargetObservation
from omokai_following.domain import TERMINAL_STATES
from omokai_following.domain import VelocityCommand


def clamp(value: float, minimum: float, maximum: float) -> float:
    return min(maximum, max(minimum, value))


class FollowController:
    """Own mission state and emit a bounded command or an explicit zero."""

    def __init__(self, config: FollowConfig = FollowConfig()) -> None:
        self.config = config
        self.state = FollowState.IDLE
        self.reason = 'not_started'
        self._started_at: float | None = None
        self._last_observation: TargetObservation | None = None
        self._loss_started_at: float | None = None
        self._last_bearing_sign = 1.0

    def start(self, now: float) -> FollowDecision:
        self.state = FollowState.SEARCHING
        self.reason = 'active_search_scan'
        self._started_at = now
        self._last_observation = None
        self._loss_started_at = None
        return self.decision()

    def cancel(self) -> FollowDecision:
        self.state = FollowState.CANCELLED
        self.reason = 'operator_cancelled'
        return self.decision()

    def fail(self, reason: str) -> FollowDecision:
        self.state = FollowState.FAILED
        self.reason = reason
        return self.decision()

    def emergency_stop(self) -> FollowDecision:
        """Stop an active mission for an independently observed obstacle."""
        if self.state is FollowState.IDLE or self.state in TERMINAL_STATES:
            return self.decision()
        self.reason = 'emergency_obstacle_stop'
        return self.decision(VelocityCommand())

    def update(
        self,
        now: float,
        observation: TargetObservation | None,
    ) -> FollowDecision:
        if self.state is FollowState.IDLE or self.state in TERMINAL_STATES:
            return self.decision()
        if (
            self._started_at is not None
            and now - self._started_at > self.config.mission_timeout_sec
        ):
            return self.fail('mission_timeout')
        if observation is not None:
            age = now - observation.received_timestamp
            if age <= self.config.observation_timeout_sec:
                return self._follow(now, observation)
        return self._handle_loss(now)

    def decision(
        self,
        command: VelocityCommand = VelocityCommand(),
    ) -> FollowDecision:
        if self.state is FollowState.IDLE or self.state in TERMINAL_STATES:
            command = VelocityCommand()
        return FollowDecision(self.state, command, self.reason)

    def _follow(
        self,
        now: float,
        observation: TargetObservation,
    ) -> FollowDecision:
        del now
        self._last_observation = observation
        self._loss_started_at = None
        # Do not let near-zero detector jitter choose the direction of a
        # later search turn.
        if abs(observation.bearing_rad) > 0.05:
            self._last_bearing_sign = math.copysign(
                1.0, observation.bearing_rad
            )
        if observation.range_m > self.config.maximum_follow_distance_m:
            return self.fail('target_beyond_maximum_follow_distance')
        if (
            observation.obstacle_distance_m is not None
            and observation.obstacle_distance_m
            < self.config.emergency_stop_distance_m
        ):
            self.state = FollowState.FOLLOWING
            self.reason = 'emergency_obstacle_stop'
            return self.decision(VelocityCommand())

        self.state = FollowState.FOLLOWING
        range_error = (
            observation.range_m - self.config.desired_standoff_m
        )
        angular = clamp(
            # Optical-frame +X points image-right, while ROS base-frame
            # positive angular Z turns left. Convert between the conventions.
            -self.config.bearing_gain * observation.bearing_rad,
            -self.config.maximum_angular_rps,
            self.config.maximum_angular_rps,
        )
        if range_error <= self.config.standoff_tolerance_m:
            linear = 0.0
            self.reason = 'holding_standoff'
        else:
            linear = clamp(
                self.config.range_gain * range_error,
                0.0,
                self.config.maximum_linear_mps,
            )
            turn_scale = clamp(
                1.0
                - abs(observation.bearing_rad)
                / self.config.turn_slow_angle_rad,
                0.0,
                1.0,
            )
            linear *= turn_scale
            self.reason = 'following_target'
        return self.decision(VelocityCommand(linear, angular))

    def _handle_loss(self, now: float) -> FollowDecision:
        if self._last_observation is None:
            self.state = FollowState.SEARCHING
            self.reason = 'active_search_scan'
            return self.decision(
                VelocityCommand(0.0, self.config.initial_search_angular_rps)
            )
        if self._loss_started_at is None:
            self._loss_started_at = now
            self.state = FollowState.REACQUIRING
            self.reason = 'target_lost_stop'
            return self.decision(VelocityCommand())
        elapsed = now - self._loss_started_at
        if elapsed > self.config.reacquisition_timeout_sec:
            return self.fail('target_lost')
        self.state = FollowState.REACQUIRING
        # Continue the last known centering direction even at stand-off.  A
        # zero-angular wait can strand the target just outside the image after
        # the final approach turn; the bounded turn brings it back while
        # linear velocity remains zero.
        self.reason = 'bounded_reacquisition'
        phase = int(
            elapsed / self.config.reacquisition_sweep_period_sec
        )
        sweep_sign = -self._last_bearing_sign
        if phase % 2 == 1:
            sweep_sign *= -1.0
        return self.decision(
            VelocityCommand(
                0.0,
                sweep_sign * self.config.reacquisition_angular_rps,
            )
        )
