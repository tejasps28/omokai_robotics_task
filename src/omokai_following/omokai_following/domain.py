"""Pure state and command models for visual following."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class FollowState(str, Enum):
    IDLE = 'idle'
    SEARCHING = 'searching'
    TARGET_ACQUIRED = 'target_acquired'
    FOLLOWING = 'following'
    REACQUIRING = 'reacquiring'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    CANCELLED = 'cancelled'


TERMINAL_STATES = {
    FollowState.SUCCEEDED,
    FollowState.FAILED,
    FollowState.CANCELLED,
}


@dataclass(frozen=True)
class FollowConfig:
    desired_standoff_m: float = 1.20
    standoff_tolerance_m: float = 0.15
    maximum_linear_mps: float = 0.38
    maximum_angular_rps: float = 0.50
    range_gain: float = 0.65
    bearing_gain: float = 1.0
    turn_slow_angle_rad: float = 0.65
    observation_timeout_sec: float = 0.75
    reacquisition_timeout_sec: float = 10.0
    reacquisition_angular_rps: float = 0.25
    reacquisition_sweep_period_sec: float = 2.0
    initial_search_angular_rps: float = 0.40
    emergency_stop_distance_m: float = 0.55
    maximum_follow_distance_m: float = 8.0
    mission_timeout_sec: float = 180.0

    def __post_init__(self) -> None:
        positive = (
            self.desired_standoff_m,
            self.standoff_tolerance_m,
            self.maximum_linear_mps,
            self.maximum_angular_rps,
            self.range_gain,
            self.bearing_gain,
            self.turn_slow_angle_rad,
            self.observation_timeout_sec,
            self.reacquisition_timeout_sec,
            self.reacquisition_angular_rps,
            self.reacquisition_sweep_period_sec,
            self.initial_search_angular_rps,
            self.emergency_stop_distance_m,
            self.maximum_follow_distance_m,
            self.mission_timeout_sec,
        )
        if any(value <= 0 for value in positive):
            raise ValueError('follow configuration values must be positive')
        if self.emergency_stop_distance_m >= self.desired_standoff_m:
            raise ValueError('emergency distance must be below stand-off')
        if self.maximum_linear_mps > 0.40:
            raise ValueError('maximum linear speed exceeds safety cap')
        if self.maximum_angular_rps > 0.80:
            raise ValueError('maximum angular speed exceeds safety cap')
        if self.reacquisition_angular_rps > 0.50:
            raise ValueError('reacquisition speed exceeds safety cap')
        if self.initial_search_angular_rps > 0.40:
            raise ValueError('initial search speed exceeds safety cap')


@dataclass(frozen=True)
class TargetObservation:
    source_timestamp: float
    received_timestamp: float
    range_m: float
    bearing_rad: float
    obstacle_distance_m: float | None
    episode: int


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class FollowDecision:
    state: FollowState
    command: VelocityCommand
    reason: str
