"""Immutable values at the compiler-to-executor boundary."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Tuple


class ExecutorState(str, Enum):
    ACCEPTED = 'accepted'
    COMPILING = 'compiling'
    DISPATCHING = 'dispatching'
    EXECUTING = 'executing'
    RETURNING_HOME = 'returning_home'
    CANCELLING = 'cancelling'
    SUCCEEDED = 'succeeded'
    CANCELLED = 'cancelled'
    FAILED = 'failed'
    TIMED_OUT = 'timed_out'

    @property
    def terminal(self) -> bool:
        return self in {
            ExecutorState.SUCCEEDED,
            ExecutorState.CANCELLED,
            ExecutorState.FAILED,
            ExecutorState.TIMED_OUT,
        }


class GoalKind(str, Enum):
    ROUTE = 'route'
    HOME = 'home'


@dataclass(frozen=True)
class GoalPose:
    x: float
    y: float
    yaw: float
    frame_id: str = 'map'

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError('goal pose coordinates must be finite')
        if self.frame_id != 'map':
            raise ValueError("Task 1 execution goals must use the 'map' frame")


@dataclass(frozen=True)
class ExecutionGoal:
    goal_id: str
    pose: GoalPose
    kind: GoalKind = GoalKind.ROUTE

    def __post_init__(self) -> None:
        if not self.goal_id.strip():
            raise ValueError('goal_id must not be blank')


@dataclass(frozen=True)
class ExecutionPlan:
    mission_id: str
    goals: Tuple[ExecutionGoal, ...]
    speed_mps: float

    def __post_init__(self) -> None:
        if not self.mission_id.strip():
            raise ValueError('mission_id must not be blank')
        if not isinstance(self.goals, tuple):
            raise ValueError('execution goals must be an immutable tuple')
        if not self.goals:
            raise ValueError('execution plan must contain at least one goal')
        if not isfinite(self.speed_mps) or self.speed_mps <= 0:
            raise ValueError('speed_mps must be finite and greater than zero')
        goal_ids = [goal.goal_id for goal in self.goals]
        if len(set(goal_ids)) != len(goal_ids):
            raise ValueError('execution goal IDs must be unique')


@dataclass(frozen=True)
class ExecutorConfig:
    goal_timeout_sec: float = 120.0
    max_retries: int = 1

    def __post_init__(self) -> None:
        if not isfinite(self.goal_timeout_sec) or self.goal_timeout_sec <= 0:
            raise ValueError('goal_timeout_sec must be finite and greater than zero')
        if (
            not isinstance(self.max_retries, int)
            or isinstance(self.max_retries, bool)
            or self.max_retries < 0
        ):
            raise ValueError('max_retries must be a non-negative integer')
