"""Immutable values shared by fleet planning and coordination."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
import os


def maximum_squad_speed_mps() -> float:
    """Return the bounded simulation override, preserving the safe default."""
    try:
        value = float(os.environ.get('OMOKAI_DEMO_MAX_SPEED_MPS', '0.18'))
    except ValueError:
        return 0.18
    return value if isfinite(value) and 0.18 <= value <= 0.40 else 0.18


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and isfinite(value)
    )


def _is_non_blank(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


class RobotId(str, Enum):
    ROBOT1 = 'robot1'
    ROBOT2 = 'robot2'
    ROBOT3 = 'robot3'


ROBOT_IDS = tuple(RobotId)


class Formation(str, Enum):
    LINE = 'line'
    WEDGE = 'wedge'


class SquadAction(str, Enum):
    FORMATION_PATROL = 'formation_patrol'


@dataclass(frozen=True)
class Robot:
    robot_id: RobotId

    def __post_init__(self) -> None:
        if not isinstance(self.robot_id, RobotId):
            raise ValueError('robot_id must be a known RobotId')

    @property
    def namespace(self) -> str:
        return f'/{self.robot_id.value}'

    @property
    def odom_frame(self) -> str:
        return f'{self.robot_id.value}/odom'

    @property
    def base_frame(self) -> str:
        return f'{self.robot_id.value}/base_footprint'

    @property
    def navigation_action(self) -> str:
        return f'{self.namespace}/navigate_to_pose'


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    frame_id: str = 'map'

    def __post_init__(self) -> None:
        if not all(
            _is_finite_number(value)
            for value in (self.x, self.y, self.yaw)
        ):
            raise ValueError('pose coordinates must be finite')
        if self.frame_id != 'map':
            raise ValueError("fleet goals must use the shared 'map' frame")


@dataclass(frozen=True)
class RobotGoal:
    phase_id: str
    goal_id: str
    robot_id: RobotId
    pose: Pose2D

    def __post_init__(self) -> None:
        if not _is_non_blank(self.phase_id):
            raise ValueError('phase_id must not be blank')
        if not _is_non_blank(self.goal_id):
            raise ValueError('goal_id must not be blank')
        if not isinstance(self.robot_id, RobotId):
            raise ValueError('robot_id must be a known RobotId')
        if not isinstance(self.pose, Pose2D):
            raise ValueError('pose must be a Pose2D')


@dataclass(frozen=True)
class SquadPlan:
    plan_id: str
    action: SquadAction
    formation: Formation
    route_id: str
    spacing_m: float
    speed_mps: float
    split_route: bool
    regroup: bool
    regroup_location: str
    robots: tuple[RobotId, ...] = ROBOT_IDS
    goal_timeout_sec: float = 120.0
    mission_timeout_sec: float = 600.0
    max_retries: int = 1

    def __post_init__(self) -> None:
        if not _is_non_blank(self.plan_id):
            raise ValueError('plan_id must not be blank')
        if not isinstance(self.action, SquadAction):
            raise ValueError('action must be a supported SquadAction')
        if not isinstance(self.formation, Formation):
            raise ValueError('formation must be a supported Formation')
        if self.route_id != 'inspection_loop':
            raise ValueError("route_id must be 'inspection_loop'")
        if (
            not _is_finite_number(self.spacing_m)
            or not 0.60 <= self.spacing_m <= 1.20
        ):
            raise ValueError('spacing_m must be between 0.60 and 1.20')
        if (
            not _is_finite_number(self.speed_mps)
            or not 0.05 <= self.speed_mps <= maximum_squad_speed_mps()
        ):
            raise ValueError(
                'speed_mps must be between 0.05 and '
                f'{maximum_squad_speed_mps():.2f}'
            )
        if not isinstance(self.split_route, bool):
            raise ValueError('split_route must be a boolean')
        if not isinstance(self.regroup, bool):
            raise ValueError('regroup must be a boolean')
        if self.regroup_location != 'home':
            raise ValueError("regroup_location must be 'home'")
        if not isinstance(self.robots, tuple) or self.robots != ROBOT_IDS:
            raise ValueError('robots must contain robot1, robot2, and robot3')
        if (
            not _is_finite_number(self.goal_timeout_sec)
            or self.goal_timeout_sec <= 0
        ):
            raise ValueError('goal_timeout_sec must be finite and positive')
        if (
            not _is_finite_number(self.mission_timeout_sec)
            or self.mission_timeout_sec <= 0
        ):
            raise ValueError('mission_timeout_sec must be finite and positive')
        if self.mission_timeout_sec < self.goal_timeout_sec:
            raise ValueError(
                'mission_timeout_sec must not be shorter than goal_timeout_sec'
            )
        if (
            not isinstance(self.max_retries, int)
            or isinstance(self.max_retries, bool)
            or self.max_retries < 0
        ):
            raise ValueError('max_retries must be a non-negative integer')
