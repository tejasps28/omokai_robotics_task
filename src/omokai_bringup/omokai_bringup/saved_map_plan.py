"""Deterministic named locations used to verify saved-map navigation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from omokai_executor import ExecutionGoal, ExecutionPlan, GoalPose


@dataclass(frozen=True)
class NamedLocation:
    name: str
    x: float
    y: float
    yaw: float = 0.0

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError('location name must not be blank')
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError('location pose must contain finite values')


VERIFICATION_LOCATIONS = (
    NamedLocation('north_station', 0.548, 2.161, 0.0),
    NamedLocation('east_station', 3.574, 1.711, 0.0),
)


def build_saved_map_plan(
    mission_id: str,
    *,
    speed_mps: float = 0.15,
    locations: Iterable[NamedLocation] = VERIFICATION_LOCATIONS,
) -> ExecutionPlan:
    """Build an immutable execution plan in the saved SLAM map frame."""

    selected = tuple(locations)
    return ExecutionPlan(
        mission_id=mission_id,
        goals=tuple(
            ExecutionGoal(
                goal_id=location.name,
                pose=GoalPose(
                    x=location.x,
                    y=location.y,
                    yaw=location.yaw,
                    frame_id='map',
                ),
            )
            for location in selected
        ),
        speed_mps=speed_mps,
    )
