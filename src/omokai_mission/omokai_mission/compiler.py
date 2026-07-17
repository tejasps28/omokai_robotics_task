"""Deterministic mission compiler.

Turns a semantically :class:`~omokai_interfaces.ValidatedMission` into an ordered,
immutable list of navigation goals by resolving its symbolic ``route_id`` against
the audited route catalog. The planner never supplies coordinates; every pose
here comes from the catalog.

Determinism is the core contract: for the same accepted mission and the same
catalog, the compiled goal sequence is always byte-for-byte identical. A patrol
compiles each validated traversal segment in order. Canonical forward or
counterclockwise segments use catalog order; reverse or clockwise segments use
a deterministic reversed ordering. Home is appended when requested.

The executor consumes ``CompiledRoute`` and owns motion. It must not implement
a second compiler.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from omokai_interfaces import TraversalDirection, ValidatedMission

from .catalog import CatalogError, Pose2D, Route, RouteCatalog, Waypoint


@dataclass(frozen=True)
class NavGoal:
    """One ordered navigation goal with an auditable label."""

    label: str
    frame_id: str
    pose: Pose2D

    def summary(self) -> str:
        """One-line human summary used by diagnostics and audit logs."""

        return (
            f'{self.label} @ ({self.pose.x:.3f}, {self.pose.y:.3f}, '
            f'yaw={self.pose.yaw:.3f}) [{self.frame_id}]'
        )


@dataclass(frozen=True)
class CompiledRoute:
    """The immutable ordered goal plan for one accepted mission."""

    mission_id: str
    route_id: str
    frame_id: str
    goals: Tuple[NavGoal, ...]

    def summary_lines(self) -> List[str]:
        """Return an ordered, numbered summary of every goal."""

        return [f'{index}. {goal.summary()}' for index, goal in enumerate(self.goals, 1)]


def compile_mission(mission: ValidatedMission, catalog: RouteCatalog) -> CompiledRoute:
    """Compile an accepted mission into an ordered goal plan.

    ``catalog.get`` raises :class:`CatalogError` if the route is unknown. Semantic
    validation should already have rejected unknown routes, so this is a
    defensive guarantee that the compiler never emits an empty or arbitrary plan.
    """

    spec = mission.mission
    route = catalog.get(spec.route_id)
    if not route.waypoints:
        raise CatalogError(f'route {spec.route_id!r} has no waypoints')

    goals: List[NavGoal] = []
    lap = 0
    for segment_index, segment in enumerate(spec.segments, 1):
        waypoints = _waypoints_for_direction(route, segment.direction)
        for _ in range(segment.repetitions):
            lap += 1
            for waypoint in waypoints:
                goals.append(
                    NavGoal(
                        label=(
                            f'{route.route_id}/segment{segment_index}/'
                            f'{segment.direction.value}/lap{lap}/{waypoint.name}'
                        ),
                        frame_id=catalog.frame_id,
                        pose=waypoint.pose,
                    )
                )

    if spec.return_home:
        goals.append(
            NavGoal(label='home', frame_id=catalog.frame_id, pose=catalog.home)
        )

    return CompiledRoute(
        mission_id=mission.mission_id,
        route_id=route.route_id,
        frame_id=catalog.frame_id,
        goals=tuple(goals),
    )


def _waypoints_for_direction(
    route: Route,
    direction: TraversalDirection,
) -> Tuple[Waypoint, ...]:
    """Return the deterministic waypoint ordering for one segment."""

    if direction not in route.allowed_directions:
        raise CatalogError(
            f'direction {direction.value!r} is not allowed for route '
            f'{route.route_id!r}'
        )
    if direction in {
        TraversalDirection.COUNTERCLOCKWISE,
        TraversalDirection.FORWARD,
    }:
        return route.waypoints

    if route.closed:
        # Preserve the audited starting anchor, then traverse the remaining
        # closed loop in the opposite direction.
        ordered = (route.waypoints[0], *reversed(route.waypoints[1:]))
    else:
        ordered = tuple(reversed(route.waypoints))
    return _orient_waypoints(tuple(ordered), closed=route.closed)


def _orient_waypoints(
    waypoints: Tuple[Waypoint, ...],
    *,
    closed: bool,
) -> Tuple[Waypoint, ...]:
    """Face each reversed goal toward the next deterministic route leg."""

    if len(waypoints) < 2:
        return waypoints
    oriented = []
    for index, waypoint in enumerate(waypoints):
        if index + 1 < len(waypoints):
            target = waypoints[index + 1]
            dx = target.pose.x - waypoint.pose.x
            dy = target.pose.y - waypoint.pose.y
        elif closed:
            target = waypoints[0]
            dx = target.pose.x - waypoint.pose.x
            dy = target.pose.y - waypoint.pose.y
        else:
            previous = waypoints[index - 1]
            dx = waypoint.pose.x - previous.pose.x
            dy = waypoint.pose.y - previous.pose.y
        oriented.append(
            Waypoint(
                name=waypoint.name,
                pose=Pose2D(
                    x=waypoint.pose.x,
                    y=waypoint.pose.y,
                    yaw=math.atan2(dy, dx),
                ),
            )
        )
    return tuple(oriented)
