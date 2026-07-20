"""Trusted coordinates for the bounded three-robot demonstration."""

from __future__ import annotations

from math import pi

from omokai_fleet.allocation import RoutePoint
from omokai_fleet.mission import SquadMission, build_squad_mission
from omokai_fleet.model import Pose2D, SquadPlan


FORMATION_REFERENCES = (
    Pose2D(-2.0, -0.5, -3.0 * pi / 4.0),
    Pose2D(-1.7, -0.5, -3.0 * pi / 4.0),
)

SPLIT_ROUTE = (
    RoutePoint('west_south', Pose2D(-1.7, -1.2, pi / 2.0)),
    RoutePoint('west_home', Pose2D(-1.7, -0.5, -3.0 * pi / 4.0)),
    RoutePoint('middle_east', Pose2D(-0.4, -0.5, pi)),
    RoutePoint(
        'middle_home',
        Pose2D(-0.851472, -0.5, -3.0 * pi / 4.0),
    ),
    RoutePoint('west_north', Pose2D(-1.7, 1.05, -pi / 2.0)),
    RoutePoint(
        'north_home',
        Pose2D(-1.7, 0.348528, -3.0 * pi / 4.0),
    ),
)

HOME = Pose2D(-1.275736, -0.075736, -3.0 * pi / 4.0)


def build_demo_mission(plan: SquadPlan) -> SquadMission:
    return build_squad_mission(
        plan,
        formation_references=FORMATION_REFERENCES,
        route_points=SPLIT_ROUTE,
        home=HOME,
    )
