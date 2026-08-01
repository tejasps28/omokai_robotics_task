"""Trusted coordinates for the bounded three-robot demonstration."""

from __future__ import annotations

from math import hypot, pi

from omokai_fleet.allocation import RoutePoint
from omokai_fleet.formation import formation_goals
from omokai_fleet.mission import SquadMission, build_squad_mission
from omokai_fleet.model import (
    ROBOT_IDS,
    Formation,
    Pose2D,
    RobotGoal,
    RobotId,
    SquadPlan,
)


DOCK_REFERENCE = Pose2D(5.4, 0.0, pi)
# A short, obstacle-free inward leg makes the formation phase observable while
# keeping the squad clear of the central doorway used by the split phase.
FORMATION_PATROL_REFERENCE = Pose2D(4.8, 0.0, pi)
DOCK_HOME = Pose2D(6.0, 0.0, pi)
DOCKING_POSES = tuple(
    goal.pose
    for goal in formation_goals(
        DOCK_REFERENCE,
        Formation.WEDGE,
        0.6,
        phase_id='docked',
        step_index=1,
    )
)

FORMATION_REFERENCES = (
    DOCK_REFERENCE,
    FORMATION_PATROL_REFERENCE,
)

ROOM_TARGETS = (
    # Keep the main-room hold clear of the central passage used by robots
    # returning from the two enclosed rooms.
    RoutePoint('main_room', Pose2D(4.0, 4.5, pi)),
    RoutePoint('lower_room', Pose2D(-3.0, -3.5, pi)),
    RoutePoint('upper_room', Pose2D(-3.5, 4.0, pi)),
)


def build_demo_mission(plan: SquadPlan) -> SquadMission:
    mission = build_squad_mission(
        plan,
        formation_references=FORMATION_REFERENCES,
        route_points=ROOM_TARGETS,
        home=DOCK_HOME,
    )
    if not plan.split_route:
        return mission
    return SquadMission(
        plan=mission.plan,
        forming=mission.forming,
        formation_movement=mission.formation_movement,
        split_execution=_closest_first_room_batches(
            mission.formation_movement[-1]
        ),
        regrouping=mission.regrouping,
    )


def _closest_first_room_batches(
    dock_goals: tuple[RobotGoal, ...],
) -> tuple[tuple[RobotGoal, ...], ...]:
    dock_poses = {goal.robot_id: goal.pose for goal in dock_goals}
    room_poses = {
        robot_id: target.pose
        for robot_id, target in zip(ROBOT_IDS, ROOM_TARGETS)
    }
    room_names = {
        robot_id: target.point_id
        for robot_id, target in zip(ROBOT_IDS, ROOM_TARGETS)
    }
    outbound = _distance_order(dock_poses, room_poses)
    # Fill the outer slots before the apex. Pure nearest-first return order
    # would park robot1 in the shared approach corridor too early.
    outer_robots = tuple(
        robot_id for robot_id in ROBOT_IDS if robot_id is not RobotId.ROBOT1
    )
    inbound = _distance_order(
        room_poses,
        dock_poses,
        robot_ids=outer_robots,
    ) + (RobotId.ROBOT1,)
    current = dict(dock_poses)
    batches = []

    for step, moving_robot in enumerate(outbound, start=1):
        current[moving_robot] = room_poses[moving_robot]
        batches.append(
            _position_batch(
                current,
                phase=f'outbound{step}',
                moving_robot=moving_robot,
                destination=room_names[moving_robot],
            )
        )
    for step, moving_robot in enumerate(inbound, start=1):
        current[moving_robot] = dock_poses[moving_robot]
        batches.append(
            _position_batch(
                current,
                phase=f'inbound{step}',
                moving_robot=moving_robot,
                destination='dock',
            )
        )
    return tuple(batches)


def _distance_order(
    starts: dict[RobotId, Pose2D],
    destinations: dict[RobotId, Pose2D],
    *,
    robot_ids: tuple[RobotId, ...] = ROBOT_IDS,
) -> tuple[RobotId, ...]:
    return tuple(
        sorted(
            robot_ids,
            key=lambda robot_id: (
                hypot(
                    starts[robot_id].x - destinations[robot_id].x,
                    starts[robot_id].y - destinations[robot_id].y,
                ),
                ROBOT_IDS.index(robot_id),
            ),
        )
    )


def _position_batch(
    positions: dict[RobotId, Pose2D],
    *,
    phase: str,
    moving_robot: RobotId,
    destination: str,
) -> tuple[RobotGoal, ...]:
    return tuple(
        RobotGoal(
            phase_id='executing_split',
            goal_id=(
                f'executing_split/{phase}/{robot_id.value}/'
                f'{destination if robot_id is moving_robot else "hold"}'
            ),
            robot_id=robot_id,
            pose=positions[robot_id],
        )
        for robot_id in ROBOT_IDS
    )
