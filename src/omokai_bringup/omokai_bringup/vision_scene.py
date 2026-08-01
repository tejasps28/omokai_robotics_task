"""ROS-independent geometry for the minimal-office vision demonstration."""

from __future__ import annotations

import math


OFFICE_X_BOUNDS_M = (-5.0, 5.0)
OFFICE_Y_BOUNDS_M = (-5.0, 5.0)
ROBOT_START = (0.0, -2.0)
ROBOT_START_YAW_RAD = math.pi
CAMERA_HORIZONTAL_FOV_RAD = math.pi / 2.0
WHITE_ACTOR_START = (-1.0, -4.2)
RED_ACTOR_START = (0.8, -4.2)
STATIONARY_ACTOR_YAW_RAD = math.pi / 2.0

# The Black Coffee plugin applies TrajectoryPose relative to the actor's
# include pose, so path commands must be expressed in that local frame.
ACTOR_WAYPOINTS = (
    (0.0, 0.0),
    (2.0, 0.0),
    (2.0, 0.6),
    (0.0, 0.6),
    (0.0, 0.0),
)


def actor_world_waypoints(
    waypoints=ACTOR_WAYPOINTS,
) -> tuple[tuple[float, float], ...]:
    """Return actor-local path points resolved into the office frame."""

    return tuple(
        (WHITE_ACTOR_START[0] + x, WHITE_ACTOR_START[1] + y)
        for x, y in waypoints
    )


def path_length_m(waypoints=ACTOR_WAYPOINTS) -> float:
    """Return total Euclidean length for an ordered two-dimensional path."""

    return sum(
        math.hypot(end_x - start_x, end_y - start_y)
        for (start_x, start_y), (end_x, end_y) in zip(
            waypoints, waypoints[1:]
        )
    )


def wrapped_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""

    return math.atan2(math.sin(angle), math.cos(angle))


def initial_actor_bearing_rad() -> float:
    """Return the first actor waypoint bearing in the world frame."""

    actor_x, actor_y = actor_world_waypoints()[0]
    return math.atan2(actor_y - ROBOT_START[1], actor_x - ROBOT_START[0])


def initial_actor_outside_camera_fov(margin_rad: float = 0.10) -> bool:
    """Prove the configured target starts behind the parked camera."""

    offset = abs(
        wrapped_angle(initial_actor_bearing_rad() - ROBOT_START_YAW_RAD)
    )
    return offset > CAMERA_HORIZONTAL_FOV_RAD / 2.0 + margin_rad
