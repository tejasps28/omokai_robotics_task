"""Namespaced AMCL pose tracking for live emergency-separation checks."""

from __future__ import annotations

from math import atan2
from typing import Any

from geometry_msgs.msg import PoseWithCovarianceStamped

from omokai_fleet.model import ROBOT_IDS, Pose2D, RobotId
from omokai_fleet.separation import (
    EMERGENCY_MINIMUM_DISTANCE_M,
    ObservedRobotPose,
    emergency_separation_reason,
)


class FleetPoseTracker:
    def __init__(
        self,
        node: Any,
        *,
        minimum_m: float = EMERGENCY_MINIMUM_DISTANCE_M,
    ) -> None:
        self._minimum_m = minimum_m
        self._poses: dict[RobotId, Pose2D] = {}
        self._subscriptions = tuple(
            node.create_subscription(
                PoseWithCovarianceStamped,
                f'/{robot_id.value}/amcl_pose',
                lambda message, current=robot_id: self._on_pose(
                    current,
                    message,
                ),
                10,
            )
            for robot_id in ROBOT_IDS
        )

    @property
    def ready(self) -> bool:
        return all(robot_id in self._poses for robot_id in ROBOT_IDS)

    def violation_reason(self) -> str | None:
        if not self.ready:
            return None
        return emergency_separation_reason(
            tuple(
                ObservedRobotPose(robot_id, self._poses[robot_id])
                for robot_id in ROBOT_IDS
            ),
            self._minimum_m,
        )

    def _on_pose(
        self,
        robot_id: RobotId,
        message: PoseWithCovarianceStamped,
    ) -> None:
        orientation = message.pose.pose.orientation
        yaw = atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        position = message.pose.pose.position
        self._poses[robot_id] = Pose2D(position.x, position.y, yaw)
