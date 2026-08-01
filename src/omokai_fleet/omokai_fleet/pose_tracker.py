"""Namespaced AMCL pose tracking for live emergency-separation checks."""

from __future__ import annotations

from math import atan2
from time import monotonic
from typing import Any

from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

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
        maximum_age_sec: float = 5.0,
        clock=monotonic,
    ) -> None:
        self._minimum_m = minimum_m
        self._maximum_age_sec = maximum_age_sec
        self._clock = clock
        self._node = node
        self._poses: dict[RobotId, Pose2D] = {}
        self._pose_messages = {}
        self._received_at: dict[RobotId, float] = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            node,
            spin_thread=False,
        )
        self._publisher = node.create_publisher(
            PoseArray,
            '/omokai_fleet/poses',
            10,
        )
        pose_qos = QoSProfile(depth=1)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._subscriptions = tuple(
            node.create_subscription(
                PoseWithCovarianceStamped,
                f'/{robot_id.value}/amcl_pose',
                lambda message, current=robot_id: self._on_pose(
                    current,
                    message,
                ),
                pose_qos,
            )
            for robot_id in ROBOT_IDS
        )

    @property
    def ready(self) -> bool:
        return all(robot_id in self._poses for robot_id in ROBOT_IDS)

    def violation_reason(self) -> str | None:
        self._refresh_from_tf()
        readiness_reason = self.readiness_reason()
        if readiness_reason:
            return readiness_reason
        return emergency_separation_reason(
            tuple(
                ObservedRobotPose(robot_id, self._poses[robot_id])
                for robot_id in ROBOT_IDS
            ),
            self._minimum_m,
        )

    def readiness_reason(self) -> str | None:
        missing = tuple(
            robot_id.value
            for robot_id in ROBOT_IDS
            if robot_id not in self._poses
        )
        if missing:
            return 'fleet pose unavailable for: ' + ', '.join(missing)
        now = self._clock()
        stale = tuple(
            robot_id.value
            for robot_id in ROBOT_IDS
            if now - self._received_at[robot_id] > self._maximum_age_sec
        )
        if stale:
            return 'fleet pose stale for: ' + ', '.join(stale)
        return None

    def _on_pose(
        self,
        robot_id: RobotId,
        message: PoseWithCovarianceStamped,
    ) -> None:
        if message.header.frame_id != 'map':
            return
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
        self._pose_messages[robot_id] = message.pose.pose
        self._received_at[robot_id] = self._clock()
        self._publish_snapshot()

    def _refresh_from_tf(self) -> None:
        for robot_id in ROBOT_IDS:
            try:
                transform = self._tf_buffer.lookup_transform(
                    'map',
                    f'{robot_id.value}/base_link',
                    Time(),
                ).transform
            except TransformException:
                continue
            rotation = transform.rotation
            yaw = atan2(
                2.0
                * (
                    rotation.w * rotation.z
                    + rotation.x * rotation.y
                ),
                1.0
                - 2.0
                * (
                    rotation.y * rotation.y
                    + rotation.z * rotation.z
                ),
            )
            translation = transform.translation
            self._poses[robot_id] = Pose2D(
                translation.x,
                translation.y,
                yaw,
            )
            pose = self._pose_messages.get(robot_id)
            if pose is None:
                pose = Pose()
                self._pose_messages[robot_id] = pose
            pose.position.x = translation.x
            pose.position.y = translation.y
            pose.position.z = translation.z
            pose.orientation = rotation
            self._received_at[robot_id] = self._clock()
        self._publish_snapshot()

    def _publish_snapshot(self) -> None:
        if self.ready:
            snapshot = PoseArray()
            snapshot.header.frame_id = 'map'
            snapshot.header.stamp = self._node.get_clock().now().to_msg()
            snapshot.poses = [
                self._pose_messages[current]
                for current in ROBOT_IDS
            ]
            self._publisher.publish(snapshot)
