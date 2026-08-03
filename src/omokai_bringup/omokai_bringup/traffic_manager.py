"""Velocity-level right-of-way arbitration for the three-robot simulation."""

from __future__ import annotations

from math import atan2, cos, sin
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from omokai_fleet.model import ROBOT_IDS, Pose2D, RobotId
from omokai_fleet.traffic import PlanarVelocity, decide_right_of_way


COMMAND_MAX_AGE_SEC = 0.5


class FleetTrafficManager(Node):
    def __init__(self) -> None:
        super().__init__('fleet_traffic_manager')
        self._buffer = Buffer()
        self._listener = TransformListener(
            self._buffer,
            self,
            spin_thread=False,
        )
        self._safe_commands: dict[RobotId, Twist] = {}
        self._safe_received_at: dict[RobotId, float] = {}
        self._intentions: dict[RobotId, Twist] = {}
        self._intent_received_at: dict[RobotId, float] = {}
        self._command_publishers = {
            robot_id: self.create_publisher(
                Twist,
                f'/{robot_id.value}/cmd_vel',
                10,
            )
            for robot_id in ROBOT_IDS
        }
        self._status_publisher = self.create_publisher(
            String,
            '/omokai_fleet/traffic_state',
            10,
        )
        self._safe_subscriptions = tuple(
            self.create_subscription(
                Twist,
                f'/{robot_id.value}/cmd_vel_safe',
                lambda message, current=robot_id: self._receive_safe(
                    current,
                    message,
                ),
                10,
            )
            for robot_id in ROBOT_IDS
        )
        self._intent_subscriptions = tuple(
            self.create_subscription(
                Twist,
                f'/{robot_id.value}/cmd_vel_smoothed',
                lambda message, current=robot_id: self._receive_intent(
                    current,
                    message,
                ),
                10,
            )
            for robot_id in ROBOT_IDS
        )
        self._last_status = ''
        self._timer = self.create_timer(0.05, self._update)

    def _receive_safe(self, robot_id: RobotId, message: Twist) -> None:
        self._safe_commands[robot_id] = message
        self._safe_received_at[robot_id] = monotonic()

    def _receive_intent(self, robot_id: RobotId, message: Twist) -> None:
        self._intentions[robot_id] = message
        self._intent_received_at[robot_id] = monotonic()

    def _update(self) -> None:
        now = monotonic()
        poses = self._read_poses()
        if poses is None:
            self._publish_commands((), now)
            self._publish_status('blocked: fleet pose unavailable')
            return
        decision = decide_right_of_way(
            poses,
            self._world_velocities(poses, now),
            now,
        )
        self._publish_commands(decision.allowed, now)
        if decision.yielding:
            priority = ','.join(
                robot_id.value for robot_id in decision.allowed
            )
            yielding = ','.join(
                robot_id.value for robot_id in decision.yielding
            )
            self._publish_status(
                f'priority={priority}; yielding={yielding}'
            )
        else:
            self._publish_status('clear')

    def _read_poses(self) -> dict[RobotId, Pose2D] | None:
        poses = {}
        for robot_id in ROBOT_IDS:
            try:
                transform = self._buffer.lookup_transform(
                    'map',
                    f'{robot_id.value}/base_link',
                    Time(),
                ).transform
            except TransformException:
                return None
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
            poses[robot_id] = Pose2D(
                transform.translation.x,
                transform.translation.y,
                yaw,
            )
        return poses

    def _world_velocities(
        self,
        poses: dict[RobotId, Pose2D],
        now: float,
    ) -> dict[RobotId, PlanarVelocity]:
        velocities = {}
        for robot_id in ROBOT_IDS:
            command = self._intentions.get(robot_id)
            received_at = self._intent_received_at.get(robot_id, 0.0)
            if command is None or now - received_at > COMMAND_MAX_AGE_SEC:
                velocities[robot_id] = PlanarVelocity(0.0, 0.0)
                continue
            yaw = poses[robot_id].yaw
            velocities[robot_id] = PlanarVelocity(
                command.linear.x * cos(yaw) - command.linear.y * sin(yaw),
                command.linear.x * sin(yaw) + command.linear.y * cos(yaw),
            )
        return velocities

    def _publish_commands(
        self,
        allowed: tuple[RobotId, ...],
        now: float,
    ) -> None:
        for robot_id in ROBOT_IDS:
            command = self._safe_commands.get(robot_id)
            received_at = self._safe_received_at.get(robot_id, 0.0)
            if (
                robot_id not in allowed
                or command is None
                or now - received_at > COMMAND_MAX_AGE_SEC
            ):
                command = Twist()
            self._command_publishers[robot_id].publish(command)

    def _publish_status(self, status: str) -> None:
        self._status_publisher.publish(String(data=status))
        if status != self._last_status:
            self.get_logger().info(status)
            self._last_status = status


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetTrafficManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
