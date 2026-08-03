"""ROS adapter for continuous leader-relative formation tracking."""

from __future__ import annotations

import json
from math import atan2

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from omokai_fleet.formation_tracking import (
    FormationControlDirective,
    FormationTrackingConfig,
    follower_command,
    tracking_error,
)
from omokai_fleet.model import Formation, Pose2D, RobotId


FORMATION_COMMAND_TOPIC = '/omokai_fleet/formation_command'
FORMATION_STATE_TOPIC = '/omokai_fleet/formation_tracking_state'
FOLLOWERS = (RobotId.ROBOT2, RobotId.ROBOT3)


class FormationController(Node):
    """Track robot1 with robot2/3 through the existing Nav2 safety chain."""

    def __init__(self) -> None:
        super().__init__('fleet_formation_controller')
        self._buffer = Buffer()
        self._listener = TransformListener(
            self._buffer,
            self,
            spin_thread=False,
        )
        self._directive = FormationControlDirective(
            enabled=False,
            formation=Formation.WEDGE,
            spacing_m=0.6,
            max_linear_mps=0.18,
            operation_id='0' * 32,
        )
        self._active = False
        # ``Node`` owns an internal ``_publishers`` list, so keep the
        # application-level mapping under a distinct name.
        self._velocity_publishers = {
            robot_id: self.create_publisher(
                Twist,
                f'/{robot_id.value}/cmd_vel_nav',
                10,
            )
            for robot_id in FOLLOWERS
        }
        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        command_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._command_subscription = self.create_subscription(
            String,
            FORMATION_COMMAND_TOPIC,
            self._receive_command,
            command_qos,
        )
        self._state_publisher = self.create_publisher(
            String,
            FORMATION_STATE_TOPIC,
            10,
        )
        self._timer = self.create_timer(0.1, self._control_loop)

    def _receive_command(self, message: String) -> None:
        try:
            directive = FormationControlDirective.from_mapping(
                json.loads(message.data)
            )
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(f'rejected formation command: {exc}')
            return

        was_active = self._active
        self._directive = directive
        self._active = directive.enabled
        if was_active and not self._active:
            self._publish_stop()
        self.get_logger().info(
            'formation tracking '
            + ('enabled' if self._active else 'disabled')
            + f': {directive.formation.value} spacing={directive.spacing_m:.2f}m'
        )

    def _control_loop(self) -> None:
        if not self._active:
            return
        try:
            leader = self._read_pose(RobotId.ROBOT1)
            followers = {
                robot_id: self._read_pose(robot_id)
                for robot_id in FOLLOWERS
            }
        except TransformException as exc:
            self._publish_stop()
            self._publish_state({
                'enabled': True,
                'operation_id': self._directive.operation_id,
                'blocked': str(exc),
            })
            return

        state = {
            'enabled': True,
            'operation_id': self._directive.operation_id,
            'followers': {},
        }
        for robot_id in FOLLOWERS:
            error = tracking_error(
                leader,
                followers[robot_id],
                self._directive.follower_offset(robot_id),
            )
            command = follower_command(
                error,
                FormationTrackingConfig(
                    max_linear_mps=self._directive.max_linear_mps,
                ),
            )
            message = Twist()
            message.linear.x = command.linear_mps
            message.angular.z = command.angular_rps
            self._velocity_publishers[robot_id].publish(message)
            state['followers'][robot_id.value] = {
                'distance_error_m': round(error.distance_m, 4),
                'forward_error_m': round(error.forward_error_m, 4),
                'left_error_m': round(error.left_error_m, 4),
                'target_reached': command.target_reached,
            }
        self._publish_state(state)

    def _read_pose(self, robot_id: RobotId) -> Pose2D:
        transform = self._buffer.lookup_transform(
            'map',
            f'{robot_id.value}/base_link',
            Time(),
        ).transform
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
        return Pose2D(
            transform.translation.x,
            transform.translation.y,
            yaw,
        )

    def _publish_stop(self) -> None:
        stop = Twist()
        for publisher in self._velocity_publishers.values():
            publisher.publish(stop)

    def _publish_state(self, value: dict) -> None:
        self._state_publisher.publish(
            String(data=json.dumps(value, sort_keys=True))
        )

    def destroy_node(self):
        self._publish_stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
