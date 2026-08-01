"""Publish a repeatable bounded path to the Gazebo actor plugin."""

from __future__ import annotations

import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from omokai_bringup.vision_scene import ACTOR_WAYPOINTS

WAYPOINTS = ACTOR_WAYPOINTS


class VisionActorPatrol(Node):
    """Republish one closed path when each bounded patrol cycle completes."""

    def __init__(self) -> None:
        super().__init__('vision_actor_patrol')
        # 5.2 m at the world's 0.28 m/s target speed, plus turn tolerance.
        self.declare_parameter('cycle_period_sec', 22.0)
        self.declare_parameter('bridge_wait_timeout_sec', 15.0)
        period = float(self.get_parameter('cycle_period_sec').value)
        bridge_timeout = float(
            self.get_parameter('bridge_wait_timeout_sec').value
        )
        if period <= 0:
            raise ValueError('cycle_period_sec must be positive')
        if bridge_timeout <= 0:
            raise ValueError('bridge_wait_timeout_sec must be positive')
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._publisher = self.create_publisher(
            PoseArray, '/vision_target/cmd_path', path_qos
        )
        self._bridge_wait_ticks = 0
        self._bridge_wait_limit = max(1, math.ceil(bridge_timeout / 0.5))
        self._startup_timer = self.create_timer(0.5, self._publish_first)
        self._cycle_timer = self.create_timer(period, self._publish)
        self._published = False

    def _publish_first(self) -> None:
        if self._published:
            return
        self._bridge_wait_ticks += 1
        if (
            self._publisher.get_subscription_count() == 0
            and self._bridge_wait_ticks < self._bridge_wait_limit
        ):
            return
        self._publish()
        self._published = True
        self._startup_timer.cancel()
        self.get_logger().info(
            'Published translating actor patrol after the ROS/Gazebo bridge '
            f'reported {self._publisher.get_subscription_count()} subscriber(s)'
        )

    def _publish(self) -> None:
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'world'
        for index, (x, y) in enumerate(WAYPOINTS):
            next_x, next_y = WAYPOINTS[(index + 1) % len(WAYPOINTS)]
            yaw = math.atan2(next_y - y, next_x - x)
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            message.poses.append(pose)
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionActorPatrol()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
