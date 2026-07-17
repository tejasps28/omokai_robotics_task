"""Publish the known simulation pose until AMCL confirms localization."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')
        self.declare_parameter('x', -2.0)
        self.declare_parameter('y', -0.5)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('timeout_sec', 60.0)

        self._x = float(self.get_parameter('x').value)
        self._y = float(self.get_parameter('y').value)
        self._yaw = float(self.get_parameter('yaw').value)
        period = float(self.get_parameter('publish_period_sec').value)
        self._timeout_sec = float(self.get_parameter('timeout_sec').value)
        if period <= 0 or self._timeout_sec <= 0:
            raise ValueError('publish period and timeout must be greater than zero')

        self._started_at = time.monotonic()
        self.localized = False
        self.timed_out = False
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        self._subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._on_amcl_pose,
            10,
        )
        self._timer = self.create_timer(period, self._publish_or_timeout)

    def _publish_or_timeout(self) -> None:
        if time.monotonic() - self._started_at >= self._timeout_sec:
            self.timed_out = True
            self.get_logger().error('Timed out waiting for AMCL localization')
            rclpy.shutdown()
            return
        if self._publisher.get_subscription_count() == 0:
            return

        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = self._x
        message.pose.pose.position.y = self._y
        message.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self._publisher.publish(message)
        self.get_logger().info(
            f'Published initial pose x={self._x:.2f}, y={self._y:.2f}, '
            f'yaw={self._yaw:.2f}'
        )

    def _on_amcl_pose(self, _: PoseWithCovarianceStamped) -> None:
        self.localized = True
        self.get_logger().info('AMCL localization confirmed')
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node.timed_out:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
