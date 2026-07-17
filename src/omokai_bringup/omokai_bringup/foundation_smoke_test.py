"""Verify that the simulated robot publishes its essential sensor interfaces."""

from __future__ import annotations

import sys
import time
from typing import Type

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class FoundationSmokeTest(Node):
    """Wait for one message on each interface required by later milestones."""

    def __init__(self) -> None:
        super().__init__('foundation_smoke_test')
        self.received = {
            '/clock': False,
            '/odom': False,
            '/scan': False,
            '/tf': False,
        }
        self._subscribe(Clock, '/clock')
        self._subscribe(Odometry, '/odom')
        self._subscribe(LaserScan, '/scan')
        self._subscribe(TFMessage, '/tf')

    def _subscribe(self, message_type: Type, topic: str) -> None:
        self.create_subscription(
            message_type,
            topic,
            lambda _message, name=topic: self._mark_received(name),
            qos_profile_sensor_data,
        )

    def _mark_received(self, topic: str) -> None:
        if not self.received[topic]:
            self.get_logger().info(f'Received required interface: {topic}')
            self.received[topic] = True

    @property
    def complete(self) -> bool:
        return all(self.received.values())


def main() -> None:
    rclpy.init()
    node = FoundationSmokeTest()
    deadline = time.monotonic() + 30.0

    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.complete:
            rclpy.spin_once(node, timeout_sec=0.5)

        if not node.complete:
            missing = [name for name, received in node.received.items() if not received]
            node.get_logger().error(f'Missing required interfaces: {missing}')
            sys.exit(1)

        node.get_logger().info('Foundation smoke test passed.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
