"""Republish namespaced fleet transforms for a single RViz view."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


ROBOT_NAMES = ('robot1', 'robot2', 'robot3')


class FleetTfAggregator(Node):
    def __init__(self) -> None:
        super().__init__('fleet_tf_aggregator')
        dynamic_input_qos = QoSProfile(depth=100)
        dynamic_input_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        dynamic_output_qos = QoSProfile(depth=100)
        dynamic_output_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos = QoSProfile(depth=10)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._dynamic_publisher = self.create_publisher(
            TFMessage,
            '/tf',
            dynamic_output_qos,
        )
        self._static_publisher = self.create_publisher(
            TFMessage,
            '/tf_static',
            static_qos,
        )
        self._subscriptions = []
        for robot_name in ROBOT_NAMES:
            self._subscriptions.extend(
                (
                    self.create_subscription(
                        TFMessage,
                        f'/{robot_name}/tf',
                        self._dynamic_publisher.publish,
                        dynamic_input_qos,
                    ),
                    self.create_subscription(
                        TFMessage,
                        f'/{robot_name}/tf_static',
                        self._static_publisher.publish,
                        static_qos,
                    ),
                )
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetTfAggregator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
