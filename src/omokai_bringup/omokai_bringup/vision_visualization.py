"""Correct Gazebo RGB-D cloud metadata and create an RViz depth preview."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from omokai_bringup.vision_depth import depth_preview


CAMERA_BODY_FRAME = 'robot1/camera_rgb_frame'


class VisionVisualization(Node):
    """Publish corrected cloud metadata and a human-readable depth image."""

    def __init__(self) -> None:
        super().__init__('vision_visualization')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._cloud_publisher = self.create_publisher(
            PointCloud2, '/robot1/camera/points', qos
        )
        self._depth_publisher = self.create_publisher(
            Image, '/robot1/camera/depth_visualization', qos
        )
        self.create_subscription(
            PointCloud2,
            '/robot1/camera/points_raw',
            self._on_cloud,
            qos,
        )
        self.create_subscription(
            Image,
            '/robot1/camera/depth_image',
            self._on_depth,
            qos,
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        # Gazebo emits XYZ in its camera-body convention (X forward), even
        # when the RGB/depth image header uses the ROS optical frame.
        message.header.frame_id = CAMERA_BODY_FRAME
        self._cloud_publisher.publish(message)

    def _on_depth(self, message: Image) -> None:
        if message.encoding != '32FC1':
            self.get_logger().warning(
                f'Expected 32FC1 depth, received {message.encoding}'
            )
            return
        preview = Image()
        preview.header = message.header
        preview.height = message.height
        preview.width = message.width
        preview.encoding = 'mono8'
        preview.is_bigendian = 0
        preview.step = message.width
        preview.data = depth_preview(bytes(message.data))
        self._depth_publisher.publish(preview)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionVisualization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
