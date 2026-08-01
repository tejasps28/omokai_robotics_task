"""ROS 2 node that detects stationary or moving people in the RGB stream."""

from __future__ import annotations

import time
import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

from omokai_perception.attributes import CoatColorSelector
from omokai_perception.mission import arm_response
from omokai_perception.mission import target_detection_id
from omokai_perception.tracker import TemporalTargetTracker
from omokai_perception.yolox import YoloXPersonDetector
from omokai_perception.yolox import annotate_selection


DEFAULT_MODEL = '/opt/omokai_models/object_detection_yolox_2022nov.onnx'


def image_to_bgr(message: Image) -> np.ndarray:
    """Convert supported ROS RGB encodings without cv_bridge."""
    if message.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported image encoding {message.encoding}')
    row_bytes = message.width * 3
    if message.step < row_bytes:
        raise ValueError('image step is shorter than one RGB row')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = rows[:, :row_bytes].reshape(message.height, message.width, 3)
    if message.encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def bgr_to_message(image: np.ndarray, source: Image) -> Image:
    result = Image()
    result.header = source.header
    result.height = image.shape[0]
    result.width = image.shape[1]
    result.encoding = 'bgr8'
    result.is_bigendian = 0
    result.step = image.shape[1] * 3
    result.data = image.tobytes()
    return result


class PersonDetectorNode(Node):
    """Bounded-rate person detector with typed and annotated outputs."""

    def __init__(self) -> None:
        super().__init__('person_detector')
        self.declare_parameter('model_path', DEFAULT_MODEL)
        # Synthetic Gazebo actors score lower than photographic COCO people.
        # Temporal confirmation will provide the later acquisition gate.
        self.declare_parameter('confidence_threshold', 0.30)
        self.declare_parameter('nms_threshold', 0.50)
        self.declare_parameter('max_inference_hz', 5.0)
        self.declare_parameter('target_coat_color', 'white')
        # Gazebo's white coat contains grey shading and a green shirt, so the
        # central crop has a small but stable white-pixel ratio.  The red
        # distractor remains far above this gate; the ambiguity margin keeps
        # the decisions mutually exclusive.
        self.declare_parameter('attribute_minimum_ratio', 0.02)
        self.declare_parameter('attribute_ambiguity_margin', 0.015)
        self.declare_parameter('confirmation_frames', 3)
        self.declare_parameter('maximum_missing_frames', 8)
        self._detector = YoloXPersonDetector(
            self.get_parameter('model_path').value,
            float(self.get_parameter('confidence_threshold').value),
            float(self.get_parameter('nms_threshold').value),
        )
        max_hz = float(self.get_parameter('max_inference_hz').value)
        if max_hz <= 0:
            raise ValueError('max_inference_hz must be positive')
        self._minimum_period = 1.0 / max_hz
        self._last_started = -self._minimum_period
        self._frames = 0
        self._active_color = str(
            self.get_parameter('target_coat_color').value
        ).lower()
        self._armed = False
        self._prepared = False
        self._mission_generation = 0
        self._tracker = TemporalTargetTracker(
            int(self.get_parameter('confirmation_frames').value),
            int(self.get_parameter('maximum_missing_frames').value),
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self._image_publisher = self.create_publisher(
            Image, '/vision/detections/image', qos
        )
        self._detection_publisher = self.create_publisher(
            Detection2DArray, '/vision/detections', qos
        )
        self._target_publisher = self.create_publisher(
            Detection2DArray, '/vision/target_detection', qos
        )
        self._status_publisher = self.create_publisher(
            String, '/vision/selection_status', qos
        )
        self.create_subscription(
            Image, '/robot1/camera/image', self._on_image, qos
        )
        self.create_service(Trigger, '/vision/arm_detector', self._arm)
        self.create_service(
            Trigger, '/vision/activate_detector', self._activate
        )
        self.create_service(Trigger, '/vision/disarm_detector', self._disarm)
        self.get_logger().info(
            'YOLOX person detector ready: /vision/detections and '
            '/vision/detections/image'
        )

    def _on_image(self, message: Image) -> None:
        started = time.monotonic()
        if started - self._last_started < self._minimum_period:
            return
        self._last_started = started
        try:
            image = image_to_bgr(message)
            detections = self._detector.detect(image)
            requested_color = str(
                self.get_parameter('target_coat_color').value
            ).strip().lower()
            if requested_color != self._active_color:
                self._tracker.reset()
                self._active_color = requested_color
            selector = CoatColorSelector(
                requested_color,
                float(
                    self.get_parameter('attribute_minimum_ratio').value
                ),
                float(
                    self.get_parameter('attribute_ambiguity_margin').value
                ),
            )
            candidates = [
                selector.classify(image, detection)
                for detection in detections
            ]
            tracking = (
                self._tracker.update(candidates)
                if self._armed
                else self._tracker.result()
            )
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f'Detection failed: {error}')
            return

        output = Detection2DArray()
        output.header = message.header
        for index, item in enumerate(detections):
            detection = Detection2D()
            detection.header = message.header
            detection.id = f'person-{index}'
            detection.bbox.center.position.x = item.x + item.width / 2.0
            detection.bbox.center.position.y = item.y + item.height / 2.0
            detection.bbox.size_x = float(item.width)
            detection.bbox.size_y = float(item.height)
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = 'person'
            result.hypothesis.score = item.confidence
            detection.results.append(result)
            output.detections.append(detection)
        self._detection_publisher.publish(output)
        target_output = Detection2DArray()
        target_output.header = message.header
        if tracking.confirmed and tracking.target is not None:
            target_output.detections.append(
                self._to_detection_message(
                    tracking.target.box.x,
                    tracking.target.box.y,
                    tracking.target.box.width,
                    tracking.target.box.height,
                    tracking.target.detection_confidence,
                    message,
                    target_detection_id(
                        self._mission_generation, tracking.episode
                    ),
                )
            )
        self._target_publisher.publish(target_output)
        selected_box = (
            tracking.target.box if tracking.target is not None else None
        )
        annotated = annotate_selection(
            image,
            candidates,
            selected_box,
            tracking.confirmed,
        )
        self._image_publisher.publish(
            bgr_to_message(annotated, message)
        )
        status = String()
        status.data = json.dumps(
            {
                'target_class': 'person',
                'target_coat_color': requested_color,
                'armed': self._armed,
                'prepared': self._prepared,
                'mission_generation': self._mission_generation,
                'episode': tracking.episode,
                'confirmed': tracking.confirmed,
                'confirmation_count': tracking.confirmation_count,
                'missing_count': tracking.missing_count,
                'accepted_count': sum(
                    candidate.decision.value == 'accepted'
                    for candidate in candidates
                ),
                'rejected_count': sum(
                    candidate.decision.value != 'accepted'
                    for candidate in candidates
                ),
                'candidates': [
                    {
                        'decision': candidate.decision.value,
                        'observed_color': candidate.observed_color,
                        'requested_color': candidate.requested_color,
                        'attribute_score': candidate.attribute_score,
                        'detection_confidence': (
                            candidate.detection_confidence
                        ),
                        'bbox': {
                            'x': candidate.box.x,
                            'y': candidate.box.y,
                            'width': candidate.box.width,
                            'height': candidate.box.height,
                        },
                    }
                    for candidate in candidates
                ],
            },
            sort_keys=True,
        )
        self._status_publisher.publish(status)
        self._frames += 1
        if detections and self._frames % 10 == 1:
            self.get_logger().info(
                f'Person detected, confidence={detections[0].confidence:.3f}'
            )

    def _arm(self, request, response):
        del request
        requested_color = str(
            self.get_parameter('target_coat_color').value
        ).strip().lower()
        if requested_color not in {'white', 'red', 'any'}:
            response.success = False
            response.message = 'unsupported target coat colour'
            return response
        self._tracker.reset()
        self._active_color = requested_color
        self._mission_generation += 1
        self._armed = False
        self._prepared = True
        response.success = True
        response.message = arm_response(
            self._mission_generation, 'person', requested_color
        )
        self.get_logger().info(
            'Detector prepared for fresh mission generation '
            f'{self._mission_generation}: person/{requested_color}'
        )
        return response

    def _activate(self, request, response):
        del request
        if not self._prepared or self._mission_generation < 1:
            response.success = False
            response.message = 'detector has no prepared mission generation'
            return response
        self._tracker.reset()
        self._armed = True
        self._prepared = False
        response.success = True
        response.message = arm_response(
            self._mission_generation, 'person', self._active_color
        )
        self.get_logger().info(
            'Detector activated with empty tracker state for generation '
            f'{self._mission_generation}'
        )
        return response

    def _disarm(self, request, response):
        del request
        self._tracker.reset()
        self._armed = False
        self._prepared = False
        response.success = True
        response.message = 'detector disarmed and tracking state cleared'
        return response

    @staticmethod
    def _to_detection_message(
        x: int,
        y: int,
        width: int,
        height: int,
        confidence: float,
        source: Image,
        detection_id: str,
    ) -> Detection2D:
        detection = Detection2D()
        detection.header = source.header
        detection.id = detection_id
        detection.bbox.center.position.x = x + width / 2.0
        detection.bbox.center.position.y = y + height / 2.0
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = 'person'
        result.hypothesis.score = confidence
        detection.results.append(result)
        return detection


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
