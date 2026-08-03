"""ROS coordinator for synchronized localization, notification, and following."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from omokai_following.artifacts import write_json
from omokai_following.controller import FollowController
from omokai_following.domain import FollowConfig
from omokai_following.domain import FollowState
from omokai_following.domain import TargetObservation
from omokai_following.domain import TERMINAL_STATES
from omokai_following.scan import BoundedScanTracker
from omokai_perception.localization import CameraIntrinsics
from omokai_perception.localization import DepthPolicy
from omokai_perception.localization import depth_array
from omokai_perception.localization import localize_box
from omokai_perception.localization import protected_sector_distance
from omokai_perception.localization import timestamp_seconds
from omokai_perception.model import BoundingBox
from omokai_perception.person_detector import image_to_bgr
from omokai_perception.mission import parse_target_detection_id
from omokai_perception.snapshots import SnapshotWriter


class VisionFollower(Node):
    """Exclusive velocity owner for the dedicated vision launch."""

    def __init__(self) -> None:
        super().__init__('vision_follower')
        self._declare_parameters()
        self._controller = FollowController(self._read_config())
        self._depth_policy = DepthPolicy(
            minimum_m=float(self.get_parameter('minimum_depth_m').value),
            maximum_m=float(self.get_parameter('maximum_depth_m').value),
            minimum_valid_ratio=float(
                self.get_parameter('minimum_valid_depth_ratio').value
            ),
            maximum_range_jump_m=float(
                self.get_parameter('maximum_range_jump_m').value
            ),
        )
        self._sync_tolerance = float(
            self.get_parameter('sync_tolerance_sec').value
        )
        self._artifact_root = Path(
            str(self.get_parameter('artifact_root').value)
        )
        self._snapshot_writer = SnapshotWriter(self._artifact_root)
        self._mission_id = ''
        self._mission_generation = 0
        self._last_started_generation = 0
        self._latest_observation: TargetObservation | None = None
        self._last_range: float | None = None
        self._last_decision_state = FollowState.IDLE
        self._terminal_written = False
        self._camera_info: CameraInfo | None = None
        self._depth_cache: deque[tuple[float, Image]] = deque(maxlen=30)
        self._latest_obstacle_distance: float | None = None
        self._latest_depth_received_at: float | None = None
        self._pending_targets: deque[Detection2DArray] = deque(maxlen=10)
        self._rgb_cache: deque[tuple[float, Image]] = deque(maxlen=30)
        self._annotated_cache: deque[tuple[float, Image]] = deque(maxlen=30)
        self._last_sync_difference: float | None = None
        self._pending_snapshot: tuple[
            float, int, int, float, str
        ] | None = None
        self._scan = BoundedScanTracker()
        self._scan.stop()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._velocity_publisher = self.create_publisher(
            Twist, '/robot1/cmd_vel', qos
        )
        self._status_publisher = self.create_publisher(
            String, '/vision/follow_status', qos
        )
        self._event_publisher = self.create_publisher(
            String, '/vision/operator_event', event_qos
        )
        self.create_subscription(
            Detection2DArray,
            '/vision/target_detection',
            self._on_target,
            qos,
        )
        self.create_subscription(
            Image, '/robot1/camera/depth_image', self._on_depth, qos
        )
        self.create_subscription(
            CameraInfo,
            '/robot1/camera/camera_info',
            self._on_camera_info,
            qos,
        )
        self.create_subscription(
            Image, '/robot1/camera/image', self._on_rgb, qos
        )
        self.create_subscription(
            Image, '/vision/detections/image', self._on_annotated, qos
        )
        self.create_subscription(
            Odometry, '/robot1/odom', self._on_odometry, qos
        )
        self.create_service(Trigger, '/vision/start', self._start)
        self.create_service(Trigger, '/vision/cancel', self._cancel)
        self.create_service(Trigger, '/vision/status', self._status)
        self.create_timer(0.10, self._tick)

    def _declare_parameters(self) -> None:
        self.declare_parameter('mission_id', 'vision-current')
        self.declare_parameter('mission_generation', 0)
        self.declare_parameter('artifact_root', '/data/artifacts')
        self.declare_parameter('target_coat_color', 'white')
        self.declare_parameter('desired_standoff_m', 1.20)
        self.declare_parameter('standoff_tolerance_m', 0.15)
        self.declare_parameter('maximum_linear_mps', 0.38)
        self.declare_parameter('maximum_angular_rps', 0.50)
        self.declare_parameter('range_gain', 0.65)
        self.declare_parameter('bearing_gain', 1.0)
        self.declare_parameter('turn_slow_angle_rad', 0.65)
        self.declare_parameter('observation_timeout_sec', 0.75)
        self.declare_parameter('reacquisition_timeout_sec', 10.0)
        self.declare_parameter('reacquisition_angular_rps', 0.25)
        self.declare_parameter('reacquisition_sweep_period_sec', 2.0)
        self.declare_parameter('initial_search_angular_rps', 0.40)
        self.declare_parameter('emergency_stop_distance_m', 0.55)
        self.declare_parameter('maximum_follow_distance_m', 8.0)
        self.declare_parameter('mission_timeout_sec', 180.0)
        self.declare_parameter('sync_tolerance_sec', 0.08)
        self.declare_parameter('minimum_depth_m', 0.12)
        self.declare_parameter('maximum_depth_m', 8.0)
        self.declare_parameter('minimum_valid_depth_ratio', 0.20)
        self.declare_parameter('maximum_range_jump_m', 1.0)

    def _read_config(self) -> FollowConfig:
        return FollowConfig(
            desired_standoff_m=float(
                self.get_parameter('desired_standoff_m').value
            ),
            standoff_tolerance_m=float(
                self.get_parameter('standoff_tolerance_m').value
            ),
            maximum_linear_mps=float(
                self.get_parameter('maximum_linear_mps').value
            ),
            maximum_angular_rps=float(
                self.get_parameter('maximum_angular_rps').value
            ),
            range_gain=float(self.get_parameter('range_gain').value),
            bearing_gain=float(self.get_parameter('bearing_gain').value),
            turn_slow_angle_rad=float(
                self.get_parameter('turn_slow_angle_rad').value
            ),
            observation_timeout_sec=float(
                self.get_parameter('observation_timeout_sec').value
            ),
            reacquisition_timeout_sec=float(
                self.get_parameter('reacquisition_timeout_sec').value
            ),
            reacquisition_angular_rps=float(
                self.get_parameter('reacquisition_angular_rps').value
            ),
            reacquisition_sweep_period_sec=float(
                self.get_parameter('reacquisition_sweep_period_sec').value
            ),
            initial_search_angular_rps=float(
                self.get_parameter('initial_search_angular_rps').value
            ),
            emergency_stop_distance_m=float(
                self.get_parameter('emergency_stop_distance_m').value
            ),
            maximum_follow_distance_m=float(
                self.get_parameter('maximum_follow_distance_m').value
            ),
            mission_timeout_sec=float(
                self.get_parameter('mission_timeout_sec').value
            ),
        )

    def _on_depth(self, message: Image) -> None:
        try:
            depth = depth_array(
                bytes(message.data),
                message.width,
                message.height,
                message.step,
                message.encoding,
                bool(message.is_bigendian),
            )
            self._latest_obstacle_distance = protected_sector_distance(
                depth, self._depth_policy
            )
            self._latest_depth_received_at = self._now()
        except ValueError:
            self._latest_obstacle_distance = None
            self._latest_depth_received_at = None
        self._depth_cache.append(
            (timestamp_seconds(message.header.stamp), message)
        )
        self._process_pending_targets()

    def _on_rgb(self, message: Image) -> None:
        self._rgb_cache.append(
            (timestamp_seconds(message.header.stamp), message)
        )

    def _on_annotated(self, message: Image) -> None:
        self._annotated_cache.append(
            (timestamp_seconds(message.header.stamp), message)
        )

    def _on_odometry(self, message: Odometry) -> None:
        if self._controller.state is not FollowState.SEARCHING:
            return
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
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
        self._scan.observe(yaw)

    def _on_camera_info(self, message: CameraInfo) -> None:
        if (
            message.width > 0
            and message.height > 0
            and message.k[0] > 0
            and message.k[4] > 0
        ):
            self._camera_info = message
            self._process_pending_targets()

    def _on_target(self, message: Detection2DArray) -> None:
        if (
            self._controller.state is FollowState.IDLE
            or self._controller.state in TERMINAL_STATES
            or not message.detections
        ):
            return
        identifier = parse_target_detection_id(message.detections[0].id)
        if identifier is None or identifier[0] != self._mission_generation:
            return
        self._pending_targets.append(message)
        self._process_pending_targets()

    def _process_pending_targets(self) -> None:
        """Consume detections only after their aligned depth frame arrives."""
        if (
            self._controller.state is FollowState.IDLE
            or self._controller.state in TERMINAL_STATES
        ):
            self._pending_targets.clear()
            return
        if self._camera_info is None or not self._depth_cache:
            return
        while self._pending_targets:
            message = self._pending_targets[0]
            source_time = timestamp_seconds(message.header.stamp)
            depth_entry = self._nearest_entry(
                self._depth_cache, source_time
            )
            if depth_entry is None:
                # DDS may deliver the detector result before the matching
                # depth sample. Wait while that sample can still arrive, but
                # discard a target once the depth stream has advanced beyond
                # the permitted synchronization window.
                if self._depth_cache[-1][0] <= (
                    source_time + self._sync_tolerance
                ):
                    return
                self._pending_targets.popleft()
                self._publish_localization_error(
                    'aligned_depth_unavailable_within_tolerance'
                )
                continue
            self._pending_targets.popleft()
            depth_time, depth_message = depth_entry
            self._last_sync_difference = abs(depth_time - source_time)
            self._process_target(message, source_time, depth_message)

    def _process_target(
        self,
        message: Detection2DArray,
        source_time: float,
        depth_message: Image,
    ) -> None:
        detection = message.detections[0]
        identifier = parse_target_detection_id(detection.id)
        if identifier is None or identifier[0] != self._mission_generation:
            return
        box = self._box_from_detection(detection)
        try:
            depth = depth_array(
                bytes(depth_message.data),
                depth_message.width,
                depth_message.height,
                depth_message.step,
                depth_message.encoding,
                bool(depth_message.is_bigendian),
            )
            calibration = CameraIntrinsics(
                self._camera_info.width,
                self._camera_info.height,
                float(self._camera_info.k[0]),
                float(self._camera_info.k[4]),
                float(self._camera_info.k[2]),
                float(self._camera_info.k[5]),
            )
            localized = localize_box(
                box,
                depth,
                calibration,
                self._depth_policy,
                self._last_range,
            )
            obstacle = protected_sector_distance(depth, self._depth_policy)
        except ValueError as error:
            self._publish_localization_error(str(error))
            return
        self._last_range = localized.horizontal_range_m
        generation, episode = identifier
        now = self._now()
        self._latest_observation = TargetObservation(
            source_time,
            now,
            localized.horizontal_range_m,
            localized.bearing_rad,
            obstacle,
            episode,
        )
        confidence = (
            float(detection.results[0].hypothesis.score)
            if detection.results
            else 0.0
        )
        self._pending_snapshot = (
            source_time,
            generation,
            episode,
            confidence,
            str(self.get_parameter('target_coat_color').value),
        )
        self._scan.stop()
        self._try_snapshot()

    def _tick(self) -> None:
        now = self._now()
        if (
            self._controller.state is FollowState.SEARCHING
            and self._scan.complete
        ):
            self._controller.fail('target_not_found_after_full_scan')
        depth_is_fresh = (
            self._latest_depth_received_at is not None
            and now - self._latest_depth_received_at
            <= self._controller.config.observation_timeout_sec
        )
        if (
            depth_is_fresh
            and self._latest_obstacle_distance is not None
            and self._latest_obstacle_distance
            < self._controller.config.emergency_stop_distance_m
        ):
            decision = self._controller.emergency_stop()
        else:
            decision = self._controller.update(
                now, self._latest_observation
            )
        self._publish_velocity(
            decision.command.linear_x,
            decision.command.angular_z,
        )
        self._publish_status(decision.reason)
        self._try_snapshot()
        if decision.state in TERMINAL_STATES and not self._terminal_written:
            self._write_result(decision.reason)
            self._terminal_written = True
        self._last_decision_state = decision.state

    def _start(self, request, response):
        del request
        if (
            self._controller.state is not FollowState.IDLE
            and self._controller.state not in TERMINAL_STATES
        ):
            response.success = False
            response.message = 'vision follow mission is already active'
            return response
        self._mission_id = str(self.get_parameter('mission_id').value)
        if not self._mission_id or '/' in self._mission_id or '..' in self._mission_id:
            response.success = False
            response.message = 'mission_id is not artifact-path safe'
            return response
        generation = int(self.get_parameter('mission_generation').value)
        if generation < 1 or generation <= self._last_started_generation:
            response.success = False
            response.message = (
                'detector must be freshly armed before starting this mission'
            )
            return response
        try:
            self._controller = FollowController(self._read_config())
        except ValueError as error:
            response.success = False
            response.message = f'unsafe follow configuration: {error}'
            return response
        self._controller.start(self._now())
        self._mission_generation = generation
        self._last_started_generation = generation
        self._scan.reset()
        self._latest_observation = None
        self._last_range = None
        self._pending_targets.clear()
        self._pending_snapshot = None
        self._last_sync_difference = None
        self._terminal_written = False
        write_json(
            self._artifact_root / self._mission_id / 'vision_plan.json',
            {
                'mission_id': self._mission_id,
                'mission_generation': self._mission_generation,
                'target_class': 'person',
                'target_coat_color': str(
                    self.get_parameter('target_coat_color').value
                ),
                'follow_config': asdict(self._controller.config),
                'started_at': datetime.now(timezone.utc).isoformat(),
            },
        )
        response.success = True
        response.message = f'vision mission {self._mission_id} started'
        return response

    def _cancel(self, request, response):
        del request
        self._controller.cancel()
        self._scan.stop()
        self._pending_targets.clear()
        self._pending_snapshot = None
        self._publish_velocity(0.0, 0.0)
        self._write_result('operator_cancelled')
        self._terminal_written = True
        response.success = True
        response.message = 'vision mission cancelled; zero velocity published'
        return response

    def _status(self, request, response):
        del request
        response.success = True
        response.message = json.dumps(self._status_payload(), sort_keys=True)
        return response

    def _try_snapshot(self) -> None:
        if self._pending_snapshot is None or not self._mission_id:
            return
        (
            source_time,
            generation,
            episode,
            confidence,
            coat_color,
        ) = self._pending_snapshot
        original_message = self._nearest(self._rgb_cache, source_time)
        annotated_message = self._nearest(
            self._annotated_cache, source_time
        )
        if original_message is None or annotated_message is None:
            return
        try:
            record = self._snapshot_writer.write_once(
                self._mission_id,
                generation,
                episode,
                'person',
                coat_color,
                confidence,
                source_time,
                image_to_bgr(original_message),
                image_to_bgr(annotated_message),
            )
        except (ValueError, OSError) as error:
            self.get_logger().error(f'snapshot failed: {error}')
            self._pending_snapshot = None
            return
        self._pending_snapshot = None
        if record is not None:
            event = String()
            event.data = json.dumps(asdict(record), sort_keys=True)
            self._event_publisher.publish(event)
            self.get_logger().info(
                f'Target acquired; operator snapshot: '
                f'{record.annotated_path}'
            )

    def _publish_velocity(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self._velocity_publisher.publish(command)

    def _publish_status(self, reason: str) -> None:
        message = String()
        payload = self._status_payload()
        payload['reason'] = reason
        message.data = json.dumps(payload, sort_keys=True)
        self._status_publisher.publish(message)

    def _publish_localization_error(self, reason: str) -> None:
        message = String()
        message.data = json.dumps(
            {'state': 'localization_unavailable', 'reason': reason},
            sort_keys=True,
        )
        self._status_publisher.publish(message)

    def _status_payload(self) -> dict:
        payload = {
            'mission_id': self._mission_id or None,
            'mission_generation': self._mission_generation or None,
            'target_class': 'person',
            'state': self._controller.state.value,
            'reason': self._controller.reason,
            'obstacle_distance_m': self._latest_obstacle_distance,
            'target_coat_color': str(
                self.get_parameter('target_coat_color').value
            ),
        }
        if self._latest_observation is not None:
            payload['observation'] = asdict(self._latest_observation)
        if self._last_sync_difference is not None:
            payload['rgb_depth_sync_difference_sec'] = (
                self._last_sync_difference
            )
        payload['search_angle_rad'] = self._scan.total_angle_rad
        return payload

    def _write_result(self, reason: str) -> None:
        if not self._mission_id:
            return
        payload = self._status_payload()
        payload['reason'] = reason
        payload['finished_at'] = datetime.now(timezone.utc).isoformat()
        write_json(
            self._artifact_root / self._mission_id / 'vision_result.json',
            payload,
        )

    def stop_motion(self) -> None:
        for _ in range(3):
            self._publish_velocity(0.0, 0.0)

    def destroy_node(self):
        self.stop_motion()
        return super().destroy_node()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _nearest(
        self,
        cache: deque[tuple[float, Image]],
        target: float,
    ) -> Image | None:
        entry = self._nearest_entry(cache, target)
        return None if entry is None else entry[1]

    def _nearest_entry(
        self,
        cache: deque[tuple[float, Image]],
        target: float,
    ) -> tuple[float, Image] | None:
        if not cache:
            return None
        timestamp, message = min(
            cache, key=lambda item: abs(item[0] - target)
        )
        if abs(timestamp - target) > self._sync_tolerance:
            return None
        return timestamp, message

    @staticmethod
    def _box_from_detection(detection) -> BoundingBox:
        width = max(1, round(detection.bbox.size_x))
        height = max(1, round(detection.bbox.size_y))
        return BoundingBox(
            max(0, round(detection.bbox.center.position.x - width / 2)),
            max(0, round(detection.bbox.center.position.y - height / 2)),
            width,
            height,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionFollower()
    try:
        rclpy.spin(node)
    finally:
        node.stop_motion()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
