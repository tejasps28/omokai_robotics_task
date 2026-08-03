"""Record evidence-backed metrics from a running vision-follow mission."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import cv2
from geometry_msgs.msg import PoseArray
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
from vision_msgs.msg import Detection2DArray

from omokai_perception.localization import timestamp_seconds
from omokai_perception.person_detector import image_to_bgr


def _yaw(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
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


def _span(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    maximum = 0.0
    for first_index, first in enumerate(points):
        for second in points[first_index + 1 :]:
            maximum = max(
                maximum,
                math.hypot(first[0] - second[0], first[1] - second[1]),
            )
    return maximum


class VisionLiveValidation(Node):
    """Collect mission, motion, actor-path, and safety telemetry."""

    def __init__(self) -> None:
        super().__init__('vision_live_validation')
        self.started_at = time.monotonic()
        self.follow_status: list[tuple[float, dict]] = []
        self.selection_status: list[dict] = []
        self.commands: list[tuple[float, float, float]] = []
        self.odometry: list[tuple[float, float, float, float]] = []
        self.actor_path_messages = 0
        self.operator_events: list[dict] = []
        self.detection_samples = 0
        self.nonempty_detection_samples = 0
        self.latest_rgb: Image | None = None
        self.latest_annotated: Image | None = None
        self.latest_detection_rgb: Image | None = None
        self.latest_detection_annotated: Image | None = None
        self._awaiting_detection_annotation = False
        self.rgb_contracts: set[tuple[int, int, str, str]] = set()
        self.depth_contracts: set[tuple[int, int, str, str]] = set()
        self.camera_contracts: set[tuple[int, int, str]] = set()
        self.rgb_timestamps: list[float] = []
        self.depth_timestamps: list[float] = []
        self.latest_odom: Odometry | None = None
        self.target_world_estimates: list[tuple[float, float]] = []
        self._estimated_observation_keys: set[tuple[float, int]] = set()

        self.create_subscription(
            String, '/vision/follow_status', self._on_follow_status, 10
        )
        self.create_subscription(
            String, '/vision/selection_status', self._on_selection_status, 10
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String,
            '/vision/operator_event',
            self._on_operator_event,
            event_qos,
        )
        self.create_subscription(
            Twist, '/robot1/cmd_vel', self._on_command, 10
        )
        self.create_subscription(
            Odometry, '/robot1/odom', self._on_odometry, 10
        )
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PoseArray,
            '/vision_target/cmd_path',
            self._on_actor_path,
            path_qos,
        )
        self.create_subscription(
            Detection2DArray,
            '/vision/detections',
            self._on_detections,
            10,
        )
        self.create_subscription(
            Image, '/robot1/camera/image', self._on_rgb, 10
        )
        self.create_subscription(
            Image,
            '/vision/detections/image',
            self._on_annotated,
            10,
        )
        self.create_subscription(
            Image, '/robot1/camera/depth_image', self._on_depth, 10
        )
        self.create_subscription(
            CameraInfo,
            '/robot1/camera/camera_info',
            self._on_camera_info,
            10,
        )

    @staticmethod
    def _decode(message: String) -> dict | None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return None
        return value if isinstance(value, dict) else None

    def _on_follow_status(self, message: String) -> None:
        payload = self._decode(message)
        if payload is None:
            return
        now = time.monotonic() - self.started_at
        self.follow_status.append((now, payload))
        observation = payload.get('observation')
        if not isinstance(observation, dict) or self.latest_odom is None:
            return
        range_m = observation.get('range_m')
        bearing_rad = observation.get('bearing_rad')
        if range_m is None or bearing_rad is None:
            return
        observation_key = (
            float(observation.get('source_timestamp', -1.0)),
            int(observation.get('episode', -1)),
        )
        if observation_key in self._estimated_observation_keys:
            return
        self._estimated_observation_keys.add(observation_key)
        robot = self.latest_odom.pose.pose.position
        target_yaw = _yaw(self.latest_odom) - float(bearing_rad)
        self.target_world_estimates.append(
            (
                robot.x + float(range_m) * math.cos(target_yaw),
                robot.y + float(range_m) * math.sin(target_yaw),
            )
        )

    def _on_selection_status(self, message: String) -> None:
        payload = self._decode(message)
        if payload is not None:
            self.selection_status.append(payload)

    def _on_operator_event(self, message: String) -> None:
        payload = self._decode(message)
        if payload is not None:
            self.operator_events.append(payload)

    def _on_command(self, message: Twist) -> None:
        self.commands.append(
            (
                time.monotonic() - self.started_at,
                float(message.linear.x),
                float(message.angular.z),
            )
        )

    def _on_odometry(self, message: Odometry) -> None:
        self.latest_odom = message
        position = message.pose.pose.position
        self.odometry.append(
            (
                time.monotonic() - self.started_at,
                float(position.x),
                float(position.y),
                _yaw(message),
            )
        )

    def _on_actor_path(self, _message: PoseArray) -> None:
        self.actor_path_messages += 1

    def _on_detections(self, message: Detection2DArray) -> None:
        self.detection_samples += 1
        if message.detections:
            self.nonempty_detection_samples += 1
            self.latest_detection_rgb = self.latest_rgb
            self._awaiting_detection_annotation = True

    def _on_rgb(self, message: Image) -> None:
        self.latest_rgb = message
        self.rgb_contracts.add(
            (
                int(message.width),
                int(message.height),
                str(message.encoding),
                str(message.header.frame_id),
            )
        )
        self.rgb_timestamps.append(timestamp_seconds(message.header.stamp))

    def _on_annotated(self, message: Image) -> None:
        self.latest_annotated = message
        if self._awaiting_detection_annotation:
            self.latest_detection_annotated = message
            self._awaiting_detection_annotation = False

    def _on_depth(self, message: Image) -> None:
        self.depth_contracts.add(
            (
                int(message.width),
                int(message.height),
                str(message.encoding),
                str(message.header.frame_id),
            )
        )
        self.depth_timestamps.append(
            timestamp_seconds(message.header.stamp)
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_contracts.add(
            (
                int(message.width),
                int(message.height),
                str(message.header.frame_id),
            )
        )

    def report(self, scenario: str, mission_id: str | None) -> dict:
        rgb_depth_differences: list[float] = []
        if self.rgb_timestamps and self.depth_timestamps:
            overlap_start = max(
                min(self.rgb_timestamps), min(self.depth_timestamps)
            )
            overlap_end = min(
                max(self.rgb_timestamps), max(self.depth_timestamps)
            )
            rgb_depth_differences = [
                min(abs(depth - rgb) for rgb in self.rgb_timestamps)
                for depth in self.depth_timestamps
                if overlap_start <= depth <= overlap_end
            ]
        statuses = [
            status
            for _, status in self.follow_status
            if mission_id is None or status.get('mission_id') == mission_id
        ]
        states = [str(status.get('state', '')) for status in statuses]
        reasons = [str(status.get('reason', '')) for status in statuses]
        observations = [
            status['observation']
            for status in statuses
            if isinstance(status.get('observation'), dict)
        ]
        obstacle_distances = [
            float(observation['obstacle_distance_m'])
            for observation in observations
            if observation.get('obstacle_distance_m') is not None
        ]
        obstacle_distances.extend(
            float(status['obstacle_distance_m'])
            for status in statuses
            if status.get('obstacle_distance_m') is not None
        )
        ranges = [
            float(observation['range_m'])
            for observation in observations
            if observation.get('range_m') is not None
        ]
        robot_points = [(sample[1], sample[2]) for sample in self.odometry]
        confirmed = sum(
            1
            for selection in self.selection_status
            if selection.get('confirmed') is True
        )
        accepted_max = max(
            (
                int(selection.get('accepted_count', 0))
                for selection in self.selection_status
            ),
            default=0,
        )
        rejected_max = max(
            (
                int(selection.get('rejected_count', 0))
                for selection in self.selection_status
            ),
            default=0,
        )
        emergency_times = [
            elapsed
            for elapsed, status in self.follow_status
            if status.get('reason') == 'emergency_obstacle_stop'
            and (
                mission_id is None
                or status.get('mission_id') == mission_id
            )
        ]
        commands_after_emergency = (
            [
                command
                for command in self.commands
                if command[0] >= emergency_times[0]
            ]
            if emergency_times
            else []
        )
        unique_events = {
            (
                event.get('mission_id'),
                event.get('metadata_path'),
                event.get('source_timestamp'),
            )
            for event in self.operator_events
            if mission_id is None or event.get('mission_id') == mission_id
        }

        report = {
            'scenario': scenario,
            'mission_id': mission_id,
            'duration_sec': time.monotonic() - self.started_at,
            'follow_status_samples': len(statuses),
            'states_observed': list(dict.fromkeys(states)),
            'reasons_observed': list(dict.fromkeys(reasons)),
            'confirmed_selection_samples': confirmed,
            'maximum_accepted_people': accepted_max,
            'maximum_rejected_people': rejected_max,
            'unique_operator_events': len(unique_events),
            'actor_path_messages': self.actor_path_messages,
            'detection_samples': self.detection_samples,
            'nonempty_detection_samples': self.nonempty_detection_samples,
            'rgb_contracts': sorted(self.rgb_contracts),
            'depth_contracts': sorted(self.depth_contracts),
            'camera_info_contracts': sorted(self.camera_contracts),
            'maximum_rgb_depth_difference_sec': max(
                rgb_depth_differences, default=None
            ),
            'robot_motion_span_m': _span(robot_points),
            'estimated_target_motion_span_m': _span(
                self.target_world_estimates
            ),
            'minimum_target_range_m': min(ranges) if ranges else None,
            'maximum_target_range_m': max(ranges) if ranges else None,
            'final_target_range_m': ranges[-1] if ranges else None,
            'minimum_obstacle_distance_m': (
                min(obstacle_distances) if obstacle_distances else None
            ),
            'maximum_linear_command_mps': max(
                (abs(command[1]) for command in self.commands), default=0.0
            ),
            'maximum_angular_command_rps': max(
                (abs(command[2]) for command in self.commands), default=0.0
            ),
            'commands_after_emergency': len(commands_after_emergency),
            'maximum_linear_after_emergency_mps': max(
                (abs(command[1]) for command in commands_after_emergency),
                default=0.0,
            ),
        }
        checks: dict[str, bool] = {
            'received_status': len(statuses) > 0,
            'linear_speed_bounded': (
                report['maximum_linear_command_mps'] <= 0.4001
            ),
            'angular_speed_bounded': (
                report['maximum_angular_command_rps'] <= 0.5001
            ),
            'one_rgb_contract': len(self.rgb_contracts) == 1,
            'one_depth_contract': len(self.depth_contracts) == 1,
            'one_camera_info_contract': len(self.camera_contracts) == 1,
            'rgb_depth_dimensions_aligned': (
                len(self.rgb_contracts) == 1
                and len(self.depth_contracts) == 1
                and next(iter(self.rgb_contracts))[:2]
                == next(iter(self.depth_contracts))[:2]
            ),
            'rgb_depth_optical_frame_aligned': (
                len(self.rgb_contracts) == 1
                and len(self.depth_contracts) == 1
                and next(iter(self.rgb_contracts))[3]
                == next(iter(self.depth_contracts))[3]
            ),
            'camera_info_matches_images': (
                len(self.camera_contracts) == 1
                and len(self.rgb_contracts) == 1
                and next(iter(self.camera_contracts))[:2]
                == next(iter(self.rgb_contracts))[:2]
                and next(iter(self.camera_contracts))[2]
                == next(iter(self.rgb_contracts))[3]
            ),
            'depth_encoding_supported': (
                len(self.depth_contracts) == 1
                and next(iter(self.depth_contracts))[2]
                in {'32FC1', '16UC1'}
            ),
            'rgb_depth_sync_bounded': (
                bool(rgb_depth_differences)
                and max(rgb_depth_differences) <= 0.0801
            ),
        }
        if scenario == 'moving':
            checks.update(
                {
                    'actor_path_commanded': self.actor_path_messages > 0,
                    'target_confirmed': confirmed >= 3,
                    'robot_moved': report['robot_motion_span_m'] >= 0.15,
                    'target_estimate_moved': (
                        report['estimated_target_motion_span_m'] >= 0.20
                    ),
                    'follow_state_observed': 'following' in states,
                }
            )
        elif scenario == 'stationary':
            checks.update(
                {
                    'target_confirmed': confirmed >= 3,
                    'one_acquisition_event': len(unique_events) == 1,
                    'accepted_target_observed': accepted_max >= 1,
                    'rejected_distractor_observed': rejected_max >= 1,
                    'follow_state_observed': 'following' in states,
                    'mission_remained_active': 'failed' not in states,
                    'robot_moved': report['robot_motion_span_m'] >= 0.15,
                    'standoff_reached': (
                        bool(ranges)
                        and any(1.05 <= value <= 1.35 for value in ranges)
                    ),
                    'stationary_target_estimate': (
                        # Pixel/depth noise is amplified while the robot
                        # turns; the actor itself has no animation or motion
                        # plugin in this scenario.
                        report['estimated_target_motion_span_m'] <= 0.80
                    ),
                }
            )
        elif scenario == 'precommand':
            checks.update(
                {
                    'at_least_three_inference_samples': (
                        self.detection_samples >= 3
                    ),
                    'target_outside_initial_camera_view': (
                        self.nonempty_detection_samples == 0
                    ),
                    'detector_not_armed': (
                        len(self.selection_status) >= 3
                        and all(
                            selection.get('armed') is False
                            for selection in self.selection_status
                        )
                    ),
                    'no_precommand_confirmation': all(
                        selection.get('confirmed') is not True
                        for selection in self.selection_status
                    ),
                    'robot_parked': (
                        all(
                            abs(linear) <= 1.0e-4
                            and abs(angular) <= 1.0e-4
                            for _, linear, angular in self.commands
                        )
                    ),
                }
            )
        elif scenario == 'obstacle':
            checks.update(
                {
                    'emergency_stop_observed': (
                        'emergency_obstacle_stop' in reasons
                    ),
                    'protected_distance_crossed': (
                        report['minimum_obstacle_distance_m'] is not None
                        and report['minimum_obstacle_distance_m'] < 0.55
                    ),
                    'forward_motion_zero_after_stop': (
                        bool(commands_after_emergency)
                        and report['maximum_linear_after_emergency_mps']
                        <= 1.0e-4
                    ),
                }
            )
        elif scenario == 'brief-loss':
            reacquiring_indexes = [
                index
                for index, state in enumerate(states)
                if state == 'reacquiring'
            ]
            checks.update(
                {
                    'reacquiring_observed': bool(reacquiring_indexes),
                    'following_resumed': (
                        bool(reacquiring_indexes)
                        and 'following'
                        in states[reacquiring_indexes[-1] + 1 :]
                    ),
                    'mission_not_failed': 'failed' not in states,
                }
            )
        elif scenario == 'prolonged-loss':
            checks.update(
                {
                    'reacquiring_observed': 'reacquiring' in states,
                    'target_lost_failure_observed': (
                        any(
                            state == 'failed' and reason == 'target_lost'
                            for state, reason in zip(states, reasons)
                        )
                    ),
                    'final_linear_command_zero': (
                        bool(self.commands)
                        and abs(self.commands[-1][1]) <= 1.0e-4
                    ),
                }
            )
        report['checks'] = checks
        report['passed'] = all(checks.values())
        return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog='vision_live_validation')
    parser.add_argument('--duration-sec', type=float, default=30.0)
    parser.add_argument('--mission-id')
    parser.add_argument(
        '--scenario',
        choices=(
            'generic',
            'precommand',
            'moving',
            'stationary',
            'obstacle',
            'brief-loss',
            'prolonged-loss',
        ),
        default='generic',
    )
    parser.add_argument('--output')
    parser.add_argument('--frame-output')
    parser.add_argument('--annotated-frame-output')
    parser.add_argument('--detection-frame-output')
    parser.add_argument('--detection-annotated-frame-output')
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = VisionLiveValidation()
    deadline = time.monotonic() + arguments.duration_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report(arguments.scenario, arguments.mission_id)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if arguments.output:
        output = Path(arguments.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + '\n', encoding='utf-8')
    if arguments.frame_output and node.latest_rgb is not None:
        frame_output = Path(arguments.frame_output)
        frame_output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(
            str(frame_output), image_to_bgr(node.latest_rgb)
        ):
            raise OSError(f'failed to write {frame_output}')
    for requested_path, message in (
        (arguments.annotated_frame_output, node.latest_annotated),
        (arguments.detection_frame_output, node.latest_detection_rgb),
        (
            arguments.detection_annotated_frame_output,
            node.latest_detection_annotated,
        ),
    ):
        if requested_path and message is not None:
            requested_output = Path(requested_path)
            requested_output.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(
                str(requested_output), image_to_bgr(message)
            ):
                raise OSError(f'failed to write {requested_output}')
    return 0 if report['passed'] else 1
