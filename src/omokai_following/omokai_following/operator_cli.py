"""Operator preview, run, status, and cancel commands for vision missions."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from omokai_perception.mission import parse_arm_response


MISSION_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')
EMERGENCY_STOP_DISTANCE_M = 0.55
MAXIMUM_ALLOWED_LINEAR_MPS = 0.40


def parameter_update_succeeded(response) -> bool:
    """Accept Jazzy SetParameters.Response and older iterable result shapes."""
    if response is None:
        return False
    results = getattr(response, 'results', response)
    try:
        return all(result.successful for result in results)
    except TypeError:
        return False


def mission_payload(args) -> dict:
    if args.target_class != 'person':
        raise ValueError('the current detector supports target class person')
    if not MISSION_ID_PATTERN.fullmatch(args.mission_id) or '..' in args.mission_id:
        raise ValueError('mission ID contains unsupported characters')
    if args.standoff <= 0 or args.max_speed <= 0:
        raise ValueError('stand-off and maximum speed must be positive')
    if args.standoff <= EMERGENCY_STOP_DISTANCE_M:
        raise ValueError('stand-off must exceed the emergency-stop distance')
    if args.max_speed > MAXIMUM_ALLOWED_LINEAR_MPS:
        raise ValueError('maximum speed exceeds the 0.40 m/s safety cap')
    return {
        'mission_id': args.mission_id,
        'target_class': args.target_class,
        'target_coat_color': args.coat_color,
        'desired_standoff_m': args.standoff,
        'maximum_linear_mps': args.max_speed,
    }


class VisionOperator(Node):
    def __init__(self) -> None:
        super().__init__('vision_operator')
        self._operator_event: dict | None = None
        self._follow_status: dict = {}
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, '/vision/operator_event', self._on_event, event_qos
        )
        self.create_subscription(
            String, '/vision/follow_status', self._on_status, 10
        )

    def configure_and_start(self, payload: dict) -> tuple[bool, str]:
        detector = AsyncParameterClient(self, '/person_detector')
        follower = AsyncParameterClient(self, '/vision_follower')
        for client in (detector, follower):
            if not client.wait_for_services(timeout_sec=5.0):
                return False, 'vision parameter service is unavailable'
        detector_future = detector.set_parameters(
            [
                Parameter(
                    'target_coat_color',
                    Parameter.Type.STRING,
                    payload['target_coat_color'],
                )
            ]
        )
        follower_future = follower.set_parameters(
            [
                Parameter(
                    'mission_id',
                    Parameter.Type.STRING,
                    payload['mission_id'],
                ),
                Parameter(
                    'target_coat_color',
                    Parameter.Type.STRING,
                    payload['target_coat_color'],
                ),
                Parameter(
                    'desired_standoff_m',
                    Parameter.Type.DOUBLE,
                    payload['desired_standoff_m'],
                ),
                Parameter(
                    'maximum_linear_mps',
                    Parameter.Type.DOUBLE,
                    payload['maximum_linear_mps'],
                ),
            ]
        )
        for future in (detector_future, follower_future):
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result() if future.done() else None
            if not parameter_update_succeeded(response):
                return False, 'vision parameters were rejected'
        armed, arm_message = self.call_trigger('/vision/arm_detector')
        if not armed:
            return False, f'detector arm failed: {arm_message}'
        try:
            arm = parse_arm_response(arm_message)
        except (ValueError, json.JSONDecodeError) as error:
            self.call_trigger('/vision/disarm_detector')
            return False, f'invalid detector arm response: {error}'
        if arm['target_coat_color'] != payload['target_coat_color']:
            self.call_trigger('/vision/disarm_detector')
            return False, 'detector armed with the wrong target descriptor'
        generation_future = follower.set_parameters(
            [
                Parameter(
                    'mission_generation',
                    Parameter.Type.INTEGER,
                    arm['mission_generation'],
                )
            ]
        )
        rclpy.spin_until_future_complete(
            self, generation_future, timeout_sec=5.0
        )
        generation_response = (
            generation_future.result() if generation_future.done() else None
        )
        if not parameter_update_succeeded(generation_response):
            self.call_trigger('/vision/disarm_detector')
            return False, 'mission generation was rejected'
        started, message = self.call_trigger('/vision/start')
        if not started:
            self.call_trigger('/vision/disarm_detector')
            return False, message
        activated, activation_message = self.call_trigger(
            '/vision/activate_detector'
        )
        if not activated:
            self.call_trigger('/vision/cancel')
            self.call_trigger('/vision/disarm_detector')
            return False, f'detector activation failed: {activation_message}'
        return True, message

    def call_trigger(self, service_name: str) -> tuple[bool, str]:
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            return False, f'{service_name} is unavailable'
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return False, f'{service_name} timed out'
        result = future.result()
        return bool(result.success), str(result.message)

    def wait_for_acquisition(
        self, mission_id: str, timeout_sec: float
    ) -> tuple[bool, dict | None, str]:
        """Stay attached until a mission event, terminal state, or timeout."""

        deadline = time.monotonic() + timeout_sec
        terminal = {'failed', 'cancelled', 'succeeded'}
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.20)
            if (
                self._operator_event is not None
                and self._operator_event.get('mission_id') == mission_id
            ):
                return True, self._operator_event, 'target acquired'
            if (
                self._follow_status.get('mission_id') == mission_id
                and self._follow_status.get('state') in terminal
            ):
                reason = str(
                    self._follow_status.get('reason', 'terminal state')
                )
                return False, None, reason
        return False, None, 'timed out waiting for target acquisition'

    def _on_event(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self._operator_event = payload

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self._follow_status = payload


def acquisition_notification(event: dict) -> str:
    """Render the single examiner-facing acquisition notification."""

    return (
        'TARGET ACQUIRED\n'
        f"  mission: {event.get('mission_id')} "
        f"(generation {event.get('mission_generation')})\n"
        f"  target: {event.get('target_class')}/"
        f"{event.get('coat_color')} coat\n"
        f"  confidence: {float(event.get('confidence', 0.0)):.3f}\n"
        f"  source timestamp: {event.get('source_timestamp')}\n"
        f"  original: {event.get('original_path')}\n"
        f"  annotated: {event.get('annotated_path')}"
    )


def command_preview(args) -> int:
    try:
        payload = mission_payload(args)
    except ValueError as error:
        print(f'Invalid vision mission: {error}', file=sys.stderr)
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


def command_run(args) -> int:
    try:
        payload = mission_payload(args)
    except ValueError as error:
        print(f'Invalid vision mission: {error}', file=sys.stderr)
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True))
    if not args.yes and not _confirm('Approve vision follow mission? [y/N] '):
        print('Approval declined; robot remains stopped.', file=sys.stderr)
        return 3
    rclpy.init()
    node = VisionOperator()
    try:
        success, message = node.configure_and_start(payload)
        print(message)
        if success and not args.detach:
            acquired, event, wait_message = node.wait_for_acquisition(
                payload['mission_id'], args.wait_timeout
            )
            if acquired and event is not None:
                print(acquisition_notification(event))
            else:
                print(
                    f'Acquisition wait ended: {wait_message}',
                    file=sys.stderr,
                )
                success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 4


def command_service(service_name: str) -> int:
    rclpy.init()
    node = VisionOperator()
    try:
        success, message = node.call_trigger(service_name)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if service_name == '/vision/status':
        try:
            print(json.dumps(json.loads(message), indent=2, sort_keys=True))
        except json.JSONDecodeError:
            print(message)
    else:
        print(message)
    return 0 if success else 4


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='omokai-vision')
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('preview', 'run'):
        command = commands.add_parser(name)
        command.add_argument('--target-class', default='person')
        command.add_argument(
            '--coat-color', choices=('any', 'white', 'red'), default='white'
        )
        command.add_argument('--standoff', type=float, default=1.2)
        command.add_argument('--max-speed', type=float, default=0.38)
        command.add_argument('--mission-id', default=_default_mission_id())
        if name == 'run':
            command.add_argument('--yes', action='store_true')
            command.add_argument('--detach', action='store_true')
            command.add_argument('--wait-timeout', type=float, default=45.0)
    commands.add_parser('status')
    commands.add_parser('cancel')
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == 'preview':
        return command_preview(args)
    if args.command == 'run':
        return command_run(args)
    if args.command == 'status':
        return command_service('/vision/status')
    return command_service('/vision/cancel')


def _default_mission_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return f'vision-{timestamp}'


def _confirm(text: str) -> bool:
    try:
        return input(text).strip().lower() in {'y', 'yes'}
    except EOFError:
        return False
