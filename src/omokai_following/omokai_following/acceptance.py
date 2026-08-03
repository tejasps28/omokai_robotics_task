"""Headless acceptance observer for a separately started vision mission."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


class AcceptanceObserver(Node):
    def __init__(self, mission_id: str | None = None) -> None:
        super().__init__('vision_acceptance')
        self.mission_id = mission_id
        self.latest_status: dict = {}
        self.operator_events = 0
        self.create_subscription(
            String, '/vision/follow_status', self._on_status, 10
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, '/vision/operator_event', self._on_event, event_qos
        )

    def _on_status(self, message: String) -> None:
        try:
            self.latest_status = json.loads(message.data)
        except json.JSONDecodeError:
            self.latest_status = {'state': 'malformed_status'}

    def _on_event(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if (
            self.mission_id is not None
            and event.get('mission_id') != self.mission_id
        ):
            return
        self.operator_events += 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog='vision_acceptance')
    parser.add_argument('--timeout-sec', type=float, default=60.0)
    parser.add_argument('--mission-id')
    parser.add_argument(
        '--required-state',
        choices=('following', 'reacquiring', 'failed', 'cancelled'),
        default='following',
    )
    args = parser.parse_args(argv)
    rclpy.init()
    observer = AcceptanceObserver(args.mission_id)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.2)
            if (
                observer.latest_status.get('state') == args.required_state
                and observer.operator_events == 1
                and (
                    args.mission_id is None
                    or observer.latest_status.get('mission_id')
                    == args.mission_id
                )
            ):
                print(
                    json.dumps(
                        {
                            'accepted': True,
                            'operator_events': observer.operator_events,
                            'status': observer.latest_status,
                        },
                        indent=2,
                        sort_keys=True,
                    )
                )
                return 0
        print(
            json.dumps(
                {
                    'accepted': False,
                    'operator_events': observer.operator_events,
                    'status': observer.latest_status,
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 1
    finally:
        observer.destroy_node()
        rclpy.shutdown()
