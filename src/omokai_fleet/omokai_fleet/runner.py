"""ROS execution boundary for an approved squad mission."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from omokai_fleet.mission import SquadMission, execute_squad_mission
from omokai_fleet.nav2_fleet_adapter import FleetNav2Adapter
from omokai_fleet.pose_tracker import FleetPoseTracker


CANCEL_SERVICE = '/omokai_fleet/cancel'


class SquadRunner:
    def __init__(self, node: Node) -> None:
        self._cancel_requested = False
        self._node = node
        self._tracker = FleetPoseTracker(node)
        self._cancel_service = node.create_service(
            Trigger,
            CANCEL_SERVICE,
            self._on_cancel,
        )

    def execute(self, mission: SquadMission):
        adapter = FleetNav2Adapter(
            self._node,
            safety_check=self._tracker.violation_reason,
            cancel_check=lambda: self._cancel_requested,
            pose_lookup=self._tracker.current_pose,
        )
        return execute_squad_mission(adapter, mission)

    def _on_cancel(self, request, response):
        del request
        self._cancel_requested = True
        response.success = True
        response.message = 'squad cancellation requested'
        return response


def run_live_mission(mission: SquadMission, artifact_dir: Path) -> int:
    rclpy.init()
    node = Node('omokai_squad_runner')
    try:
        lifecycle = SquadRunner(node).execute(mission)
        snapshot = lifecycle.snapshot()
        _write_json(
            artifact_dir / 'squad_result.json',
            {
                'plan_id': mission.plan.plan_id,
                'state': snapshot.state.value,
                'reason': snapshot.reason,
                'phases': [
                    {
                        'phase': record.phase.value,
                        'robots': [
                            {
                                'robot_id': result.robot_id.value,
                                'outcome': result.outcome.value,
                                'reason': result.reason,
                            }
                            for result in record.results
                        ],
                    }
                    for record in snapshot.history
                ],
            },
        )
        return 0 if snapshot.state.value == 'succeeded' else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def request_live_cancel(timeout_sec: float = 5.0) -> tuple[bool, str]:
    rclpy.init()
    node = Node('omokai_squad_cancel')
    try:
        client = node.create_client(Trigger, CANCEL_SERVICE)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, 'no active squad runner'
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=timeout_sec,
        )
        response = future.result()
        if response is None:
            return False, 'cancellation request timed out'
        return bool(response.success), str(response.message)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)
