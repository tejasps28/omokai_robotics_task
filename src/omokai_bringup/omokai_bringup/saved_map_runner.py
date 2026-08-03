"""Navigate to named locations after restarting against a saved SLAM map."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from omokai_executor import (
    ArtifactKind,
    ExecutorConfig,
    JsonlAuditSink,
    MissionArtifactWriter,
    MissionExecutor,
)
from omokai_executor.nav2_adapter import Nav2NavigationAdapter

from .saved_map_plan import build_saved_map_plan


def _default_mission_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return f'saved-map-{timestamp}'


class _LoggingAuditSink:
    def __init__(self, node: Node, delegate: JsonlAuditSink) -> None:
        self._node = node
        self._delegate = delegate

    def record(self, event) -> None:
        self._delegate.record(event)
        if event.event_type == 'goal_dispatched':
            self._node.get_logger().info(
                f"Navigating to {event.details.get('goal_id')}"
            )
        elif event.event_type == 'goal_succeeded':
            self._node.get_logger().info(
                f"Reached {event.details.get('goal_id')}"
            )


class SavedMapVerificationRunner(Node):
    def __init__(
        self,
        *,
        mission_id: str,
        artifact_root: Path,
        speed_mps: float,
        goal_timeout_sec: float,
        max_retries: int,
        server_timeout_sec: float,
    ) -> None:
        super().__init__(
            'saved_map_verifier',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self._mission_id = mission_id
        self._plan = build_saved_map_plan(
            mission_id,
            speed_mps=speed_mps,
        )
        self._artifacts = MissionArtifactWriter(artifact_root)
        self._navigation = Nav2NavigationAdapter(self)
        self._executor = MissionExecutor(
            mission_id,
            self._navigation,
            _LoggingAuditSink(self, JsonlAuditSink(artifact_root)),
            config=ExecutorConfig(
                goal_timeout_sec=goal_timeout_sec,
                max_retries=max_retries,
            ),
        )
        self._navigation.bind_outcomes(self._executor)
        self._server_deadline = monotonic() + server_timeout_sec
        self._started = False
        self._done = False
        self._exit_code = 1
        self._finalized = False
        self._timer = self.create_timer(0.25, self._tick)
        names = ', '.join(goal.goal_id for goal in self._plan.goals)
        self.get_logger().info(
            f'Waiting for Nav2; verification locations: {names}'
        )

    @property
    def done(self) -> bool:
        return self._done

    @property
    def exit_code(self) -> int:
        return self._exit_code

    def request_cancel(self) -> bool:
        if self._done or self._executor.state.terminal:
            return False
        return self._executor.request_cancel()

    def _tick(self) -> None:
        if not self._started:
            if not self._navigation.server_is_ready():
                if monotonic() >= self._server_deadline:
                    self._finish_without_executor(
                        'failed',
                        'nav2_server_timeout',
                    )
                return
            self._started = True
            self._executor.begin_compilation()
            self._executor.load_plan(self._plan)

        if not self._executor.state.terminal:
            self._executor.tick()
        if self._executor.state.terminal:
            self._finalize()

    def _finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        state = self._executor.state.value
        completed = self._executor.current_goal_index
        self._write_result(state, None, completed)
        self._exit_code = 0 if state == 'succeeded' else 1
        self._done = True
        self.get_logger().info(
            f'Saved-map verification finished: state={state}, '
            f'completed_goals={completed}/{len(self._plan.goals)}'
        )

    def _finish_without_executor(self, state: str, reason: str) -> None:
        self._write_result(state, reason, 0)
        self._done = True

    def _write_result(
        self,
        state: str,
        reason: Optional[str],
        completed_goals: int,
    ) -> None:
        payload = {
            'mission_id': self._mission_id,
            'state': state,
            'completed_goals': completed_goals,
            'total_goals': len(self._plan.goals),
            'locations': [goal.goal_id for goal in self._plan.goals],
        }
        if reason is not None:
            payload['reason'] = reason
        self._artifacts.write_json(
            self._mission_id,
            ArtifactKind.RESULT,
            payload,
        )


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Verify localization by reaching two saved-map locations.'
    )
    parser.add_argument('--mission-id', default=_default_mission_id())
    parser.add_argument(
        '--artifact-root',
        type=Path,
        default=Path('/data/artifacts'),
    )
    parser.add_argument('--speed-mps', type=float, default=0.15)
    parser.add_argument('--goal-timeout-sec', type=float, default=180.0)
    parser.add_argument('--max-retries', type=int, default=1)
    parser.add_argument('--server-timeout-sec', type=float, default=30.0)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    options = _parse_args(argv)
    rclpy.init()
    node = SavedMapVerificationRunner(
        mission_id=options.mission_id,
        artifact_root=options.artifact_root,
        speed_mps=options.speed_mps,
        goal_timeout_sec=options.goal_timeout_sec,
        max_retries=options.max_retries,
        server_timeout_sec=options.server_timeout_sec,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted by operator')
        node.request_cancel()
        deadline = monotonic() + 10.0
        while rclpy.ok() and not node.done and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
