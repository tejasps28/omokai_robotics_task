"""Run the validated Task 1 mission through the live Nav2 action server."""

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
from omokai_interfaces import MissionProposal, PlanRequest
from omokai_mission import (
    FakePlanner,
    GeminiPlanner,
    GeminiPlannerError,
    OpenAIPlanner,
    OpenAIPlannerError,
    Planner,
    PlannerError,
    load_catalog,
)
from omokai_pipeline import prepare_mission, prepare_proposal


DEFAULT_PROMPT = 'Patrol the inspection loop twice and return home.'
PLANNER_CHOICES = ('fake', 'openai', 'gemini')


def _default_mission_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return f'task1-{timestamp}'


def _create_planner(name: str) -> Planner:
    if name == 'fake':
        return FakePlanner()
    if name == 'openai':
        return OpenAIPlanner()
    if name == 'gemini':
        return GeminiPlanner()
    raise ValueError(f'unsupported planner {name!r}')


class _LoggingAuditSink:
    """Persist every event and print the execution milestones."""

    def __init__(self, node: Node, delegate: JsonlAuditSink) -> None:
        self._node = node
        self._delegate = delegate

    def record(self, event) -> None:
        self._delegate.record(event)
        if event.event_type == 'goal_dispatched':
            self._node.get_logger().info(
                'Dispatched goal %s (index=%s, handle=%s)'
                % (
                    event.details.get('goal_id'),
                    event.details.get('goal_index'),
                    event.details.get('goal_handle'),
                )
            )
        elif event.event_type == 'state_transition':
            self._node.get_logger().info(
                'Mission state: %s -> %s'
                % (
                    event.details.get('from_state'),
                    event.details.get('to_state'),
                )
            )


class Task1MissionRunner(Node):
    """Compose the deterministic mission pipeline with the Nav2 adapter."""

    def __init__(
        self,
        *,
        prompt: str,
        mission_id: str,
        artifact_root: Path,
        goal_timeout_sec: float,
        max_retries: int,
        server_timeout_sec: float,
        cancel_after_sec: Optional[float],
        planner_name: str,
        approved_proposal_json: Optional[str],
        approved_proposal_provider: Optional[str],
    ) -> None:
        super().__init__(
            'task1_mission_runner',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self._mission_id = mission_id
        self._artifact_writer = MissionArtifactWriter(artifact_root)
        self._done = False
        self._exit_code = 1
        self._finalized = False
        self._server_deadline = monotonic() + server_timeout_sec
        self._cancel_after_sec = cancel_after_sec
        self._execution_started_at: Optional[float] = None
        self._cancel_requested_by_timer = False

        request = PlanRequest(request_id=f'{mission_id}-request', prompt=prompt)
        try:
            catalog = load_catalog()
            if approved_proposal_json is None:
                result = prepare_mission(
                    request=request,
                    mission_id=mission_id,
                    planner=_create_planner(planner_name),
                    catalog=catalog,
                    artifact_writer=self._artifact_writer,
                )
                provider_label = planner_name
            else:
                assert approved_proposal_provider is not None
                proposal = MissionProposal(
                    request_id=request.request_id,
                    provider=approved_proposal_provider,
                    content=approved_proposal_json,
                )
                result = prepare_proposal(
                    request=request,
                    mission_id=mission_id,
                    proposal=proposal,
                    catalog=catalog,
                    artifact_writer=self._artifact_writer,
                )
                provider_label = f'approved:{approved_proposal_provider}'
        except (
            PlannerError,
            OpenAIPlannerError,
            GeminiPlannerError,
            ValueError,
        ) as error:
            self.get_logger().error(f'Mission planning failed: {error}')
            self._write_result('failed', 'planner_error', 0, 0)
            self._done = True
            return
        if not result.accepted:
            self.get_logger().error('Mission proposal was rejected')
            self._write_result('rejected', 'validation_rejected', 0, 0)
            self._done = True
            return

        assert result.preparation is not None
        self._plan = result.preparation.execution_plan
        self._navigation = Nav2NavigationAdapter(self)
        event_sink = _LoggingAuditSink(self, JsonlAuditSink(artifact_root))
        self._executor = MissionExecutor(
            mission_id,
            self._navigation,
            event_sink,
            config=ExecutorConfig(
                goal_timeout_sec=goal_timeout_sec,
                max_retries=max_retries,
            ),
        )
        self._navigation.bind_outcomes(self._executor)
        self._started = False
        self._timer = self.create_timer(0.25, self._tick)
        self.get_logger().info(
            f'Prepared mission {mission_id} with provider={provider_label}: '
            f'{len(self._plan.goals)} goals at {self._plan.speed_mps:.2f} m/s; '
            f'waiting for Nav2'
        )

    @property
    def done(self) -> bool:
        return self._done

    @property
    def exit_code(self) -> int:
        return self._exit_code

    def request_cancel(self) -> bool:
        """Request deterministic cancellation during operator interruption."""

        if self._done or self._executor.state.terminal:
            return False
        accepted = self._executor.request_cancel()
        if self._executor.state.terminal:
            self._finalize()
        return accepted

    def _tick(self) -> None:
        if not self._started:
            if not self._navigation.server_is_ready():
                if monotonic() >= self._server_deadline:
                    self.get_logger().error('Timed out waiting for Nav2 action server')
                    self._write_result(
                        'failed', 'nav2_server_timeout', 0, len(self._plan.goals)
                    )
                    self._done = True
                return

            self._started = True
            self._execution_started_at = monotonic()
            self.get_logger().info('Nav2 is ready; starting deterministic execution')
            self._executor.begin_compilation()
            self._executor.load_plan(self._plan)

        if (
            self._cancel_after_sec is not None
            and not self._cancel_requested_by_timer
            and self._execution_started_at is not None
            and monotonic() - self._execution_started_at >= self._cancel_after_sec
        ):
            self._cancel_requested_by_timer = True
            self.get_logger().warning(
                f'Requesting deterministic cancellation after '
                f'{self._cancel_after_sec:.2f}s'
            )
            self.request_cancel()

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
        self._write_result(state, None, completed, len(self._plan.goals))
        self._exit_code = 0 if state == 'succeeded' else 1
        self._done = True
        self.get_logger().info(
            f'Mission {self._mission_id} finished: state={state}, '
            f'completed_goals={completed}/{len(self._plan.goals)}'
        )

    def _write_result(
        self,
        state: str,
        reason: Optional[str],
        completed_goals: int,
        total_goals: int,
    ) -> None:
        payload = {
            'mission_id': self._mission_id,
            'state': state,
            'completed_goals': completed_goals,
            'total_goals': total_goals,
        }
        if reason is not None:
            payload['reason'] = reason
        self._artifact_writer.write_json(
            self._mission_id,
            ArtifactKind.RESULT,
            payload,
        )


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run the Task 1 fake-planner mission through live Nav2.'
    )
    parser.add_argument('--prompt', default=DEFAULT_PROMPT)
    parser.add_argument('--planner', choices=PLANNER_CHOICES, default='fake')
    parser.add_argument(
        '--approved-proposal-json',
        default=None,
        help='Exact proposal JSON already reviewed at the operator approval gate.',
    )
    parser.add_argument(
        '--approved-proposal-provider',
        default=None,
        help='Provider label associated with --approved-proposal-json.',
    )
    parser.add_argument('--mission-id', default=_default_mission_id())
    parser.add_argument('--artifact-root', type=Path, default=Path('/data/artifacts'))
    parser.add_argument('--goal-timeout-sec', type=float, default=180.0)
    parser.add_argument('--max-retries', type=int, default=1)
    parser.add_argument('--server-timeout-sec', type=float, default=30.0)
    parser.add_argument(
        '--cancel-after-sec',
        type=float,
        default=None,
        help='Testing hook: request deterministic cancellation after execution starts.',
    )
    options = parser.parse_args(argv)
    approved_fields = (
        options.approved_proposal_json,
        options.approved_proposal_provider,
    )
    if (approved_fields[0] is None) != (approved_fields[1] is None):
        parser.error(
            '--approved-proposal-json and --approved-proposal-provider '
            'must be supplied together'
        )
    return options


def main(argv: Optional[Sequence[str]] = None) -> int:
    options = _parse_args(argv)
    rclpy.init()
    node = Task1MissionRunner(
        prompt=options.prompt,
        mission_id=options.mission_id,
        artifact_root=options.artifact_root,
        goal_timeout_sec=options.goal_timeout_sec,
        max_retries=options.max_retries,
        server_timeout_sec=options.server_timeout_sec,
        cancel_after_sec=options.cancel_after_sec,
        planner_name=options.planner,
        approved_proposal_json=options.approved_proposal_json,
        approved_proposal_provider=options.approved_proposal_provider,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted by operator')
        node.request_cancel()
        cancel_deadline = monotonic() + 10.0
        while rclpy.ok() and not node.done and monotonic() < cancel_deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
