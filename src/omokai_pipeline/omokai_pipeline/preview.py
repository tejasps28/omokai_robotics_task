"""Offline proof of the complete deterministic preparation/execution boundary."""

from typing import List, Tuple

from omokai_executor import ExecutorState, InMemoryEventSink, MissionExecutor
from omokai_interfaces import PlanRequest
from omokai_mission import FakePlanner, load_catalog

from .service import prepare_mission


DEMO_PROMPT = 'Patrol the inspection loop twice and return home.'


class _RecordingNavigation:
    def __init__(self) -> None:
        self.dispatched: List[str] = []

    def dispatch(self, goal, speed_mps: float) -> str:
        del speed_mps
        self.dispatched.append(goal.goal_id)
        return f'offline-goal-{len(self.dispatched):03d}'

    def cancel(self, goal_handle: str) -> None:
        del goal_handle


def run_preview(prompt: str = DEMO_PROMPT) -> Tuple[List[str], bool]:
    request = PlanRequest(request_id='offline-request', prompt=prompt)
    result = prepare_mission(
        request=request,
        mission_id='offline-mission',
        planner=FakePlanner(),
        catalog=load_catalog(),
    )
    lines = [
        '=== Omokai core pipeline preview ===',
        f'Prompt: {prompt}',
        f'Proposal: {result.proposal.content}',
        f'Validation: accepted={result.validation.accepted} '
        f'stage={result.validation.stage}',
    ]
    if not result.accepted:
        lines.extend(
            f'Rejection: {issue.code} {issue.path}'
            for issue in result.validation.errors
        )
        return lines, False

    assert result.preparation is not None
    navigation = _RecordingNavigation()
    events = InMemoryEventSink()
    executor = MissionExecutor(
        result.preparation.execution_plan.mission_id,
        navigation,
        events,
    )
    executor.begin_compilation()
    executor.load_plan(result.preparation.execution_plan)
    while not executor.state.terminal:
        handle = executor.active_goal_handle
        assert handle is not None
        executor.navigation_succeeded(handle)

    plan = result.preparation.execution_plan
    lines.extend(
        [
            f'Execution plan: goals={len(plan.goals)} speed={plan.speed_mps:.2f}m/s',
            f'Dispatched: {len(navigation.dispatched)}',
            f'Final state: {executor.state.value}',
            f'Audit events: {len(events.events)}',
            f'Final goal: {plan.goals[-1].goal_id} ({plan.goals[-1].kind.value})',
        ]
    )
    return lines, executor.state is ExecutorState.SUCCEEDED


def main() -> int:
    lines, succeeded = run_preview()
    print('\n'.join(lines))
    return 0 if succeeded else 1


if __name__ == '__main__':
    raise SystemExit(main())
