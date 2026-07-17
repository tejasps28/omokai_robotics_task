"""Operator command-line interface for the Task 1 mission pipeline.

This is the human-facing entry point that sits on top of the existing
application boundary. It intentionally reuses the shared preparation service
(:func:`omokai_pipeline.prepare_mission`) for planning, validation, and
compilation, and delegates live execution to the existing mission runner so
that no core validation or execution logic is duplicated here.

Commands
--------
``preview``  Plan a prompt offline and show the proposal, validation outcome,
             and compiled route summary. Never touches the robot.
``run``      Plan a prompt, present an approval gate, then delegate to the
             existing Task 1 runner (live Nav2 execution).
``status``   Read the latest persisted mission artifacts and print a concise
             status summary.
``cancel``   Placeholder: live cancellation is only available while the runner
             is attached to Nav2, so this exits with a clear message in the
             offline CLI.

The ``preview`` and ``status`` commands are pure Python and require neither ROS
nor a running simulator, which keeps them unit-testable on the host.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, List, Optional, Sequence

from omokai_interfaces import PlanRequest
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

from .service import PreparationResult, prepare_mission


DEFAULT_PROMPT = 'Patrol the inspection loop twice and return home.'
DEFAULT_ARTIFACT_ROOT = 'runtime/artifacts'
RUNNER_PACKAGE = 'omokai_bringup'
RUNNER_EXECUTABLE = 'task1_mission_runner'

# Exit codes shared by the commands so tooling can react deterministically.
EXIT_OK = 0
EXIT_REJECTED = 1
EXIT_DECLINED = 3
EXIT_UNAVAILABLE = 4
PLANNER_CHOICES = ('fake', 'openai', 'gemini')


def _default_artifact_root() -> Path:
    return Path(os.environ.get('OMOKAI_ARTIFACT_ROOT', DEFAULT_ARTIFACT_ROOT))


def _default_mission_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return f'task1-{timestamp}'


def _decode_proposal_content(content: str) -> object:
    """Return parsed proposal JSON when possible, otherwise the raw text.

    The planner boundary deliberately keeps untrusted proposal text verbatim,
    so malformed JSON is preserved for the operator instead of raising here.
    """

    try:
        return json.loads(content)
    except (json.JSONDecodeError, TypeError):
        return content


# ---------------------------------------------------------------------------
# preview
# ---------------------------------------------------------------------------


def create_planner(name: str) -> Planner:
    """Create a planner selected by the operator."""

    if name == 'fake':
        return FakePlanner()
    if name == 'openai':
        return OpenAIPlanner()
    if name == 'gemini':
        return GeminiPlanner()
    raise ValueError(f'unsupported planner {name!r}')


def build_preview(
    prompt: str,
    mission_id: str = 'preview',
    *,
    planner_name: str = 'fake',
) -> PreparationResult:
    """Run the shared preparation service offline (no artifacts written)."""

    if not prompt.strip():
        raise PlannerError('prompt must not be blank')
    request = PlanRequest(request_id=f'{mission_id}-request', prompt=prompt)
    return prepare_mission(
        request=request,
        mission_id=mission_id,
        planner=create_planner(planner_name),
        catalog=load_catalog(),
    )


def preview_as_dict(prompt: str, result: PreparationResult) -> dict:
    """Machine-readable rendering of a preview result."""

    payload: dict = {
        'prompt': prompt,
        'accepted': result.accepted,
        'proposal': {
            'provider': result.proposal.provider,
            'content': _decode_proposal_content(result.proposal.content),
        },
        'validation': {
            'accepted': result.validation.accepted,
            'stage': result.validation.stage,
            'policy_version': result.validation.policy_version,
            'errors': result.validation.audit_records(),
        },
        'route': None,
    }
    if result.preparation is not None:
        compiled = result.preparation.compiled
        plan = result.preparation.execution_plan
        payload['route'] = {
            'route_id': compiled.route_id,
            'frame_id': compiled.frame_id,
            'goal_count': len(plan.goals),
            'speed_mps': plan.speed_mps,
            'goals': compiled.summary_lines(),
        }
    return payload


def format_preview_lines(prompt: str, result: PreparationResult) -> List[str]:
    """Human-readable rendering of a preview result."""

    proposal_json = json.dumps(
        _decode_proposal_content(result.proposal.content),
        indent=2,
        sort_keys=True,
    )
    lines = [
        '=== Mission preview ===',
        f'Prompt: {prompt}',
        f'Provider: {result.proposal.provider}',
        'Proposal JSON:',
    ]
    lines.extend(f'  {line}' for line in proposal_json.splitlines())
    lines.append(
        f'Validation: accepted={result.validation.accepted} '
        f'stage={result.validation.stage} '
        f'policy={result.validation.policy_version}'
    )
    if not result.accepted:
        lines.append('Rejected. Issues:')
        for issue in result.validation.errors:
            lines.append(f'  [{issue.stage}] {issue.code} {issue.path}: {issue.message}')
        return lines

    assert result.preparation is not None
    compiled = result.preparation.compiled
    plan = result.preparation.execution_plan
    lines.append(
        f'Compiled route: {compiled.route_id} '
        f'({len(plan.goals)} goals @ {plan.speed_mps:.2f} m/s, frame={compiled.frame_id})'
    )
    lines.extend(f'  {line}' for line in compiled.summary_lines())
    return lines


def cmd_preview(args: argparse.Namespace) -> int:
    try:
        result = build_preview(args.prompt, planner_name=args.planner)
    except (PlannerError, OpenAIPlannerError, GeminiPlannerError) as error:
        print(f'Planner could not interpret the prompt: {error}', file=sys.stderr)
        return EXIT_REJECTED
    if args.json:
        print(json.dumps(preview_as_dict(args.prompt, result), indent=2, sort_keys=True))
    else:
        print('\n'.join(format_preview_lines(args.prompt, result)))
    return EXIT_OK if result.accepted else EXIT_REJECTED


# ---------------------------------------------------------------------------
# run
# ---------------------------------------------------------------------------


def _confirm(prompt_text: str) -> bool:
    try:
        answer = input(prompt_text)
    except EOFError:
        return False
    return answer.strip().lower() in {'y', 'yes'}


def _default_runner(command: Sequence[str]) -> int:
    """Invoke the existing Task 1 runner via ``ros2 run``."""

    completed = subprocess.run(list(command))
    return completed.returncode


def cmd_run(
    args: argparse.Namespace,
    runner: Callable[[Sequence[str]], int] = _default_runner,
) -> int:
    try:
        result = build_preview(
            args.prompt,
            mission_id=args.mission_id,
            planner_name=args.planner,
        )
    except (PlannerError, OpenAIPlannerError, GeminiPlannerError) as error:
        print(f'Planner could not interpret the prompt: {error}', file=sys.stderr)
        return EXIT_REJECTED
    print('\n'.join(format_preview_lines(args.prompt, result)))

    if not result.accepted:
        print('\nMission was rejected by validation; refusing to execute.', file=sys.stderr)
        return EXIT_REJECTED

    command = [
        'ros2', 'run', RUNNER_PACKAGE, RUNNER_EXECUTABLE,
        '--prompt', args.prompt,
        '--mission-id', args.mission_id,
        '--artifact-root', str(args.artifact_root),
        '--goal-timeout-sec', str(args.goal_timeout_sec),
        '--max-retries', str(args.max_retries),
        '--server-timeout-sec', str(args.server_timeout_sec),
        '--approved-proposal-json', result.proposal.content,
        '--approved-proposal-provider', result.proposal.provider,
    ]

    if not args.yes:
        if not _confirm('\nApprove and execute this mission on the robot? [y/N] '):
            print('Approval declined; mission not started.', file=sys.stderr)
            return EXIT_DECLINED

    if args.dry_run:
        print('\nDry run; would execute:')
        print('  ' + ' '.join(command))
        return EXIT_OK

    print(f'\nDelegating to {RUNNER_PACKAGE} {RUNNER_EXECUTABLE} ...')
    return runner(command)


# ---------------------------------------------------------------------------
# status
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MissionStatus:
    mission_id: str
    state: str
    completed_goals: Optional[int]
    total_goals: Optional[int]
    reason: Optional[str]
    artifacts: List[str]

    def summary_lines(self) -> List[str]:
        lines = [
            f'Mission: {self.mission_id}',
            f'State: {self.state}',
        ]
        if self.total_goals is not None:
            completed = self.completed_goals if self.completed_goals is not None else '?'
            lines.append(f'Goals: {completed}/{self.total_goals}')
        if self.reason:
            lines.append(f'Reason: {self.reason}')
        lines.append(f'Artifacts: {", ".join(self.artifacts) if self.artifacts else "none"}')
        return lines


def latest_mission_id(artifact_root: Path) -> Optional[str]:
    """Return the most recently modified mission directory name, if any."""

    if not artifact_root.is_dir():
        return None
    mission_dirs = [child for child in artifact_root.iterdir() if child.is_dir()]
    if not mission_dirs:
        return None
    newest = max(mission_dirs, key=lambda path: path.stat().st_mtime)
    return newest.name


def load_mission_status(artifact_root: Path, mission_id: str) -> MissionStatus:
    """Assemble a concise status from whatever evidence exists on disk."""

    mission_dir = artifact_root / mission_id
    artifacts = sorted(
        child.name for child in mission_dir.glob('*') if child.is_file()
    ) if mission_dir.is_dir() else []

    result_path = mission_dir / 'result.json'
    if result_path.is_file():
        payload = json.loads(result_path.read_text(encoding='utf-8'))
        return MissionStatus(
            mission_id=payload.get('mission_id', mission_id),
            state=payload.get('state', 'unknown'),
            completed_goals=payload.get('completed_goals'),
            total_goals=payload.get('total_goals'),
            reason=payload.get('reason'),
            artifacts=artifacts,
        )

    # No terminal result yet; infer a coarse state from partial evidence.
    if (mission_dir / 'accepted_mission.json').is_file():
        state = 'accepted (no result yet)'
    elif (mission_dir / 'validation.json').is_file():
        state = 'validated (no result yet)'
    else:
        state = 'unknown'
    return MissionStatus(
        mission_id=mission_id,
        state=state,
        completed_goals=None,
        total_goals=None,
        reason=None,
        artifacts=artifacts,
    )


def cmd_status(args: argparse.Namespace) -> int:
    artifact_root = args.artifact_root
    mission_id = args.mission_id or latest_mission_id(artifact_root)
    if mission_id is None:
        print(f'No mission artifacts found under {artifact_root}', file=sys.stderr)
        return EXIT_REJECTED

    status = load_mission_status(artifact_root, mission_id)
    if args.json:
        print(json.dumps(
            {
                'mission_id': status.mission_id,
                'state': status.state,
                'completed_goals': status.completed_goals,
                'total_goals': status.total_goals,
                'reason': status.reason,
                'artifacts': status.artifacts,
            },
            indent=2,
            sort_keys=True,
        ))
    else:
        print('\n'.join(status.summary_lines()))
    return EXIT_OK


# ---------------------------------------------------------------------------
# cancel
# ---------------------------------------------------------------------------


def cmd_cancel(args: argparse.Namespace) -> int:
    del args
    print(
        'Cancellation is not available in offline CLI mode.\n'
        'The deterministic executor supports cancellation only while the mission '
        'runner is attached to Nav2. Interrupt the active `run` process (Ctrl-C) '
        'to trigger a deterministic, audited cancellation.',
        file=sys.stderr,
    )
    return EXIT_UNAVAILABLE


# ---------------------------------------------------------------------------
# parser / entry point
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='omokai-operator',
        description='Operator interface for the Task 1 mission pipeline.',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    preview = subparsers.add_parser(
        'preview', help='Plan a prompt offline and show the proposal, validation, and route.'
    )
    preview.add_argument('prompt', nargs='?', default=DEFAULT_PROMPT)
    preview.add_argument(
        '--planner',
        choices=PLANNER_CHOICES,
        default='fake',
        help='Planner provider to use for the proposal.',
    )
    preview.add_argument('--json', action='store_true', help='Emit machine-readable JSON.')
    preview.set_defaults(func=cmd_preview)

    run = subparsers.add_parser(
        'run', help='Plan a prompt, confirm at an approval gate, then run on the robot.'
    )
    run.add_argument('prompt', nargs='?', default=DEFAULT_PROMPT)
    run.add_argument('--mission-id', default=_default_mission_id())
    run.add_argument('--artifact-root', type=Path, default=Path('/data/artifacts'))
    run.add_argument('--goal-timeout-sec', type=float, default=180.0)
    run.add_argument('--max-retries', type=int, default=1)
    run.add_argument('--server-timeout-sec', type=float, default=30.0)
    run.add_argument(
        '--planner',
        choices=PLANNER_CHOICES,
        default='fake',
        help='Planner provider to use for the proposal.',
    )
    run.add_argument(
        '--yes', action='store_true', help='Skip the interactive approval gate.'
    )
    run.add_argument(
        '--dry-run', action='store_true',
        help='Plan and show the runner command without executing it.',
    )
    run.set_defaults(func=cmd_run)

    status = subparsers.add_parser(
        'status', help='Print the latest mission status from persisted artifacts.'
    )
    status.add_argument(
        '--mission-id', default=None, help='Mission to inspect (default: latest).'
    )
    status.add_argument('--artifact-root', type=Path, default=_default_artifact_root())
    status.add_argument('--json', action='store_true', help='Emit machine-readable JSON.')
    status.set_defaults(func=cmd_status)

    cancel = subparsers.add_parser(
        'cancel', help='Cancellation entry point (unavailable in offline CLI mode).'
    )
    cancel.set_defaults(func=cmd_cancel)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())
