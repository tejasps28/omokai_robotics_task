"""Operator preview, run, status, and cancel commands for squad missions."""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import asdict, replace
from datetime import datetime, timezone
from pathlib import Path

from omokai_interfaces import PlanRequest

from omokai_fleet.gemini_planner import GeminiSquadPlanner
from omokai_fleet.planner import (
    FakeSquadPlanner,
    SquadPlannerError,
    validate_squad_proposal,
)
from omokai_fleet.scenario import build_demo_mission


DEFAULT_PROMPT = (
    'You three split the inspection route in a wedge and regroup home.'
)
MISSION_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')


def create_planner(name: str):
    if name == 'fake':
        return FakeSquadPlanner()
    if name == 'gemini':
        return GeminiSquadPlanner()
    raise ValueError(f'unsupported planner {name!r}')


def build_preview(prompt: str, planner_name: str, plan_id: str):
    if not prompt.strip():
        raise SquadPlannerError('prompt must not be blank')
    proposal = create_planner(planner_name).propose(
        PlanRequest(f'{plan_id}-request', prompt)
    )
    validation = validate_squad_proposal(proposal, plan_id=plan_id)
    return proposal, validation


def preview_dict(prompt, proposal, validation) -> dict:
    payload = {
        'prompt': prompt,
        'provider': proposal.provider,
        'proposal': _decode_json(proposal.content),
        'accepted': validation.accepted,
        'issues': [asdict(issue) for issue in validation.issues],
        'mission': None,
    }
    if validation.accepted:
        mission = build_demo_mission(validation.plan)
        payload['mission'] = {
            'robots': 3,
            'formation_batches': 1 + len(mission.formation_movement),
            'split_batches': len(mission.split_execution),
            'regroup_batches': 1 if mission.regrouping else 0,
        }
    return payload


def cmd_preview(args) -> int:
    try:
        proposal, validation = build_preview(
            args.prompt,
            args.planner,
            'preview',
        )
    except Exception as exc:
        print(f'Planner failed: {exc}', file=sys.stderr)
        return 1
    payload = preview_dict(args.prompt, proposal, validation)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if validation.accepted else 1


def cmd_run(args) -> int:
    try:
        _validate_mission_id(args.mission_id)
        if args.goal_timeout_sec <= 0:
            raise ValueError('goal timeout must be positive')
        if args.mission_timeout_sec < args.goal_timeout_sec:
            raise ValueError(
                'mission timeout must not be shorter than goal timeout'
            )
    except ValueError as exc:
        print(f'Invalid run configuration: {exc}', file=sys.stderr)
        return 2
    try:
        proposal, validation = build_preview(
            args.prompt,
            args.planner,
            args.mission_id,
        )
    except Exception as exc:
        print(f'Planner failed: {exc}', file=sys.stderr)
        return 1
    payload = preview_dict(args.prompt, proposal, validation)
    print(json.dumps(payload, indent=2, sort_keys=True))
    if not validation.accepted:
        print('Squad proposal rejected; no robot goal was sent.', file=sys.stderr)
        return 1
    if not args.yes and not _confirm('Approve squad mission? [y/N] '):
        print('Approval declined; squad mission not started.', file=sys.stderr)
        return 3

    artifact_dir = args.artifact_root / args.mission_id
    _write_json(artifact_dir / 'squad_plan.json', payload)
    plan = replace(
        validation.plan,
        goal_timeout_sec=args.goal_timeout_sec,
        mission_timeout_sec=args.mission_timeout_sec,
    )
    mission = build_demo_mission(plan)
    if args.dry_run:
        print(
            f'Dry run accepted: {len(mission.forming)} robots, '
            f'{len(mission.split_execution)} split batches.'
        )
        return 0

    from omokai_fleet.runner import run_live_mission

    return run_live_mission(mission, artifact_dir)


def cmd_status(args) -> int:
    mission_id = args.mission_id or _latest_mission_id(args.artifact_root)
    if mission_id is None:
        print('No squad mission artifacts found.', file=sys.stderr)
        return 1
    try:
        _validate_mission_id(mission_id)
    except ValueError as exc:
        print(f'Invalid status request: {exc}', file=sys.stderr)
        return 2
    mission_dir = args.artifact_root / mission_id
    result_path = mission_dir / 'squad_result.json'
    if result_path.is_file():
        print(result_path.read_text(encoding='utf-8').rstrip())
        return 0
    if (mission_dir / 'squad_plan.json').is_file():
        print(
            json.dumps(
                {'plan_id': mission_id, 'state': 'accepted'},
                indent=2,
                sort_keys=True,
            )
        )
        return 0
    print(f'No squad status found for {mission_id}.', file=sys.stderr)
    return 1


def cmd_cancel(args) -> int:
    del args
    from omokai_fleet.runner import request_live_cancel

    success, message = request_live_cancel()
    print(message)
    return 0 if success else 4


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='omokai-squad')
    commands = parser.add_subparsers(dest='command', required=True)

    preview = commands.add_parser('preview')
    preview.add_argument('prompt', nargs='?', default=DEFAULT_PROMPT)
    preview.add_argument('--planner', choices=('fake', 'gemini'), default='fake')
    preview.set_defaults(func=cmd_preview)

    run = commands.add_parser('run')
    run.add_argument('prompt', nargs='?', default=DEFAULT_PROMPT)
    run.add_argument('--planner', choices=('fake', 'gemini'), default='fake')
    run.add_argument('--mission-id', default=_default_mission_id())
    run.add_argument('--artifact-root', type=Path, default=Path('/data/artifacts'))
    run.add_argument('--goal-timeout-sec', type=float, default=300.0)
    run.add_argument('--mission-timeout-sec', type=float, default=1800.0)
    run.add_argument('--yes', action='store_true')
    run.add_argument('--dry-run', action='store_true')
    run.set_defaults(func=cmd_run)

    status = commands.add_parser('status')
    status.add_argument('mission_id', nargs='?')
    status.add_argument('--artifact-root', type=Path, default=Path('/data/artifacts'))
    status.set_defaults(func=cmd_status)

    cancel = commands.add_parser('cancel')
    cancel.set_defaults(func=cmd_cancel)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


def _default_mission_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return f'squad-{timestamp}'


def _validate_mission_id(value: str) -> None:
    if not MISSION_ID_PATTERN.fullmatch(value) or '..' in value:
        raise ValueError('mission ID contains unsupported characters')


def _latest_mission_id(root: Path):
    candidates = [
        item
        for item in root.iterdir()
        if item.is_dir() and (item / 'squad_plan.json').is_file()
    ] if root.is_dir() else []
    if not candidates:
        return None
    return max(candidates, key=lambda item: item.stat().st_mtime).name


def _confirm(text: str) -> bool:
    try:
        return input(text).strip().lower() in {'y', 'yes'}
    except EOFError:
        return False


def _decode_json(value: str):
    try:
        return json.loads(value)
    except json.JSONDecodeError:
        return value


def _write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)
