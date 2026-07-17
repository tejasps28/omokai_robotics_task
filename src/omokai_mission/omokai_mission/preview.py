"""Developer-only offline preview of the mission-planning pipeline.

Demonstrates, without any ROS runtime, Gazebo, network access, or credentials:

    prompt -> fake proposal -> validation -> compiled route summary

This is a diagnostic aid for developers, not the operator API. It prints the
raw proposal JSON, the validation result, and — when the mission is accepted —
the number of compiled goals and their ordered summary.

Run from the repository root:

    PYTHONPATH="src/omokai_interfaces:src/omokai_mission" \
        python3 -m omokai_mission.preview

An alternative prompt may be supplied as a single argument:

    python3 -m omokai_mission.preview "Patrol the inspection loop once."
"""

from __future__ import annotations

import json
import sys
from typing import List, Optional

from omokai_interfaces import PlanRequest

from .catalog import load_catalog
from .compiler import compile_mission
from .planner import FakePlanner
from .validation import validate_proposal

DEMO_PROMPT = 'Patrol the inspection loop twice and return home.'
# Fixed IDs keep the preview output deterministic across runs.
PREVIEW_REQUEST_ID = 'preview-request'
PREVIEW_MISSION_ID = 'preview-mission'


def _run_preview(prompt: str) -> tuple[List[str], bool]:
    """Run the offline pipeline and return output lines plus acceptance."""

    lines: List[str] = []
    lines.append('=== Omokai mission preview (offline) ===')
    lines.append(f'Prompt: {prompt}')
    lines.append('')

    request = PlanRequest(request_id=PREVIEW_REQUEST_ID, prompt=prompt)
    planner = FakePlanner()
    proposal = planner.propose(request)

    lines.append(f'Planner provider: {proposal.provider}')
    lines.append('Proposal JSON:')
    # Re-dump with indentation for readability; the raw content is preserved
    # verbatim on the proposal object for auditing.
    lines.append(json.dumps(json.loads(proposal.content), indent=2, sort_keys=True))
    lines.append('')

    catalog = load_catalog()
    result = validate_proposal(
        proposal,
        route_directions=catalog.direction_policy(),
    )

    lines.append(f'Validation: accepted={result.accepted} stage={result.stage}')
    if not result.accepted:
        lines.append('Errors:')
        for issue in result.errors:
            lines.append(f'  - [{issue.code}] {issue.path}: {issue.message}')
        lines.append('')
        lines.append('Rejected — no route compiled and nothing would execute.')
        return lines, False

    lines.append(f'Policy: {result.policy_version}')
    lines.append('')

    validated = result.build_validated_mission(PREVIEW_MISSION_ID)
    compiled = compile_mission(validated, catalog)

    lines.append(f'Compiled route: {compiled.route_id} (frame={compiled.frame_id})')
    lines.append(f'Number of goals: {len(compiled.goals)}')
    lines.append('Ordered goals:')
    lines.extend(f'  {line}' for line in compiled.summary_lines())
    return lines, True


def _render(prompt: str) -> List[str]:
    """Compatibility helper returning deterministic output lines."""

    return _run_preview(prompt)[0]


def main(argv: Optional[List[str]] = None) -> int:
    """Entry point. Returns 0 on an accepted mission, 1 on rejection."""

    args = list(sys.argv[1:] if argv is None else argv)
    prompt = args[0] if args else DEMO_PROMPT

    lines, accepted = _run_preview(prompt)
    print('\n'.join(lines))
    return 0 if accepted else 1


if __name__ == '__main__':
    raise SystemExit(main())
