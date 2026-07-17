"""Planner abstraction and a deterministic fake planner.

A planner turns a trusted operator :class:`~omokai_interfaces.PlanRequest` into an
untrusted :class:`~omokai_interfaces.MissionProposal`. The proposal is text and is
never trusted: it must pass :func:`~omokai_mission.validation.validate_proposal`
before anything downstream uses it. Keeping the interface provider-neutral means
the hosted providers and the offline fake below are interchangeable behind the
same contract.

The :class:`FakePlanner` interprets a small, fixed grammar deterministically so
automated tests need no network access or credentials. It always records the
exact request ID and returns the raw generated JSON verbatim in
``MissionProposal.content`` so the provider output stays auditable.
"""

from __future__ import annotations

import json
import re
from typing import Protocol, runtime_checkable

from omokai_interfaces import MissionProposal, PlanRequest

# Fixed default speed for the fake planner's proposals. Matches the audited
# demonstration mission and stays within the semantic speed bound.
DEFAULT_SPEED_MPS = 0.18

# Word-to-count mapping the fake planner understands for repetitions.
_COUNT_WORDS = {
    'once': 1,
    'twice': 2,
    'thrice': 3,
    'one': 1,
    'two': 2,
    'three': 3,
    'four': 4,
    'five': 5,
    'six': 6,
    'seven': 7,
    'eight': 8,
    'nine': 9,
    'ten': 10,
}


class PlannerError(ValueError):
    """Raised when a deterministic planner cannot interpret a request."""


@runtime_checkable
class Planner(Protocol):
    """Provider-neutral planner interface.

    Implementations expose a stable ``provider`` label and turn a request into an
    untrusted proposal. They must not execute motion or call navigation.
    """

    provider: str

    def propose(self, request: PlanRequest) -> MissionProposal:
        """Return an untrusted mission proposal for ``request``."""
        ...


def _extract_repetitions(prompt: str) -> int:
    """Deterministically read a repetition count from the prompt (default 1)."""

    # Numeric forms such as "2 times", "3 laps", "x2".
    numeric = re.search(r'(\d+)\s*(?:times?|laps?|loops?|x)\b', prompt)
    if numeric:
        return int(numeric.group(1))
    bare_x = re.search(r'\bx\s*(\d+)\b', prompt)
    if bare_x:
        return int(bare_x.group(1))
    # Word forms such as "twice", "three times".
    for word, count in _COUNT_WORDS.items():
        if re.search(rf'\b{word}\b', prompt):
            return count
    return 1


def _wants_return_home(prompt: str) -> bool:
    """Detect an explicit request to return/come back home."""

    patterns = (
        r'return\s+home',
        r'return\s+to\s+home',
        r'\breturn\b',
        r'come\s+back',
        r'back\s+home',
        r'go\s+(?:to\s+)?home',
        r'return\s+to\s+(?:the\s+)?start',
        r'back\s+to\s+(?:the\s+)?start',
    )
    return any(re.search(pattern, prompt) for pattern in patterns)


_LOOP_DIRECTION = re.compile(
    r'\b(?:counter[- ]?clock[- ]?wise|anti[- ]?clock[- ]?wise|clock[- ]?wise)\b'
)
_PATH_DIRECTION = re.compile(r'\b(?:forward|reverse|backwards?)\b')
_FULL_AREA = re.compile(
    r'\b(?:full|whole|entire)\s+(?:area|space|floor|room)\b'
    r'|\b(?:all|every)\s+(?:aisles?|lanes?|corridors?)\b'
)


def _direction_value(token: str, route_id: str) -> str:
    normalized = token.replace('-', '').replace(' ', '')
    if route_id == 'inspection_loop':
        return (
            'counterclockwise'
            if normalized.startswith(('counter', 'anti'))
            else 'clockwise'
        )
    return 'reverse' if normalized.startswith(('reverse', 'backward')) else 'forward'


def _extract_segments(prompt: str, route_id: str) -> list[dict[str, object]]:
    """Extract ordered, bounded route segments from the fake-planner grammar."""

    pattern = _LOOP_DIRECTION if route_id == 'inspection_loop' else _PATH_DIRECTION
    matches = list(pattern.finditer(prompt))
    if not matches:
        default = 'counterclockwise' if route_id == 'inspection_loop' else 'forward'
        return [{'direction': default, 'repetitions': _extract_repetitions(prompt)}]
    if len(matches) == 1:
        return [
            {
                'direction': _direction_value(matches[0].group(0), route_id),
                'repetitions': _extract_repetitions(prompt),
            }
        ]

    segments = []
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(prompt)
        clause = prompt[match.end():end]
        segments.append(
            {
                'direction': _direction_value(match.group(0), route_id),
                'repetitions': _extract_repetitions(clause),
            }
        )
    return segments


def _route_id_from_prompt(prompt: str) -> str:
    if _FULL_AREA.search(prompt):
        return 'full_area_sweep'
    if re.search(r'\b(?:aisles?|lanes?)\b', prompt):
        return 'aisle_sweep'
    if (
        re.search(r'\b(?:inspection|perimeter)\s+(?:loop|route)\b', prompt)
        or re.search(r'\b(?:same\s+|patrol\s+)?loop\b', prompt)
        or _LOOP_DIRECTION.search(prompt)
    ):
        return 'inspection_loop'
    raise PlannerError(
        'fake planner supports inspection loop, aisle sweep, and full area sweep routes'
    )


class FakePlanner:
    """A deterministic, offline planner for tests and diagnostics.

    It recognises the v1.1 patrol grammar. For the demonstration prompt
    "Patrol the inspection loop twice and return home." it emits exactly:

        {"action": "patrol", "segments": [{"direction":
         "counterclockwise", "repetitions": 2}], "return_home": true,
         "route_id": "inspection_loop", "schema_version": "1.1",
         "speed_mps": 0.18}
    """

    provider = 'fake'

    def __init__(self, speed_mps: float = DEFAULT_SPEED_MPS) -> None:
        self._speed_mps = speed_mps

    def propose(self, request: PlanRequest) -> MissionProposal:
        prompt = request.prompt.lower()
        if not re.search(
            r'\b(?:patrol|drive|sweep|cover|navigate|go\s+through)\b',
            prompt,
        ):
            raise PlannerError(
                'fake planner supports patrol, drive, sweep, cover, and navigate requests'
            )
        route_id = _route_id_from_prompt(prompt)
        spec = {
            'schema_version': '1.1',
            'action': 'patrol',
            'route_id': route_id,
            'segments': _extract_segments(prompt, route_id),
            'speed_mps': self._speed_mps,
            'return_home': _wants_return_home(prompt),
        }
        # sort_keys keeps the raw provider output byte-stable and auditable.
        content = json.dumps(spec, sort_keys=True)
        return MissionProposal(
            request_id=request.request_id,
            provider=self.provider,
            content=content,
        )
