"""Provider-neutral squad planning and strict local validation."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import Any, Mapping, Protocol, runtime_checkable

from jsonschema import Draft7Validator
from omokai_interfaces import MissionProposal, PlanRequest

from omokai_fleet.model import Formation, SquadAction, SquadPlan


SQUAD_SCHEMA_VERSION = '1.0'
DEFAULT_SPACING_M = 0.6
DEFAULT_SPEED_MPS = 0.12

SQUAD_PLAN_SCHEMA: Mapping[str, Any] = {
    '$schema': 'http://json-schema.org/draft-07/schema#',
    'type': 'object',
    'additionalProperties': False,
    'required': [
        'schema_version',
        'action',
        'formation',
        'route_id',
        'spacing_m',
        'speed_mps',
        'split_route',
        'regroup',
        'regroup_location',
    ],
    'properties': {
        'schema_version': {'const': SQUAD_SCHEMA_VERSION},
        'action': {'const': 'formation_patrol'},
        'formation': {'type': 'string', 'enum': ['line', 'wedge']},
        'route_id': {'const': 'inspection_loop'},
        'spacing_m': {
            'type': 'number',
            'minimum': 0.6,
            'maximum': 1.2,
        },
        'speed_mps': {
            'type': 'number',
            'minimum': 0.05,
            'maximum': 0.18,
        },
        'split_route': {'type': 'boolean'},
        'regroup': {'type': 'boolean'},
        'regroup_location': {'const': 'home'},
    },
}


class SquadPlannerError(ValueError):
    """Raised when a planner cannot safely interpret a squad request."""


@runtime_checkable
class SquadPlanner(Protocol):
    provider: str

    def propose(self, request: PlanRequest) -> MissionProposal:
        """Return untrusted squad JSON without executing robot motion."""
        ...


@dataclass(frozen=True)
class SquadValidationIssue:
    path: str
    message: str


@dataclass(frozen=True)
class SquadValidationResult:
    accepted: bool
    plan: SquadPlan | None
    issues: tuple[SquadValidationIssue, ...]

    def __post_init__(self) -> None:
        if self.accepted != (self.plan is not None and not self.issues):
            raise ValueError('accepted must match the plan and issues')


class FakeSquadPlanner:
    """Deterministic offline planner for tests and demonstrations."""

    provider = 'fake-squad'

    def __init__(
        self,
        *,
        spacing_m: float = DEFAULT_SPACING_M,
        speed_mps: float = DEFAULT_SPEED_MPS,
    ) -> None:
        self._spacing_m = spacing_m
        self._speed_mps = speed_mps

    def propose(self, request: PlanRequest) -> MissionProposal:
        prompt = request.prompt.lower()
        if re.search(r'\bwedge\b', prompt):
            formation = 'wedge'
        elif re.search(r'\bline\b', prompt):
            formation = 'line'
        else:
            raise SquadPlannerError(
                'fake squad planner requires a line or wedge formation'
            )

        proposal = {
            'schema_version': SQUAD_SCHEMA_VERSION,
            'action': 'formation_patrol',
            'formation': formation,
            'route_id': 'inspection_loop',
            'spacing_m': self._spacing_m,
            'speed_mps': self._speed_mps,
            'split_route': bool(
                re.search(r'\b(?:split|divide|share|assign)\b', prompt)
            ),
            'regroup': bool(
                re.search(r'\b(?:regroup|rendezvous|return|home)\b', prompt)
            ),
            'regroup_location': 'home',
        }
        return MissionProposal(
            request_id=request.request_id,
            provider=self.provider,
            content=json.dumps(proposal, sort_keys=True),
        )


def validate_squad_proposal(
    proposal: MissionProposal,
    *,
    plan_id: str,
) -> SquadValidationResult:
    """Validate untrusted planner JSON and build an immutable squad plan."""
    if not isinstance(proposal, MissionProposal):
        raise ValueError('proposal must be a MissionProposal')
    try:
        value = json.loads(proposal.content)
    except json.JSONDecodeError as exc:
        return SquadValidationResult(
            False,
            None,
            (
                SquadValidationIssue(
                    '$',
                    f'malformed JSON: {exc.msg}',
                ),
            ),
        )

    schema_errors = sorted(
        Draft7Validator(SQUAD_PLAN_SCHEMA).iter_errors(value),
        key=lambda error: (
            tuple(str(item) for item in error.absolute_path),
            error.validator or '',
            error.message,
        ),
    )
    if schema_errors:
        return SquadValidationResult(
            False,
            None,
            tuple(
                SquadValidationIssue(
                    _json_path(error.absolute_path),
                    error.message,
                )
                for error in schema_errors
            ),
        )

    try:
        plan = SquadPlan(
            plan_id=plan_id,
            action=SquadAction(value['action']),
            formation=Formation(value['formation']),
            route_id=value['route_id'],
            spacing_m=value['spacing_m'],
            speed_mps=value['speed_mps'],
            split_route=value['split_route'],
            regroup=value['regroup'],
            regroup_location=value['regroup_location'],
        )
    except (KeyError, TypeError, ValueError) as exc:
        return SquadValidationResult(
            False,
            None,
            (SquadValidationIssue('$', f'policy rejected proposal: {exc}'),),
        )
    return SquadValidationResult(True, plan, ())


def _json_path(parts: Any) -> str:
    path = '$'
    for part in parts:
        if isinstance(part, int):
            path += f'[{part}]'
        else:
            path += f'.{part}'
    return path
