"""Two-stage mission validation for the Task 1 core pipeline.

A planner returns an untrusted :class:`~omokai_interfaces.MissionProposal` whose
``content`` is arbitrary text. Before any mission may reach the executor it must
pass, in order:

1. **Structural validation** — the text must parse as JSON and satisfy the
   packaged strict Draft 7 schema (``omokai_interfaces`` owns the schema). This
   rejects malformed JSON, missing/extra fields, wrong types, and unsupported
   enum/const values. A typed :class:`~omokai_interfaces.MissionV1` is built only
   after this stage succeeds.
2. **Semantic validation** — a named safety policy checks the typed mission
   against operational limits: the action must be a supported capability, the
   route must exist in the audited catalog, and repetitions and speed must be
   within bounds.

The result is deterministic: for a given proposal and policy the accepted flag,
stage, and the ordered list of errors are always identical. Every error carries a
stable :mod:`~omokai_mission.errors` code and a JSON path so it can be written to
an audit log without additional interpretation. No mission is ever executed while
any error is present; ``ValidationResult.accepted`` is the single gate.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

from jsonschema import Draft7Validator

from omokai_interfaces import (
    MissionAction,
    MissionProposal,
    MissionV1,
    TraversalDirection,
    ValidatedMission,
    load_mission_v1_schema,
)

from . import errors

# Version of the semantic policy expressed by this module. It is recorded on the
# accepted mission so audit logs can attribute an acceptance to a specific set of
# limits. Bump this string whenever the limits below change.
POLICY_VERSION = 'core-safety-2'

# Route identifiers this policy will accept. The compiler's route catalog is the
# operational source of truth for poses; this constant keeps structural policy
# self-contained. ``test_catalog`` asserts the two agree.
KNOWN_ROUTE_DIRECTIONS: Mapping[str, Tuple[TraversalDirection, ...]] = {
    'aisle_sweep': (
        TraversalDirection.FORWARD,
        TraversalDirection.REVERSE,
    ),
    'inspection_loop': (
        TraversalDirection.CLOCKWISE,
        TraversalDirection.COUNTERCLOCKWISE,
    ),
    'full_area_sweep': (
        TraversalDirection.FORWARD,
        TraversalDirection.REVERSE,
    ),
}
KNOWN_ROUTE_IDS: Tuple[str, ...] = tuple(sorted(KNOWN_ROUTE_DIRECTIONS))

# Capabilities the ground robot supports in schema 1.1.
SUPPORTED_ACTIONS: Tuple[MissionAction, ...] = (MissionAction.PATROL,)

# Operational bounds. The JSON Schema only enforces repetitions >= 1 and
# speed > 0; the upper bounds are an operational safety decision and live here.
MAX_REPETITIONS = 10
# TurtleBot3 Waffle Pi maximum rated linear velocity (m/s), per the ROBOTIS
# specification. Proposals above this are physically unrealizable and rejected.
MAX_SPEED_MPS = 0.26


@dataclass(frozen=True)
class ValidationIssue:
    """A single, audit-ready validation failure."""

    code: str
    message: str
    path: str
    stage: str

    def as_dict(self) -> Dict[str, str]:
        """Return a stable dictionary suitable for JSON audit records."""

        return {
            'code': self.code,
            'message': self.message,
            'path': self.path,
            'stage': self.stage,
        }


@dataclass(frozen=True)
class ValidationResult:
    """Outcome of validating one proposal.

    ``accepted`` is the only gate the executor should consult. When accepted,
    ``mission`` holds the typed mission and ``policy_version`` records the policy
    that accepted it. When rejected, ``errors`` explains why and ``mission`` is
    ``None``.
    """

    request_id: str
    accepted: bool
    stage: str
    mission: Optional[MissionV1] = None
    policy_version: Optional[str] = None
    errors: Tuple[ValidationIssue, ...] = field(default_factory=tuple)

    def build_validated_mission(self, mission_id: str) -> ValidatedMission:
        """Promote an accepted result to a :class:`ValidatedMission`.

        Raises ``ValueError`` if the result was not accepted, guaranteeing that
        only semantically accepted missions can obtain a mission identity.
        """

        if not self.accepted or self.mission is None or self.policy_version is None:
            raise ValueError('cannot build a validated mission from a rejected result')
        return ValidatedMission(
            mission_id=mission_id,
            request_id=self.request_id,
            mission=self.mission,
            policy_version=self.policy_version,
        )

    def audit_records(self) -> List[Dict[str, str]]:
        """Return the errors as a stable list of dictionaries for audit logs."""

        return [issue.as_dict() for issue in self.errors]


def _json_path(validator_error: Any) -> str:
    """Build a ``$``-rooted JSON path from a Draft 7 validation error."""

    parts = list(validator_error.absolute_path)
    if not parts:
        prop = _offending_property(validator_error)
        if prop is not None:
            return f'$.{prop}'
        return '$'
    path = '$'
    for part in parts:
        if isinstance(part, int):
            path += f'[{part}]'
        else:
            path += f'.{part}'
    return path


def _offending_property(validator_error: Any) -> Optional[str]:
    """Extract the property name for keyword errors that lack an instance path."""

    if validator_error.validator == 'required':
        match = re.search(r"'([^']+)' is a required property", validator_error.message)
        return match.group(1) if match else None
    if validator_error.validator == 'additionalProperties':
        found = re.findall(r"'([^']+)'", validator_error.message)
        return found[0] if found else None
    return None


def _structural_errors(schema_errors: Any) -> Tuple[ValidationIssue, ...]:
    """Map raw Draft 7 errors to stable, deterministically ordered issues."""

    issues = []
    for schema_error in schema_errors:
        code = errors.SCHEMA_KEYWORD_CODES.get(
            schema_error.validator, errors.SCHEMA_VIOLATION
        )
        issues.append(
            ValidationIssue(
                code=code,
                message=schema_error.message,
                path=_json_path(schema_error),
                stage=errors.STAGE_STRUCTURAL,
            )
        )
    # jsonschema does not guarantee a stable iteration order; sort so identical
    # input always yields an identical error list.
    issues.sort(key=lambda issue: (issue.path, issue.code, issue.message))
    return tuple(issues)


def _semantic_errors(
    mission: MissionV1,
    route_directions: Mapping[str, Tuple[TraversalDirection, ...]],
) -> Tuple[ValidationIssue, ...]:
    """Apply the safety policy in a fixed, deterministic order."""

    issues: List[ValidationIssue] = []

    if mission.action not in SUPPORTED_ACTIONS:
        issues.append(
            ValidationIssue(
                code=errors.UNSUPPORTED_ACTION,
                message=f'action {mission.action.value!r} is not a supported capability',
                path='$.action',
                stage=errors.STAGE_SEMANTIC,
            )
        )

    allowed_directions = route_directions.get(mission.route_id)
    if allowed_directions is None:
        issues.append(
            ValidationIssue(
                code=errors.UNKNOWN_ROUTE,
                message=f'route {mission.route_id!r} is not in the known route catalog',
                path='$.route_id',
                stage=errors.STAGE_SEMANTIC,
            )
        )
    else:
        for index, segment in enumerate(mission.segments):
            if segment.direction not in allowed_directions:
                issues.append(
                    ValidationIssue(
                        code=errors.UNSUPPORTED_DIRECTION,
                        message=(
                            f'direction {segment.direction.value!r} is not allowed '
                            f'for route {mission.route_id!r}'
                        ),
                        path=f'$.segments[{index}].direction',
                        stage=errors.STAGE_SEMANTIC,
                    )
                )

    if mission.repetitions > MAX_REPETITIONS:
        issues.append(
            ValidationIssue(
                code=errors.REPETITIONS_OUT_OF_RANGE,
                message=(
                    f'total repetitions {mission.repetitions} exceeds the maximum of '
                    f'{MAX_REPETITIONS}'
                ),
                path='$.segments',
                stage=errors.STAGE_SEMANTIC,
            )
        )

    if mission.speed_mps > MAX_SPEED_MPS:
        issues.append(
            ValidationIssue(
                code=errors.SPEED_OUT_OF_RANGE,
                message=(
                    f'speed {mission.speed_mps} m/s exceeds the maximum of '
                    f'{MAX_SPEED_MPS} m/s'
                ),
                path='$.speed_mps',
                stage=errors.STAGE_SEMANTIC,
            )
        )

    return tuple(issues)


def validate_proposal(
    proposal: MissionProposal,
    route_directions: Mapping[
        str, Tuple[TraversalDirection, ...]
    ] = KNOWN_ROUTE_DIRECTIONS,
) -> ValidationResult:
    """Validate an untrusted proposal and return a deterministic result.

    ``route_directions`` is injectable so callers validate against the live
    catalog's route and traversal policy; it defaults to the audited policy.
    """

    # Stage 0: parse untrusted text. A parse failure stops here.
    try:
        parsed = json.loads(
            proposal.content,
            parse_constant=lambda value: _reject_json_constant(value),
        )
    except (json.JSONDecodeError, ValueError) as exc:
        issue = ValidationIssue(
            code=errors.MALFORMED_JSON,
            message=f'content is not valid strict JSON: {exc}',
            path='$',
            stage=errors.STAGE_STRUCTURAL,
        )
        return ValidationResult(
            request_id=proposal.request_id,
            accepted=False,
            stage=errors.STAGE_STRUCTURAL,
            errors=(issue,),
        )

    # Stage 1: strict JSON Schema.
    validator = Draft7Validator(load_mission_v1_schema())
    structural = _structural_errors(validator.iter_errors(parsed))
    if structural:
        return ValidationResult(
            request_id=proposal.request_id,
            accepted=False,
            stage=errors.STAGE_STRUCTURAL,
            errors=structural,
        )

    # The schema guarantees an object with exactly the 1.1 fields, so this build
    # cannot fail; guard defensively and surface any mismatch as structural.
    try:
        mission = MissionV1.from_mapping(_as_mapping(parsed))
    except (ValueError, TypeError) as exc:
        issue = ValidationIssue(
            code=errors.SCHEMA_VIOLATION,
            message=f'schema-valid content could not be typed: {exc}',
            path='$',
            stage=errors.STAGE_STRUCTURAL,
        )
        return ValidationResult(
            request_id=proposal.request_id,
            accepted=False,
            stage=errors.STAGE_STRUCTURAL,
            errors=(issue,),
        )

    # Stage 2: semantic safety policy.
    semantic = _semantic_errors(mission, route_directions)
    if semantic:
        return ValidationResult(
            request_id=proposal.request_id,
            accepted=False,
            stage=errors.STAGE_SEMANTIC,
            errors=semantic,
        )

    return ValidationResult(
        request_id=proposal.request_id,
        accepted=True,
        stage=errors.STAGE_ACCEPTED,
        mission=mission,
        policy_version=POLICY_VERSION,
    )


def _as_mapping(parsed: Any) -> Mapping[str, Any]:
    """Return ``parsed`` as a mapping or raise if it is not an object."""

    if not isinstance(parsed, Mapping):
        raise TypeError('top-level JSON value is not an object')
    return parsed


def _reject_json_constant(value: str) -> Any:
    raise ValueError(f'non-standard numeric constant {value!r} is forbidden')
