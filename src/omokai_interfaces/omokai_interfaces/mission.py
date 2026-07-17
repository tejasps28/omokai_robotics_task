"""Immutable mission-domain values for the Task 1 pipeline."""

from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType
from typing import Any, Mapping


class MissionAction(str, Enum):
    """Actions supported by mission schema 1.1."""

    PATROL = 'patrol'


class TraversalDirection(str, Enum):
    """Audited route-order choices supported by mission schema v1.1."""

    CLOCKWISE = 'clockwise'
    COUNTERCLOCKWISE = 'counterclockwise'
    FORWARD = 'forward'
    REVERSE = 'reverse'


@dataclass(frozen=True)
class MissionSegment:
    """One ordered block of route traversals in a validated direction."""

    direction: TraversalDirection
    repetitions: int

    def __post_init__(self) -> None:
        if not isinstance(self.direction, TraversalDirection):
            raise ValueError('segment direction must be a TraversalDirection')
        if (
            isinstance(self.repetitions, bool)
            or not isinstance(self.repetitions, int)
            or self.repetitions < 1
        ):
            raise ValueError('segment repetitions must be a positive integer')

    def as_mapping(self) -> Mapping[str, object]:
        return MappingProxyType(
            {
                'direction': self.direction.value,
                'repetitions': self.repetitions,
            }
        )


@dataclass(frozen=True)
class PlanRequest:
    """A trusted request submitted by the operator-facing boundary."""

    request_id: str
    prompt: str

    def __post_init__(self) -> None:
        if not self.request_id.strip():
            raise ValueError('request_id must not be blank')
        if not self.prompt.strip():
            raise ValueError('prompt must not be blank')


@dataclass(frozen=True)
class MissionProposal:
    """Untrusted planner output retained verbatim until validation."""

    request_id: str
    provider: str
    content: str

    def __post_init__(self) -> None:
        if not self.request_id.strip():
            raise ValueError('request_id must not be blank')
        if not self.provider.strip():
            raise ValueError('provider must not be blank')


@dataclass(frozen=True)
class MissionV1:
    """Typed representation constructed only after structural validation."""

    schema_version: str
    action: MissionAction
    route_id: str
    segments: tuple[MissionSegment, ...]
    speed_mps: float
    return_home: bool

    def __post_init__(self) -> None:
        if self.schema_version != '1.1':
            raise ValueError("schema_version must be '1.1'")
        if not isinstance(self.action, MissionAction):
            raise ValueError('action must be a MissionAction')
        if not self.route_id.strip():
            raise ValueError('route_id must not be blank')
        if not isinstance(self.segments, tuple) or not self.segments:
            raise ValueError('segments must be a non-empty tuple')
        if any(not isinstance(segment, MissionSegment) for segment in self.segments):
            raise ValueError('segments must contain only MissionSegment values')
        if (
            isinstance(self.speed_mps, bool)
            or not isinstance(self.speed_mps, (int, float))
            or self.speed_mps <= 0
        ):
            raise ValueError('speed_mps must be a positive number')
        if not isinstance(self.return_home, bool):
            raise ValueError('return_home must be a boolean')

    @property
    def repetitions(self) -> int:
        """Return the safety-relevant total number of route traversals."""

        return sum(segment.repetitions for segment in self.segments)

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> 'MissionV1':
        """Construct from a mapping already accepted by the v1 JSON Schema."""

        expected = {
            'schema_version',
            'action',
            'route_id',
            'segments',
            'speed_mps',
            'return_home',
        }
        if set(value) != expected:
            raise ValueError('mapping does not contain the exact MissionV1 fields')

        raw_segments = value['segments']
        if not isinstance(raw_segments, list) or not raw_segments:
            raise ValueError('segments must be a non-empty list')

        segments = []
        for raw_segment in raw_segments:
            if not isinstance(raw_segment, Mapping) or set(raw_segment) != {
                'direction',
                'repetitions',
            }:
                raise ValueError('each segment must contain direction and repetitions')
            segments.append(
                MissionSegment(
                    direction=TraversalDirection(raw_segment['direction']),
                    repetitions=raw_segment['repetitions'],
                )
            )

        return cls(
            schema_version=str(value['schema_version']),
            action=MissionAction(value['action']),
            route_id=str(value['route_id']),
            segments=tuple(segments),
            speed_mps=float(value['speed_mps']),
            return_home=bool(value['return_home']),
        )

    def as_mapping(self) -> Mapping[str, object]:
        """Return a read-only mapping suitable for stable JSON serialization."""

        return MappingProxyType(
            {
                'schema_version': self.schema_version,
                'action': self.action.value,
                'route_id': self.route_id,
                'segments': [
                    dict(segment.as_mapping()) for segment in self.segments
                ],
                'speed_mps': self.speed_mps,
                'return_home': self.return_home,
            }
        )


@dataclass(frozen=True)
class ValidatedMission:
    """A structurally and semantically accepted mission."""

    mission_id: str
    request_id: str
    mission: MissionV1
    policy_version: str

    def __post_init__(self) -> None:
        if not self.mission_id.strip():
            raise ValueError('mission_id must not be blank')
        if not self.request_id.strip():
            raise ValueError('request_id must not be blank')
        if not self.policy_version.strip():
            raise ValueError('policy_version must not be blank')
