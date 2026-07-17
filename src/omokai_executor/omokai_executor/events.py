"""Audit events emitted by deterministic mission execution."""

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Dict, Mapping

from .model import ExecutorState


@dataclass(frozen=True)
class ExecutionEvent:
    sequence: int
    mission_id: str
    event_type: str
    state: ExecutorState
    timestamp_utc: str
    details: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.sequence < 1:
            raise ValueError('event sequence starts at one')
        if not self.mission_id.strip() or not self.event_type.strip():
            raise ValueError('mission_id and event_type must not be blank')
        object.__setattr__(self, 'details', MappingProxyType(dict(self.details)))

    def as_dict(self) -> Dict[str, Any]:
        return {
            'sequence': self.sequence,
            'mission_id': self.mission_id,
            'event_type': self.event_type,
            'state': self.state.value,
            'timestamp_utc': self.timestamp_utc,
            'details': dict(self.details),
        }
