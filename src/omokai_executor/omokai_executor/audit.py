"""Append-only execution events and atomic mission artifacts."""

import json
import os
import re
import tempfile
from enum import Enum
from pathlib import Path
from threading import Lock
from typing import Any, Dict, List, Mapping

from .events import ExecutionEvent


_SAFE_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')


def _validate_identifier(value: str, field_name: str) -> None:
    if not _SAFE_ID.fullmatch(value) or '..' in value:
        raise ValueError(f'{field_name} contains unsupported characters')


class ArtifactKind(str, Enum):
    PROPOSAL = 'proposal'
    VALIDATION = 'validation'
    ACCEPTED_MISSION = 'accepted_mission'
    RESULT = 'result'


class InMemoryEventSink:
    """Test sink that preserves event order."""

    def __init__(self) -> None:
        self.events: List[ExecutionEvent] = []

    def record(self, event: ExecutionEvent) -> None:
        self.events.append(event)


class JsonlAuditSink:
    """Durably append one canonical JSON object per execution event."""

    def __init__(self, artifact_root: Path, *, fsync: bool = True) -> None:
        self._artifact_root = Path(artifact_root)
        self._fsync = fsync
        self._lock = Lock()

    def record(self, event: ExecutionEvent) -> None:
        _validate_identifier(event.mission_id, 'mission_id')
        mission_dir = self._artifact_root / event.mission_id
        mission_dir.mkdir(parents=True, exist_ok=True)
        encoded = json.dumps(
            event.as_dict(), sort_keys=True, separators=(',', ':'), allow_nan=False
        )

        with self._lock:
            with (mission_dir / 'execution.jsonl').open('a', encoding='utf-8') as stream:
                stream.write(encoded + '\n')
                stream.flush()
                if self._fsync:
                    os.fsync(stream.fileno())


class MissionArtifactWriter:
    """Atomically store the non-event evidence for a mission."""

    def __init__(self, artifact_root: Path, *, fsync: bool = True) -> None:
        self._artifact_root = Path(artifact_root)
        self._fsync = fsync

    def write_json(
        self,
        mission_id: str,
        kind: ArtifactKind,
        payload: Mapping[str, Any],
    ) -> Path:
        _validate_identifier(mission_id, 'mission_id')
        mission_dir = self._artifact_root / mission_id
        mission_dir.mkdir(parents=True, exist_ok=True)
        destination = mission_dir / f'{kind.value}.json'

        descriptor, temporary_name = tempfile.mkstemp(
            dir=mission_dir, prefix=f'.{kind.value}.', suffix='.tmp'
        )
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(
                    dict(payload),
                    stream,
                    sort_keys=True,
                    separators=(',', ':'),
                    allow_nan=False,
                )
                stream.write('\n')
                stream.flush()
                if self._fsync:
                    os.fsync(stream.fileno())
            os.replace(temporary_name, destination)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

        return destination
