"""Atomic runtime evidence for an exploration session."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import tempfile
from typing import Iterable

from .session import ExplorationEvent, ExplorationResult, ExplorationSnapshot


_SAFE_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')


class ExplorationArtifactWriter:
    """Persist status, terminal result, and append-only ordered events."""

    def __init__(
        self,
        artifact_root: Path,
        exploration_id: str,
        *,
        fsync: bool = True,
    ) -> None:
        if not _SAFE_ID.fullmatch(exploration_id) or '..' in exploration_id:
            raise ValueError('exploration_id contains unsupported characters')
        self._directory = Path(artifact_root) / exploration_id
        self._directory.mkdir(parents=True, exist_ok=True)
        self._fsync = fsync

    @property
    def directory(self) -> Path:
        return self._directory

    def append_events(self, events: Iterable[ExplorationEvent]) -> None:
        path = self._directory / 'exploration_events.jsonl'
        with path.open('a', encoding='utf-8') as stream:
            for event in events:
                payload = {
                    'sequence': event.sequence,
                    'event_type': event.event_type,
                    'state': event.state.value,
                    'elapsed_sec': event.elapsed_sec,
                    'details': dict(event.details),
                }
                stream.write(
                    json.dumps(
                        payload,
                        sort_keys=True,
                        separators=(',', ':'),
                        allow_nan=False,
                    )
                    + '\n'
                )
            stream.flush()
            if self._fsync:
                os.fsync(stream.fileno())

    def write_status(self, snapshot: ExplorationSnapshot) -> Path:
        return self._write_json(
            'exploration_status.json',
            {
                'exploration_id': snapshot.exploration_id,
                'state': snapshot.state.value,
                'completed_goals': snapshot.completed_goals,
                'failed_goals': snapshot.failed_goals,
                'visited_count': snapshot.visited_count,
                'blacklist_count': snapshot.blacklist_count,
                'active_goal_handle': snapshot.active_goal_handle,
            },
        )

    def write_result(self, result: ExplorationResult) -> Path:
        return self._write_json(
            'exploration_result.json',
            {
                'exploration_id': result.exploration_id,
                'state': result.state.value,
                'reason': result.reason,
                'completed_goals': result.completed_goals,
                'failed_goals': result.failed_goals,
                'blacklisted_points': result.blacklisted_points,
                'visited_points': result.visited_points,
                'duration_sec': result.duration_sec,
            },
        )

    def _write_json(self, filename: str, payload: dict) -> Path:
        destination = self._directory / filename
        descriptor, temporary_name = tempfile.mkstemp(
            dir=self._directory,
            prefix=f'.{filename}.',
            suffix='.tmp',
        )
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(
                    payload,
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
