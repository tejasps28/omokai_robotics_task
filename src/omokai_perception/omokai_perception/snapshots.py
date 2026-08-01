"""Exactly-once snapshot artifacts for confirmed vision missions."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class SnapshotRecord:
    mission_id: str
    mission_generation: int
    episode: int
    target_class: str
    coat_color: str
    confidence: float
    source_timestamp: float
    original_path: str
    annotated_path: str
    metadata_path: str


class SnapshotWriter:
    def __init__(self, artifact_root: Path) -> None:
        self.artifact_root = artifact_root
        self._written: set[str] = set()

    def write_once(
        self,
        mission_id: str,
        mission_generation: int,
        episode: int,
        target_class: str,
        coat_color: str,
        confidence: float,
        source_timestamp: float,
        original: np.ndarray,
        annotated: np.ndarray,
    ) -> SnapshotRecord | None:
        if mission_id in self._written:
            return None
        if not mission_id or '/' in mission_id or '..' in mission_id:
            raise ValueError('mission ID is not artifact-path safe')
        if mission_generation < 1:
            raise ValueError('mission generation must be positive')
        directory = self.artifact_root / mission_id / 'vision'
        directory.mkdir(parents=True, exist_ok=True)
        stem = f'acquisition-{episode:03d}-{source_timestamp:.3f}'
        original_path = directory / f'{stem}-original.png'
        annotated_path = directory / f'{stem}-annotated.png'
        metadata_path = directory / f'{stem}.json'
        if not cv2.imwrite(str(original_path), original):
            raise OSError(f'failed to write {original_path}')
        if not cv2.imwrite(str(annotated_path), annotated):
            raise OSError(f'failed to write {annotated_path}')
        record = SnapshotRecord(
            mission_id,
            mission_generation,
            episode,
            target_class,
            coat_color,
            confidence,
            source_timestamp,
            str(original_path),
            str(annotated_path),
            str(metadata_path),
        )
        temporary = metadata_path.with_suffix('.json.tmp')
        temporary.write_text(
            json.dumps(asdict(record), indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        temporary.replace(metadata_path)
        self._written.add(mission_id)
        return record
