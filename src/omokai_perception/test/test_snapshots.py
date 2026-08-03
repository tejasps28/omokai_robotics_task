import json
from pathlib import Path

import numpy as np
import pytest

from omokai_perception.snapshots import SnapshotWriter


def test_snapshot_is_written_exactly_once_per_mission(tmp_path) -> None:
    writer = SnapshotWriter(tmp_path)
    image = np.zeros((20, 30, 3), dtype=np.uint8)

    first = writer.write_once(
        'mission-1', 4, 1, 'person', 'white', 0.9, 12.5, image, image
    )
    second = writer.write_once(
        'mission-1', 4, 2, 'person', 'white', 0.9, 12.6, image, image
    )

    assert first is not None
    assert second is None
    assert len(list((tmp_path / 'mission-1' / 'vision').glob('*.png'))) == 2
    metadata = json.loads(Path(first.metadata_path).read_text())
    assert metadata['episode'] == 1
    assert metadata['mission_generation'] == 4


def test_snapshot_rejects_unsafe_mission_id(tmp_path) -> None:
    image = np.zeros((2, 2, 3), dtype=np.uint8)

    with pytest.raises(ValueError, match='not artifact-path safe'):
        SnapshotWriter(tmp_path).write_once(
            '../escape', 1, 1, 'person', 'white', 0.9, 1.0, image, image
        )
