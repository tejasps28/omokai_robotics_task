import json
import tempfile
import unittest
from pathlib import Path

from omokai_executor import ArtifactKind, JsonlAuditSink, MissionArtifactWriter
from omokai_executor.events import ExecutionEvent
from omokai_executor.model import ExecutorState


class AuditTest(unittest.TestCase):
    def test_appends_canonical_json_lines(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            sink = JsonlAuditSink(Path(directory), fsync=False)
            for sequence in (1, 2):
                sink.record(
                    ExecutionEvent(
                        sequence=sequence,
                        mission_id='mission-1',
                        event_type='test_event',
                        state=ExecutorState.ACCEPTED,
                        timestamp_utc='2026-07-02T00:00:00Z',
                        details={'value': sequence},
                    )
                )

            path = Path(directory) / 'mission-1' / 'execution.jsonl'
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([1, 2], [record['sequence'] for record in records])

    def test_writes_artifact_atomically(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            writer = MissionArtifactWriter(Path(directory), fsync=False)
            destination = writer.write_json(
                'mission-1', ArtifactKind.VALIDATION, {'accepted': True}
            )
            self.assertEqual({'accepted': True}, json.loads(destination.read_text()))
            self.assertEqual([], list(destination.parent.glob('*.tmp')))

    def test_rejects_unsafe_mission_identifier(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            writer = MissionArtifactWriter(Path(directory), fsync=False)
            with self.assertRaises(ValueError):
                writer.write_json('../outside', ArtifactKind.RESULT, {})


if __name__ == '__main__':
    unittest.main()
