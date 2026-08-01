import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from omokai_exploration import (
    ExplorationEvent,
    ExplorationResult,
    ExplorationSnapshot,
    ExplorationState,
)
from omokai_exploration.artifacts import ExplorationArtifactWriter


class ExplorationArtifactWriterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.writer = ExplorationArtifactWriter(
            self.root,
            'slam-test',
            fsync=False,
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_writes_atomic_status_snapshot(self) -> None:
        path = self.writer.write_status(
            ExplorationSnapshot(
                exploration_id='slam-test',
                state=ExplorationState.NAVIGATING,
                completed_goals=2,
                failed_goals=1,
                visited_count=2,
                blacklist_count=1,
                active_goal_handle='nav-goal-3',
            )
        )

        payload = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual('navigating', payload['state'])
        self.assertEqual(2, payload['completed_goals'])
        self.assertEqual('nav-goal-3', payload['active_goal_handle'])
        self.assertEqual([], list(path.parent.glob('*.tmp')))

    def test_appends_canonical_ordered_events(self) -> None:
        self.writer.append_events(
            (
                ExplorationEvent(
                    sequence=1,
                    event_type='session_created',
                    state=ExplorationState.CREATED,
                    elapsed_sec=0.0,
                ),
                ExplorationEvent(
                    sequence=2,
                    event_type='state_transition',
                    state=ExplorationState.WAITING_FOR_MAP,
                    elapsed_sec=0.1,
                    details=(('to_state', 'waiting_for_map'),),
                ),
            )
        )

        lines = (
            self.writer.directory / 'exploration_events.jsonl'
        ).read_text(encoding='utf-8').splitlines()
        self.assertEqual([1, 2], [json.loads(line)['sequence'] for line in lines])

    def test_writes_terminal_result(self) -> None:
        path = self.writer.write_result(
            ExplorationResult(
                exploration_id='slam-test',
                state=ExplorationState.COMPLETED,
                reason='no_frontiers',
                completed_goals=4,
                failed_goals=1,
                blacklisted_points=((1.0, 2.0),),
                visited_points=((0.0, 0.0),),
                duration_sec=42.0,
            )
        )

        payload = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual('completed', payload['state'])
        self.assertEqual('no_frontiers', payload['reason'])
        self.assertEqual([[1.0, 2.0]], payload['blacklisted_points'])

    def test_rejects_unsafe_identifier(self) -> None:
        with self.assertRaises(ValueError):
            ExplorationArtifactWriter(self.root, '../unsafe', fsync=False)


if __name__ == '__main__':
    unittest.main()
