import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

from omokai_fleet.operator_cli import main


PROMPT = 'Form a wedge at the dock, inspect separate rooms, and regroup home.'


class OperatorCliTest(unittest.TestCase):
    def run_cli(self, *arguments):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = main(list(arguments))
        return code, stdout.getvalue(), stderr.getvalue()

    def test_preview_accepts_fake_prompt(self) -> None:
        code, output, error = self.run_cli('preview', PROMPT)

        self.assertEqual(0, code)
        self.assertEqual('', error)
        self.assertTrue(json.loads(output)['accepted'])

    def test_preview_rejects_unsupported_prompt(self) -> None:
        code, _, error = self.run_cli('preview', 'Move somehow.')

        self.assertEqual(1, code)
        self.assertIn('Planner failed', error)

    def test_dry_run_writes_plan_and_does_not_import_ros_runner(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            code, output, error = self.run_cli(
                'run',
                '--yes',
                '--dry-run',
                '--mission-id',
                'dry-run',
                '--artifact-root',
                directory,
                PROMPT,
            )

            self.assertEqual(0, code)
            self.assertEqual('', error)
            self.assertIn('Dry run accepted', output)
            self.assertTrue(
                (Path(directory) / 'dry-run' / 'squad_plan.json').is_file()
            )

    def test_status_reads_terminal_result(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            mission_dir = Path(directory) / 'finished'
            mission_dir.mkdir()
            (mission_dir / 'squad_plan.json').write_text('{}')
            (mission_dir / 'squad_result.json').write_text(
                json.dumps({'plan_id': 'finished', 'state': 'succeeded'})
            )

            code, output, error = self.run_cli(
                'status',
                'finished',
                '--artifact-root',
                directory,
            )

            self.assertEqual(0, code)
            self.assertEqual('', error)
            self.assertEqual('succeeded', json.loads(output)['state'])

    def test_status_reports_accepted_without_result(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            mission_dir = Path(directory) / 'active'
            mission_dir.mkdir()
            (mission_dir / 'squad_plan.json').write_text('{}')

            code, output, _ = self.run_cli(
                'status',
                '--artifact-root',
                directory,
            )

            self.assertEqual(0, code)
            self.assertEqual('accepted', json.loads(output)['state'])

    def test_run_rejects_unsafe_id_and_timeout_order(self) -> None:
        code, _, error = self.run_cli(
            'run',
            '--yes',
            '--dry-run',
            '--mission-id',
            '../escape',
            PROMPT,
        )
        self.assertEqual(2, code)
        self.assertIn('Invalid run configuration', error)

        code, _, error = self.run_cli(
            'run',
            '--yes',
            '--dry-run',
            '--goal-timeout-sec',
            '500',
            '--mission-timeout-sec',
            '300',
            PROMPT,
        )
        self.assertEqual(2, code)
        self.assertIn('mission timeout', error)


if __name__ == '__main__':
    unittest.main()
