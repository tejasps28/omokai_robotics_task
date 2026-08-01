from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[3]
START = ROOT / 'scripts' / 'start.sh'
SLAM = ROOT / 'scripts' / 'run_slam.sh'


def run_script(*arguments: object) -> subprocess.CompletedProcess:
    return subprocess.run(
        tuple(str(argument) for argument in arguments),
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


class OperatorScriptFailureTest(unittest.TestCase):
    def test_start_rejects_conflicting_modes_before_docker(self) -> None:
        result = run_script(START, '--slam', '--map', 'saved')

        self.assertEqual(2, result.returncode)
        self.assertIn('mutually exclusive', result.stderr)

    def test_start_rejects_unsafe_map_name(self) -> None:
        result = run_script(START, '--map', '../private')

        self.assertEqual(2, result.returncode)
        self.assertIn('Map name may contain only', result.stderr)

    def test_start_rejects_missing_saved_map(self) -> None:
        result = run_script(START, '--map', 'definitely-not-present')

        self.assertEqual(1, result.returncode)
        self.assertIn('Saved map does not exist', result.stderr)

    def test_localize_requires_map_name(self) -> None:
        result = run_script(SLAM, 'localize')

        self.assertEqual(2, result.returncode)
        self.assertIn('localize MAP', result.stderr)

    def test_status_rejects_traversal_identifier(self) -> None:
        result = run_script(SLAM, 'status', '../private')

        self.assertEqual(2, result.returncode)
        self.assertIn('unsupported characters', result.stderr)


if __name__ == '__main__':
    unittest.main()
