from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[3]
START = ROOT / 'scripts' / 'start.sh'


def run_script(*arguments: object) -> subprocess.CompletedProcess:
    return subprocess.run(
        tuple(str(argument) for argument in arguments),
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


class OperatorScriptFailureTest(unittest.TestCase):
    def test_start_rejects_duplicate_multi_robot_mode(self) -> None:
        result = run_script(START, '--multi-agent', '--multi')

        self.assertEqual(2, result.returncode)
        self.assertIn('Only one multi-agent', result.stderr)

    def test_start_rejects_unavailable_slam_mode(self) -> None:
        result = run_script(START, '--slam')

        self.assertEqual(2, result.returncode)
        self.assertIn('Usage:', result.stderr)

    def test_start_help_documents_multi_agent_mode(self) -> None:
        result = run_script(START, '--help')

        self.assertEqual(0, result.returncode)
        self.assertIn('--multi-agent', result.stdout)


if __name__ == '__main__':
    unittest.main()
