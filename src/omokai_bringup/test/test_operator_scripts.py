from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[3]
START = ROOT / 'scripts' / 'start.sh'
RUN = ROOT / 'scripts' / 'run.sh'
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
    def test_consolidated_runner_requires_one_challenge(self) -> None:
        result = run_script(RUN, 'status')

        self.assertEqual(2, result.returncode)
        self.assertIn('--perception', result.stderr)

    def test_consolidated_runner_rejects_conflicting_challenges(self) -> None:
        result = run_script(RUN, '--slam', '--perception', 'start')

        self.assertEqual(2, result.returncode)
        self.assertIn('exactly one', result.stderr)

    def test_consolidated_runner_selects_only_slam_launch(self) -> None:
        result = run_script(RUN, '--slam', 'start', '--print-config')

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn('mode=slam', result.stdout)
        self.assertIn('launch_file=slam_navigation.launch.py', result.stdout)

    def test_consolidated_runner_selects_only_multi_launch(self) -> None:
        result = run_script(
            RUN, '--multi-agent', 'start', '--print-config'
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn('mode=multi', result.stdout)
        self.assertIn(
            'launch_file=multi_robot_simulation.launch.py', result.stdout
        )

    def test_perception_defaults_to_translating_actor(self) -> None:
        result = run_script(
            RUN, '--perception', 'start', '--print-config'
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn('mode=vision', result.stdout)
        self.assertIn('launch_file=vision_rgbd.launch.py', result.stdout)
        self.assertIn('actor_motion=true', result.stdout)

    def test_perception_stationary_actor_override(self) -> None:
        result = run_script(
            RUN,
            '--perception',
            'start',
            '--stationary-actor',
            '--print-config',
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn('actor_motion=false', result.stdout)

    def test_start_rejects_conflicting_modes_before_docker(self) -> None:
        result = run_script(START, '--slam', '--map', 'saved')

        self.assertEqual(2, result.returncode)
        self.assertIn('mutually exclusive', result.stderr)

    def test_start_rejects_multi_robot_with_another_mode(self) -> None:
        result = run_script(START, '--multi', '--slam')

        self.assertEqual(2, result.returncode)
        self.assertIn('mutually exclusive', result.stderr)

    def test_start_rejects_vision_with_another_mode(self) -> None:
        result = run_script(START, '--vision', '--multi')

        self.assertEqual(2, result.returncode)
        self.assertIn('mutually exclusive', result.stderr)

    def test_moving_actor_is_rejected_outside_vision_mode(self) -> None:
        result = run_script(START, '--moving-actor')

        self.assertEqual(2, result.returncode)
        self.assertIn('valid only with --perception', result.stderr)

    def test_stationary_actor_is_rejected_outside_vision_mode(self) -> None:
        result = run_script(START, '--stationary-actor')

        self.assertEqual(2, result.returncode)
        self.assertIn('valid only with --perception', result.stderr)

    def test_red_actor_is_rejected_outside_vision_mode(self) -> None:
        result = run_script(START, '--red-actor')

        self.assertEqual(2, result.returncode)
        self.assertIn('valid only with --perception', result.stderr)

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
