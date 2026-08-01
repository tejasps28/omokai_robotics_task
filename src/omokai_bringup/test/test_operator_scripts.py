from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[3]
START = ROOT / 'scripts' / 'start.sh'
VISION = ROOT / 'scripts' / 'run_vision.sh'


def run_script(*arguments: object) -> subprocess.CompletedProcess:
    return subprocess.run(
        tuple(str(argument) for argument in arguments),
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def test_vision_mode_selects_only_the_vision_launch() -> None:
    result = run_script(START, '--perception', '--print-config')

    assert result.returncode == 0
    assert 'mode=vision' in result.stdout
    assert 'launch_file=vision_rgbd.launch.py' in result.stdout
    assert 'actor_motion=true' in result.stdout


def test_stationary_and_red_actor_flags_are_explicit() -> None:
    result = run_script(
        START,
        '--vision',
        '--stationary-actor',
        '--red-actor',
        '--print-config',
    )

    assert result.returncode == 0
    assert 'actor_motion=false' in result.stdout
    assert 'red_actor=true' in result.stdout


def test_actor_flags_are_rejected_outside_vision_mode() -> None:
    result = run_script(START, '--red-actor', '--print-config')

    assert result.returncode == 2
    assert 'valid only with --perception' in result.stderr


def test_unavailable_challenge_modes_are_rejected() -> None:
    for mode in ('--slam', '--multi-agent', '--map'):
        result = run_script(START, mode, '--print-config')
        assert result.returncode == 2
        assert 'Usage:' in result.stderr


def test_vision_wrapper_rejects_unknown_commands_before_docker() -> None:
    result = run_script(VISION, 'unknown-command')

    assert result.returncode == 2
    assert 'Usage:' in result.stderr
