import pytest

from omokai_following.controller import FollowController
from omokai_following.domain import FollowConfig
from omokai_following.domain import FollowState
from omokai_following.domain import TargetObservation


def observation(
    now: float,
    range_m: float = 2.0,
    bearing: float = 0.0,
    obstacle: float | None = None,
) -> TargetObservation:
    return TargetObservation(now, now, range_m, bearing, obstacle, 1)


def test_controller_actively_scans_before_target() -> None:
    controller = FollowController()

    assert controller.start(0.0).command.linear_x == 0.0
    waiting = controller.update(0.1, None)

    assert waiting.state is FollowState.SEARCHING
    assert waiting.reason == 'active_search_scan'
    assert waiting.command.linear_x == 0.0
    assert waiting.command.angular_z == pytest.approx(0.40)


def test_follow_command_is_bounded_and_turning_slows_translation() -> None:
    config = FollowConfig(maximum_linear_mps=0.2, maximum_angular_rps=0.5)
    controller = FollowController(config)
    controller.start(0.0)

    straight = controller.update(0.1, observation(0.1, 4.0, 0.0))
    turning = controller.update(0.2, observation(0.2, 4.0, 0.60))

    assert straight.command.linear_x == pytest.approx(0.2)
    assert turning.command.linear_x < straight.command.linear_x
    assert turning.command.angular_z == pytest.approx(-0.5)


def test_controller_holds_standoff_without_reversing() -> None:
    controller = FollowController()
    controller.start(0.0)

    decision = controller.update(0.1, observation(0.1, 1.1, 0.1))

    assert decision.reason == 'holding_standoff'
    assert decision.command.linear_x == 0.0


def test_obstacle_emergency_stop_overrides_following() -> None:
    controller = FollowController()
    controller.start(0.0)

    stopped = controller.update(
        0.1, observation(0.1, range_m=3.0, obstacle=0.4)
    )

    assert stopped.reason == 'emergency_obstacle_stop'
    assert stopped.command.linear_x == 0.0
    assert stopped.command.angular_z == 0.0


def test_independent_obstacle_stop_works_before_target_confirmation() -> None:
    controller = FollowController()
    controller.start(0.0)

    stopped = controller.emergency_stop()

    assert stopped.state is FollowState.SEARCHING
    assert stopped.reason == 'emergency_obstacle_stop'
    assert stopped.command.linear_x == 0.0
    assert stopped.command.angular_z == 0.0


def test_stale_observation_enters_bounded_reacquisition_then_fails() -> None:
    config = FollowConfig(
        observation_timeout_sec=0.2,
        reacquisition_timeout_sec=1.0,
    )
    controller = FollowController(config)
    controller.start(0.0)
    controller.update(0.1, observation(0.1, bearing=-0.2))

    initial_loss = controller.update(0.4, None)
    reacquiring = controller.update(0.5, None)
    failed = controller.update(1.5, None)

    assert initial_loss.command.linear_x == 0.0
    assert initial_loss.command.angular_z == 0.0
    assert reacquiring.state is FollowState.REACQUIRING
    assert reacquiring.command.linear_x == 0.0
    assert reacquiring.command.angular_z > 0.0
    assert failed.state is FollowState.FAILED
    assert failed.command.linear_x == 0.0
    assert failed.command.angular_z == 0.0


def test_cancel_and_mission_timeout_always_zero_motion() -> None:
    controller = FollowController(FollowConfig(mission_timeout_sec=1.0))
    controller.start(0.0)
    timed_out = controller.update(1.1, observation(1.1))
    assert timed_out.state is FollowState.FAILED
    assert timed_out.command.linear_x == 0.0

    controller.start(2.0)
    cancelled = controller.cancel()
    assert cancelled.state is FollowState.CANCELLED
    assert cancelled.command.linear_x == 0.0


def test_loss_at_standoff_reuses_last_centering_direction() -> None:
    config = FollowConfig(
        observation_timeout_sec=0.2,
        reacquisition_timeout_sec=1.0,
    )
    controller = FollowController(config)
    controller.start(0.0)
    controller.update(0.1, observation(0.1, range_m=1.25, bearing=0.2))

    controller.update(0.4, None)
    reacquiring = controller.update(0.5, None)

    assert reacquiring.state is FollowState.REACQUIRING
    assert reacquiring.reason == 'bounded_reacquisition'
    assert reacquiring.command.linear_x == 0.0
    assert reacquiring.command.angular_z < 0.0


def test_reacquisition_reverses_instead_of_rotating_away() -> None:
    config = FollowConfig(
        observation_timeout_sec=0.2,
        reacquisition_timeout_sec=6.0,
        reacquisition_sweep_period_sec=2.0,
    )
    controller = FollowController(config)
    controller.start(0.0)
    controller.update(0.1, observation(0.1, bearing=0.2))

    controller.update(0.4, None)
    first_sweep = controller.update(0.5, None)
    reverse_sweep = controller.update(2.5, None)

    assert first_sweep.command.angular_z < 0.0
    assert reverse_sweep.command.angular_z > 0.0


def test_configuration_rejects_commands_above_safety_caps() -> None:
    with pytest.raises(ValueError, match='linear speed'):
        FollowConfig(maximum_linear_mps=0.41)
    with pytest.raises(ValueError, match='angular speed'):
        FollowConfig(maximum_angular_rps=0.81)
    with pytest.raises(ValueError, match='initial search speed'):
        FollowConfig(initial_search_angular_rps=0.41)
