import unittest

from omokai_fleet import (
    Formation,
    LifecycleError,
    RobotId,
    RobotOutcome,
    SquadAction,
    SquadLifecycle,
    SquadPlan,
    SquadState,
)


def plan(**overrides) -> SquadPlan:
    values = {
        'plan_id': 'fleet-lifecycle',
        'action': SquadAction.FORMATION_PATROL,
        'formation': Formation.WEDGE,
        'route_id': 'inspection_loop',
        'spacing_m': 0.8,
        'speed_mps': 0.12,
        'split_route': True,
        'regroup': True,
        'regroup_location': 'home',
    }
    values.update(overrides)
    return SquadPlan(**values)


def complete_phase(lifecycle: SquadLifecycle) -> None:
    for robot_id in RobotId:
        lifecycle.record_success(robot_id)


class LifecycleSuccessTest(unittest.TestCase):
    def test_starts_idle_and_accepts_once(self) -> None:
        lifecycle = SquadLifecycle()

        self.assertEqual(SquadState.IDLE, lifecycle.state)
        self.assertIsNone(lifecycle.snapshot().plan_id)
        lifecycle.accept(plan())
        self.assertEqual(SquadState.VALIDATED, lifecycle.state)

        with self.assertRaises(LifecycleError):
            lifecycle.accept(plan(plan_id='second'))

    def test_complete_four_phase_mission(self) -> None:
        lifecycle = SquadLifecycle()
        lifecycle.accept(plan())
        lifecycle.start_phase(SquadState.FORMING)
        complete_phase(lifecycle)
        lifecycle.start_phase(SquadState.FORMATION_MOVING)
        complete_phase(lifecycle)
        lifecycle.prepare_split()
        lifecycle.start_phase(SquadState.EXECUTING_SPLIT)
        complete_phase(lifecycle)
        lifecycle.start_phase(SquadState.REGROUPING)
        complete_phase(lifecycle)
        lifecycle.finish_success()

        snapshot = lifecycle.snapshot()
        self.assertEqual(SquadState.SUCCEEDED, snapshot.state)
        self.assertEqual(
            (
                SquadState.FORMING,
                SquadState.FORMATION_MOVING,
                SquadState.EXECUTING_SPLIT,
                SquadState.REGROUPING,
            ),
            tuple(record.phase for record in snapshot.history),
        )
        self.assertTrue(
            all(
                result.outcome is RobotOutcome.SUCCEEDED
                for record in snapshot.history
                for result in record.results
            )
        )

    def test_barrier_blocks_early_advance(self) -> None:
        lifecycle = SquadLifecycle()
        lifecycle.accept(plan())
        lifecycle.start_phase(SquadState.FORMING)
        lifecycle.record_success(RobotId.ROBOT1)

        with self.assertRaises(LifecycleError):
            lifecycle.start_phase(SquadState.FORMATION_MOVING)

    def test_result_order_is_stable_when_robots_finish_out_of_order(self) -> None:
        lifecycle = SquadLifecycle()
        lifecycle.accept(plan())
        lifecycle.start_phase(SquadState.FORMING)
        lifecycle.record_success(RobotId.ROBOT3)
        lifecycle.record_success(RobotId.ROBOT1)
        lifecycle.record_success(RobotId.ROBOT2)

        self.assertEqual(
            (RobotId.ROBOT1, RobotId.ROBOT2, RobotId.ROBOT3),
            tuple(item.robot_id for item in lifecycle.snapshot().current_results),
        )

    def test_duplicate_robot_result_is_rejected(self) -> None:
        lifecycle = SquadLifecycle()
        lifecycle.accept(plan())
        lifecycle.start_phase(SquadState.FORMING)
        lifecycle.record_success(RobotId.ROBOT1)

        with self.assertRaises(LifecycleError):
            lifecycle.record_success(RobotId.ROBOT1)

    def test_optional_split_and_regroup_can_be_skipped(self) -> None:
        lifecycle = SquadLifecycle()
        lifecycle.accept(plan(split_route=False, regroup=False))
        lifecycle.start_phase(SquadState.FORMING)
        complete_phase(lifecycle)
        lifecycle.start_phase(SquadState.FORMATION_MOVING)
        complete_phase(lifecycle)
        lifecycle.finish_success()

        self.assertEqual(SquadState.SUCCEEDED, lifecycle.state)


class LifecycleFailureTest(unittest.TestCase):
    def setUp(self) -> None:
        self.lifecycle = SquadLifecycle()
        self.lifecycle.accept(plan())
        self.lifecycle.start_phase(SquadState.FORMING)

    def test_one_failure_requires_other_active_goals_to_cancel(self) -> None:
        self.lifecycle.record_failure(
            RobotId.ROBOT2,
            'navigation aborted',
        )

        self.assertEqual(SquadState.CANCELLING, self.lifecycle.state)
        with self.assertRaises(LifecycleError):
            self.lifecycle.finish_cancellation()

        self.lifecycle.record_cancelled(RobotId.ROBOT1)
        self.lifecycle.record_cancelled(RobotId.ROBOT3)
        self.lifecycle.finish_cancellation()

        snapshot = self.lifecycle.snapshot()
        self.assertEqual(SquadState.FAILED, snapshot.state)
        self.assertEqual(
            (
                RobotOutcome.CANCELLED,
                RobotOutcome.FAILED,
                RobotOutcome.CANCELLED,
            ),
            tuple(
                item.outcome
                for item in snapshot.history[-1].results
            ),
        )

    def test_goal_timeout_finishes_squad_timed_out(self) -> None:
        self.lifecycle.record_failure(
            RobotId.ROBOT3,
            'goal exceeded 120 seconds',
            timed_out=True,
        )
        self.lifecycle.record_cancelled(RobotId.ROBOT1)
        self.lifecycle.record_cancelled(RobotId.ROBOT2)
        self.lifecycle.finish_cancellation()

        self.assertEqual(SquadState.TIMED_OUT, self.lifecycle.state)

    def test_operator_cancel_fans_out_then_finishes_cancelled(self) -> None:
        self.lifecycle.request_cancel()
        for robot_id in RobotId:
            self.lifecycle.record_cancelled(robot_id)
        self.lifecycle.finish_cancellation()

        self.assertEqual(SquadState.CANCELLED, self.lifecycle.state)

    def test_cancel_only_unresolved_robots_after_partial_success(self) -> None:
        self.lifecycle.record_success(RobotId.ROBOT1)
        self.lifecycle.request_cancel()
        self.lifecycle.record_cancelled(RobotId.ROBOT2)
        self.lifecycle.record_cancelled(RobotId.ROBOT3)
        self.lifecycle.finish_cancellation()

        outcomes = tuple(
            item.outcome
            for item in self.lifecycle.snapshot().history[-1].results
        )
        self.assertEqual(
            (
                RobotOutcome.SUCCEEDED,
                RobotOutcome.CANCELLED,
                RobotOutcome.CANCELLED,
            ),
            outcomes,
        )

    def test_late_callbacks_cannot_override_failure_cancellation(self) -> None:
        self.lifecycle.record_failure(RobotId.ROBOT2, 'navigation aborted')

        with self.assertRaises(LifecycleError):
            self.lifecycle.record_success(RobotId.ROBOT1)
        with self.assertRaises(LifecycleError):
            self.lifecycle.record_failure(RobotId.ROBOT3, 'late failure')
        with self.assertRaises(LifecycleError):
            self.lifecycle.request_cancel()

    def test_mission_timeout_requires_all_active_goals_to_resolve(self) -> None:
        self.lifecycle.request_timeout()
        for robot_id in RobotId:
            self.lifecycle.record_cancelled(
                robot_id,
                'cancelled after mission timeout',
            )
        self.lifecycle.finish_cancellation()

        snapshot = self.lifecycle.snapshot()
        self.assertEqual(SquadState.TIMED_OUT, snapshot.state)
        self.assertEqual('squad mission timed out', snapshot.reason)

    def test_unconfirmed_operator_cancellation_finishes_failed(self) -> None:
        self.lifecycle.request_cancel()
        self.lifecycle.record_cancellation_failed(
            RobotId.ROBOT1,
            'Nav2 rejected cancellation',
        )
        self.lifecycle.record_cancelled(RobotId.ROBOT2)
        self.lifecycle.record_cancelled(RobotId.ROBOT3)
        self.lifecycle.finish_cancellation()

        self.assertEqual(SquadState.FAILED, self.lifecycle.state)

    def test_cancel_before_dispatch_is_immediately_terminal(self) -> None:
        lifecycle = SquadLifecycle()
        lifecycle.accept(plan())
        lifecycle.request_cancel('operator cancelled before dispatch')

        self.assertEqual(SquadState.CANCELLED, lifecycle.state)


class LifecycleOrderTest(unittest.TestCase):
    def test_cannot_start_before_validation_or_out_of_order(self) -> None:
        lifecycle = SquadLifecycle()
        with self.assertRaises(LifecycleError):
            lifecycle.start_phase(SquadState.FORMING)

        lifecycle.accept(plan())
        with self.assertRaises(LifecycleError):
            lifecycle.start_phase(SquadState.REGROUPING)

    def test_prepare_split_requires_completed_movement(self) -> None:
        lifecycle = SquadLifecycle()
        lifecycle.accept(plan())
        lifecycle.start_phase(SquadState.FORMING)

        with self.assertRaises(LifecycleError):
            lifecycle.prepare_split()

    def test_finish_success_rejects_missing_required_phases(self) -> None:
        lifecycle = SquadLifecycle()
        lifecycle.accept(plan())
        lifecycle.start_phase(SquadState.FORMING)
        complete_phase(lifecycle)

        with self.assertRaises(LifecycleError):
            lifecycle.finish_success()


if __name__ == '__main__':
    unittest.main()
