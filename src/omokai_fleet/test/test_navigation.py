import unittest

from omokai_fleet import (
    Formation,
    NavigationBatch,
    NavigationResult,
    NavigationStatus,
    RobotId,
    RobotOutcome,
    SquadAction,
    SquadLifecycle,
    SquadPlan,
    SquadState,
    apply_navigation_batch,
    is_hold_goal_id,
)


def lifecycle() -> SquadLifecycle:
    instance = SquadLifecycle()
    instance.accept(
        SquadPlan(
            plan_id='navigation-batch',
            action=SquadAction.FORMATION_PATROL,
            formation=Formation.WEDGE,
            route_id='inspection_loop',
            spacing_m=0.8,
            speed_mps=0.12,
            split_route=True,
            regroup=True,
            regroup_location='home',
        )
    )
    instance.start_phase(SquadState.FORMING)
    return instance


def result(
    robot_id: RobotId,
    status: NavigationStatus,
    reason: str = '',
) -> NavigationResult:
    return NavigationResult(robot_id, status, reason)


class NavigationBatchTest(unittest.TestCase):
    def test_hold_goal_id_marks_only_explicit_hold_reservations(self) -> None:
        self.assertTrue(is_hold_goal_id('executing_split/outbound/robot2/hold'))
        self.assertFalse(is_hold_goal_id('executing_split/outbound/robot2/room'))
        self.assertFalse(is_hold_goal_id('hold'))
        self.assertFalse(is_hold_goal_id(None))

    def test_success_batch_completes_active_barrier(self) -> None:
        instance = lifecycle()
        batch = NavigationBatch(
            tuple(
                result(robot_id, NavigationStatus.SUCCEEDED)
                for robot_id in RobotId
            )
        )

        apply_navigation_batch(instance, batch)

        self.assertTrue(batch.succeeded)
        self.assertTrue(instance.barrier_complete)
        self.assertEqual(SquadState.FORMING, instance.state)

    def test_failure_cancels_other_active_robots(self) -> None:
        instance = lifecycle()
        batch = NavigationBatch(
            (
                result(
                    RobotId.ROBOT1,
                    NavigationStatus.CANCELLED,
                    'cancelled after robot2 failed',
                ),
                result(
                    RobotId.ROBOT2,
                    NavigationStatus.FAILED,
                    'Nav2 aborted',
                ),
                result(
                    RobotId.ROBOT3,
                    NavigationStatus.CANCELLED,
                    'cancelled after robot2 failed',
                ),
            )
        )

        apply_navigation_batch(instance, batch)

        snapshot = instance.snapshot()
        self.assertEqual(SquadState.FAILED, snapshot.state)
        self.assertEqual(
            (
                RobotOutcome.CANCELLED,
                RobotOutcome.FAILED,
                RobotOutcome.CANCELLED,
            ),
            tuple(item.outcome for item in snapshot.history[-1].results),
        )

    def test_partial_success_is_preserved_on_timeout(self) -> None:
        instance = lifecycle()
        batch = NavigationBatch(
            (
                result(RobotId.ROBOT1, NavigationStatus.SUCCEEDED),
                result(
                    RobotId.ROBOT2,
                    NavigationStatus.TIMED_OUT,
                    'goal timeout',
                ),
                result(
                    RobotId.ROBOT3,
                    NavigationStatus.CANCELLED,
                    'cancelled after robot2 timed out',
                ),
            )
        )

        apply_navigation_batch(instance, batch)

        snapshot = instance.snapshot()
        self.assertEqual(SquadState.TIMED_OUT, snapshot.state)
        self.assertEqual(
            RobotOutcome.SUCCEEDED,
            snapshot.history[-1].results[0].outcome,
        )

    def test_operator_cancel_batch_finishes_cancelled(self) -> None:
        instance = lifecycle()
        batch = NavigationBatch(
            tuple(
                result(
                    robot_id,
                    NavigationStatus.CANCELLED,
                    'operator cancelled',
                )
                for robot_id in RobotId
            )
        )

        apply_navigation_batch(instance, batch)

        self.assertEqual(SquadState.CANCELLED, instance.state)

    def test_unconfirmed_cancellation_cannot_report_cancelled(self) -> None:
        instance = lifecycle()
        batch = NavigationBatch(
            (
                result(
                    RobotId.ROBOT1,
                    NavigationStatus.CANCEL_FAILED,
                    'cancel rejected',
                ),
                result(
                    RobotId.ROBOT2,
                    NavigationStatus.CANCELLED,
                    'operator cancelled',
                ),
                result(
                    RobotId.ROBOT3,
                    NavigationStatus.CANCELLED,
                    'operator cancelled',
                ),
            )
        )

        apply_navigation_batch(instance, batch)

        self.assertEqual(SquadState.FAILED, instance.state)

    def test_rejects_wrong_order_and_multiple_primary_failures(self) -> None:
        with self.assertRaises(ValueError):
            NavigationBatch(
                (
                    result(RobotId.ROBOT2, NavigationStatus.SUCCEEDED),
                    result(RobotId.ROBOT1, NavigationStatus.SUCCEEDED),
                    result(RobotId.ROBOT3, NavigationStatus.SUCCEEDED),
                )
            )
        with self.assertRaises(ValueError):
            NavigationBatch(
                (
                    result(
                        RobotId.ROBOT1,
                        NavigationStatus.FAILED,
                        'first',
                    ),
                    result(
                        RobotId.ROBOT2,
                        NavigationStatus.REJECTED,
                        'second',
                    ),
                    result(
                        RobotId.ROBOT3,
                        NavigationStatus.CANCELLED,
                        'cancelled',
                    ),
                )
            )

    def test_non_success_requires_reason(self) -> None:
        with self.assertRaises(ValueError):
            result(RobotId.ROBOT1, NavigationStatus.FAILED)


if __name__ == '__main__':
    unittest.main()
