import unittest

from omokai_fleet.model import Pose2D, RobotGoal, RobotId
from omokai_fleet.split_execution import (
    SplitReservationScheduler,
    compile_split_execution,
)


def batch(moving: RobotId, step: int):
    return tuple(
        RobotGoal(
            'executing_split',
            f'executing_split/step{step}/{robot_id.value}/'
            + ('room' if robot_id is moving else 'hold'),
            robot_id,
            Pose2D(float(step), float(index), 0.0),
        )
        for index, robot_id in enumerate(RobotId)
    )


class SplitExecutionPlanTest(unittest.TestCase):
    def test_separates_per_robot_work_from_reservations(self) -> None:
        plan = compile_split_execution(
            (
                batch(RobotId.ROBOT2, 1),
                batch(RobotId.ROBOT1, 2),
                batch(RobotId.ROBOT3, 3),
            )
        )

        self.assertEqual(
            (RobotId.ROBOT1, RobotId.ROBOT2, RobotId.ROBOT3),
            tuple(item.robot_id for item in plan.work),
        )
        self.assertTrue(
            all(not goal.goal_id.endswith('/hold') for item in plan.work for goal in item.goals)
        )
        self.assertEqual(
            ((RobotId.ROBOT2,), (RobotId.ROBOT1,), (RobotId.ROBOT3,)),
            tuple(item.robot_ids for item in plan.reservations),
        )

    def test_rejects_empty_and_hold_only_batches(self) -> None:
        with self.assertRaises(ValueError):
            compile_split_execution(())
        holds = tuple(
            RobotGoal('executing_split', f'x/{robot.value}/hold', robot, Pose2D(0, 0, 0))
            for robot in RobotId
        )
        with self.assertRaises(ValueError):
            compile_split_execution((holds,))

    def test_non_conflicting_routes_release_concurrently(self) -> None:
        plan = compile_split_execution(
            (
                batch(RobotId.ROBOT1, 1),
                batch(RobotId.ROBOT2, 2),
                batch(RobotId.ROBOT3, 3),
            )
        )
        scheduler = SplitReservationScheduler(plan)

        ready = scheduler.ready_goals()

        self.assertEqual(tuple(RobotId), tuple(goal.robot_id for goal in ready))
        scheduler.claim(ready)
        for robot_id in RobotId:
            scheduler.release(robot_id)
        self.assertTrue(scheduler.complete)

    def test_shared_dock_corridor_releases_one_robot_at_a_time(self) -> None:
        batches = tuple(
            tuple(
                RobotGoal(
                    'executing_split',
                    f'executing_split/outbound{step}/{robot_id.value}/'
                    + ('room' if robot_id is moving else 'hold'),
                    robot_id,
                    Pose2D(float(step), float(index), 0.0),
                )
                for index, robot_id in enumerate(RobotId)
            )
            for step, moving in enumerate(RobotId, start=1)
        )
        scheduler = SplitReservationScheduler(compile_split_execution(batches))

        first = scheduler.ready_goals()

        self.assertEqual((RobotId.ROBOT1,), tuple(goal.robot_id for goal in first))
        scheduler.claim(first)
        scheduler.release(RobotId.ROBOT1)
        second = scheduler.ready_goals()
        self.assertEqual((RobotId.ROBOT2,), tuple(goal.robot_id for goal in second))

    def test_private_route_failure_can_be_isolated_and_work_is_skipped(self) -> None:
        scheduler = SplitReservationScheduler(
            compile_split_execution(
                (
                    batch(RobotId.ROBOT1, 1),
                    batch(RobotId.ROBOT2, 2),
                    batch(RobotId.ROBOT3, 3),
                )
            )
        )
        ready = scheduler.ready_goals()
        scheduler.claim(ready)

        self.assertTrue(scheduler.failure_is_isolatable(RobotId.ROBOT2))
        skipped = scheduler.abandon(RobotId.ROBOT2)

        self.assertEqual(1, len(skipped))
        self.assertEqual((), scheduler.remaining_goals(RobotId.ROBOT2))
        self.assertNotIn(RobotId.ROBOT2, scheduler.active_robot_ids)

    def test_shared_corridor_failure_is_not_isolatable(self) -> None:
        batches = tuple(
            tuple(
                RobotGoal(
                    'executing_split',
                    f'executing_split/outbound{step}/{robot_id.value}/'
                    + ('room' if robot_id is moving else 'hold'),
                    robot_id,
                    Pose2D(float(step), float(index), 0.0),
                )
                for index, robot_id in enumerate(RobotId)
            )
            for step, moving in enumerate(RobotId, start=1)
        )
        scheduler = SplitReservationScheduler(compile_split_execution(batches))
        ready = scheduler.ready_goals()
        scheduler.claim(ready)

        self.assertFalse(scheduler.failure_is_isolatable(RobotId.ROBOT1))


if __name__ == '__main__':
    unittest.main()
