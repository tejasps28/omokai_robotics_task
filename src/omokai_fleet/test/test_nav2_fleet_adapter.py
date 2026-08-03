"""ROS-runtime adapter tests; skipped on the minimal host test environment."""

from __future__ import annotations

import unittest
from unittest.mock import patch

try:
    from omokai_fleet.nav2_fleet_adapter import FleetNav2Adapter
    ROS_ADAPTER_AVAILABLE = True
except ModuleNotFoundError:
    ROS_ADAPTER_AVAILABLE = False

from omokai_fleet.model import Pose2D, RobotGoal, RobotId
from omokai_fleet.split_execution import (
    SplitReservationScheduler,
    compile_split_execution,
)


class _Publisher:
    def publish(self, message) -> None:
        del message


class _Node:
    def __init__(self) -> None:
        self.pending = []

    def create_publisher(self, *args):
        del args
        return _Publisher()

    def create_subscription(self, *args):
        del args
        return object()


class _Adapter:
    def __init__(self, node: _Node, owner, robot_id: RobotId) -> None:
        self.node = node
        self.owner = owner
        self.robot_id = robot_id
        self.dispatched = []
        self.auto_succeed = True
        self.failure_reason = ''

    def server_is_ready(self) -> bool:
        return True

    def dispatch(self, goal, speed_mps: float) -> str:
        self.dispatched.append((goal, speed_mps))
        token = f'{self.robot_id.value}-{len(self.dispatched)}'
        if self.failure_reason:
            self.node.pending.append(
                lambda: self.owner._navigation_failed(
                    self.robot_id, token, self.failure_reason
                )
            )
        elif self.auto_succeed:
            self.node.pending.append(
                lambda: self.owner._navigation_succeeded(self.robot_id, token)
            )
        return token

    def cancel(self, token: str) -> None:
        self.node.pending.append(
            lambda: self.owner._cancellation_confirmed(self.robot_id, token)
        )


@unittest.skipUnless(ROS_ADAPTER_AVAILABLE, 'ROS adapter dependencies unavailable')
class FleetNav2AdapterHoldTest(unittest.TestCase):
    def setUp(self) -> None:
        self.node = _Node()
        placeholders = {robot_id: object() for robot_id in RobotId}
        self.adapter = FleetNav2Adapter(self.node, adapters=placeholders)
        adapters = {
            robot_id: _Adapter(self.node, self.adapter, robot_id)
            for robot_id in RobotId
        }
        self.adapter._adapters = adapters
        self.spin_patch = patch(
            'omokai_fleet.nav2_fleet_adapter.rclpy.spin_once',
            self._spin_once,
        )
        self.spin_patch.start()
        self.addCleanup(self.spin_patch.stop)

    def _spin_once(self, node, timeout_sec: float = 0.0) -> None:
        self.assertIs(self.node, node)
        del timeout_sec
        if self.node.pending:
            self.node.pending.pop(0)()

    @staticmethod
    def _goals(*suffixes: str) -> tuple[RobotGoal, ...]:
        return tuple(
            RobotGoal(
                phase_id='executing_split',
                goal_id=f'executing_split/step/{robot_id.value}/{suffix}',
                robot_id=robot_id,
                pose=Pose2D(float(index), 0.0, 0.0),
            )
            for index, (robot_id, suffix) in enumerate(
                zip(RobotId, suffixes),
            )
        )

    def test_holds_are_not_dispatched_but_moving_goal_is(self) -> None:
        result = self.adapter.execute(
            self._goals('room', 'hold', 'hold'),
            speed_mps=0.12,
            timeout_sec=1.0,
        )

        self.assertTrue(result.succeeded)
        self.assertEqual(1, len(self.adapter._adapters[RobotId.ROBOT1].dispatched))
        self.assertEqual(0, len(self.adapter._adapters[RobotId.ROBOT2].dispatched))
        self.assertEqual(0, len(self.adapter._adapters[RobotId.ROBOT3].dispatched))

    def test_regular_goals_are_all_dispatched(self) -> None:
        result = self.adapter.execute(
            self._goals('room', 'room', 'room'),
            speed_mps=0.12,
            timeout_sec=1.0,
        )

        self.assertTrue(result.succeeded)
        self.assertEqual(
            (1, 1, 1),
            tuple(len(self.adapter._adapters[robot_id].dispatched) for robot_id in RobotId),
        )

    def test_safety_stop_cancels_the_only_dispatched_goal(self) -> None:
        checks = iter((None, 'test separation stop'))
        self.adapter._safety_check = lambda: next(checks, 'test separation stop')
        self.adapter._adapters[RobotId.ROBOT1].auto_succeed = False

        result = self.adapter.execute(
            self._goals('room', 'hold', 'hold'),
            speed_mps=0.12,
            timeout_sec=1.0,
        )

        self.assertEqual('failed', result.results[0].status.value)
        self.assertEqual('succeeded', result.results[1].status.value)
        self.assertEqual('succeeded', result.results[2].status.value)

    def test_split_isolates_private_failure_and_finishes_survivors(self) -> None:
        batches = tuple(
            self._goals(*suffixes)
            for suffixes in (
                ('room1', 'hold', 'hold'),
                ('room2', 'hold', 'hold'),
                ('hold', 'room3', 'hold'),
                ('hold', 'hold', 'room4'),
            )
        )
        self.adapter._adapters[RobotId.ROBOT2].failure_reason = 'path blocked'

        result = self.adapter.execute_split(
            compile_split_execution(batches),
            speed_mps=0.12,
            timeout_sec=1.0,
        )

        self.assertEqual(
            ('succeeded', 'failed', 'succeeded'),
            tuple(item.status.value for item in result.results),
        )
        self.assertEqual(2, result.results[0].completed_goals)
        self.assertEqual(1, result.results[1].skipped_goals)

    def test_split_advances_robot_without_waiting_for_sibling_barrier(self) -> None:
        batches = tuple(
            self._goals(*suffixes)
            for suffixes in (
                ('room1', 'hold', 'hold'),
                ('room2', 'hold', 'hold'),
                ('hold', 'room3', 'hold'),
                ('hold', 'hold', 'room4'),
            )
        )

        result = self.adapter.execute_split(
            compile_split_execution(batches),
            speed_mps=0.12,
            timeout_sec=1.0,
        )

        self.assertTrue(result.succeeded)
        self.assertEqual(2, len(self.adapter._adapters[RobotId.ROBOT1].dispatched))

    def test_shared_corridor_failure_stops_undispatched_siblings(self) -> None:
        batches = tuple(
            self._goals(*suffixes)
            for suffixes in (
                ('outbound-room1', 'hold', 'hold'),
                ('hold', 'outbound-room2', 'hold'),
                ('hold', 'hold', 'outbound-room3'),
            )
        )
        self.adapter._adapters[RobotId.ROBOT1].failure_reason = 'corridor blocked'

        result = self.adapter.execute_split(
            compile_split_execution(batches),
            speed_mps=0.12,
            timeout_sec=1.0,
        )

        self.assertEqual(
            ('failed', 'cancelled', 'cancelled'),
            tuple(item.status.value for item in result.results),
        )
        self.assertEqual(0, len(self.adapter._adapters[RobotId.ROBOT2].dispatched))
        self.assertEqual(0, len(self.adapter._adapters[RobotId.ROBOT3].dispatched))

    def test_departure_queue_releases_after_one_metre_of_live_travel(self) -> None:
        batches = tuple(
            self._goals(*suffixes)
            for suffixes in (
                ('outbound-room1', 'hold', 'hold'),
                ('hold', 'outbound-room2', 'hold'),
                ('hold', 'hold', 'outbound-room3'),
            )
        )
        scheduler = SplitReservationScheduler(compile_split_execution(batches))
        first = scheduler.ready_goals()
        scheduler.claim(first)
        self.adapter._split_scheduler = scheduler
        self.adapter._split_departure_origins = {
            RobotId.ROBOT1: Pose2D(0.0, 0.0, 0.0),
        }
        poses = {RobotId.ROBOT1: Pose2D(0.99, 0.0, 0.0)}
        self.adapter._pose_lookup = poses.get

        self.adapter._release_cleared_departures()
        self.assertEqual((), scheduler.ready_goals())

        poses[RobotId.ROBOT1] = Pose2D(1.0, 0.0, 0.0)
        self.adapter._release_cleared_departures()

        self.assertEqual(
            (RobotId.ROBOT2,),
            tuple(goal.robot_id for goal in scheduler.ready_goals()),
        )
        self.assertIn(RobotId.ROBOT1, scheduler.active_robot_ids)

    def test_operator_cancel_stops_queued_split_dispatch(self) -> None:
        batches = tuple(
            self._goals(*suffixes)
            for suffixes in (
                ('outbound-room1', 'hold', 'hold'),
                ('hold', 'outbound-room2', 'hold'),
                ('hold', 'hold', 'outbound-room3'),
            )
        )
        scheduler = SplitReservationScheduler(compile_split_execution(batches))
        first = scheduler.ready_goals()
        scheduler.claim(first)
        self.adapter._split_scheduler = scheduler
        self.adapter._pending = {RobotId.ROBOT1}
        self.adapter._tokens = {RobotId.ROBOT1: 'robot1-1'}

        self.adapter._begin_operator_cancel()

        self.assertTrue(self.adapter._split_stop_requested)
        self.assertEqual(
            'cancelled',
            self.adapter._cancel_status[RobotId.ROBOT1].value,
        )


if __name__ == '__main__':
    unittest.main()
