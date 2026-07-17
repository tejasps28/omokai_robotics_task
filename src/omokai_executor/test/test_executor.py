import unittest

from omokai_executor import (
    ExecutionGoal,
    ExecutionPlan,
    ExecutorConfig,
    ExecutorState,
    GoalKind,
    GoalPose,
    MissionExecutor,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def monotonic(self) -> float:
        return self.now

    def utc_now(self) -> str:
        return f'2026-07-02T00:00:{int(self.now):02d}Z'


class FakeNavigation:
    def __init__(self) -> None:
        self.dispatched = []
        self.cancelled = []
        self.dispatch_error = None

    def dispatch(self, goal, speed_mps):
        if self.dispatch_error:
            raise self.dispatch_error
        handle = f'handle-{len(self.dispatched)}'
        self.dispatched.append((handle, goal, speed_mps))
        return handle

    def cancel(self, goal_handle):
        self.cancelled.append(goal_handle)


class FakeEventSink:
    def __init__(self) -> None:
        self.events = []

    def record(self, event) -> None:
        self.events.append(event)


def plan() -> ExecutionPlan:
    return ExecutionPlan(
        mission_id='mission-1',
        goals=(
            ExecutionGoal('route-0', GoalPose(1.0, 0.0, 0.0)),
            ExecutionGoal(
                'home', GoalPose(0.0, 0.0, 0.0), kind=GoalKind.HOME
            ),
        ),
        speed_mps=0.18,
    )


class MissionExecutorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeClock()
        self.navigation = FakeNavigation()
        self.events = FakeEventSink()
        self.executor = MissionExecutor(
            'mission-1',
            self.navigation,
            self.events,
            config=ExecutorConfig(goal_timeout_sec=5.0, max_retries=1),
            clock=self.clock,
        )

    def start(self) -> str:
        self.executor.begin_compilation()
        self.executor.load_plan(plan())
        return self.executor.active_goal_handle

    def test_executes_route_then_home(self) -> None:
        first_handle = self.start()
        self.assertEqual(ExecutorState.EXECUTING, self.executor.state)

        self.executor.navigation_succeeded(first_handle)
        second_handle = self.executor.active_goal_handle
        self.assertEqual(ExecutorState.RETURNING_HOME, self.executor.state)

        self.executor.navigation_succeeded(second_handle)
        self.assertEqual(ExecutorState.SUCCEEDED, self.executor.state)
        self.assertEqual(
            ['route-0', 'home'],
            [item[1].goal_id for item in self.navigation.dispatched],
        )

    def test_retries_same_goal_once_then_fails(self) -> None:
        first_handle = self.start()
        self.executor.navigation_failed(first_handle, 'blocked')
        retry_handle = self.executor.active_goal_handle

        self.assertEqual('route-0', self.navigation.dispatched[1][1].goal_id)
        self.executor.navigation_failed(retry_handle, 'still_blocked')
        self.assertEqual(ExecutorState.FAILED, self.executor.state)
        self.assertEqual(2, len(self.navigation.dispatched))

    def test_cancels_active_navigation(self) -> None:
        handle = self.start()
        self.assertTrue(self.executor.request_cancel())
        self.assertEqual(ExecutorState.CANCELLING, self.executor.state)
        self.assertEqual([handle], self.navigation.cancelled)

        self.executor.cancellation_confirmed(handle)
        self.assertEqual(ExecutorState.CANCELLED, self.executor.state)

    def test_cancel_before_compilation_is_terminal(self) -> None:
        self.assertTrue(self.executor.request_cancel())
        self.assertEqual(ExecutorState.CANCELLED, self.executor.state)
        self.assertFalse(self.executor.request_cancel())

    def test_timeout_cancels_goal_and_stops_dispatch(self) -> None:
        handle = self.start()
        self.clock.now = 15.0

        self.assertTrue(self.executor.tick())
        self.assertEqual(ExecutorState.TIMED_OUT, self.executor.state)
        self.assertEqual([handle], self.navigation.cancelled)
        self.assertEqual(1, len(self.navigation.dispatched))

    def test_tick_before_deadline_has_no_effect(self) -> None:
        self.start()
        self.clock.now = 14.99
        self.assertFalse(self.executor.tick())
        self.assertEqual(ExecutorState.EXECUTING, self.executor.state)

    def test_rejects_result_for_wrong_handle(self) -> None:
        self.start()
        with self.assertRaises(ValueError):
            self.executor.navigation_succeeded('stale-handle')
        self.assertEqual(ExecutorState.EXECUTING, self.executor.state)

    def test_dispatch_error_fails_without_active_goal(self) -> None:
        self.navigation.dispatch_error = RuntimeError('adapter unavailable')
        self.executor.begin_compilation()
        self.executor.load_plan(plan())
        self.assertEqual(ExecutorState.FAILED, self.executor.state)
        self.assertIsNone(self.executor.active_goal_handle)

    def test_events_have_contiguous_sequence_numbers(self) -> None:
        handle = self.start()
        self.executor.navigation_succeeded(handle)
        self.assertEqual(
            list(range(1, len(self.events.events) + 1)),
            [event.sequence for event in self.events.events],
        )

    def test_terminal_executor_rejects_late_navigation_result(self) -> None:
        first_handle = self.start()
        self.executor.navigation_succeeded(first_handle)
        second_handle = self.executor.active_goal_handle
        self.executor.navigation_succeeded(second_handle)

        with self.assertRaises(RuntimeError):
            self.executor.navigation_succeeded(second_handle)
        self.assertEqual(ExecutorState.SUCCEEDED, self.executor.state)


class ExecutionPlanTest(unittest.TestCase):
    def test_rejects_duplicate_goal_ids(self) -> None:
        goal = ExecutionGoal('same', GoalPose(0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            ExecutionPlan('mission-1', (goal, goal), 0.1)

    def test_rejects_non_map_goal(self) -> None:
        with self.assertRaises(ValueError):
            GoalPose(0.0, 0.0, 0.0, frame_id='odom')

    def test_rejects_mutable_goal_sequence(self) -> None:
        goal = ExecutionGoal('route-0', GoalPose(0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            ExecutionPlan('mission-1', [goal], 0.1)

    def test_rejects_non_integer_retry_limit(self) -> None:
        with self.assertRaises(ValueError):
            ExecutorConfig(max_retries=1.5)


if __name__ == '__main__':
    unittest.main()
