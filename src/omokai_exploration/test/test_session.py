import unittest

from omokai_exploration import (
    ExplorationConfig,
    ExplorationSession,
    ExplorationState,
    FrontierConfig,
    GridMetadata,
    OccupancyGrid,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def monotonic(self) -> float:
        return self.now


class FakeNavigation:
    def __init__(self) -> None:
        self.dispatched = []
        self.cancelled = []
        self.dispatch_error = None
        self.cancel_error = None

    def dispatch(self, candidate):
        if self.dispatch_error is not None:
            raise self.dispatch_error
        handle = f'goal-{len(self.dispatched)}'
        self.dispatched.append((handle, candidate))
        return handle

    def cancel(self, goal_handle):
        if self.cancel_error is not None:
            raise self.cancel_error
        self.cancelled.append(goal_handle)


def grid_from_rows(rows) -> OccupancyGrid:
    return OccupancyGrid(
        GridMetadata(
            width=len(rows[0]),
            height=len(rows),
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
        ),
        (value for row in rows for value in row),
    )


def frontier_grid() -> OccupancyGrid:
    return grid_from_rows(
        [
            [-1, -1, -1, -1, -1],
            [-1, 0, -1, 0, -1],
            [100, 100, 100, 100, 100],
        ]
    )


def complete_grid() -> OccupancyGrid:
    return grid_from_rows(
        [
            [0, 0, 0],
            [0, 0, 0],
        ]
    )


def config(**changes) -> ExplorationConfig:
    values = {
        'frontier': FrontierConfig(
            min_cluster_size=1,
            clearance_m=0.0,
            blacklist_radius_m=1.0,
        ),
        'goal_timeout_sec': 5.0,
        'mission_timeout_sec': 30.0,
        'map_timeout_sec': 4.0,
        'cancel_timeout_sec': 2.0,
        'completion_confirmations': 2,
        'max_failed_goals': 2,
    }
    return ExplorationConfig(**(values | changes))


class ExplorationConfigTest(unittest.TestCase):
    def test_rejects_invalid_timeouts_and_limits(self) -> None:
        for changes in (
            {'frontier': None},
            {'goal_timeout_sec': 0.0},
            {'mission_timeout_sec': -1.0},
            {'map_timeout_sec': True},
            {'cancel_timeout_sec': float('inf')},
            {'completion_confirmations': 0},
            {'max_failed_goals': 1.5},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                ExplorationConfig(**changes)


class ExplorationSessionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeClock()
        self.navigation = FakeNavigation()
        self.session = ExplorationSession(
            'exploration-1',
            self.navigation,
            config=config(),
            clock=self.clock,
        )

    def start_and_dispatch(self) -> str:
        self.session.start()
        self.assertTrue(self.session.observe_map(frontier_grid(), 2.5, 1.5))
        return self.session.active_goal_handle

    def test_dispatches_frontier_then_waits_for_new_map_after_success(self) -> None:
        handle = self.start_and_dispatch()

        self.assertEqual(ExplorationState.NAVIGATING, self.session.state)
        self.session.navigation_succeeded(handle)

        self.assertEqual(
            ExplorationState.WAITING_FOR_MAP,
            self.session.state,
        )
        self.assertIsNone(self.session.active_goal_handle)
        self.assertEqual(1, len(self.navigation.dispatched))

    def test_requires_repeated_empty_maps_before_completion(self) -> None:
        self.session.start()

        self.assertFalse(self.session.observe_map(complete_grid(), 0.5, 0.5))
        self.assertEqual(
            ExplorationState.WAITING_FOR_MAP,
            self.session.state,
        )
        self.assertFalse(self.session.observe_map(complete_grid(), 0.5, 0.5))

        self.assertEqual(ExplorationState.COMPLETED, self.session.state)
        self.assertEqual('no_frontiers', self.session.result.reason)

    def test_failed_frontier_is_blacklisted_before_next_selection(self) -> None:
        first_handle = self.start_and_dispatch()
        first_goal = self.navigation.dispatched[0][1]
        self.session.navigation_failed(first_handle, 'blocked')

        self.assertEqual(
            ((first_goal.goal_x, first_goal.goal_y),),
            self.session.blacklist,
        )
        self.assertTrue(self.session.observe_map(frontier_grid(), 2.5, 1.5))
        second_goal = self.navigation.dispatched[1][1]
        self.assertNotEqual(first_goal.goal, second_goal.goal)

    def test_failure_limit_stops_exploration(self) -> None:
        first_handle = self.start_and_dispatch()
        self.session.navigation_failed(first_handle, 'blocked')
        self.session.observe_map(frontier_grid(), 2.5, 1.5)
        second_handle = self.session.active_goal_handle
        self.session.navigation_failed(second_handle, 'blocked_again')

        self.assertEqual(ExplorationState.FAILED, self.session.state)
        self.assertEqual('failure_limit', self.session.result.reason)
        self.assertEqual(2, self.session.result.failed_goals)

    def test_goal_timeout_cancels_then_blacklists_and_continues(self) -> None:
        handle = self.start_and_dispatch()
        self.clock.now = 15.0

        self.assertTrue(self.session.tick())
        self.assertEqual(ExplorationState.CANCELLING, self.session.state)
        self.assertEqual([handle], self.navigation.cancelled)

        self.session.cancellation_confirmed(handle)
        self.assertEqual(
            ExplorationState.WAITING_FOR_MAP,
            self.session.state,
        )
        self.assertEqual(1, len(self.session.blacklist))

    def test_mission_timeout_while_waiting_is_terminal(self) -> None:
        self.session.start()
        self.clock.now = 40.0

        self.assertTrue(self.session.tick())
        self.assertEqual(ExplorationState.TIMED_OUT, self.session.state)
        self.assertEqual('mission_timeout', self.session.result.reason)

    def test_mission_timeout_cancels_active_goal_before_terminal_result(self) -> None:
        handle = self.start_and_dispatch()
        self.clock.now = 40.0

        self.assertTrue(self.session.tick())
        self.assertEqual(ExplorationState.CANCELLING, self.session.state)
        self.assertEqual([handle], self.navigation.cancelled)

        self.session.cancellation_confirmed(handle)
        self.assertEqual(ExplorationState.TIMED_OUT, self.session.state)

    def test_mission_timeout_still_bounds_unconfirmed_cancellation(self) -> None:
        self.start_and_dispatch()
        self.clock.now = 40.0
        self.session.tick()
        self.clock.now = 42.0

        self.assertTrue(self.session.tick())
        self.assertEqual(ExplorationState.FAILED, self.session.state)
        self.assertEqual('cancellation_timeout', self.session.result.reason)

    def test_map_timeout_fails_without_dispatching(self) -> None:
        self.session.start()
        self.clock.now = 14.0

        self.assertTrue(self.session.tick())
        self.assertEqual(ExplorationState.FAILED, self.session.state)
        self.assertEqual('map_timeout', self.session.result.reason)
        self.assertEqual([], self.navigation.dispatched)

    def test_operator_can_cancel_waiting_or_active_session(self) -> None:
        self.session.start()
        self.assertTrue(self.session.request_cancel())
        self.assertEqual(ExplorationState.CANCELLED, self.session.state)

        active = ExplorationSession(
            'exploration-2',
            self.navigation,
            config=config(),
            clock=self.clock,
        )
        active.start()
        active.observe_map(frontier_grid(), 2.5, 1.5)
        handle = active.active_goal_handle
        self.assertTrue(active.request_cancel())
        self.assertEqual(ExplorationState.CANCELLING, active.state)
        active.cancellation_confirmed(handle)
        self.assertEqual(ExplorationState.CANCELLED, active.state)

    def test_cancellation_error_is_terminal_failure(self) -> None:
        handle = self.start_and_dispatch()
        self.navigation.cancel_error = RuntimeError('adapter unavailable')

        self.assertFalse(self.session.request_cancel())
        self.assertEqual(ExplorationState.FAILED, self.session.state)
        self.assertEqual(
            'navigation_cancel_error',
            self.session.result.reason,
        )
        self.assertIsNone(self.session.active_goal_handle)
        self.assertNotIn(handle, self.navigation.cancelled)

    def test_cancellation_confirmation_timeout_is_failure(self) -> None:
        self.start_and_dispatch()
        self.session.request_cancel()
        self.clock.now += 2.0

        self.assertTrue(self.session.tick())
        self.assertEqual(ExplorationState.FAILED, self.session.state)
        self.assertEqual('cancellation_timeout', self.session.result.reason)

    def test_rejects_stale_navigation_handle(self) -> None:
        self.start_and_dispatch()

        with self.assertRaises(ValueError):
            self.session.navigation_succeeded('stale-goal')
        self.assertEqual(ExplorationState.NAVIGATING, self.session.state)

    def test_dispatch_error_fails_without_active_goal(self) -> None:
        self.navigation.dispatch_error = RuntimeError('server unavailable')
        self.session.start()

        self.assertFalse(
            self.session.observe_map(frontier_grid(), 2.5, 1.5)
        )
        self.assertEqual(ExplorationState.FAILED, self.session.state)
        self.assertEqual(
            'navigation_dispatch_error',
            self.session.result.reason,
        )
        self.assertIsNone(self.session.active_goal_handle)

    def test_result_and_events_are_immutable_snapshots(self) -> None:
        self.session.start()
        self.session.observe_map(complete_grid(), 0.5, 0.5)
        self.clock.now = 11.0
        self.session.observe_map(complete_grid(), 0.5, 0.5)

        result = self.session.result
        self.assertEqual(1.0, result.duration_sec)
        self.assertEqual(
            list(range(1, len(self.session.events) + 1)),
            [event.sequence for event in self.session.events],
        )
        self.assertIsInstance(result.blacklisted_points, tuple)


if __name__ == '__main__':
    unittest.main()
