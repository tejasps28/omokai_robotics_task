import unittest
from math import pi

from omokai_fleet import (
    Formation,
    NavigationBatch,
    NavigationResult,
    NavigationStatus,
    Pose2D,
    RobotId,
    RoutePoint,
    SquadAction,
    SquadPlan,
    SquadState,
    build_squad_mission,
    execute_squad_mission,
)
from omokai_fleet.split_execution import SplitExecutionResult, SplitRobotResult


def plan(**overrides) -> SquadPlan:
    values = {
        'plan_id': 'full-fleet-demo',
        'action': SquadAction.FORMATION_PATROL,
        'formation': Formation.LINE,
        'route_id': 'inspection_loop',
        'spacing_m': 0.6,
        'speed_mps': 0.12,
        'split_route': True,
        'regroup': True,
        'regroup_location': 'home',
    }
    values.update(overrides)
    return SquadPlan(**values)


ROUTE = (
    RoutePoint('corner_sw', Pose2D(-1.5, -1.5, 0.0)),
    RoutePoint('corner_se', Pose2D(1.5, -1.5, pi / 2.0)),
    RoutePoint('corner_ne', Pose2D(1.5, 1.5, pi)),
    RoutePoint('corner_nw', Pose2D(-1.5, 1.5, -pi / 2.0)),
)
REFERENCES = (
    Pose2D(-1.5, 0.9, pi / 2.0),
    Pose2D(-1.5, 1.5, pi / 2.0),
)
HOME = Pose2D(-2.0, -0.5, 0.0)


class FakeNavigation:
    def __init__(self, fail_call: int | None = None) -> None:
        self.fail_call = fail_call
        self.calls = []

    def execute(self, goals, *, speed_mps, timeout_sec):
        self.calls.append((goals, speed_mps, timeout_sec))
        call_number = len(self.calls)
        if call_number == self.fail_call:
            return NavigationBatch(
                (
                    NavigationResult(
                        RobotId.ROBOT1,
                        NavigationStatus.FAILED,
                        'injected failure',
                    ),
                    NavigationResult(
                        RobotId.ROBOT2,
                        NavigationStatus.CANCELLED,
                        'cancelled after robot1 failed',
                    ),
                    NavigationResult(
                        RobotId.ROBOT3,
                        NavigationStatus.CANCELLED,
                        'cancelled after robot1 failed',
                    ),
                )
            )
        return NavigationBatch(
            tuple(
                NavigationResult(robot_id, NavigationStatus.SUCCEEDED)
                for robot_id in RobotId
            )
        )

    def execute_formation(
        self,
        batches,
        *,
        formation,
        spacing_m,
        speed_mps,
        timeout_sec,
    ):
        self.calls.append(
            (batches, formation, spacing_m, speed_mps, timeout_sec)
        )
        call_number = len(self.calls)
        if call_number == self.fail_call:
            return NavigationBatch(
                (
                    NavigationResult(
                        RobotId.ROBOT1,
                        NavigationStatus.FAILED,
                        'injected leader failure',
                    ),
                    NavigationResult(
                        RobotId.ROBOT2,
                        NavigationStatus.CANCELLED,
                        'formation tracking stopped',
                    ),
                    NavigationResult(
                        RobotId.ROBOT3,
                        NavigationStatus.CANCELLED,
                        'formation tracking stopped',
                    ),
                )
            )
        return NavigationBatch(
            tuple(
                NavigationResult(robot_id, NavigationStatus.SUCCEEDED)
                for robot_id in RobotId
            )
        )


class IsolatedFailureNavigation(FakeNavigation):
    def execute_split(self, split_plan, *, speed_mps, timeout_sec):
        self.calls.append((split_plan, speed_mps, timeout_sec))
        return SplitExecutionResult(
            (
                SplitRobotResult(RobotId.ROBOT1, NavigationStatus.SUCCEEDED, 2),
                SplitRobotResult(
                    RobotId.ROBOT2,
                    NavigationStatus.FAILED,
                    0,
                    1,
                    'private route blocked',
                ),
                SplitRobotResult(RobotId.ROBOT3, NavigationStatus.SUCCEEDED, 1),
            )
        )


class MissionBuildTest(unittest.TestCase):
    def test_builds_formation_two_split_batches_and_regroup(self) -> None:
        mission = build_squad_mission(
            plan(),
            formation_references=REFERENCES,
            route_points=ROUTE,
            home=HOME,
        )

        self.assertEqual(3, len(mission.forming))
        self.assertEqual(1, len(mission.formation_movement))
        self.assertEqual(2, len(mission.split_execution))
        self.assertEqual(3, len(mission.regrouping))

    def test_route_points_appear_exactly_once_excluding_hold_goals(self) -> None:
        mission = build_squad_mission(
            plan(),
            formation_references=REFERENCES,
            route_points=ROUTE,
            home=HOME,
        )
        goal_ids = tuple(
            goal.goal_id
            for batch in mission.split_execution
            for goal in batch
            if '/hold' not in goal.goal_id
        )

        self.assertEqual(
            (
                'executing_split/robot1/corner_sw',
                'executing_split/robot2/corner_ne',
                'executing_split/robot3/corner_nw',
                'executing_split/robot1/corner_se',
            ),
            goal_ids,
        )

    def test_line_regroup_places_tail_at_home(self) -> None:
        mission = build_squad_mission(
            plan(),
            formation_references=REFERENCES,
            route_points=ROUTE,
            home=HOME,
        )

        self.assertEqual(HOME, mission.regrouping[2].pose)
        self.assertEqual(
            (-0.8, -1.4, -2.0),
            tuple(round(goal.pose.x, 1) for goal in mission.regrouping),
        )

    def test_rejects_missing_movement_reference(self) -> None:
        with self.assertRaises(ValueError):
            build_squad_mission(
                plan(),
                formation_references=(REFERENCES[0],),
                route_points=ROUTE,
                home=HOME,
            )


class MissionExecutionTest(unittest.TestCase):
    def test_runs_complete_mission_to_success(self) -> None:
        mission = build_squad_mission(
            plan(),
            formation_references=REFERENCES,
            route_points=ROUTE,
            home=HOME,
        )
        navigation = FakeNavigation()

        lifecycle = execute_squad_mission(navigation, mission)

        self.assertEqual(SquadState.SUCCEEDED, lifecycle.state)
        self.assertEqual(5, len(navigation.calls))
        self.assertEqual(
            (
                SquadState.FORMING,
                SquadState.FORMATION_MOVING,
                SquadState.EXECUTING_SPLIT,
                SquadState.REGROUPING,
            ),
            tuple(item.phase for item in lifecycle.snapshot().history),
        )

    def test_stops_without_later_phases_after_batch_failure(self) -> None:
        mission = build_squad_mission(
            plan(),
            formation_references=REFERENCES,
            route_points=ROUTE,
            home=HOME,
        )
        navigation = FakeNavigation(fail_call=2)

        lifecycle = execute_squad_mission(navigation, mission)

        self.assertEqual(SquadState.FAILED, lifecycle.state)
        self.assertEqual(2, len(navigation.calls))

    def test_records_partial_split_and_does_not_attempt_regroup(self) -> None:
        mission = build_squad_mission(
            plan(),
            formation_references=REFERENCES,
            route_points=ROUTE,
            home=HOME,
        )
        navigation = IsolatedFailureNavigation()

        lifecycle = execute_squad_mission(navigation, mission)

        self.assertEqual(SquadState.FAILED, lifecycle.state)
        self.assertEqual(3, len(navigation.calls))
        split = lifecycle.snapshot().history[-1]
        self.assertEqual(
            ('succeeded', 'failed', 'succeeded'),
            tuple(result.outcome.value for result in split.results),
        )
        self.assertIn('private route blocked', lifecycle.snapshot().reason)

    def test_formation_path_is_one_continuous_navigation_operation(self) -> None:
        references = REFERENCES + (Pose2D(-0.5, 1.5, 0.0),)
        mission = build_squad_mission(
            plan(split_route=False, regroup=False),
            formation_references=references,
            route_points=ROUTE,
            home=HOME,
        )
        navigation = FakeNavigation()

        lifecycle = execute_squad_mission(navigation, mission)

        self.assertEqual(SquadState.SUCCEEDED, lifecycle.state)
        self.assertEqual(2, len(navigation.calls))
        formation_call = navigation.calls[1]
        self.assertEqual(2, len(formation_call[0]))
        self.assertEqual(Formation.LINE, formation_call[1])

    def test_can_skip_split_and_regroup(self) -> None:
        mission = build_squad_mission(
            plan(split_route=False, regroup=False),
            formation_references=REFERENCES,
            route_points=ROUTE,
            home=HOME,
        )
        navigation = FakeNavigation()

        lifecycle = execute_squad_mission(navigation, mission)

        self.assertEqual(SquadState.SUCCEEDED, lifecycle.state)
        self.assertEqual(2, len(navigation.calls))

    def test_mission_deadline_stops_before_later_batch_dispatch(self) -> None:
        mission = build_squad_mission(
            plan(),
            formation_references=REFERENCES,
            route_points=ROUTE,
            home=HOME,
        )
        navigation = FakeNavigation()
        times = iter((0.0, 0.0, 601.0))

        lifecycle = execute_squad_mission(
            navigation,
            mission,
            clock=lambda: next(times),
        )

        self.assertEqual(SquadState.TIMED_OUT, lifecycle.state)
        self.assertEqual(1, len(navigation.calls))


if __name__ == '__main__':
    unittest.main()
