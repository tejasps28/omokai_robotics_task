import unittest
from dataclasses import FrozenInstanceError
from math import inf, nan

from omokai_fleet import (
    ROBOT_IDS,
    Formation,
    Pose2D,
    Robot,
    RobotGoal,
    RobotId,
    SquadAction,
    SquadPlan,
)


def valid_plan(**overrides) -> SquadPlan:
    values = {
        'plan_id': 'fleet-demo',
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


class RobotTest(unittest.TestCase):
    def test_derives_namespaced_interfaces_and_frames(self) -> None:
        robot = Robot(RobotId.ROBOT2)

        self.assertEqual('/robot2', robot.namespace)
        self.assertEqual('robot2/odom', robot.odom_frame)
        self.assertEqual('robot2/base_footprint', robot.base_frame)
        self.assertEqual('/robot2/navigate_to_pose', robot.navigation_action)

    def test_rejects_unknown_robot(self) -> None:
        with self.assertRaises(ValueError):
            Robot('robot4')


class PoseAndGoalTest(unittest.TestCase):
    def test_accepts_finite_map_pose(self) -> None:
        self.assertEqual('map', Pose2D(1.0, -2.0, 0.5).frame_id)

    def test_rejects_non_finite_pose(self) -> None:
        for value in (nan, inf, -inf, True, 'north'):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    Pose2D(value, 0.0, 0.0)

    def test_rejects_robot_local_goal_frame(self) -> None:
        with self.assertRaises(ValueError):
            Pose2D(0.0, 0.0, 0.0, frame_id='robot1/odom')

    def test_goal_is_correlated_by_phase_and_robot(self) -> None:
        goal = RobotGoal(
            phase_id='forming',
            goal_id='forming/robot3/step1',
            robot_id=RobotId.ROBOT3,
            pose=Pose2D(0.0, 1.0, 0.0),
        )

        self.assertEqual(RobotId.ROBOT3, goal.robot_id)
        self.assertEqual('forming', goal.phase_id)

    def test_rejects_blank_goal_identity(self) -> None:
        with self.assertRaises(ValueError):
            RobotGoal('', 'goal', RobotId.ROBOT1, Pose2D(0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            RobotGoal('phase', ' ', RobotId.ROBOT1, Pose2D(0.0, 0.0, 0.0))


class SquadPlanTest(unittest.TestCase):
    def test_accepts_frozen_contract(self) -> None:
        plan = valid_plan()

        self.assertEqual(ROBOT_IDS, plan.robots)
        self.assertEqual(120.0, plan.goal_timeout_sec)
        self.assertEqual(600.0, plan.mission_timeout_sec)

    def test_is_immutable(self) -> None:
        plan = valid_plan()

        with self.assertRaises(FrozenInstanceError):
            plan.spacing_m = 1.0

    def test_rejects_unknown_action_formation_route_and_rendezvous(self) -> None:
        invalid_values = (
            {'action': 'patrol'},
            {'formation': 'column'},
            {'route_id': 'aisle_sweep'},
            {'regroup_location': 'charger'},
        )
        for overrides in invalid_values:
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):
                    valid_plan(**overrides)

    def test_enforces_spacing_bounds_and_finiteness(self) -> None:
        for spacing in (0.59, 1.21, nan, inf, True, 'wide'):
            with self.subTest(spacing=spacing):
                with self.assertRaises(ValueError):
                    valid_plan(spacing_m=spacing)

        self.assertEqual(0.60, valid_plan(spacing_m=0.60).spacing_m)
        self.assertEqual(1.20, valid_plan(spacing_m=1.20).spacing_m)

    def test_enforces_speed_bounds_and_finiteness(self) -> None:
        for speed in (0.049, 0.181, nan, inf, True, 'fast'):
            with self.subTest(speed=speed):
                with self.assertRaises(ValueError):
                    valid_plan(speed_mps=speed)

        self.assertEqual(0.05, valid_plan(speed_mps=0.05).speed_mps)
        self.assertEqual(0.18, valid_plan(speed_mps=0.18).speed_mps)

    def test_requires_exactly_the_frozen_robot_order(self) -> None:
        invalid_robots = (
            list(ROBOT_IDS),
            (RobotId.ROBOT1, RobotId.ROBOT2),
            tuple(reversed(ROBOT_IDS)),
        )
        for robots in invalid_robots:
            with self.subTest(robots=robots):
                with self.assertRaises(ValueError):
                    valid_plan(robots=robots)

    def test_requires_boolean_phase_choices(self) -> None:
        with self.assertRaises(ValueError):
            valid_plan(split_route=1)
        with self.assertRaises(ValueError):
            valid_plan(regroup='yes')

    def test_rejects_invalid_timeouts_and_retries(self) -> None:
        invalid_values = (
            {'goal_timeout_sec': 0.0},
            {'mission_timeout_sec': 0.0},
            {'goal_timeout_sec': 121.0, 'mission_timeout_sec': 120.0},
            {'max_retries': -1},
            {'max_retries': True},
        )
        for overrides in invalid_values:
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):
                    valid_plan(**overrides)


if __name__ == '__main__':
    unittest.main()
