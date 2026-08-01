import unittest

from omokai_fleet import (
    Formation,
    Pose2D,
    RobotGoal,
    RobotId,
    RouteAssignment,
    RoutePoint,
    goal_separations,
    partition_route,
    regroup_goals,
)


def route_points(count: int) -> tuple[RoutePoint, ...]:
    return tuple(
        RoutePoint(
            point_id=f'point{index}',
            pose=Pose2D(float(index), float(index % 2), 0.0),
        )
        for index in range(count)
    )


class RoutePointTest(unittest.TestCase):
    def test_rejects_blank_id_and_wrong_pose(self) -> None:
        with self.assertRaises(ValueError):
            RoutePoint(' ', Pose2D(0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            RoutePoint('point', object())


class RouteAssignmentTest(unittest.TestCase):
    def test_rejects_empty_or_cross_robot_goals(self) -> None:
        with self.assertRaises(ValueError):
            RouteAssignment(RobotId.ROBOT1, ())

        robot2_goal = RobotGoal(
            'split',
            'split/robot2/a',
            RobotId.ROBOT2,
            Pose2D(0.0, 0.0, 0.0),
        )
        with self.assertRaises(ValueError):
            RouteAssignment(RobotId.ROBOT1, (robot2_goal,))


class PartitionRouteTest(unittest.TestCase):
    def test_four_points_split_two_one_one_contiguously(self) -> None:
        assignments = partition_route(route_points(4))

        self.assertEqual(
            (RobotId.ROBOT1, RobotId.ROBOT2, RobotId.ROBOT3),
            tuple(item.robot_id for item in assignments),
        )
        self.assertEqual(
            (2, 1, 1),
            tuple(len(item.goals) for item in assignments),
        )
        self.assertEqual(
            (
                ('point0', 'point1'),
                ('point2',),
                ('point3',),
            ),
            tuple(
                tuple(goal.goal_id.rsplit('/', 1)[-1] for goal in item.goals)
                for item in assignments
            ),
        )

    def test_seven_points_are_balanced_three_two_two(self) -> None:
        assignments = partition_route(route_points(7))

        self.assertEqual(
            (3, 2, 2),
            tuple(len(item.goals) for item in assignments),
        )

    def test_preserves_complete_unique_coverage(self) -> None:
        points = route_points(8)
        assignments = partition_route(points, phase_id='inspection')
        assigned_ids = tuple(
            goal.goal_id.rsplit('/', 1)[-1]
            for assignment in assignments
            for goal in assignment.goals
        )

        self.assertEqual(
            tuple(point.point_id for point in points),
            assigned_ids,
        )
        self.assertEqual(len(assigned_ids), len(set(assigned_ids)))
        self.assertTrue(
            all(
                goal.phase_id == 'inspection'
                for assignment in assignments
                for goal in assignment.goals
            )
        )

    def test_rejects_too_few_or_mutable_points(self) -> None:
        with self.assertRaises(ValueError):
            partition_route(route_points(2))
        with self.assertRaises(ValueError):
            partition_route(list(route_points(4)))

    def test_rejects_duplicate_point_ids(self) -> None:
        points = (
            RoutePoint('a', Pose2D(0.0, 0.0, 0.0)),
            RoutePoint('a', Pose2D(1.0, 0.0, 0.0)),
            RoutePoint('b', Pose2D(2.0, 0.0, 0.0)),
        )

        with self.assertRaises(ValueError):
            partition_route(points)

    def test_rejects_blank_phase(self) -> None:
        with self.assertRaises(ValueError):
            partition_route(route_points(3), phase_id=' ')

    def test_assigns_contiguous_segments_by_minimum_approach_distance(self) -> None:
        points = tuple(
            RoutePoint(f'p{index}', Pose2D(float(index * 10), 0.0, 0.0))
            for index in range(6)
        )
        poses = {
            RobotId.ROBOT1: Pose2D(40.0, 0.0, 0.0),
            RobotId.ROBOT2: Pose2D(0.0, 0.0, 0.0),
            RobotId.ROBOT3: Pose2D(20.0, 0.0, 0.0),
        }

        assignments = partition_route(points, current_poses=poses)

        self.assertEqual(
            (('p4', 'p5'), ('p0', 'p1'), ('p2', 'p3')),
            tuple(
                tuple(goal.goal_id.rsplit('/', 1)[-1] for goal in item.goals)
                for item in assignments
            ),
        )

    def test_equal_costs_use_stable_segment_order(self) -> None:
        poses = {
            robot_id: Pose2D(0.0, 0.0, 0.0)
            for robot_id in RobotId
        }

        assignments = partition_route(route_points(3), current_poses=poses)

        self.assertEqual(
            ('point0', 'point1', 'point2'),
            tuple(item.goals[0].goal_id.rsplit('/', 1)[-1] for item in assignments),
        )

    def test_missing_live_pose_data_is_rejected_and_none_falls_back(self) -> None:
        points = route_points(3)
        with self.assertRaises(ValueError):
            partition_route(
                points,
                current_poses={RobotId.ROBOT1: Pose2D(0.0, 0.0, 0.0)},
            )

        fallback = partition_route(points, current_poses=None)
        self.assertEqual(
            ('point0', 'point1', 'point2'),
            tuple(item.goals[0].goal_id.rsplit('/', 1)[-1] for item in fallback),
        )


class RegroupTest(unittest.TestCase):
    def test_produces_three_distinct_safe_goals(self) -> None:
        reference = Pose2D(-1.0, -0.5, 0.0)
        goals = regroup_goals(reference, Formation.WEDGE, 0.8)

        self.assertEqual(3, len(goals))
        self.assertEqual(reference, goals[0].pose)
        self.assertEqual(3, len({goal.pose for goal in goals}))
        self.assertGreaterEqual(
            min(item.distance_m for item in goal_separations(goals)),
            0.5,
        )

    def test_preserves_custom_phase_id(self) -> None:
        goals = regroup_goals(
            Pose2D(0.0, 0.0, 1.0),
            Formation.LINE,
            0.6,
            phase_id='return_to_home',
        )

        self.assertTrue(
            all(goal.phase_id == 'return_to_home' for goal in goals)
        )
        self.assertEqual(
            'return_to_home/step1/robot2',
            goals[1].goal_id,
        )


if __name__ == '__main__':
    unittest.main()
