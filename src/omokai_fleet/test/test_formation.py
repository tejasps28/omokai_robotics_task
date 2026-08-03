import unittest
from math import pi, sqrt

from omokai_fleet import (
    Formation,
    FormationOffset,
    Pose2D,
    RobotGoal,
    RobotId,
    formation_goals,
    formation_offsets,
    goal_separations,
    transform_offset,
    validate_goal_separation,
)


class FormationOffsetsTest(unittest.TestCase):
    def test_line_offsets_have_stable_leader_to_tail_order(self) -> None:
        offsets = formation_offsets(Formation.LINE, 0.8)

        self.assertEqual(
            (
                (RobotId.ROBOT1, 0.0, 0.0),
                (RobotId.ROBOT2, -0.8, 0.0),
                (RobotId.ROBOT3, -1.6, 0.0),
            ),
            tuple(
                (item.robot_id, item.forward_m, item.left_m)
                for item in offsets
            ),
        )

    def test_wedge_offsets_are_symmetric(self) -> None:
        offsets = formation_offsets(Formation.WEDGE, 0.7)

        self.assertEqual(0.7, offsets[1].left_m)
        self.assertEqual(-0.7, offsets[2].left_m)
        self.assertEqual(offsets[1].forward_m, offsets[2].forward_m)

    def test_rejects_unknown_formation_and_unsafe_spacing(self) -> None:
        with self.assertRaises(ValueError):
            formation_offsets('diamond', 0.8)
        for spacing in (0.59, 1.21, True, 'wide'):
            with self.subTest(spacing=spacing):
                with self.assertRaises(ValueError):
                    formation_offsets(Formation.LINE, spacing)


class TransformTest(unittest.TestCase):
    def test_zero_yaw_uses_reference_axes(self) -> None:
        pose = transform_offset(
            Pose2D(2.0, 3.0, 0.0),
            FormationOffset(RobotId.ROBOT2, -1.0, 0.5),
        )

        self.assertAlmostEqual(1.0, pose.x)
        self.assertAlmostEqual(3.5, pose.y)
        self.assertEqual(0.0, pose.yaw)

    def test_quarter_turn_rotates_offset(self) -> None:
        pose = transform_offset(
            Pose2D(2.0, 3.0, pi / 2.0),
            FormationOffset(RobotId.ROBOT2, -1.0, 0.5),
        )

        self.assertAlmostEqual(1.5, pose.x)
        self.assertAlmostEqual(2.0, pose.y)
        self.assertEqual(pi / 2.0, pose.yaw)

    def test_arbitrary_yaw_preserves_distance_from_reference(self) -> None:
        reference = Pose2D(-1.0, 2.0, 0.37)
        offset = FormationOffset(RobotId.ROBOT3, -0.8, -0.8)
        pose = transform_offset(reference, offset)

        distance = sqrt(
            (pose.x - reference.x) ** 2
            + (pose.y - reference.y) ** 2
        )
        self.assertAlmostEqual(sqrt(2.0) * 0.8, distance)


class FormationGoalsTest(unittest.TestCase):
    def test_generates_stable_correlated_goals(self) -> None:
        goals = formation_goals(
            Pose2D(1.5, 1.5, pi),
            Formation.WEDGE,
            0.8,
            phase_id='forming',
            step_index=2,
        )

        self.assertEqual(
            (RobotId.ROBOT1, RobotId.ROBOT2, RobotId.ROBOT3),
            tuple(goal.robot_id for goal in goals),
        )
        self.assertEqual(
            'forming/step2/robot3',
            goals[2].goal_id,
        )
        self.assertTrue(all(goal.phase_id == 'forming' for goal in goals))

    def test_rejects_invalid_step_index(self) -> None:
        for step_index in (0, -1, True, 1.5):
            with self.subTest(step_index=step_index):
                with self.assertRaises(ValueError):
                    formation_goals(
                        Pose2D(0.0, 0.0, 0.0),
                        Formation.LINE,
                        0.8,
                        phase_id='forming',
                        step_index=step_index,
                    )


class SeparationTest(unittest.TestCase):
    def test_line_distances_are_deterministic(self) -> None:
        goals = formation_goals(
            Pose2D(0.0, 0.0, 0.0),
            Formation.LINE,
            0.8,
            phase_id='forming',
            step_index=1,
        )

        self.assertEqual(
            (0.8, 1.6, 0.8),
            tuple(item.distance_m for item in goal_separations(goals)),
        )

    def test_wedge_distances_remain_safe_after_rotation(self) -> None:
        goals = formation_goals(
            Pose2D(2.0, -1.0, 1.1),
            Formation.WEDGE,
            0.6,
            phase_id='moving',
            step_index=4,
        )

        separations = validate_goal_separation(goals)

        self.assertEqual(3, len(separations))
        self.assertGreaterEqual(
            min(item.distance_m for item in separations),
            0.5,
        )

    def test_rejects_unsafe_pair_and_names_it(self) -> None:
        goals = (
            RobotGoal(
                'forming',
                'a',
                RobotId.ROBOT1,
                Pose2D(0.0, 0.0, 0.0),
            ),
            RobotGoal(
                'forming',
                'b',
                RobotId.ROBOT2,
                Pose2D(0.2, 0.0, 0.0),
            ),
        )

        with self.assertRaisesRegex(ValueError, 'robot1/robot2'):
            validate_goal_separation(goals)

    def test_rejects_duplicate_robot_assignments(self) -> None:
        goals = (
            RobotGoal(
                'forming',
                'a',
                RobotId.ROBOT1,
                Pose2D(0.0, 0.0, 0.0),
            ),
            RobotGoal(
                'forming',
                'b',
                RobotId.ROBOT1,
                Pose2D(1.0, 0.0, 0.0),
            ),
        )

        with self.assertRaises(ValueError):
            goal_separations(goals)

    def test_rejects_invalid_minimum(self) -> None:
        goals = formation_goals(
            Pose2D(0.0, 0.0, 0.0),
            Formation.LINE,
            0.8,
            phase_id='forming',
            step_index=1,
        )
        for minimum in (0.0, -1.0, True, 'near'):
            with self.subTest(minimum=minimum):
                with self.assertRaises(ValueError):
                    validate_goal_separation(goals, minimum)


if __name__ == '__main__':
    unittest.main()
