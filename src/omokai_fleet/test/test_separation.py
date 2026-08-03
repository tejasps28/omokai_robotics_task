import unittest

from omokai_fleet import (
    ObservedRobotPose,
    Pose2D,
    RobotId,
    emergency_separation_reason,
    observed_separations,
)


def poses(robot2_x: float = 0.6) -> tuple[ObservedRobotPose, ...]:
    return (
        ObservedRobotPose(RobotId.ROBOT1, Pose2D(0.0, 0.0, 0.0)),
        ObservedRobotPose(RobotId.ROBOT2, Pose2D(robot2_x, 0.0, 0.0)),
        ObservedRobotPose(RobotId.ROBOT3, Pose2D(0.0, 0.8, 0.0)),
    )


class ObservedSeparationTest(unittest.TestCase):
    def test_computes_all_pairs_in_stable_order(self) -> None:
        values = observed_separations(poses())

        self.assertEqual(
            (
                (RobotId.ROBOT1, RobotId.ROBOT2),
                (RobotId.ROBOT1, RobotId.ROBOT3),
                (RobotId.ROBOT2, RobotId.ROBOT3),
            ),
            tuple((item.first, item.second) for item in values),
        )
        self.assertEqual((0.6, 0.8, 1.0), tuple(item.distance_m for item in values))

    def test_safe_poses_return_no_reason(self) -> None:
        self.assertIsNone(emergency_separation_reason(poses()))

    def test_violation_names_pair_distance_and_threshold(self) -> None:
        reason = emergency_separation_reason(poses(robot2_x=0.2))

        self.assertIn('robot1/robot2=0.200m', reason)
        self.assertIn('0.30 m', reason)

    def test_exact_threshold_is_safe(self) -> None:
        self.assertIsNone(
            emergency_separation_reason(poses(robot2_x=0.30))
        )

    def test_close_but_not_emergency_distance_remains_active(self) -> None:
        self.assertIsNone(
            emergency_separation_reason(poses(robot2_x=0.31))
        )

    def test_rejects_wrong_order_and_invalid_threshold(self) -> None:
        with self.assertRaises(ValueError):
            observed_separations(tuple(reversed(poses())))
        for minimum in (0.0, True, 'near'):
            with self.subTest(minimum=minimum):
                with self.assertRaises(ValueError):
                    emergency_separation_reason(poses(), minimum)


if __name__ == '__main__':
    unittest.main()
