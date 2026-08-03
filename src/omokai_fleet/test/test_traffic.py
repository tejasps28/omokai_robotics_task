import unittest

from omokai_fleet.model import ROBOT_IDS, Pose2D, RobotId
from omokai_fleet.traffic import (
    PlanarVelocity,
    decide_right_of_way,
    predicted_separation,
)


def poses(*coordinates):
    return {
        robot_id: Pose2D(x, y, 0.0)
        for robot_id, (x, y) in zip(ROBOT_IDS, coordinates)
    }


def velocities(*coordinates):
    return {
        robot_id: PlanarVelocity(x, y)
        for robot_id, (x, y) in zip(ROBOT_IDS, coordinates)
    }


class TrafficPolicyTest(unittest.TestCase):
    def test_predicts_head_on_conflict(self) -> None:
        distance = predicted_separation(
            Pose2D(-1.0, 0.0, 0.0),
            PlanarVelocity(0.2, 0.0),
            Pose2D(1.0, 0.0, 0.0),
            PlanarVelocity(-0.2, 0.0),
            3.0,
        )

        self.assertAlmostEqual(0.8, distance)

    def test_clear_parallel_motion_allows_every_robot(self) -> None:
        decision = decide_right_of_way(
            poses((-1.0, 0.0), (-1.0, 0.6), (-1.0, 1.2)),
            velocities((0.1, 0.0), (0.1, 0.0), (0.1, 0.0)),
            0.0,
        )

        self.assertEqual(ROBOT_IDS, decision.allowed)
        self.assertEqual((), decision.yielding)

    def test_predicted_crossing_gives_robot1_initial_priority(self) -> None:
        decision = decide_right_of_way(
            poses((-0.4, 0.0), (0.0, -0.4), (3.0, 3.0)),
            velocities((0.2, 0.0), (0.0, 0.2), (0.1, 0.0)),
            0.0,
        )

        self.assertIn((RobotId.ROBOT1, RobotId.ROBOT2), decision.conflicts)
        self.assertIn(RobotId.ROBOT1, decision.allowed)
        self.assertEqual((RobotId.ROBOT2,), decision.yielding)

    def test_priority_rotates_to_avoid_starvation(self) -> None:
        current_poses = poses((-0.4, 0.0), (0.0, -0.4), (3.0, 3.0))
        current_velocities = velocities(
            (0.2, 0.0),
            (0.0, 0.2),
            (0.1, 0.0),
        )

        first = decide_right_of_way(
            current_poses,
            current_velocities,
            0.0,
        )
        second = decide_right_of_way(
            current_poses,
            current_velocities,
            5.0,
        )

        self.assertIn(RobotId.ROBOT1, first.allowed)
        self.assertIn(RobotId.ROBOT2, second.allowed)

    def test_rejects_missing_robot_state(self) -> None:
        with self.assertRaises(ValueError):
            decide_right_of_way(
                {RobotId.ROBOT1: Pose2D(0.0, 0.0, 0.0)},
                velocities((0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),
                0.0,
            )


if __name__ == '__main__':
    unittest.main()
