import unittest
from math import pi

from omokai_fleet import (
    Formation,
    FormationControlDirective,
    FormationOffset,
    FormationTrackingConfig,
    Pose2D,
    RobotId,
    follower_command,
    is_current_formation_state,
    normalize_angle,
    tracking_error,
)


class FormationTrackingTest(unittest.TestCase):
    def test_zero_error_stops_at_rotated_wedge_offset(self) -> None:
        leader = Pose2D(2.0, 3.0, pi / 2.0)
        offset = FormationOffset(RobotId.ROBOT2, -0.6, 0.6)
        follower = Pose2D(1.4, 2.4, pi / 2.0)

        error = tracking_error(leader, follower, offset)
        command = follower_command(error)

        self.assertAlmostEqual(0.0, error.distance_m)
        self.assertTrue(command.target_reached)
        self.assertEqual(0.0, command.linear_mps)
        self.assertEqual(0.0, command.angular_rps)

    def test_position_error_is_reported_in_leader_frame(self) -> None:
        error = tracking_error(
            Pose2D(0.0, 0.0, pi / 2.0),
            Pose2D(-0.5, -1.0, pi / 2.0),
            FormationOffset(RobotId.ROBOT2, -0.6, 0.6),
        )

        self.assertAlmostEqual(0.4, error.forward_error_m)
        self.assertAlmostEqual(0.1, error.left_error_m)

    def test_large_heading_error_turns_before_advancing(self) -> None:
        error = tracking_error(
            Pose2D(1.0, 0.0, 0.0),
            Pose2D(0.0, 0.0, pi),
            FormationOffset(RobotId.ROBOT2, 0.0, 0.0),
        )

        command = follower_command(error)

        self.assertEqual(0.0, command.linear_mps)
        self.assertLessEqual(abs(command.angular_rps), 1.2)

    def test_outputs_are_bounded_and_forward_only(self) -> None:
        config = FormationTrackingConfig(
            linear_gain=10.0,
            angular_gain=10.0,
            max_linear_mps=0.18,
            max_angular_rps=0.7,
        )
        error = tracking_error(
            Pose2D(10.0, 1.0, 0.0),
            Pose2D(0.0, 0.0, 0.0),
            FormationOffset(RobotId.ROBOT3, 0.0, 0.0),
        )

        command = follower_command(error, config)

        self.assertGreaterEqual(command.linear_mps, 0.0)
        self.assertLessEqual(command.linear_mps, 0.18)
        self.assertLessEqual(abs(command.angular_rps), 0.7)

    def test_at_position_corrects_yaw_without_translation(self) -> None:
        error = tracking_error(
            Pose2D(0.0, 0.0, pi / 2.0),
            Pose2D(0.0, 0.0, 0.0),
            FormationOffset(RobotId.ROBOT2, 0.0, 0.0),
        )

        command = follower_command(error)

        self.assertEqual(0.0, command.linear_mps)
        self.assertGreater(command.angular_rps, 0.0)
        self.assertFalse(command.target_reached)

    def test_normalize_angle_and_config_reject_invalid_values(self) -> None:
        self.assertAlmostEqual(-pi, normalize_angle(pi))
        with self.assertRaises(ValueError):
            normalize_angle(float('nan'))
        with self.assertRaises(ValueError):
            FormationTrackingConfig(max_linear_mps=True)

    def test_directive_validates_exact_runtime_control_contract(self) -> None:
        directive = FormationControlDirective.from_mapping(
            {
                'enabled': True,
                'formation': 'wedge',
                'spacing_m': 0.8,
                'max_linear_mps': 0.18,
                'operation_id': 'a' * 32,
            }
        )

        self.assertEqual(Formation.WEDGE, directive.formation)
        self.assertEqual(
            RobotId.ROBOT3,
            directive.follower_offset(RobotId.ROBOT3).robot_id,
        )
        with self.assertRaises(ValueError):
            FormationControlDirective.from_mapping(
                {
                    'enabled': True,
                    'formation': 'wedge',
                    'spacing_m': 0.8,
                    'max_linear_mps': 0.18,
                    'operation_id': 'a' * 32,
                    'coordinates': [1, 2],
                }
            )
        with self.assertRaises(ValueError):
            directive.follower_offset(RobotId.ROBOT1)
        with self.assertRaises(ValueError):
            FormationControlDirective.from_mapping(
                {
                    'enabled': True,
                    'formation': 'wedge',
                    'spacing_m': 0.8,
                    'max_linear_mps': 0.18,
                    'operation_id': 'stale',
                }
            )

    def test_only_the_active_operation_state_is_accepted(self) -> None:
        active = 'a' * 32
        self.assertTrue(
            is_current_formation_state({'operation_id': active}, active)
        )
        self.assertFalse(
            is_current_formation_state({'operation_id': 'b' * 32}, active)
        )
        self.assertFalse(is_current_formation_state({'operation_id': active}, None))
        self.assertFalse(is_current_formation_state([], active))


if __name__ == '__main__':
    unittest.main()
