import math
import unittest

from omokai_bringup.saved_map_plan import (
    NamedLocation,
    VERIFICATION_LOCATIONS,
    build_saved_map_plan,
)


class SavedMapPlanTest(unittest.TestCase):
    def test_builds_two_named_map_goals_in_order(self) -> None:
        plan = build_saved_map_plan('saved-map-test')

        self.assertEqual(
            ('north_station', 'east_station'),
            tuple(goal.goal_id for goal in plan.goals),
        )
        self.assertTrue(
            all(goal.pose.frame_id == 'map' for goal in plan.goals)
        )
        self.assertEqual(0.15, plan.speed_mps)

    def test_locations_match_successful_exploration_regions(self) -> None:
        self.assertEqual(2, len(VERIFICATION_LOCATIONS))
        self.assertAlmostEqual(0.548, VERIFICATION_LOCATIONS[0].x)
        self.assertAlmostEqual(3.574, VERIFICATION_LOCATIONS[1].x)

    def test_accepts_an_explicit_location_sequence(self) -> None:
        plan = build_saved_map_plan(
            'one-location',
            locations=(NamedLocation('inspection_point', 1.0, 2.0),),
        )

        self.assertEqual(('inspection_point',), tuple(
            goal.goal_id for goal in plan.goals
        ))

    def test_rejects_blank_location_name(self) -> None:
        with self.assertRaises(ValueError):
            NamedLocation(' ', 0.0, 0.0)

    def test_rejects_empty_location_sequence(self) -> None:
        with self.assertRaises(ValueError):
            build_saved_map_plan('empty', locations=())

    def test_rejects_duplicate_location_names(self) -> None:
        duplicate = (
            NamedLocation('same', 0.0, 0.0),
            NamedLocation('same', 1.0, 1.0),
        )

        with self.assertRaises(ValueError):
            build_saved_map_plan('duplicate', locations=duplicate)

    def test_rejects_non_finite_location_pose(self) -> None:
        with self.assertRaises(ValueError):
            NamedLocation('invalid', math.nan, 0.0)

    def test_rejects_invalid_speed(self) -> None:
        with self.assertRaises(ValueError):
            build_saved_map_plan('invalid-speed', speed_mps=0.0)


if __name__ == '__main__':
    unittest.main()
