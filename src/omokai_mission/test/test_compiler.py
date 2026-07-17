"""Deterministic mission compiler tests."""

import copy
import unittest

from omokai_interfaces import MissionV1, ValidatedMission

from omokai_mission.catalog import CatalogError, parse_catalog
from omokai_mission.compiler import compile_mission


VALID_CATALOG = {
    'catalog_version': '1.0',
    'frame_id': 'map',
    'home': {'name': 'home', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'routes': {
        'inspection_loop': {
            'description': 'test loop',
            'closed': True,
            'allowed_directions': ['clockwise', 'counterclockwise'],
            'waypoints': [
                {'name': 'a', 'x': 1.0, 'y': 1.0, 'yaw': 0.0},
                {'name': 'b', 'x': -1.0, 'y': 1.0, 'yaw': 3.14},
            ],
        }
    },
}


def make_mission(
    repetitions: int,
    return_home: bool,
    route_id: str = 'inspection_loop',
    mission_id: str = 'mission-1',
    segments=None,
) -> ValidatedMission:
    spec = MissionV1.from_mapping(
        {
            'schema_version': '1.1',
            'action': 'patrol',
            'route_id': route_id,
            'segments': segments or [
                {
                    'direction': 'counterclockwise',
                    'repetitions': repetitions,
                }
            ],
            'speed_mps': 0.18,
            'return_home': return_home,
        }
    )
    return ValidatedMission(
        mission_id=mission_id,
        request_id='request-1',
        mission=spec,
        policy_version='core-safety-2',
    )


class CompilerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.catalog = parse_catalog(copy.deepcopy(VALID_CATALOG))
        self.waypoints = self.catalog.get('inspection_loop').waypoints

    def test_one_lap_without_return(self) -> None:
        compiled = compile_mission(make_mission(1, False), self.catalog)
        self.assertEqual(len(self.waypoints), len(compiled.goals))
        self.assertEqual(
            [
                'inspection_loop/segment1/counterclockwise/lap1/a',
                'inspection_loop/segment1/counterclockwise/lap1/b',
            ],
            [goal.label for goal in compiled.goals],
        )
        self.assertNotIn('home', [goal.label for goal in compiled.goals])

    def test_two_laps_then_home(self) -> None:
        compiled = compile_mission(make_mission(2, True), self.catalog)
        # exactly N * waypoints traversals, then one home goal.
        self.assertEqual(2 * len(self.waypoints) + 1, len(compiled.goals))
        self.assertEqual(
            [
                'inspection_loop/segment1/counterclockwise/lap1/a',
                'inspection_loop/segment1/counterclockwise/lap1/b',
                'inspection_loop/segment1/counterclockwise/lap2/a',
                'inspection_loop/segment1/counterclockwise/lap2/b',
                'home',
            ],
            [goal.label for goal in compiled.goals],
        )
        self.assertEqual(self.catalog.home, compiled.goals[-1].pose)

    def test_return_home_false_appends_no_home(self) -> None:
        compiled = compile_mission(make_mission(3, False), self.catalog)
        self.assertEqual(3 * len(self.waypoints), len(compiled.goals))
        self.assertNotIn('home', [goal.label for goal in compiled.goals])

    def test_goals_use_catalog_poses_and_frame(self) -> None:
        compiled = compile_mission(make_mission(1, False), self.catalog)
        self.assertEqual('map', compiled.frame_id)
        self.assertEqual(self.waypoints[0].pose, compiled.goals[0].pose)
        self.assertTrue(all(goal.frame_id == 'map' for goal in compiled.goals))

    def test_identical_input_produces_identical_goals(self) -> None:
        first = compile_mission(make_mission(2, True), self.catalog)
        second = compile_mission(make_mission(2, True), self.catalog)
        self.assertEqual(first, second)
        self.assertEqual(first.summary_lines(), second.summary_lines())

    def test_mixed_directions_preserve_segment_order(self) -> None:
        from omokai_mission.catalog import load_catalog

        catalog = load_catalog()
        mission = make_mission(
            2,
            True,
            segments=[
                {'direction': 'clockwise', 'repetitions': 1},
                {'direction': 'counterclockwise', 'repetitions': 1},
            ],
        )
        compiled = compile_mission(mission, catalog)
        names = [goal.label.rsplit('/', 1)[-1] for goal in compiled.goals[:-1]]
        self.assertEqual(
            [
                'corner_sw', 'corner_nw', 'corner_ne', 'corner_se',
                'corner_sw', 'corner_se', 'corner_ne', 'corner_nw',
            ],
            names,
        )
        self.assertIn('/segment1/clockwise/lap1/', compiled.goals[0].label)
        self.assertIn('/segment2/counterclockwise/lap2/', compiled.goals[4].label)

    def test_reverse_aisle_uses_reverse_catalog_order(self) -> None:
        from omokai_mission.catalog import load_catalog

        catalog = load_catalog()
        mission = make_mission(
            1,
            False,
            route_id='aisle_sweep',
            segments=[{'direction': 'reverse', 'repetitions': 1}],
        )
        compiled = compile_mission(mission, catalog)
        self.assertEqual(
            [
                'east_aisle_south',
                'east_aisle_north',
                'west_aisle_north',
                'west_aisle_south',
            ],
            [goal.label.rsplit('/', 1)[-1] for goal in compiled.goals],
        )

    def test_full_area_sweep_compiles_all_coverage_poses_then_home(self) -> None:
        from omokai_mission.catalog import load_catalog

        catalog = load_catalog()
        mission = make_mission(
            1,
            True,
            route_id='full_area_sweep',
            segments=[{'direction': 'forward', 'repetitions': 1}],
        )
        compiled = compile_mission(mission, catalog)
        names = [goal.label.rsplit('/', 1)[-1] for goal in compiled.goals]
        self.assertEqual(13, len(names))
        self.assertEqual('outer_south_west', names[0])
        self.assertEqual('east_aisle_north', names[-2])
        self.assertEqual('home', names[-1])

    def test_reverse_full_area_sweep_reverses_every_coverage_pose(self) -> None:
        from omokai_mission.catalog import load_catalog

        catalog = load_catalog()
        mission = make_mission(
            1,
            False,
            route_id='full_area_sweep',
            segments=[{'direction': 'reverse', 'repetitions': 1}],
        )
        compiled = compile_mission(mission, catalog)
        names = [goal.label.rsplit('/', 1)[-1] for goal in compiled.goals]
        self.assertEqual('east_aisle_north', names[0])
        self.assertEqual('outer_south_west', names[-1])

    def test_unknown_route_raises(self) -> None:
        with self.assertRaises(CatalogError):
            compile_mission(make_mission(1, True, route_id='ghost_route'), self.catalog)

    def test_mission_identity_is_preserved(self) -> None:
        compiled = compile_mission(make_mission(1, True, mission_id='abc'), self.catalog)
        self.assertEqual('abc', compiled.mission_id)
        self.assertEqual('inspection_loop', compiled.route_id)


if __name__ == '__main__':
    unittest.main()
