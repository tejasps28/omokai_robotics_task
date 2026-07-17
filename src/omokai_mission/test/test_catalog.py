"""Route catalog loading, validation, and policy synchronisation tests."""

import copy
import math
import unittest
from types import MappingProxyType

from omokai_mission.catalog import (
    CatalogError,
    RouteCatalog,
    load_catalog,
    parse_catalog,
)
from omokai_mission.validation import KNOWN_ROUTE_DIRECTIONS, KNOWN_ROUTE_IDS


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


def catalog_without(*keys: str) -> dict:
    data = copy.deepcopy(VALID_CATALOG)
    for key in keys:
        data.pop(key, None)
    return data


class CatalogParseTest(unittest.TestCase):
    def test_parses_valid_catalog(self) -> None:
        catalog = parse_catalog(copy.deepcopy(VALID_CATALOG))
        self.assertIsInstance(catalog, RouteCatalog)
        self.assertEqual('map', catalog.frame_id)
        self.assertEqual(('inspection_loop',), catalog.route_ids())
        route = catalog.get('inspection_loop')
        self.assertEqual(2, len(route.waypoints))
        self.assertTrue(route.closed)
        self.assertEqual('a', route.waypoints[0].name)
        self.assertEqual(1.0, route.waypoints[0].pose.x)

    def test_missing_top_level_key_is_rejected(self) -> None:
        for key in ('catalog_version', 'frame_id', 'home', 'routes'):
            with self.assertRaises(CatalogError):
                parse_catalog(catalog_without(key))

    def test_empty_routes_is_rejected(self) -> None:
        data = copy.deepcopy(VALID_CATALOG)
        data['routes'] = {}
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_empty_waypoints_is_rejected(self) -> None:
        data = copy.deepcopy(VALID_CATALOG)
        data['routes']['inspection_loop']['waypoints'] = []
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_non_numeric_pose_is_rejected(self) -> None:
        data = copy.deepcopy(VALID_CATALOG)
        data['routes']['inspection_loop']['waypoints'][0]['x'] = 'north'
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_boolean_pose_value_is_rejected(self) -> None:
        # bool is a subclass of int; it must not slip through as a coordinate.
        data = copy.deepcopy(VALID_CATALOG)
        data['routes']['inspection_loop']['waypoints'][0]['y'] = True
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_blank_waypoint_name_is_rejected(self) -> None:
        data = copy.deepcopy(VALID_CATALOG)
        data['routes']['inspection_loop']['waypoints'][0]['name'] = '  '
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_non_finite_pose_is_rejected(self) -> None:
        data = copy.deepcopy(VALID_CATALOG)
        data['routes']['inspection_loop']['waypoints'][0]['x'] = math.nan
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_unknown_catalog_property_is_rejected(self) -> None:
        data = copy.deepcopy(VALID_CATALOG)
        data['shell_command'] = 'forbidden'
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_duplicate_waypoint_name_is_rejected(self) -> None:
        data = copy.deepcopy(VALID_CATALOG)
        data['routes']['inspection_loop']['waypoints'][1]['name'] = 'a'
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_direction_policy_must_match_route_kind(self) -> None:
        data = copy.deepcopy(VALID_CATALOG)
        data['routes']['inspection_loop']['allowed_directions'] = [
            'forward',
            'reverse',
        ]
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_duplicate_direction_is_rejected(self) -> None:
        data = copy.deepcopy(VALID_CATALOG)
        data['routes']['inspection_loop']['allowed_directions'] = [
            'clockwise',
            'clockwise',
        ]
        with self.assertRaises(CatalogError):
            parse_catalog(data)

    def test_routes_mapping_is_read_only(self) -> None:
        catalog = parse_catalog(copy.deepcopy(VALID_CATALOG))
        self.assertIsInstance(catalog.routes, MappingProxyType)
        with self.assertRaises(TypeError):
            catalog.routes['new_route'] = catalog.get('inspection_loop')

    def test_unknown_route_lookup_raises(self) -> None:
        catalog = parse_catalog(copy.deepcopy(VALID_CATALOG))
        with self.assertRaises(CatalogError):
            catalog.get('ghost_route')


class DefaultCatalogTest(unittest.TestCase):
    def test_packaged_catalog_loads(self) -> None:
        catalog = load_catalog()
        self.assertIn('inspection_loop', catalog.route_ids())
        self.assertIn('aisle_sweep', catalog.route_ids())
        loop = catalog.get('inspection_loop')
        self.assertEqual(4, len(loop.waypoints))
        self.assertEqual(-2.0, catalog.home.x)
        self.assertEqual(-0.5, catalog.home.y)
        self.assertEqual('corner_sw', loop.waypoints[0].name)
        aisle = catalog.get('aisle_sweep')
        self.assertFalse(aisle.closed)
        self.assertEqual(4, len(aisle.waypoints))
        full_sweep = catalog.get('full_area_sweep')
        self.assertFalse(full_sweep.closed)
        self.assertEqual(12, len(full_sweep.waypoints))
        self.assertEqual('outer_south_west', full_sweep.waypoints[0].name)
        self.assertEqual('east_aisle_north', full_sweep.waypoints[-1].name)

    def test_known_routes_stay_in_sync_with_catalog(self) -> None:
        # Single source of truth guard: semantic route and direction policy must
        # exactly match the audited catalog.
        catalog = load_catalog()
        self.assertEqual(tuple(sorted(KNOWN_ROUTE_IDS)), catalog.route_ids())
        self.assertEqual(
            dict(KNOWN_ROUTE_DIRECTIONS),
            dict(catalog.direction_policy()),
        )


if __name__ == '__main__':
    unittest.main()
