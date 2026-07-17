import unittest

from jsonschema import Draft7Validator

from omokai_interfaces import load_mission_v1_schema


VALID_MISSION = {
    'schema_version': '1.1',
    'action': 'patrol',
    'route_id': 'inspection_loop',
    'segments': [{'direction': 'counterclockwise', 'repetitions': 2}],
    'speed_mps': 0.18,
    'return_home': True,
}


class MissionSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.validator = Draft7Validator(load_mission_v1_schema())

    def assert_invalid(self, **changes: object) -> None:
        candidate = {**VALID_MISSION, **changes}
        self.assertTrue(list(self.validator.iter_errors(candidate)))

    def test_accepts_core_patrol(self) -> None:
        self.validator.validate(VALID_MISSION)

    def test_rejects_unknown_action(self) -> None:
        self.assert_invalid(action='publish_cmd_vel')

    def test_rejects_additional_property(self) -> None:
        self.assert_invalid(shell_command='rm -rf /')

    def test_rejects_non_symbolic_route(self) -> None:
        self.assert_invalid(route_id='../../arbitrary/path')

    def test_rejects_zero_repetitions(self) -> None:
        self.assert_invalid(
            segments=[{'direction': 'counterclockwise', 'repetitions': 0}]
        )

    def test_rejects_empty_segments(self) -> None:
        self.assert_invalid(segments=[])

    def test_rejects_unknown_direction(self) -> None:
        self.assert_invalid(segments=[{'direction': 'sideways', 'repetitions': 1}])

    def test_allows_unknown_but_well_formed_route_for_semantic_validation(self) -> None:
        candidate = {**VALID_MISSION, 'route_id': 'unknown_route'}
        self.validator.validate(candidate)


if __name__ == '__main__':
    unittest.main()
