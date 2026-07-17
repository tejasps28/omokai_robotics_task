import json
import unittest

from omokai_interfaces import (
    MissionAction,
    MissionSegment,
    MissionProposal,
    MissionV1,
    PlanRequest,
    TraversalDirection,
)


class MissionModelTest(unittest.TestCase):
    def test_request_rejects_blank_prompt(self) -> None:
        with self.assertRaises(ValueError):
            PlanRequest(request_id='request-1', prompt='  ')

    def test_proposal_keeps_malformed_json_for_validator(self) -> None:
        proposal = MissionProposal(
            request_id='request-1', provider='test', content='{not-json'
        )
        self.assertEqual('{not-json', proposal.content)

    def test_typed_mission_has_stable_mapping(self) -> None:
        mission = MissionV1.from_mapping(
            {
                'schema_version': '1.1',
                'action': 'patrol',
                'route_id': 'inspection_loop',
                'segments': [
                    {'direction': 'clockwise', 'repetitions': 1},
                    {'direction': 'counterclockwise', 'repetitions': 1},
                ],
                'speed_mps': 0.18,
                'return_home': True,
            }
        )

        self.assertEqual(MissionAction.PATROL, mission.action)
        self.assertEqual(2, mission.repetitions)
        self.assertEqual(TraversalDirection.CLOCKWISE, mission.segments[0].direction)
        self.assertEqual(
            '{"action": "patrol", "return_home": true, '
            '"route_id": "inspection_loop", "schema_version": "1.1", '
            '"segments": [{"direction": "clockwise", "repetitions": 1}, '
            '{"direction": "counterclockwise", "repetitions": 1}], '
            '"speed_mps": 0.18}',
            json.dumps(dict(mission.as_mapping()), sort_keys=True),
        )

    def test_segment_rejects_untyped_direction(self) -> None:
        with self.assertRaises(ValueError):
            MissionSegment(direction='clockwise', repetitions=1)

    def test_direct_mission_rejects_empty_segments(self) -> None:
        with self.assertRaises(ValueError):
            MissionV1(
                schema_version='1.1',
                action=MissionAction.PATROL,
                route_id='inspection_loop',
                segments=(),
                speed_mps=0.18,
                return_home=True,
            )


if __name__ == '__main__':
    unittest.main()
