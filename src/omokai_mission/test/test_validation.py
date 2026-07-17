"""Structural and semantic validation tests for the safety policy matrix."""

import json
import unittest

from omokai_interfaces import MissionAction, MissionProposal, TraversalDirection

from omokai_mission import errors
from omokai_mission.validation import (
    MAX_REPETITIONS,
    MAX_SPEED_MPS,
    POLICY_VERSION,
    validate_proposal,
)


VALID_MISSION = {
    'schema_version': '1.1',
    'action': 'patrol',
    'route_id': 'inspection_loop',
    'segments': [{'direction': 'counterclockwise', 'repetitions': 2}],
    'speed_mps': 0.18,
    'return_home': True,
}


def proposal_from(content: str) -> MissionProposal:
    return MissionProposal(request_id='request-1', provider='test', content=content)


def proposal_json(**changes: object) -> MissionProposal:
    payload = {**VALID_MISSION, **changes}
    return proposal_from(json.dumps(payload))


def codes(result) -> set:
    return {issue.code for issue in result.errors}


class StructuralValidationTest(unittest.TestCase):
    def test_accepts_canonical_patrol(self) -> None:
        result = validate_proposal(proposal_json())
        self.assertTrue(result.accepted)
        self.assertEqual(errors.STAGE_ACCEPTED, result.stage)
        self.assertIsNotNone(result.mission)
        self.assertEqual(MissionAction.PATROL, result.mission.action)
        self.assertEqual(POLICY_VERSION, result.policy_version)
        self.assertEqual((), result.errors)

    def test_malformed_json_is_rejected(self) -> None:
        result = validate_proposal(proposal_from('{not-json'))
        self.assertFalse(result.accepted)
        self.assertEqual(errors.STAGE_STRUCTURAL, result.stage)
        self.assertEqual({errors.MALFORMED_JSON}, codes(result))
        self.assertEqual('$', result.errors[0].path)

    def test_non_object_json_is_rejected(self) -> None:
        result = validate_proposal(proposal_from('[1, 2, 3]'))
        self.assertFalse(result.accepted)
        self.assertEqual(errors.STAGE_STRUCTURAL, result.stage)
        self.assertEqual({errors.WRONG_TYPE}, codes(result))

    def test_missing_field_is_rejected(self) -> None:
        payload = {k: v for k, v in VALID_MISSION.items() if k != 'route_id'}
        result = validate_proposal(proposal_from(json.dumps(payload)))
        self.assertFalse(result.accepted)
        self.assertEqual({errors.MISSING_FIELD}, codes(result))
        self.assertEqual('$.route_id', result.errors[0].path)

    def test_extra_field_is_rejected(self) -> None:
        result = validate_proposal(proposal_json(shell_command='rm -rf /'))
        self.assertFalse(result.accepted)
        self.assertEqual({errors.UNEXPECTED_FIELD}, codes(result))

    def test_unsupported_action_is_rejected_structurally(self) -> None:
        result = validate_proposal(proposal_json(action='publish_cmd_vel'))
        self.assertFalse(result.accepted)
        self.assertEqual(errors.STAGE_STRUCTURAL, result.stage)
        self.assertEqual({errors.UNSUPPORTED_VALUE}, codes(result))
        self.assertEqual('$.action', result.errors[0].path)

    def test_wrong_type_is_rejected(self) -> None:
        result = validate_proposal(
            proposal_json(
                segments=[{'direction': 'counterclockwise', 'repetitions': '2'}]
            )
        )
        self.assertFalse(result.accepted)
        self.assertEqual({errors.WRONG_TYPE}, codes(result))
        self.assertEqual('$.segments[0].repetitions', result.errors[0].path)

    def test_wrong_type_boolean_is_rejected(self) -> None:
        result = validate_proposal(proposal_json(return_home='yes'))
        self.assertFalse(result.accepted)
        self.assertEqual({errors.WRONG_TYPE}, codes(result))
        self.assertEqual('$.return_home', result.errors[0].path)

    def test_zero_repetitions_is_rejected_by_schema(self) -> None:
        result = validate_proposal(
            proposal_json(
                segments=[{'direction': 'counterclockwise', 'repetitions': 0}]
            )
        )
        self.assertFalse(result.accepted)
        self.assertEqual(errors.STAGE_STRUCTURAL, result.stage)
        self.assertEqual({errors.OUT_OF_RANGE}, codes(result))

    def test_non_positive_speed_is_rejected_by_schema(self) -> None:
        result = validate_proposal(proposal_json(speed_mps=0))
        self.assertFalse(result.accepted)
        self.assertEqual(errors.STAGE_STRUCTURAL, result.stage)
        self.assertEqual({errors.OUT_OF_RANGE}, codes(result))

    def test_non_standard_nan_is_rejected_before_schema(self) -> None:
        content = json.dumps({**VALID_MISSION, 'speed_mps': float('nan')})
        result = validate_proposal(proposal_from(content))
        self.assertFalse(result.accepted)
        self.assertEqual({errors.MALFORMED_JSON}, codes(result))


class SemanticValidationTest(unittest.TestCase):
    def test_unknown_route_is_rejected(self) -> None:
        result = validate_proposal(proposal_json(route_id='unknown_route'))
        self.assertFalse(result.accepted)
        self.assertEqual(errors.STAGE_SEMANTIC, result.stage)
        self.assertEqual({errors.UNKNOWN_ROUTE}, codes(result))
        self.assertEqual('$.route_id', result.errors[0].path)

    def test_excessive_repetitions_is_rejected(self) -> None:
        result = validate_proposal(
            proposal_json(
                segments=[
                    {
                        'direction': 'counterclockwise',
                        'repetitions': MAX_REPETITIONS + 1,
                    }
                ]
            )
        )
        self.assertFalse(result.accepted)
        self.assertEqual(errors.STAGE_SEMANTIC, result.stage)
        self.assertEqual({errors.REPETITIONS_OUT_OF_RANGE}, codes(result))

    def test_total_repetitions_at_limit_is_accepted(self) -> None:
        result = validate_proposal(
            proposal_json(
                segments=[
                    {'direction': 'clockwise', 'repetitions': 5},
                    {'direction': 'counterclockwise', 'repetitions': 5},
                ]
            )
        )
        self.assertTrue(result.accepted)

    def test_total_repetitions_across_segments_is_bounded(self) -> None:
        result = validate_proposal(
            proposal_json(
                segments=[
                    {'direction': 'clockwise', 'repetitions': 6},
                    {'direction': 'counterclockwise', 'repetitions': 5},
                ]
            )
        )
        self.assertFalse(result.accepted)
        self.assertEqual({errors.REPETITIONS_OUT_OF_RANGE}, codes(result))
        self.assertEqual('$.segments', result.errors[0].path)

    def test_excessive_speed_is_rejected(self) -> None:
        result = validate_proposal(proposal_json(speed_mps=MAX_SPEED_MPS + 0.01))
        self.assertFalse(result.accepted)
        self.assertEqual(errors.STAGE_SEMANTIC, result.stage)
        self.assertEqual({errors.SPEED_OUT_OF_RANGE}, codes(result))

    def test_speed_at_limit_is_accepted(self) -> None:
        result = validate_proposal(proposal_json(speed_mps=MAX_SPEED_MPS))
        self.assertTrue(result.accepted)

    def test_multiple_semantic_failures_are_all_reported(self) -> None:
        result = validate_proposal(
            proposal_json(
                route_id='unknown_route',
                segments=[
                    {
                        'direction': 'counterclockwise',
                        'repetitions': MAX_REPETITIONS + 5,
                    }
                ],
                speed_mps=MAX_SPEED_MPS + 1,
            )
        )
        self.assertFalse(result.accepted)
        self.assertEqual(
            {
                errors.UNKNOWN_ROUTE,
                errors.REPETITIONS_OUT_OF_RANGE,
                errors.SPEED_OUT_OF_RANGE,
            },
            codes(result),
        )

    def test_injectable_route_direction_policy(self) -> None:
        result = validate_proposal(
            proposal_json(route_id='alt_loop'),
            route_directions={
                'alt_loop': (TraversalDirection.COUNTERCLOCKWISE,),
            },
        )
        self.assertTrue(result.accepted)

    def test_direction_must_match_route_policy(self) -> None:
        result = validate_proposal(
            proposal_json(
                route_id='aisle_sweep',
                segments=[{'direction': 'clockwise', 'repetitions': 1}],
            )
        )
        self.assertFalse(result.accepted)
        self.assertEqual({errors.UNSUPPORTED_DIRECTION}, codes(result))
        self.assertEqual('$.segments[0].direction', result.errors[0].path)

    def test_aisle_forward_is_accepted(self) -> None:
        result = validate_proposal(
            proposal_json(
                route_id='aisle_sweep',
                segments=[{'direction': 'forward', 'repetitions': 1}],
            )
        )
        self.assertTrue(result.accepted)

    def test_full_area_forward_is_accepted(self) -> None:
        result = validate_proposal(
            proposal_json(
                route_id='full_area_sweep',
                segments=[{'direction': 'forward', 'repetitions': 1}],
            )
        )
        self.assertTrue(result.accepted)


class ResultContractTest(unittest.TestCase):
    def test_structural_failure_skips_semantic_stage(self) -> None:
        # An unknown route combined with malformed structure must stop at the
        # structural stage; semantic checks never run, so no execution can occur.
        result = validate_proposal(proposal_json(action='nope', route_id='ghost'))
        self.assertEqual(errors.STAGE_STRUCTURAL, result.stage)
        self.assertNotIn(errors.UNKNOWN_ROUTE, codes(result))

    def test_rejected_result_cannot_build_validated_mission(self) -> None:
        result = validate_proposal(proposal_json(route_id='unknown_route'))
        with self.assertRaises(ValueError):
            result.build_validated_mission('mission-1')

    def test_accepted_result_builds_validated_mission(self) -> None:
        result = validate_proposal(proposal_json())
        validated = result.build_validated_mission('mission-1')
        self.assertEqual('mission-1', validated.mission_id)
        self.assertEqual('request-1', validated.request_id)
        self.assertEqual(POLICY_VERSION, validated.policy_version)
        self.assertEqual(MissionAction.PATROL, validated.mission.action)

    def test_errors_are_deterministic_and_audit_ready(self) -> None:
        first = validate_proposal(proposal_json(action='x', shell='y'))
        second = validate_proposal(proposal_json(action='x', shell='y'))
        self.assertEqual(first.audit_records(), second.audit_records())
        for record in first.audit_records():
            self.assertEqual({'code', 'message', 'path', 'stage'}, set(record))


if __name__ == '__main__':
    unittest.main()
