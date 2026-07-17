"""Gemini planner adapter tests.

These tests use a fake transport. They do not require network access or an API
key and do not verify Gemini service behaviour.
"""

import json
import unittest
from typing import Any, Mapping

from omokai_interfaces import MissionProposal, PlanRequest

from omokai_mission.gemini_planner import GeminiPlanner, GeminiPlannerError
from omokai_mission.planner import Planner
from omokai_mission.validation import validate_proposal


DEMO_SPEC = {
    'schema_version': '1.1',
    'action': 'patrol',
    'route_id': 'inspection_loop',
    'segments': [{'direction': 'counterclockwise', 'repetitions': 2}],
    'speed_mps': 0.18,
    'return_home': True,
}


class FakeTransport:
    def __init__(self, response: Mapping[str, Any]) -> None:
        self.response = response
        self.calls = []

    def generate_content(
        self,
        *,
        api_key: str,
        model: str,
        payload: Mapping[str, Any],
        timeout_sec: float,
    ) -> Mapping[str, Any]:
        self.calls.append(
            {
                'api_key': api_key,
                'model': model,
                'payload': payload,
                'timeout_sec': timeout_sec,
            }
        )
        return self.response


class GeminiPlannerTest(unittest.TestCase):
    def test_satisfies_planner_protocol(self) -> None:
        planner = GeminiPlanner(
            api_key='test-key',
            model='test-model',
            transport=FakeTransport({'output_text': json.dumps(DEMO_SPEC)}),
        )
        self.assertIsInstance(planner, Planner)

    def test_builds_structured_output_payload(self) -> None:
        transport = FakeTransport({'output_text': json.dumps(DEMO_SPEC)})
        planner = GeminiPlanner(
            api_key='test-key',
            model='test-model',
            transport=transport,
            timeout_sec=12.5,
            max_output_tokens=321,
        )
        proposal = planner.propose(
            PlanRequest(
                request_id='req-1',
                prompt='Patrol the inspection loop twice and return home.',
            )
        )

        self.assertIsInstance(proposal, MissionProposal)
        self.assertEqual('req-1', proposal.request_id)
        self.assertEqual('gemini:test-model', proposal.provider)
        self.assertEqual(DEMO_SPEC, json.loads(proposal.content))

        call = transport.calls[0]
        self.assertEqual('test-key', call['api_key'])
        self.assertEqual('test-model', call['model'])
        self.assertEqual(12.5, call['timeout_sec'])
        payload = call['payload']
        text = payload['contents'][0]['parts'][0]['text']
        self.assertIn('Operator request:', text)
        self.assertIn('clockwise', text)
        self.assertIn('aisle_sweep', text)
        self.assertIn('full_area_sweep', text)
        generation_config = payload['generationConfig']
        self.assertEqual(321, generation_config['maxOutputTokens'])
        self.assertEqual('application/json', generation_config['responseMimeType'])
        schema = generation_config['responseSchema']
        self.assertNotIn('additionalProperties', schema)
        self.assertEqual('string', schema['properties']['schema_version']['type'])
        self.assertEqual(['1.1'], schema['properties']['schema_version']['enum'])
        self.assertEqual('string', schema['properties']['action']['type'])
        self.assertNotIn('exclusiveMinimum', schema['properties']['speed_mps'])

    def test_output_still_passes_local_validator(self) -> None:
        planner = GeminiPlanner(
            api_key='test-key',
            model='test-model',
            transport=FakeTransport({'output_text': json.dumps(DEMO_SPEC)}),
        )
        proposal = planner.propose(
            PlanRequest(
                request_id='req-1',
                prompt='Patrol the inspection loop twice and return home.',
            )
        )
        result = validate_proposal(proposal)
        self.assertTrue(result.accepted, msg=result.audit_records())

    def test_candidate_response_output_shape_is_supported(self) -> None:
        transport = FakeTransport(
            {
                'candidates': [
                    {
                        'content': {
                            'parts': [
                                {
                                    'text': json.dumps(DEMO_SPEC),
                                }
                            ]
                        }
                    }
                ]
            }
        )
        proposal = GeminiPlanner(
            api_key='test-key',
            model='test-model',
            transport=transport,
        ).propose(PlanRequest(request_id='req-1', prompt='Patrol twice.'))
        self.assertEqual(DEMO_SPEC, json.loads(proposal.content))

    def test_missing_api_key_fails_before_transport(self) -> None:
        transport = FakeTransport({'output_text': json.dumps(DEMO_SPEC)})
        planner = GeminiPlanner(api_key='', model='test-model', transport=transport)
        with self.assertRaisesRegex(GeminiPlannerError, 'GEMINI_API_KEY'):
            planner.propose(PlanRequest(request_id='req-1', prompt='Patrol.'))
        self.assertEqual([], transport.calls)

    def test_non_json_output_fails_closed(self) -> None:
        planner = GeminiPlanner(
            api_key='test-key',
            model='test-model',
            transport=FakeTransport({'output_text': 'not json'}),
        )
        with self.assertRaisesRegex(GeminiPlannerError, 'not JSON'):
            planner.propose(PlanRequest(request_id='req-1', prompt='Patrol.'))

    def test_missing_output_text_fails_closed(self) -> None:
        planner = GeminiPlanner(
            api_key='test-key',
            model='test-model',
            transport=FakeTransport({'id': 'interaction_123'}),
        )
        with self.assertRaisesRegex(GeminiPlannerError, 'output text'):
            planner.propose(PlanRequest(request_id='req-1', prompt='Patrol.'))

    def test_blocked_prompt_fails_closed(self) -> None:
        planner = GeminiPlanner(
            api_key='test-key',
            model='test-model',
            transport=FakeTransport(
                {'promptFeedback': {'blockReason': 'SAFETY'}}
            ),
        )
        with self.assertRaisesRegex(GeminiPlannerError, 'blocked.*SAFETY'):
            planner.propose(PlanRequest(request_id='req-1', prompt='Patrol.'))

    def test_candidate_without_json_reports_finish_reason(self) -> None:
        planner = GeminiPlanner(
            api_key='test-key',
            model='test-model',
            transport=FakeTransport(
                {
                    'candidates': [
                        {
                            'finishReason': 'MAX_TOKENS',
                            'content': {'parts': []},
                        }
                    ]
                }
            ),
        )
        with self.assertRaisesRegex(GeminiPlannerError, 'MAX_TOKENS'):
            planner.propose(PlanRequest(request_id='req-1', prompt='Patrol.'))


if __name__ == '__main__':
    unittest.main()
