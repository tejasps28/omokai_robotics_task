import json
import unittest

from omokai_interfaces import MissionProposal, PlanRequest

from omokai_fleet import (
    FakeSquadPlanner,
    Formation,
    SquadPlannerError,
    validate_squad_proposal,
)
from omokai_fleet.gemini_planner import GeminiSquadPlanner


PROMPT = 'You three split the inspection route in a wedge and regroup home.'


class FakeSquadPlannerTest(unittest.TestCase):
    def test_produces_deterministic_valid_wedge_plan(self) -> None:
        request = PlanRequest('request-1', PROMPT)
        planner = FakeSquadPlanner()

        first = planner.propose(request)
        second = planner.propose(request)
        result = validate_squad_proposal(first, plan_id='plan-1')

        self.assertEqual(first, second)
        self.assertTrue(result.accepted)
        self.assertEqual(Formation.WEDGE, result.plan.formation)
        self.assertTrue(result.plan.split_route)
        self.assertTrue(result.plan.regroup)

    def test_supports_line_without_optional_phases(self) -> None:
        proposal = FakeSquadPlanner().propose(
            PlanRequest('request-2', 'Move the three robots in a line.')
        )
        result = validate_squad_proposal(proposal, plan_id='plan-2')

        self.assertTrue(result.accepted)
        self.assertEqual(Formation.LINE, result.plan.formation)
        self.assertFalse(result.plan.split_route)
        self.assertFalse(result.plan.regroup)

    def test_different_rooms_requests_split_assignment(self) -> None:
        proposal = FakeSquadPlanner().propose(
            PlanRequest(
                'request-rooms',
                'Form a wedge and send each robot to a different room.',
            )
        )
        result = validate_squad_proposal(proposal, plan_id='plan-rooms')

        self.assertTrue(result.accepted)
        self.assertTrue(result.plan.split_route)

    def test_rejects_prompt_without_supported_formation(self) -> None:
        with self.assertRaises(SquadPlannerError):
            FakeSquadPlanner().propose(
                PlanRequest('request-3', 'Send the robots somewhere.')
            )


class SquadValidationTest(unittest.TestCase):
    def proposal(self, **changes) -> MissionProposal:
        value = json.loads(
            FakeSquadPlanner().propose(
                PlanRequest('request-4', PROMPT)
            ).content
        )
        value.update(changes)
        return MissionProposal('request-4', 'test', json.dumps(value))

    def test_rejects_malformed_json(self) -> None:
        result = validate_squad_proposal(
            MissionProposal('request-5', 'test', '{'),
            plan_id='plan-5',
        )

        self.assertFalse(result.accepted)
        self.assertEqual('$', result.issues[0].path)

    def test_rejects_extra_coordinates(self) -> None:
        result = validate_squad_proposal(
            self.proposal(goal={'x': 1.0, 'y': 2.0}),
            plan_id='plan-6',
        )

        self.assertFalse(result.accepted)
        self.assertIn('Additional properties', result.issues[0].message)

    def test_rejects_unsafe_speed_and_spacing(self) -> None:
        result = validate_squad_proposal(
            self.proposal(speed_mps=1.0, spacing_m=0.2),
            plan_id='plan-7',
        )

        self.assertFalse(result.accepted)
        self.assertEqual(2, len(result.issues))

    def test_rejects_unverified_live_spacing(self) -> None:
        result = validate_squad_proposal(
            self.proposal(spacing_m=0.8),
            plan_id='plan-spacing',
        )

        self.assertFalse(result.accepted)
        self.assertIn('maximum of 0.6', result.issues[0].message)

    def test_rejects_unknown_route_formation_and_rendezvous(self) -> None:
        result = validate_squad_proposal(
            self.proposal(
                route_id='warehouse',
                formation='circle',
                regroup_location='charging_station',
            ),
            plan_id='plan-8',
        )

        self.assertFalse(result.accepted)
        self.assertEqual(3, len(result.issues))

    def test_rejects_wrong_boolean_type(self) -> None:
        result = validate_squad_proposal(
            self.proposal(split_route='yes'),
            plan_id='plan-9',
        )

        self.assertFalse(result.accepted)


class FakeGeminiTransport:
    def __init__(self, output):
        self.output = output
        self.calls = []

    def generate_content(self, **kwargs):
        self.calls.append(kwargs)
        return {'output_text': json.dumps(self.output)}


class GeminiSquadPlannerTest(unittest.TestCase):
    def test_requests_structured_squad_output(self) -> None:
        output = json.loads(
            FakeSquadPlanner().propose(
                PlanRequest('request-10', PROMPT)
            ).content
        )
        transport = FakeGeminiTransport(output)
        planner = GeminiSquadPlanner(
            api_key='test-key',
            model='gemini-test',
            transport=transport,
        )

        proposal = planner.propose(PlanRequest('request-10', PROMPT))
        result = validate_squad_proposal(proposal, plan_id='plan-10')
        config = transport.calls[0]['payload']['generationConfig']

        self.assertTrue(result.accepted)
        self.assertEqual('gemini-squad:gemini-test', proposal.provider)
        self.assertEqual('application/json', config['responseMimeType'])
        self.assertNotIn(
            'additionalProperties',
            config['responseSchema'],
        )
        self.assertEqual(
            ['formation_patrol'],
            config['responseSchema']['properties']['action']['enum'],
        )

    def test_provider_output_still_requires_local_validation(self) -> None:
        transport = FakeGeminiTransport(
            {
                'schema_version': '1.0',
                'action': 'formation_patrol',
                'formation': 'wedge',
                'route_id': 'inspection_loop',
                'spacing_m': 0.1,
                'speed_mps': 0.12,
                'split_route': True,
                'regroup': True,
                'regroup_location': 'home',
            }
        )
        proposal = GeminiSquadPlanner(
            api_key='test-key',
            transport=transport,
        ).propose(PlanRequest('request-11', PROMPT))

        result = validate_squad_proposal(proposal, plan_id='plan-11')

        self.assertFalse(result.accepted)


if __name__ == '__main__':
    unittest.main()
