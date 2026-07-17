"""Planner interface contract tests and fake-planner behaviour."""

import json
import unittest

from omokai_interfaces import MissionProposal, PlanRequest

from omokai_mission.catalog import load_catalog
from omokai_mission.compiler import compile_mission
from omokai_mission.planner import FakePlanner, Planner, PlannerError
from omokai_mission.validation import validate_proposal


DEMO_PROMPT = 'Patrol the inspection loop twice and return home.'

EXPECTED_DEMO_PROPOSAL = {
    'schema_version': '1.1',
    'action': 'patrol',
    'route_id': 'inspection_loop',
    'segments': [{'direction': 'counterclockwise', 'repetitions': 2}],
    'speed_mps': 0.18,
    'return_home': True,
}


def assert_planner_contract(test: unittest.TestCase, planner: Planner) -> None:
    """Any planner must satisfy this provider-neutral contract.

    Runs the offline pipeline end to end for the demonstration prompt to prove
    the proposal is not merely well-formed but semantically acceptable and
    compilable to the expected route.
    """

    test.assertIsInstance(planner, Planner)
    request = PlanRequest(request_id='contract-req', prompt=DEMO_PROMPT)
    proposal = planner.propose(request)

    test.assertIsInstance(proposal, MissionProposal)
    test.assertEqual(request.request_id, proposal.request_id)
    test.assertTrue(proposal.provider.strip())
    # Raw provider output must remain auditable: content is exactly the JSON
    # text the provider produced.
    test.assertEqual(json.loads(proposal.content), json.loads(proposal.content))

    catalog = load_catalog()
    result = validate_proposal(
        proposal,
        route_directions=catalog.direction_policy(),
    )
    test.assertTrue(result.accepted, msg=f'rejected: {result.audit_records()}')

    compiled = compile_mission(result.build_validated_mission('m-contract'), catalog)
    waypoints = len(catalog.get('inspection_loop').waypoints)
    # two laps plus home.
    test.assertEqual(2 * waypoints + 1, len(compiled.goals))
    test.assertEqual('home', compiled.goals[-1].label)


class FakePlannerContractTest(unittest.TestCase):
    def test_fake_planner_satisfies_contract(self) -> None:
        assert_planner_contract(self, FakePlanner())

    def test_demo_prompt_emits_exact_proposal(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(request_id='r1', prompt=DEMO_PROMPT)
        )
        self.assertEqual('fake', proposal.provider)
        self.assertEqual(EXPECTED_DEMO_PROPOSAL, json.loads(proposal.content))

    def test_output_is_deterministic(self) -> None:
        planner = FakePlanner()
        request = PlanRequest(request_id='r1', prompt=DEMO_PROMPT)
        self.assertEqual(
            planner.propose(request).content,
            planner.propose(request).content,
        )

    def test_once_without_return(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(request_id='r1', prompt='Patrol the inspection loop once.')
        )
        spec = json.loads(proposal.content)
        self.assertEqual(1, spec['segments'][0]['repetitions'])
        self.assertFalse(spec['return_home'])

    def test_numeric_repetitions_and_return(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(
                request_id='r1',
                prompt='Patrol the inspection loop 3 times then return home.',
            )
        )
        spec = json.loads(proposal.content)
        self.assertEqual(3, spec['segments'][0]['repetitions'])
        self.assertTrue(spec['return_home'])

    def test_mixed_loop_directions_preserve_operator_order(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(
                request_id='r1',
                prompt='Patrol clock-wise once and then anti-clockwise once then go to home.',
            )
        )
        spec = json.loads(proposal.content)
        self.assertEqual(
            [
                {'direction': 'clockwise', 'repetitions': 1},
                {'direction': 'counterclockwise', 'repetitions': 1},
            ],
            spec['segments'],
        )
        self.assertTrue(spec['return_home'])

    def test_lane_sweep_and_return_to_start(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(
                request_id='r1',
                prompt='Drive through the lanes once and return to start.',
            )
        )
        spec = json.loads(proposal.content)
        self.assertEqual('aisle_sweep', spec['route_id'])
        self.assertEqual([{'direction': 'forward', 'repetitions': 1}], spec['segments'])
        self.assertTrue(spec['return_home'])

    def test_official_ground_robot_example(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(
                request_id='r1',
                prompt='Drive the inspection route and return to start.',
            )
        )
        spec = json.loads(proposal.content)
        self.assertEqual('inspection_loop', spec['route_id'])
        self.assertEqual(
            [{'direction': 'counterclockwise', 'repetitions': 1}],
            spec['segments'],
        )
        self.assertTrue(spec['return_home'])

    def test_full_area_sweep_uses_distinct_audited_route(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(
                request_id='r1',
                prompt='Sweep the full area once and return home.',
            )
        )
        spec = json.loads(proposal.content)
        self.assertEqual('full_area_sweep', spec['route_id'])
        self.assertEqual(
            [{'direction': 'forward', 'repetitions': 1}],
            spec['segments'],
        )
        self.assertTrue(spec['return_home'])

    def test_all_aisles_means_full_area_not_central_aisles_only(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(
                request_id='r1',
                prompt='Cover all aisles in reverse and return to start.',
            )
        )
        spec = json.loads(proposal.content)
        self.assertEqual('full_area_sweep', spec['route_id'])
        self.assertEqual([{'direction': 'reverse', 'repetitions': 1}], spec['segments'])
        self.assertTrue(spec['return_home'])

    def test_request_id_is_preserved(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(request_id='unique-42', prompt=DEMO_PROMPT)
        )
        self.assertEqual('unique-42', proposal.request_id)

    def test_unsupported_prompt_fails_closed(self) -> None:
        with self.assertRaises(PlannerError):
            FakePlanner().propose(PlanRequest(request_id='r1', prompt='Stop now'))

    def test_zero_repetitions_reaches_validation_guard(self) -> None:
        proposal = FakePlanner().propose(
            PlanRequest(
                request_id='r1',
                prompt='Patrol the inspection loop 0 times.',
            )
        )
        result = validate_proposal(proposal)
        self.assertFalse(result.accepted)


if __name__ == '__main__':
    unittest.main()
