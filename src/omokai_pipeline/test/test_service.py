import json
import tempfile
import unittest
from pathlib import Path

from omokai_executor import MissionArtifactWriter
from omokai_interfaces import MissionProposal, PlanRequest
from omokai_mission import FakePlanner, load_catalog

from omokai_pipeline import prepare_mission, prepare_proposal


class RejectingPlanner:
    provider = 'rejecting-test'

    def propose(self, request):
        return MissionProposal(
            request_id=request.request_id,
            provider=self.provider,
            content='{"action":"unsupported"}',
        )


class PipelineServiceTest(unittest.TestCase):
    def test_prepares_demo_and_writes_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            result = prepare_mission(
                request=PlanRequest(
                    request_id='request-1',
                    prompt='Patrol the inspection loop twice and return home.',
                ),
                mission_id='mission-1',
                planner=FakePlanner(),
                catalog=load_catalog(),
                artifact_writer=MissionArtifactWriter(Path(directory), fsync=False),
            )
            self.assertTrue(result.accepted)
            mission_dir = Path(directory) / 'mission-1'
            self.assertEqual(
                {'proposal.json', 'validation.json', 'accepted_mission.json'},
                {path.name for path in mission_dir.iterdir()},
            )
            accepted = json.loads((mission_dir / 'accepted_mission.json').read_text())
            self.assertEqual(9, len(accepted['goals']))

    def test_rejection_never_creates_execution_plan(self) -> None:
        result = prepare_mission(
            request=PlanRequest(request_id='request-1', prompt='unsupported'),
            mission_id='mission-1',
            planner=RejectingPlanner(),
            catalog=load_catalog(),
        )
        self.assertFalse(result.accepted)
        self.assertIsNone(result.preparation)

    def test_prepares_an_existing_proposal_without_replanning(self) -> None:
        request = PlanRequest(
            request_id='request-1',
            prompt='Patrol the inspection loop once and return home.',
        )
        proposal = MissionProposal(
            request_id=request.request_id,
            provider='approved:test-provider',
            content=json.dumps(
                {
                    'schema_version': '1.1',
                    'action': 'patrol',
                    'route_id': 'inspection_loop',
                    'segments': [
                        {'direction': 'counterclockwise', 'repetitions': 1}
                    ],
                    'speed_mps': 0.2,
                    'return_home': True,
                },
                sort_keys=True,
            ),
        )

        result = prepare_proposal(
            request=request,
            mission_id='mission-1',
            proposal=proposal,
            catalog=load_catalog(),
        )

        self.assertTrue(result.accepted)
        self.assertIs(result.proposal, proposal)
        self.assertEqual(5, len(result.preparation.execution_plan.goals))

    def test_existing_proposal_must_match_the_request(self) -> None:
        with self.assertRaisesRegex(ValueError, 'request ID'):
            prepare_proposal(
                request=PlanRequest(request_id='request-1', prompt='Patrol.'),
                mission_id='mission-1',
                proposal=MissionProposal(
                    request_id='different-request',
                    provider='test-provider',
                    content='{}',
                ),
                catalog=load_catalog(),
            )


if __name__ == '__main__':
    unittest.main()
