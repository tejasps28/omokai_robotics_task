import unittest
from dataclasses import replace

from omokai_executor import GoalKind
from omokai_interfaces import PlanRequest
from omokai_mission import FakePlanner, compile_mission, load_catalog, validate_proposal

from omokai_pipeline import to_execution_plan


def prepared_values():
    catalog = load_catalog()
    proposal = FakePlanner().propose(
        PlanRequest(
            request_id='request-1',
            prompt='Patrol the inspection loop twice and return home.',
        )
    )
    result = validate_proposal(
        proposal,
        route_directions=catalog.direction_policy(),
    )
    validated = result.build_validated_mission('mission-1')
    return validated, compile_mission(validated, catalog)


class ExecutionPlanAdapterTest(unittest.TestCase):
    def test_preserves_order_speed_and_home_semantics(self) -> None:
        validated, compiled = prepared_values()
        plan = to_execution_plan(compiled, validated)

        self.assertEqual(9, len(plan.goals))
        self.assertEqual(0.18, plan.speed_mps)
        self.assertTrue(all(goal.kind is GoalKind.ROUTE for goal in plan.goals[:-1]))
        self.assertEqual(GoalKind.HOME, plan.goals[-1].kind)
        self.assertEqual(-2.0, plan.goals[-1].pose.x)
        self.assertEqual(-0.5, plan.goals[-1].pose.y)

    def test_rejects_mission_identity_mismatch(self) -> None:
        validated, compiled = prepared_values()
        with self.assertRaises(ValueError):
            to_execution_plan(replace(compiled, mission_id='other'), validated)


if __name__ == '__main__':
    unittest.main()
