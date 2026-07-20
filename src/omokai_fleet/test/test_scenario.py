import unittest

from omokai_fleet import Formation, SquadAction, SquadPlan
from omokai_fleet.scenario import build_demo_mission


class DemoScenarioTest(unittest.TestCase):
    def test_builds_complete_separated_demo(self) -> None:
        mission = build_demo_mission(
            SquadPlan(
                plan_id='demo',
                action=SquadAction.FORMATION_PATROL,
                formation=Formation.WEDGE,
                route_id='inspection_loop',
                spacing_m=0.6,
                speed_mps=0.12,
                split_route=True,
                regroup=True,
                regroup_location='home',
            )
        )

        self.assertEqual(3, len(mission.forming))
        self.assertEqual(1, len(mission.formation_movement))
        self.assertEqual(2, len(mission.split_execution))
        self.assertEqual(3, len(mission.regrouping))
        self.assertEqual(
            6,
            len(
                {
                    goal.goal_id
                    for batch in mission.split_execution
                    for goal in batch
                }
            ),
        )


if __name__ == '__main__':
    unittest.main()
