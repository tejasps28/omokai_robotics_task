import unittest

from omokai_fleet import Formation, RobotId, SquadAction, SquadPlan
from omokai_fleet.scenario import (
    DOCKING_POSES,
    FORMATION_PATROL_REFERENCE,
    ROOM_TARGETS,
    build_demo_mission,
)


class DemoScenarioTest(unittest.TestCase):
    def build_mission(self, formation: Formation):
        return build_demo_mission(
            SquadPlan(
                plan_id='demo',
                action=SquadAction.FORMATION_PATROL,
                formation=formation,
                route_id='inspection_loop',
                spacing_m=0.6,
                speed_mps=0.12,
                split_route=True,
                regroup=True,
                regroup_location='home',
            )
        )

    def test_builds_complete_separated_wedge_demo(self) -> None:
        mission = self.build_mission(Formation.WEDGE)

        self.assertEqual(3, len(mission.forming))
        self.assertEqual(1, len(mission.formation_movement))
        self.assertEqual(6, len(mission.split_execution))
        self.assertEqual(3, len(mission.regrouping))
        self.assertEqual(
            18,
            len(
                {
                    goal.goal_id
                    for batch in mission.split_execution
                    for goal in batch
                }
            ),
        )

    def test_each_robot_is_assigned_to_a_different_room(self) -> None:
        for formation in (Formation.LINE, Formation.WEDGE):
            with self.subTest(formation=formation.value):
                mission = self.build_mission(formation)
                room_batches = mission.split_execution[:3]

                self.assertEqual(
                    (RobotId.ROBOT1, RobotId.ROBOT2, RobotId.ROBOT3),
                    tuple(goal.robot_id for goal in room_batches[-1]),
                )
                self.assertEqual(
                    tuple(point.pose for point in ROOM_TARGETS),
                    tuple(goal.pose for goal in room_batches[-1]),
                )
                self.assertEqual(3, len({goal.pose for goal in room_batches[-1]}))

    def test_default_wedge_starts_and_ends_at_window_dock(self) -> None:
        mission = self.build_mission(Formation.WEDGE)

        self.assertEqual(DOCKING_POSES, tuple(goal.pose for goal in mission.forming))
        for dock, regroup in zip(DOCKING_POSES, mission.regrouping):
            self.assertAlmostEqual(dock.x, regroup.pose.x)
            self.assertAlmostEqual(dock.y, regroup.pose.y)
            self.assertAlmostEqual(dock.yaw, regroup.pose.yaw)

    def test_window_dock_has_safe_distinct_parking_poses(self) -> None:
        self.assertEqual(3, len(set(DOCKING_POSES)))

        distances = (
            ((first.x - second.x) ** 2 + (first.y - second.y) ** 2) ** 0.5
            for index, first in enumerate(DOCKING_POSES)
            for second in DOCKING_POSES[index + 1:]
        )
        self.assertTrue(all(distance >= 0.6 for distance in distances))

    def test_formation_phase_has_a_real_leader_movement_leg(self) -> None:
        mission = self.build_mission(Formation.WEDGE)

        leader_goal = mission.formation_movement[0][0]
        self.assertEqual(FORMATION_PATROL_REFERENCE, leader_goal.pose)
        self.assertNotEqual(mission.forming[0].pose, leader_goal.pose)

    def test_room_assignment_preserves_upper_and_lower_dock_lanes(self) -> None:
        self.assertEqual(
            ('main_room', 'lower_room', 'upper_room'),
            tuple(point.point_id for point in ROOM_TARGETS),
        )
        self.assertLess(DOCKING_POSES[1].y, 0.0)
        self.assertLess(ROOM_TARGETS[1].pose.y, 0.0)
        self.assertGreater(DOCKING_POSES[2].y, 0.0)
        self.assertGreater(ROOM_TARGETS[2].pose.y, 0.0)

    def test_main_room_hold_is_clear_of_central_return_corridor(self) -> None:
        main_room = ROOM_TARGETS[0].pose

        self.assertGreater(main_room.x, 3.0)
        self.assertGreater(main_room.y, 3.0)

    def test_nearest_robot_moves_first_outbound(self) -> None:
        mission = self.build_mission(Formation.WEDGE)

        self.assertIn(
            '/robot1/main_room',
            mission.split_execution[0][0].goal_id,
        )
        self.assertIn('/robot2/lower_room', mission.split_execution[1][1].goal_id)
        self.assertIn('/robot3/upper_room', mission.split_execution[2][2].goal_id)

    def test_outer_dock_slots_are_filled_before_apex(self) -> None:
        mission = self.build_mission(Formation.WEDGE)

        self.assertIn('/robot2/dock', mission.split_execution[3][1].goal_id)
        self.assertIn('/robot3/dock', mission.split_execution[4][2].goal_id)
        self.assertIn('/robot1/dock', mission.split_execution[5][0].goal_id)

    def test_each_release_holds_every_other_robot(self) -> None:
        mission = self.build_mission(Formation.WEDGE)

        for batch in mission.split_execution:
            self.assertEqual(
                1,
                sum(not goal.goal_id.endswith('/hold') for goal in batch),
            )

    def test_every_goal_stays_inside_the_house(self) -> None:
        for formation in (Formation.LINE, Formation.WEDGE):
            mission = self.build_mission(formation)
            batches = (
                (mission.forming,)
                + mission.formation_movement
                + mission.split_execution
                + (mission.regrouping,)
            )
            for batch in batches:
                for goal in batch:
                    self.assertLessEqual(abs(goal.pose.x), 7.2)
                    self.assertLessEqual(abs(goal.pose.y), 7.8)


if __name__ == '__main__':
    unittest.main()
