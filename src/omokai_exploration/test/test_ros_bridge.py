import math
import unittest
from types import SimpleNamespace

from omokai_exploration import Cell, FrontierCandidate
from omokai_exploration.ros_bridge import (
    ExplorationNav2Adapter,
    grid_from_message,
    robot_xy_from_transform,
)


def occupancy_message(*, frame_id='map', yaw=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        info=SimpleNamespace(
            width=2,
            height=2,
            resolution=0.5,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.0, y=2.0),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            ),
        ),
        data=(-1, 0, 50, 100),
    )


def candidate():
    return FrontierCandidate(
        cells=(Cell(0, 0),),
        goal=Cell(0, 0),
        goal_x=1.25,
        goal_y=-0.5,
        distance_m=1.0,
        information_gain_m=0.5,
        score=-0.5,
    )


class FakeDelegate:
    def __init__(self) -> None:
        self.outcomes = None
        self.ready = True
        self.dispatched = []
        self.cancelled = []

    def bind_outcomes(self, outcomes):
        self.outcomes = outcomes

    def server_is_ready(self):
        return self.ready

    def dispatch(self, goal, speed_mps):
        self.dispatched.append((goal, speed_mps))
        return 'nav-token'

    def cancel(self, handle):
        self.cancelled.append(handle)


class RosGridConversionTest(unittest.TestCase):
    def test_converts_message_geometry_and_cells(self) -> None:
        grid = grid_from_message(occupancy_message())

        self.assertEqual(2, grid.metadata.width)
        self.assertEqual(0.5, grid.metadata.resolution)
        self.assertEqual(-1.0, grid.metadata.origin_x)
        self.assertEqual((-1, 0, 50, 100), grid.cells)

    def test_rejects_wrong_frame_or_rotated_origin(self) -> None:
        with self.assertRaises(ValueError):
            grid_from_message(occupancy_message(frame_id='odom'))
        with self.assertRaises(ValueError):
            grid_from_message(occupancy_message(yaw=0.1))

    def test_extracts_finite_robot_position(self) -> None:
        transform = SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.5, y=-2.0)
            )
        )

        self.assertEqual((1.5, -2.0), robot_xy_from_transform(transform))
        transform.transform.translation.x = math.nan
        with self.assertRaises(ValueError):
            robot_xy_from_transform(transform)


class ExplorationNav2AdapterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.delegate = FakeDelegate()
        self.adapter = ExplorationNav2Adapter(
            object(),
            speed_mps=0.12,
            delegate=self.delegate,
        )

    def test_binds_outcomes_and_reports_readiness(self) -> None:
        outcomes = object()
        self.adapter.bind_outcomes(outcomes)

        self.assertIs(outcomes, self.delegate.outcomes)
        self.assertTrue(self.adapter.server_is_ready())

    def test_translates_frontier_to_unique_map_goal(self) -> None:
        first = self.adapter.dispatch(candidate())
        self.adapter.dispatch(candidate())

        goal, speed = self.delegate.dispatched[0]
        self.assertEqual('nav-token', first)
        self.assertEqual('frontier-000001', goal.goal_id)
        self.assertEqual('frontier-000002', self.delegate.dispatched[1][0].goal_id)
        self.assertEqual('map', goal.pose.frame_id)
        self.assertEqual(1.25, goal.pose.x)
        self.assertEqual(-0.5, goal.pose.y)
        self.assertEqual(0.12, speed)

    def test_forwards_cancellation(self) -> None:
        self.adapter.cancel('nav-token')

        self.assertEqual(['nav-token'], self.delegate.cancelled)

    def test_rejects_invalid_speed(self) -> None:
        with self.assertRaises(ValueError):
            ExplorationNav2Adapter(
                object(),
                speed_mps=0.0,
                delegate=self.delegate,
            )


if __name__ == '__main__':
    unittest.main()
