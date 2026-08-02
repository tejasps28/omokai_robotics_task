"""Fleet pose freshness regression tests."""

from __future__ import annotations

import unittest
from unittest.mock import Mock

try:
    from geometry_msgs.msg import TransformStamped
    from omokai_fleet.pose_tracker import FleetPoseTracker
    from omokai_fleet.model import ROBOT_IDS
    ROS_AVAILABLE = True
except ModuleNotFoundError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, 'ROS fleet pose dependencies unavailable')
class FleetPoseTrackerRefreshTest(unittest.TestCase):
    def test_valid_tf_refresh_keeps_quiet_amcl_poses_fresh(self) -> None:
        now = [100.0]
        tracker = object.__new__(FleetPoseTracker)
        tracker._clock = lambda: now[0]
        tracker._maximum_age_sec = 5.0
        tracker._poses = {}
        tracker._pose_messages = {}
        tracker._received_at = {}
        tracker._node = Mock()
        tracker._publisher = Mock()

        transforms = {}
        for index, robot_id in enumerate(ROBOT_IDS):
            message = TransformStamped()
            message.transform.translation.x = float(index)
            message.transform.translation.y = 0.0
            message.transform.rotation.w = 1.0
            transforms[f'{robot_id.value}/base_link'] = message

        tracker._tf_buffer = Mock()
        tracker._tf_buffer.lookup_transform.side_effect = (
            lambda target, source, time: transforms[source]
        )

        tracker._refresh_from_tf()
        self.assertIsNone(tracker.readiness_reason())

        # AMCL may remain quiet while a stationary robot is healthy. Repeated
        # valid TF refreshes must keep every pose within the safety age bound.
        now[0] += 10.0
        tracker._refresh_from_tf()
        self.assertIsNone(tracker.readiness_reason())
        self.assertEqual(set(ROBOT_IDS), set(tracker._poses))


if __name__ == '__main__':
    unittest.main()
