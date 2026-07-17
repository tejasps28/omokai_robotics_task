import math
import unittest
from types import SimpleNamespace

try:
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Time
except ModuleNotFoundError as exc:
    raise unittest.SkipTest(f'ROS message packages are unavailable: {exc}')

from omokai_executor import ExecutionGoal, GoalPose
from omokai_executor.nav2_adapter import Nav2NavigationAdapter


class FakeFuture:
    def __init__(self) -> None:
        self._callbacks = []
        self._result = None
        self._exception = None

    def add_done_callback(self, callback) -> None:
        self._callbacks.append(callback)

    def complete(self, result=None, exception=None) -> None:
        self._result = result
        self._exception = exception
        for callback in list(self._callbacks):
            callback(self)

    def result(self):
        if self._exception:
            raise self._exception
        return self._result


class FakeRosGoalHandle:
    def __init__(self, *, accepted=True) -> None:
        self.accepted = accepted
        self.result_future = FakeFuture()
        self.cancel_future = FakeFuture()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


class FakeActionClient:
    def __init__(self) -> None:
        self.ready = True
        self.sent_goals = []
        self.send_future = FakeFuture()

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, request):
        self.sent_goals.append(request)
        return self.send_future


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def get_clock(self):
        return self

    def now(self):
        return self

    def to_msg(self):
        return Time(sec=123, nanosec=0)


class FakeOutcomes:
    def __init__(self) -> None:
        self.succeeded = []
        self.failed = []
        self.cancelled = []
        self.cancel_failed = []

    def navigation_succeeded(self, token):
        self.succeeded.append(token)

    def navigation_failed(self, token, reason):
        self.failed.append((token, reason))

    def cancellation_confirmed(self, token):
        self.cancelled.append(token)

    def cancellation_failed(self, token, reason):
        self.cancel_failed.append((token, reason))


def execution_goal():
    return ExecutionGoal('route-0', GoalPose(1.25, -0.5, math.pi / 2.0))


class Nav2AdapterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.client = FakeActionClient()
        self.publisher = FakePublisher()
        self.outcomes = FakeOutcomes()
        self.adapter = Nav2NavigationAdapter(
            FakeNode(),
            action_client=self.client,
            speed_limit_publisher=self.publisher,
        )
        self.adapter.bind_outcomes(self.outcomes)

    def dispatch(self):
        return self.adapter.dispatch(execution_goal(), 0.18)

    def test_translates_pose_and_reports_success(self) -> None:
        token = self.dispatch()
        request = self.client.sent_goals[0]
        self.assertEqual('map', request.pose.header.frame_id)
        self.assertAlmostEqual(1.25, request.pose.pose.position.x)
        self.assertAlmostEqual(math.sin(math.pi / 4.0), request.pose.pose.orientation.z)
        self.assertEqual(1, len(self.publisher.messages))
        self.assertFalse(self.publisher.messages[0].percentage)
        self.assertAlmostEqual(0.18, self.publisher.messages[0].speed_limit)

        ros_handle = FakeRosGoalHandle()
        self.client.send_future.complete(ros_handle)
        ros_handle.result_future.complete(
            SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED,
                result=SimpleNamespace(error_code=0, error_msg=''),
            )
        )
        self.assertEqual([token], self.outcomes.succeeded)

    def test_rejects_dispatch_when_server_is_unavailable(self) -> None:
        self.client.ready = False
        self.assertFalse(self.adapter.server_is_ready())
        with self.assertRaises(RuntimeError):
            self.dispatch()

    def test_reports_server_readiness(self) -> None:
        self.assertTrue(self.adapter.server_is_ready())

    def test_reports_rejected_goal(self) -> None:
        token = self.dispatch()
        self.client.send_future.complete(FakeRosGoalHandle(accepted=False))
        self.assertEqual(token, self.outcomes.failed[0][0])
        self.assertEqual('goal_rejected', self.outcomes.failed[0][1])

    def test_cancellation_requested_before_goal_acceptance(self) -> None:
        token = self.dispatch()
        self.adapter.cancel(token)
        ros_handle = FakeRosGoalHandle()
        self.client.send_future.complete(ros_handle)
        self.assertEqual(1, ros_handle.cancel_calls)
        ros_handle.cancel_future.complete(SimpleNamespace(goals_canceling=[object()]))
        ros_handle.result_future.complete(
            SimpleNamespace(
                status=GoalStatus.STATUS_CANCELED,
                result=SimpleNamespace(error_code=0, error_msg=''),
            )
        )
        self.assertEqual([token], self.outcomes.cancelled)

    def test_reports_rejected_cancellation(self) -> None:
        token = self.dispatch()
        ros_handle = FakeRosGoalHandle()
        self.client.send_future.complete(ros_handle)
        self.adapter.cancel(token)
        ros_handle.cancel_future.complete(SimpleNamespace(goals_canceling=[]))
        self.assertEqual(token, self.outcomes.cancel_failed[0][0])

    def test_reports_nav2_abort_details(self) -> None:
        token = self.dispatch()
        ros_handle = FakeRosGoalHandle()
        self.client.send_future.complete(ros_handle)
        ros_handle.result_future.complete(
            SimpleNamespace(
                status=GoalStatus.STATUS_ABORTED,
                result=SimpleNamespace(error_code=42, error_msg='blocked'),
            )
        )
        self.assertEqual(token, self.outcomes.failed[0][0])
        self.assertIn('error_code=42', self.outcomes.failed[0][1])


if __name__ == '__main__':
    unittest.main()
