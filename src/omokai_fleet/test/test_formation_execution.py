import unittest

from omokai_fleet.formation_execution import (
    FormationDecision,
    decide_formation_execution,
)


class FormationExecutionDecisionTest(unittest.TestCase):
    def test_success_requires_leader_and_followers(self) -> None:
        self.assertIs(
            FormationDecision.SUCCEED,
            decide_formation_execution(
                leader_succeeded=True,
                followers_settled=True,
            ),
        )

    def test_leader_failure_is_fail_fast(self) -> None:
        self.assertIs(
            FormationDecision.FAIL,
            decide_formation_execution(
                leader_succeeded=False,
                followers_settled=False,
            ),
        )

    def test_unsettled_follower_times_out(self) -> None:
        self.assertIs(
            FormationDecision.TIMEOUT,
            decide_formation_execution(
                leader_succeeded=True,
                followers_settled=False,
                deadline_expired=True,
            ),
        )

    def test_blocked_tf_fails_before_timeout(self) -> None:
        self.assertIs(
            FormationDecision.FAIL,
            decide_formation_execution(
                leader_succeeded=True,
                followers_settled=False,
                blocked_reason='robot2 TF unavailable',
                deadline_expired=True,
            ),
        )

    def test_operator_cancel_has_highest_precedence(self) -> None:
        self.assertIs(
            FormationDecision.CANCEL,
            decide_formation_execution(
                leader_succeeded=True,
                followers_settled=False,
                blocked_reason='robot2 TF unavailable',
                operator_cancelled=True,
            ),
        )


if __name__ == '__main__':
    unittest.main()
