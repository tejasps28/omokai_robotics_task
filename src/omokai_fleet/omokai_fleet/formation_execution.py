"""Pure decisions for the final continuous-formation convergence barrier."""

from __future__ import annotations

from enum import Enum


class FormationDecision(str, Enum):
    WAIT = 'wait'
    SUCCEED = 'succeed'
    FAIL = 'fail'
    CANCEL = 'cancel'
    TIMEOUT = 'timeout'


def decide_formation_execution(
    *,
    leader_succeeded: bool,
    followers_settled: bool,
    blocked_reason: str | None = None,
    operator_cancelled: bool = False,
    deadline_expired: bool = False,
) -> FormationDecision:
    """Choose the fail-fast terminal/wait decision from observed state."""
    if operator_cancelled:
        return FormationDecision.CANCEL
    if blocked_reason:
        return FormationDecision.FAIL
    if not leader_succeeded:
        return FormationDecision.FAIL
    if followers_settled:
        return FormationDecision.SUCCEED
    if deadline_expired:
        return FormationDecision.TIMEOUT
    return FormationDecision.WAIT
