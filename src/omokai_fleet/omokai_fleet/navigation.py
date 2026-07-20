"""ROS-independent navigation batch results and lifecycle integration."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from omokai_fleet.lifecycle import SquadLifecycle
from omokai_fleet.model import ROBOT_IDS, RobotId


class NavigationStatus(str, Enum):
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    REJECTED = 'rejected'
    TIMED_OUT = 'timed_out'
    CANCELLED = 'cancelled'
    CANCEL_FAILED = 'cancel_failed'


PRIMARY_FAILURES = {
    NavigationStatus.FAILED,
    NavigationStatus.REJECTED,
    NavigationStatus.TIMED_OUT,
}


@dataclass(frozen=True)
class NavigationResult:
    robot_id: RobotId
    status: NavigationStatus
    reason: str = ''

    def __post_init__(self) -> None:
        if not isinstance(self.robot_id, RobotId):
            raise ValueError('robot_id must be a known RobotId')
        if not isinstance(self.status, NavigationStatus):
            raise ValueError('status must be a NavigationStatus')
        if not isinstance(self.reason, str):
            raise ValueError('reason must be a string')
        if self.status is not NavigationStatus.SUCCEEDED and not self.reason.strip():
            raise ValueError('non-success navigation results require a reason')


@dataclass(frozen=True)
class NavigationBatch:
    results: tuple[NavigationResult, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.results, tuple) or len(self.results) != 3:
            raise ValueError('navigation batch must contain exactly three results')
        if tuple(item.robot_id for item in self.results) != ROBOT_IDS:
            raise ValueError('navigation results must use stable robot order')
        primary_failures = tuple(
            item for item in self.results if item.status in PRIMARY_FAILURES
        )
        if len(primary_failures) > 1:
            raise ValueError(
                'navigation batch may contain only one primary failure'
            )

    @property
    def succeeded(self) -> bool:
        return all(
            item.status is NavigationStatus.SUCCEEDED
            for item in self.results
        )


def apply_navigation_batch(
    lifecycle: SquadLifecycle,
    batch: NavigationBatch,
) -> None:
    """Apply one complete adapter batch to the active lifecycle phase."""
    if not isinstance(lifecycle, SquadLifecycle):
        raise ValueError('lifecycle must be a SquadLifecycle')
    if not isinstance(batch, NavigationBatch):
        raise ValueError('batch must be a NavigationBatch')

    for result in batch.results:
        if result.status is NavigationStatus.SUCCEEDED:
            lifecycle.record_success(result.robot_id)

    primary = next(
        (
            result
            for result in batch.results
            if result.status in PRIMARY_FAILURES
        ),
        None,
    )
    cancelled = tuple(
        result
        for result in batch.results
        if result.status is NavigationStatus.CANCELLED
    )
    cancellation_failed = tuple(
        result
        for result in batch.results
        if result.status is NavigationStatus.CANCEL_FAILED
    )

    if primary is not None:
        lifecycle.record_failure(
            primary.robot_id,
            primary.reason,
            timed_out=primary.status is NavigationStatus.TIMED_OUT,
        )
        for result in cancelled:
            lifecycle.record_cancelled(result.robot_id, result.reason)
        for result in cancellation_failed:
            lifecycle.record_cancellation_failed(
                result.robot_id,
                result.reason,
            )
        lifecycle.finish_cancellation()
        return

    if cancelled or cancellation_failed:
        reason = (cancelled + cancellation_failed)[0].reason
        lifecycle.request_cancel(reason)
        for result in cancelled:
            lifecycle.record_cancelled(result.robot_id, result.reason)
        for result in cancellation_failed:
            lifecycle.record_cancellation_failed(
                result.robot_id,
                result.reason,
            )
        lifecycle.finish_cancellation()
