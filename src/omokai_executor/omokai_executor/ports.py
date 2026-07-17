"""Ports that keep the execution core independent of ROS and wall-clock time."""

from datetime import datetime, timezone
from time import monotonic
from typing import Protocol

from .events import ExecutionEvent
from .model import ExecutionGoal


class NavigationPort(Protocol):
    def dispatch(self, goal: ExecutionGoal, speed_mps: float) -> str:
        """Submit one goal and return the adapter's unique handle."""

    def cancel(self, goal_handle: str) -> None:
        """Request cancellation of the active goal."""


class EventSink(Protocol):
    def record(self, event: ExecutionEvent) -> None:
        """Durably record an execution event or raise an error."""


class RuntimeClock(Protocol):
    def monotonic(self) -> float:
        """Return monotonic seconds for deadlines."""

    def utc_now(self) -> str:
        """Return an RFC 3339 UTC timestamp for audit records."""


class SystemRuntimeClock:
    def monotonic(self) -> float:
        return monotonic()

    def utc_now(self) -> str:
        return (
            datetime.now(timezone.utc)
            .isoformat(timespec='milliseconds')
            .replace('+00:00', 'Z')
        )
