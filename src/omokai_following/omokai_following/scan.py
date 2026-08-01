"""Odometry-backed bounded in-place scan accounting."""

from __future__ import annotations

import math


def wrapped_delta(current: float, previous: float) -> float:
    """Return the shortest signed angular delta."""

    return math.atan2(
        math.sin(current - previous), math.cos(current - previous)
    )


class BoundedScanTracker:
    """Accumulate absolute odometry yaw without wrap-around errors."""

    def __init__(self, required_angle_rad: float = 2.0 * math.pi) -> None:
        if required_angle_rad <= 0:
            raise ValueError('required scan angle must be positive')
        self.required_angle_rad = required_angle_rad
        self.reset()

    def reset(self) -> None:
        self.total_angle_rad = 0.0
        self._previous_yaw: float | None = None
        self.active = True

    def stop(self) -> None:
        self.active = False

    def observe(self, yaw_rad: float) -> float:
        if not self.active:
            return self.total_angle_rad
        if self._previous_yaw is not None:
            self.total_angle_rad += abs(
                wrapped_delta(yaw_rad, self._previous_yaw)
            )
        self._previous_yaw = yaw_rad
        return self.total_angle_rad

    @property
    def complete(self) -> bool:
        return self.total_angle_rad >= self.required_angle_rad
