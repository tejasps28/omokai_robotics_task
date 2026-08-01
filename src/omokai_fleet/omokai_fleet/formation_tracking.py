"""Pure continuous formation-tracking geometry and bounded control."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin
import re

from omokai_fleet.formation import FormationOffset, transform_offset
from omokai_fleet.formation import formation_offsets
from omokai_fleet.model import (
    Formation,
    Pose2D,
    RobotId,
    maximum_squad_speed_mps,
)


def _finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and isfinite(value)
    )


def normalize_angle(value: float) -> float:
    """Normalize a finite angle to the closed-open interval [-pi, pi)."""
    if not _finite_number(value):
        raise ValueError('angle must be finite')
    return (value + pi) % (2.0 * pi) - pi


def is_current_formation_state(
    value: object,
    operation_id: str | None,
) -> bool:
    """Return whether untrusted state belongs to the active operation."""
    return (
        isinstance(value, dict)
        and isinstance(operation_id, str)
        and value.get('operation_id') == operation_id
    )


@dataclass(frozen=True)
class FormationTrackingConfig:
    """Local follower gains and hard output bounds."""

    linear_gain: float = 0.8
    angular_gain: float = 1.8
    yaw_gain: float = 1.2
    max_linear_mps: float = 0.22
    max_angular_rps: float = 1.2
    position_tolerance_m: float = 0.08
    yaw_tolerance_rad: float = 0.12
    turn_in_place_threshold_rad: float = 0.75

    def __post_init__(self) -> None:
        positive = (
            ('linear_gain', self.linear_gain),
            ('angular_gain', self.angular_gain),
            ('yaw_gain', self.yaw_gain),
            ('max_linear_mps', self.max_linear_mps),
            ('max_angular_rps', self.max_angular_rps),
            ('position_tolerance_m', self.position_tolerance_m),
            ('yaw_tolerance_rad', self.yaw_tolerance_rad),
            ('turn_in_place_threshold_rad', self.turn_in_place_threshold_rad),
        )
        for name, value in positive:
            if not _finite_number(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self.turn_in_place_threshold_rad > pi:
            raise ValueError('turn_in_place_threshold_rad must not exceed pi')


@dataclass(frozen=True)
class FormationTrackingError:
    """Measured deviation from one leader-relative desired pose."""

    desired_pose: Pose2D
    distance_m: float
    bearing_error_rad: float
    yaw_error_rad: float
    forward_error_m: float
    left_error_m: float


@dataclass(frozen=True)
class VelocityCommand:
    linear_mps: float
    angular_rps: float
    target_reached: bool


@dataclass(frozen=True)
class FormationControlDirective:
    """Validated runtime ownership command for continuous follower control."""

    enabled: bool
    formation: Formation
    spacing_m: float
    max_linear_mps: float
    operation_id: str

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ValueError('enabled must be a boolean')
        if not isinstance(self.operation_id, str) or not re.fullmatch(
            r'[a-f0-9]{32}', self.operation_id
        ):
            raise ValueError('operation_id must be a 32-character lowercase hex ID')
        if not isinstance(self.formation, Formation):
            raise ValueError('formation must be a supported Formation')
        # Reuse the formation policy as the source of truth for spacing.
        formation_offsets(self.formation, self.spacing_m)
        if (
            not _finite_number(self.max_linear_mps)
            or not 0.05 <= self.max_linear_mps <= maximum_squad_speed_mps()
        ):
            raise ValueError('max_linear_mps is outside the configured range')

    def follower_offset(self, robot_id: RobotId) -> FormationOffset:
        if robot_id not in {RobotId.ROBOT2, RobotId.ROBOT3}:
            raise ValueError('continuous tracking is defined for followers only')
        return next(
            offset
            for offset in formation_offsets(self.formation, self.spacing_m)
            if offset.robot_id is robot_id
        )

    @classmethod
    def from_mapping(cls, value: object) -> 'FormationControlDirective':
        if not isinstance(value, dict) or set(value) != {
            'enabled',
            'formation',
            'spacing_m',
            'max_linear_mps',
            'operation_id',
        }:
            raise ValueError('formation directive contains unsupported fields')
        try:
            formation = Formation(value['formation'])
        except (TypeError, ValueError) as exc:
            raise ValueError('formation must be line or wedge') from exc
        return cls(
            enabled=value['enabled'],
            formation=formation,
            spacing_m=value['spacing_m'],
            max_linear_mps=value['max_linear_mps'],
            operation_id=value['operation_id'],
        )


def tracking_error(
    leader_pose: Pose2D,
    follower_pose: Pose2D,
    offset: FormationOffset,
) -> FormationTrackingError:
    """Return follower error relative to the leader's live formation pose."""
    if not isinstance(leader_pose, Pose2D):
        raise ValueError('leader_pose must be a Pose2D')
    if not isinstance(follower_pose, Pose2D):
        raise ValueError('follower_pose must be a Pose2D')
    if not isinstance(offset, FormationOffset):
        raise ValueError('offset must be a FormationOffset')

    desired = transform_offset(leader_pose, offset)
    delta_x = desired.x - follower_pose.x
    delta_y = desired.y - follower_pose.y
    bearing = atan2(delta_y, delta_x)
    bearing_error = normalize_angle(bearing - follower_pose.yaw)
    yaw_error = normalize_angle(desired.yaw - follower_pose.yaw)

    # Express position error in the leader frame for auditable formation
    # quality metrics, independent of the follower controller frame.
    forward_error = delta_x * cos(leader_pose.yaw) + delta_y * sin(
        leader_pose.yaw
    )
    left_error = -delta_x * sin(leader_pose.yaw) + delta_y * cos(
        leader_pose.yaw
    )
    return FormationTrackingError(
        desired_pose=desired,
        distance_m=hypot(delta_x, delta_y),
        bearing_error_rad=bearing_error,
        yaw_error_rad=yaw_error,
        forward_error_m=forward_error,
        left_error_m=left_error,
    )


def follower_command(
    error: FormationTrackingError,
    config: FormationTrackingConfig = FormationTrackingConfig(),
) -> VelocityCommand:
    """Calculate a bounded differential-drive command for one follower."""
    if not isinstance(error, FormationTrackingError):
        raise ValueError('error must be a FormationTrackingError')
    if not isinstance(config, FormationTrackingConfig):
        raise ValueError('config must be a FormationTrackingConfig')

    if error.distance_m <= config.position_tolerance_m:
        if abs(error.yaw_error_rad) <= config.yaw_tolerance_rad:
            return VelocityCommand(0.0, 0.0, True)
        return VelocityCommand(
            0.0,
            _clamp(
                config.yaw_gain * error.yaw_error_rad,
                config.max_angular_rps,
            ),
            False,
        )

    angular = _clamp(
        config.angular_gain * error.bearing_error_rad,
        config.max_angular_rps,
    )
    if abs(error.bearing_error_rad) >= config.turn_in_place_threshold_rad:
        linear = 0.0
    else:
        # The cosine term smoothly slows forward motion while the follower is
        # misaligned and never asks a first-version follower to reverse.
        linear = min(
            config.max_linear_mps,
            config.linear_gain
            * error.distance_m
            * max(0.0, cos(error.bearing_error_rad)),
        )
    return VelocityCommand(linear, angular, False)


def _clamp(value: float, magnitude: float) -> float:
    return max(-magnitude, min(magnitude, value))
