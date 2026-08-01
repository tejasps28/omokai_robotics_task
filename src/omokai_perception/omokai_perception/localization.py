"""Aligned RGB-D conversion and robust target-region localization."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from omokai_perception.model import BoundingBox


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError('calibration dimensions must be positive')
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError('camera focal lengths must be positive')


@dataclass(frozen=True)
class LocalizedTarget:
    bearing_rad: float
    horizontal_range_m: float
    optical_x_m: float
    optical_y_m: float
    optical_z_m: float
    valid_depth_ratio: float


@dataclass(frozen=True)
class DepthPolicy:
    minimum_m: float = 0.12
    maximum_m: float = 8.0
    minimum_valid_ratio: float = 0.20
    maximum_range_jump_m: float = 1.0

    def __post_init__(self) -> None:
        if self.minimum_m <= 0 or self.maximum_m <= self.minimum_m:
            raise ValueError('depth bounds are invalid')
        if not 0.0 < self.minimum_valid_ratio <= 1.0:
            raise ValueError('minimum valid-depth ratio must be in (0, 1]')
        if self.maximum_range_jump_m <= 0:
            raise ValueError('maximum range jump must be positive')


def depth_array(
    data: bytes,
    width: int,
    height: int,
    step: int,
    encoding: str,
    is_bigendian: bool = False,
) -> np.ndarray:
    """Convert 32FC1 metres or 16UC1 millimetres to a float-metre matrix."""
    if width <= 0 or height <= 0:
        raise ValueError('depth dimensions must be positive')
    if encoding == '32FC1':
        dtype = np.dtype('>f4' if is_bigendian else '<f4')
        element_size = 4
        scale = 1.0
    elif encoding == '16UC1':
        dtype = np.dtype('>u2' if is_bigendian else '<u2')
        element_size = 2
        scale = 0.001
    else:
        raise ValueError(f'unsupported depth encoding {encoding!r}')
    if step < width * element_size:
        raise ValueError('depth step is shorter than one image row')
    raw = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    pixels = raw[:, : width * element_size].copy().view(dtype).reshape(
        height, width
    )
    return pixels.astype(np.float32) * scale


def localize_box(
    box: BoundingBox,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    policy: DepthPolicy = DepthPolicy(),
    previous_range_m: float | None = None,
) -> LocalizedTarget:
    if depth_m.shape != (intrinsics.height, intrinsics.width):
        raise ValueError('depth dimensions do not match camera calibration')
    x0 = max(0, box.x + round(box.width * 0.25))
    x1 = min(intrinsics.width, box.x + round(box.width * 0.75))
    y0 = max(0, box.y + round(box.height * 0.30))
    y1 = min(intrinsics.height, box.y + round(box.height * 0.78))
    if x1 <= x0 or y1 <= y0:
        raise ValueError('target depth region is empty')
    region = depth_m[y0:y1, x0:x1]
    valid = (
        np.isfinite(region)
        & (region >= policy.minimum_m)
        & (region <= policy.maximum_m)
    )
    valid_ratio = float(np.count_nonzero(valid) / valid.size)
    if valid_ratio < policy.minimum_valid_ratio:
        raise ValueError('target region has insufficient valid depth')
    z = float(np.median(region[valid]))
    u, v = box.center
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy
    horizontal_range = math.hypot(x, z)
    if (
        previous_range_m is not None
        and abs(horizontal_range - previous_range_m)
        > policy.maximum_range_jump_m
    ):
        raise ValueError('target range jump exceeds configured bound')
    return LocalizedTarget(
        bearing_rad=math.atan2(x, z),
        horizontal_range_m=horizontal_range,
        optical_x_m=x,
        optical_y_m=y,
        optical_z_m=z,
        valid_depth_ratio=valid_ratio,
    )


def protected_sector_distance(
    depth_m: np.ndarray,
    policy: DepthPolicy = DepthPolicy(),
    percentile: float = 5.0,
) -> float | None:
    """Return a conservative forward-sector distance for safety gating."""
    height, width = depth_m.shape
    # The vision camera is pitched upward to retain a full person at the
    # requested stand-off. Keep this safety ROI above the lower image band,
    # where the nearby floor otherwise looks like a false frontal obstacle.
    sector = depth_m[
        round(height * 0.32):round(height * 0.60),
        round(width * 0.35):round(width * 0.65),
    ]
    valid = (
        np.isfinite(sector)
        & (sector >= policy.minimum_m)
        & (sector <= policy.maximum_m)
    )
    if not np.any(valid):
        return None
    return float(np.percentile(sector[valid], percentile))


def timestamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
