"""Pure depth-image conversion helpers."""

from __future__ import annotations

import numpy as np


DEPTH_NEAR_METERS = 0.12
DEPTH_FAR_METERS = 8.0


def depth_preview(
    data: bytes,
    *,
    near: float = DEPTH_NEAR_METERS,
    far: float = DEPTH_FAR_METERS,
) -> bytes:
    """Map metric 32FC1 depth to mono8, with nearer valid pixels brighter."""
    depth = np.frombuffer(data, dtype=np.float32)
    valid = np.isfinite(depth) & (depth >= near) & (depth <= far)
    preview = np.zeros(depth.shape, dtype=np.uint8)
    scaled = 255.0 * (far - depth[valid]) / (far - near)
    preview[valid] = np.clip(scaled, 1.0, 255.0).astype(np.uint8)
    return preview.tobytes()
