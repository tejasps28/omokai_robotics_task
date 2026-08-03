import math

import numpy as np
import pytest

from omokai_perception.localization import CameraIntrinsics
from omokai_perception.localization import DepthPolicy
from omokai_perception.localization import depth_array
from omokai_perception.localization import localize_box
from omokai_perception.localization import protected_sector_distance
from omokai_perception.model import BoundingBox


INTRINSICS = CameraIntrinsics(640, 480, 400.0, 400.0, 320.0, 240.0)
BOX = BoundingBox(270, 140, 100, 200)


def test_32fc1_depth_and_centered_projection() -> None:
    source = np.full((480, 640), 2.0, dtype=np.float32)
    converted = depth_array(
        source.tobytes(), 640, 480, 640 * 4, '32FC1'
    )

    target = localize_box(BOX, converted, INTRINSICS)

    assert target.horizontal_range_m == pytest.approx(2.0)
    assert target.bearing_rad == pytest.approx(0.0)


def test_16uc1_depth_converts_millimetres_to_metres() -> None:
    source = np.full((2, 3), 1750, dtype=np.uint16)

    converted = depth_array(source.tobytes(), 3, 2, 6, '16UC1')

    np.testing.assert_allclose(converted, 1.75)


def test_projection_reports_expected_right_bearing() -> None:
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    target = localize_box(
        BoundingBox(370, 140, 100, 200), depth, INTRINSICS
    )

    assert target.bearing_rad == pytest.approx(math.atan2(0.5, 2.0))


def test_invalid_depth_and_large_range_jump_fail_closed() -> None:
    depth = np.full((480, 640), np.nan, dtype=np.float32)
    with pytest.raises(ValueError, match='insufficient valid depth'):
        localize_box(BOX, depth, INTRINSICS)

    depth.fill(4.0)
    with pytest.raises(ValueError, match='range jump'):
        localize_box(
            BOX,
            depth,
            INTRINSICS,
            DepthPolicy(maximum_range_jump_m=0.5),
            previous_range_m=2.0,
        )


def test_protected_sector_uses_near_percentile() -> None:
    depth = np.full((100, 100), 3.0, dtype=np.float32)
    depth[40:75, 40:60] = 0.5

    assert protected_sector_distance(depth) == pytest.approx(0.5)


def test_protected_sector_excludes_near_floor_band() -> None:
    depth = np.full((100, 100), 3.0, dtype=np.float32)
    depth[62:, :] = 0.4

    assert protected_sector_distance(depth) == pytest.approx(3.0)


def test_invalid_depth_policy_is_rejected() -> None:
    with pytest.raises(ValueError):
        DepthPolicy(minimum_m=2.0, maximum_m=1.0)
