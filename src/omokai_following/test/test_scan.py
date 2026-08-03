import math

import pytest

from omokai_following.scan import BoundedScanTracker


def test_scan_accumulates_across_pi_wrap() -> None:
    scan = BoundedScanTracker(required_angle_rad=0.30)

    scan.observe(math.pi - 0.10)
    total = scan.observe(-math.pi + 0.10)

    assert total == pytest.approx(0.20)
    assert not scan.complete


def test_full_scan_is_bounded_and_can_be_stopped_on_acquisition() -> None:
    scan = BoundedScanTracker(required_angle_rad=0.30)
    scan.observe(0.0)
    scan.observe(0.20)
    scan.observe(0.35)

    assert scan.complete
    scan.stop()
    assert scan.observe(1.0) == pytest.approx(0.35)
