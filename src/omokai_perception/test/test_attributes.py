import numpy as np
import pytest

from omokai_perception.attributes import CoatColorSelector
from omokai_perception.attributes import coat_color_scores
from omokai_perception.model import AttributeDecision
from omokai_perception.yolox import Detection


def person_image(color: tuple[int, int, int]) -> np.ndarray:
    image = np.zeros((120, 80, 3), dtype=np.uint8)
    image[20:80, 20:60] = color
    return image


def detection() -> Detection:
    return Detection(10, 5, 60, 105, 0.8, 0)


def test_white_target_is_accepted_from_stationary_frame() -> None:
    candidate = CoatColorSelector('white').classify(
        person_image((245, 245, 245)), detection()
    )

    assert candidate.decision is AttributeDecision.ACCEPTED
    assert candidate.observed_color == 'white'


def test_non_matching_person_is_rejected() -> None:
    candidate = CoatColorSelector('white').classify(
        person_image((0, 0, 230)), detection()
    )

    assert candidate.decision is AttributeDecision.REJECTED


def test_dark_or_ambiguous_garment_is_not_accepted() -> None:
    candidate = CoatColorSelector('white').classify(
        person_image((40, 40, 40)), detection()
    )

    assert candidate.decision is AttributeDecision.UNCERTAIN


def test_any_color_bypasses_attribute_gate() -> None:
    candidate = CoatColorSelector('any').classify(
        person_image((10, 30, 50)), detection()
    )

    assert candidate.decision is AttributeDecision.ACCEPTED
    assert candidate.attribute_score == 1.0


def test_synthetic_white_ratio_gate_stays_separate_from_red() -> None:
    white = CoatColorSelector('white', 0.02, 0.015).classify(
        person_image((150, 150, 150)), detection()
    )
    red = CoatColorSelector('white', 0.02, 0.015).classify(
        person_image((0, 0, 180)), detection()
    )

    assert white.decision is AttributeDecision.ACCEPTED
    assert red.decision is AttributeDecision.REJECTED


def test_invalid_crop_and_unsupported_color_fail_closed() -> None:
    with pytest.raises(ValueError):
        coat_color_scores(np.zeros((0, 0, 3), dtype=np.uint8))
    with pytest.raises(ValueError):
        CoatColorSelector('blue')
