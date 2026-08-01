"""Deterministic clothing-colour selection inside detected person boxes."""

from __future__ import annotations

import cv2
import numpy as np

from omokai_perception.model import AttributeDecision
from omokai_perception.model import BoundingBox
from omokai_perception.model import PersonCandidate
from omokai_perception.yolox import Detection


SUPPORTED_COAT_COLORS = ('any', 'white', 'red')


def upper_torso_crop(
    image: np.ndarray,
    box: BoundingBox,
) -> np.ndarray:
    """Crop the central upper garment while avoiding face and background."""
    left = box.x + round(box.width * 0.18)
    right = box.x + round(box.width * 0.82)
    top = box.y + round(box.height * 0.20)
    bottom = box.y + round(box.height * 0.64)
    left = max(0, min(image.shape[1] - 1, left))
    right = max(left + 1, min(image.shape[1], right))
    top = max(0, min(image.shape[0] - 1, top))
    bottom = max(top + 1, min(image.shape[0], bottom))
    return image[top:bottom, left:right]


def coat_color_scores(crop: np.ndarray) -> dict[str, float]:
    """Return white/red pixel ratios in a lighting-tolerant HSV gate."""
    if crop.ndim != 3 or crop.shape[2] != 3 or crop.size == 0:
        raise ValueError('coat crop must be a non-empty BGR image')
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    credible = value >= 55
    credible_count = int(np.count_nonzero(credible))
    if credible_count == 0:
        return {'white': 0.0, 'red': 0.0}
    white = credible & (saturation <= 72) & (value >= 135)
    red = credible & (
        ((hue <= 12) | (hue >= 170))
        & (saturation >= 80)
        & (value >= 70)
    )
    return {
        'white': float(np.count_nonzero(white) / credible_count),
        'red': float(np.count_nonzero(red) / credible_count),
    }


class CoatColorSelector:
    """Classify supported coat colours without granting motion authority."""

    def __init__(
        self,
        requested_color: str,
        minimum_ratio: float = 0.28,
        ambiguity_margin: float = 0.08,
    ) -> None:
        normalized = requested_color.strip().lower()
        if normalized not in SUPPORTED_COAT_COLORS:
            raise ValueError(
                f'unsupported coat color {requested_color!r}; '
                f'choose from {SUPPORTED_COAT_COLORS}'
            )
        if not 0.0 < minimum_ratio <= 1.0:
            raise ValueError('minimum_ratio must be in (0, 1]')
        if not 0.0 <= ambiguity_margin < 1.0:
            raise ValueError('ambiguity_margin must be in [0, 1)')
        self.requested_color = normalized
        self.minimum_ratio = minimum_ratio
        self.ambiguity_margin = ambiguity_margin

    def classify(
        self,
        image: np.ndarray,
        detection: Detection,
    ) -> PersonCandidate:
        box = BoundingBox(
            detection.x,
            detection.y,
            detection.width,
            detection.height,
        )
        if self.requested_color == 'any':
            return PersonCandidate(
                box,
                detection.confidence,
                'any',
                'unconstrained',
                1.0,
                AttributeDecision.ACCEPTED,
            )
        scores = coat_color_scores(upper_torso_crop(image, box))
        requested_score = scores[self.requested_color]
        other_color = 'red' if self.requested_color == 'white' else 'white'
        other_score = scores[other_color]
        observed = max(scores, key=scores.get)
        if (
            requested_score >= self.minimum_ratio
            and requested_score >= other_score + self.ambiguity_margin
        ):
            decision = AttributeDecision.ACCEPTED
        elif (
            other_score >= self.minimum_ratio
            and other_score >= requested_score + self.ambiguity_margin
        ):
            decision = AttributeDecision.REJECTED
        else:
            decision = AttributeDecision.UNCERTAIN
        return PersonCandidate(
            box,
            detection.confidence,
            self.requested_color,
            observed,
            requested_score,
            decision,
        )
