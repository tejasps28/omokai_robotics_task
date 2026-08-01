"""ROS-independent models used by target selection and tracking."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class AttributeDecision(str, Enum):
    ACCEPTED = 'accepted'
    REJECTED = 'rejected'
    UNCERTAIN = 'uncertain'


@dataclass(frozen=True)
class BoundingBox:
    x: int
    y: int
    width: int
    height: int

    def __post_init__(self) -> None:
        if self.x < 0 or self.y < 0:
            raise ValueError('bounding-box origin cannot be negative')
        if self.width <= 0 or self.height <= 0:
            raise ValueError('bounding-box dimensions must be positive')

    @property
    def center(self) -> tuple[float, float]:
        return self.x + self.width / 2.0, self.y + self.height / 2.0

    @property
    def area(self) -> int:
        return self.width * self.height

    def iou(self, other: 'BoundingBox') -> float:
        left = max(self.x, other.x)
        top = max(self.y, other.y)
        right = min(self.x + self.width, other.x + other.width)
        bottom = min(self.y + self.height, other.y + other.height)
        intersection = max(0, right - left) * max(0, bottom - top)
        union = self.area + other.area - intersection
        return intersection / union if union else 0.0

    def normalized_center_distance(self, other: 'BoundingBox') -> float:
        x1, y1 = self.center
        x2, y2 = other.center
        scale = max(self.width, self.height, other.width, other.height)
        return math.hypot(x1 - x2, y1 - y2) / scale


@dataclass(frozen=True)
class PersonCandidate:
    box: BoundingBox
    detection_confidence: float
    requested_color: str
    observed_color: str
    attribute_score: float
    decision: AttributeDecision

    def __post_init__(self) -> None:
        if not 0.0 <= self.detection_confidence <= 1.0:
            raise ValueError('detection confidence must be in [0, 1]')
        if not 0.0 <= self.attribute_score <= 1.0:
            raise ValueError('attribute score must be in [0, 1]')


@dataclass(frozen=True)
class TrackingResult:
    target: PersonCandidate | None
    confirmed: bool
    episode: int
    confirmation_count: int
    missing_count: int
