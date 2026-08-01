"""Bounded temporal confirmation and appearance-constrained identity lock."""

from __future__ import annotations

from omokai_perception.model import AttributeDecision
from omokai_perception.model import PersonCandidate
from omokai_perception.model import TrackingResult


class TemporalTargetTracker:
    """Confirm a target and never jump to a spatially unrelated distractor."""

    def __init__(
        self,
        confirmation_frames: int = 3,
        maximum_missing_frames: int = 8,
        minimum_iou: float = 0.12,
        maximum_center_distance: float = 0.45,
    ) -> None:
        if confirmation_frames < 2:
            raise ValueError('confirmation_frames must be at least two')
        if maximum_missing_frames < 1:
            raise ValueError('maximum_missing_frames must be positive')
        self.confirmation_frames = confirmation_frames
        self.maximum_missing_frames = maximum_missing_frames
        self.minimum_iou = minimum_iou
        self.maximum_center_distance = maximum_center_distance
        self.reset()

    def reset(self) -> None:
        self._target: PersonCandidate | None = None
        self._confirmation_count = 0
        self._missing_count = 0
        self._confirmed = False
        self._episode = getattr(self, '_episode', 0)

    def update(self, candidates: list[PersonCandidate]) -> TrackingResult:
        accepted = [
            candidate
            for candidate in candidates
            if candidate.decision is AttributeDecision.ACCEPTED
        ]
        match = self._select_match(accepted)
        if match is None:
            self._missing_count += 1
            # Retain the identity anchor for bounded reacquisition, but never
            # publish a stale box as a currently confirmed observation.
            self._confirmed = False
            if self._missing_count > self.maximum_missing_frames:
                self.reset()
            return self.result()

        self._missing_count = 0
        if self._target is None:
            self._episode += 1
            self._confirmation_count = 1
        else:
            self._confirmation_count += 1
        self._target = match
        self._confirmed = (
            self._confirmation_count >= self.confirmation_frames
        )
        return self.result()

    def result(self) -> TrackingResult:
        return TrackingResult(
            self._target,
            self._confirmed,
            self._episode,
            self._confirmation_count,
            self._missing_count,
        )

    def _select_match(
        self,
        candidates: list[PersonCandidate],
    ) -> PersonCandidate | None:
        if not candidates:
            return None
        if self._target is None:
            return max(
                candidates,
                key=lambda item: (
                    item.attribute_score,
                    item.detection_confidence,
                    item.box.area,
                ),
            )
        compatible = [
            candidate
            for candidate in candidates
            if (
                self._target.box.iou(candidate.box) >= self.minimum_iou
                or self._target.box.normalized_center_distance(candidate.box)
                <= self.maximum_center_distance
            )
        ]
        if not compatible:
            return None
        return max(
            compatible,
            key=lambda item: (
                self._target.box.iou(item.box),
                -self._target.box.normalized_center_distance(item.box),
                item.attribute_score,
            ),
        )
