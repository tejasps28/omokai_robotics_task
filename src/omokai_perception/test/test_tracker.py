from omokai_perception.model import AttributeDecision
from omokai_perception.model import BoundingBox
from omokai_perception.model import PersonCandidate
from omokai_perception.tracker import TemporalTargetTracker


def candidate(
    x: int,
    decision: AttributeDecision = AttributeDecision.ACCEPTED,
    score: float = 0.8,
) -> PersonCandidate:
    return PersonCandidate(
        BoundingBox(x, 10, 40, 100),
        0.9,
        'white',
        'white',
        score,
        decision,
    )


def test_target_requires_temporal_confirmation() -> None:
    tracker = TemporalTargetTracker(confirmation_frames=3)

    assert not tracker.update([candidate(10)]).confirmed
    assert not tracker.update([candidate(12)]).confirmed
    assert tracker.update([candidate(14)]).confirmed


def test_rejected_candidates_never_become_target() -> None:
    tracker = TemporalTargetTracker(confirmation_frames=2)

    result = tracker.update(
        [candidate(10, AttributeDecision.REJECTED)]
    )

    assert result.target is None
    assert not result.confirmed


def test_missing_frame_never_republishes_stale_confirmation() -> None:
    tracker = TemporalTargetTracker(confirmation_frames=2)
    tracker.update([candidate(10)])
    assert tracker.update([candidate(12)]).confirmed

    missing = tracker.update([])

    assert not missing.confirmed
    assert missing.target is not None


def test_tracker_does_not_jump_to_unrelated_person() -> None:
    tracker = TemporalTargetTracker(
        confirmation_frames=2,
        maximum_center_distance=0.30,
    )
    tracker.update([candidate(10)])
    tracker.update([candidate(12)])

    unrelated = tracker.update([candidate(250, score=1.0)])

    assert not unrelated.confirmed
    assert unrelated.target.box.x == 12


def test_rejected_person_crossing_target_never_takes_identity() -> None:
    tracker = TemporalTargetTracker(
        confirmation_frames=2,
        maximum_center_distance=0.60,
    )
    rejected = AttributeDecision.REJECTED

    tracker.update([candidate(10), candidate(100, rejected, score=1.0)])
    confirmed = tracker.update(
        [candidate(25), candidate(85, rejected, score=1.0)]
    )
    crossing = tracker.update(
        [candidate(55), candidate(45, rejected, score=1.0)]
    )
    crossed = tracker.update(
        [candidate(80), candidate(20, rejected, score=1.0)]
    )

    assert confirmed.confirmed
    assert crossing.target.box.x == 55
    assert crossed.target.box.x == 80
    assert crossed.confirmed


def test_expired_identity_starts_new_episode() -> None:
    tracker = TemporalTargetTracker(
        confirmation_frames=2,
        maximum_missing_frames=1,
    )
    tracker.update([candidate(10)])
    tracker.update([])
    tracker.update([])

    restarted = tracker.update([candidate(250)])

    assert restarted.episode == 2
    assert restarted.confirmation_count == 1
