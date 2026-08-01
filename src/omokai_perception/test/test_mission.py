import json

import pytest

from omokai_perception.mission import arm_response
from omokai_perception.mission import parse_arm_response
from omokai_perception.mission import parse_target_detection_id
from omokai_perception.mission import target_detection_id


def test_target_id_round_trip_is_mission_scoped() -> None:
    value = target_detection_id(7, 3)

    assert value == 'target-g7-e3'
    assert parse_target_detection_id(value) == (7, 3)


@pytest.mark.parametrize(
    'value', ('target-1', 'target-g0-e1', 'target-g1-e0', 'junk')
)
def test_legacy_or_malformed_target_id_is_rejected(value: str) -> None:
    assert parse_target_detection_id(value) is None


def test_arm_response_round_trip() -> None:
    payload = parse_arm_response(arm_response(2, 'person', 'red'))

    assert payload['mission_generation'] == 2
    assert payload['target_coat_color'] == 'red'


def test_arm_response_rejects_unsupported_descriptor() -> None:
    with pytest.raises(ValueError):
        parse_arm_response(
            json.dumps(
                {
                    'mission_generation': 1,
                    'target_class': 'car',
                    'target_coat_color': 'red',
                }
            )
        )
