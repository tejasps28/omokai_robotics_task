import argparse
from types import SimpleNamespace

import pytest

from omokai_following.operator_cli import build_parser
from omokai_following.operator_cli import acquisition_notification
from omokai_following.operator_cli import mission_payload
from omokai_following.operator_cli import parameter_update_succeeded


def arguments(**overrides):
    values = {
        'target_class': 'person',
        'coat_color': 'white',
        'standoff': 1.2,
        'max_speed': 0.18,
        'mission_id': 'vision-test-1',
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_valid_mission_is_normalized_to_structured_payload() -> None:
    payload = mission_payload(arguments())

    assert payload['target_class'] == 'person'
    assert payload['target_coat_color'] == 'white'
    assert payload['desired_standoff_m'] == pytest.approx(1.2)


@pytest.mark.parametrize(
    'overrides',
    [
        {'target_class': 'car'},
        {'mission_id': '../escape'},
        {'standoff': 0.0},
        {'standoff': 0.5},
        {'max_speed': -0.1},
        {'max_speed': 0.41},
    ],
)
def test_invalid_or_unsafe_missions_are_rejected(overrides) -> None:
    with pytest.raises(ValueError):
        mission_payload(arguments(**overrides))


def test_cli_exposes_preview_run_status_and_cancel() -> None:
    parser = build_parser()

    assert parser.parse_args(['preview']).command == 'preview'
    assert parser.parse_args(['run', '--yes']).yes
    assert parser.parse_args(['status']).command == 'status'
    assert parser.parse_args(['cancel']).command == 'cancel'


def test_acquisition_notification_contains_descriptor_and_paths() -> None:
    rendered = acquisition_notification(
        {
            'mission_id': 'vision-test-1',
            'mission_generation': 2,
            'target_class': 'person',
            'coat_color': 'white',
            'confidence': 0.91,
            'source_timestamp': 12.5,
            'original_path': '/data/original.png',
            'annotated_path': '/data/annotated.png',
        }
    )

    assert rendered.count('TARGET ACQUIRED') == 1
    assert 'person/white coat' in rendered
    assert '/data/original.png' in rendered
    assert '/data/annotated.png' in rendered


def test_jazzy_parameter_response_is_unwrapped() -> None:
    response = SimpleNamespace(
        results=[
            SimpleNamespace(successful=True),
            SimpleNamespace(successful=True),
        ]
    )

    assert parameter_update_succeeded(response)


def test_parameter_rejection_and_invalid_response_fail_closed() -> None:
    rejected = SimpleNamespace(
        results=[SimpleNamespace(successful=False)]
    )

    assert not parameter_update_succeeded(rejected)
    assert not parameter_update_succeeded(None)
    assert not parameter_update_succeeded(SimpleNamespace())
