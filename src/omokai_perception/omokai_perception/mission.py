"""Mission-generation identifiers shared by detection and following."""

from __future__ import annotations

import json
import re


TARGET_ID_PATTERN = re.compile(r'^target-g([1-9][0-9]*)-e([1-9][0-9]*)$')


def target_detection_id(generation: int, episode: int) -> str:
    """Return a typed target ID that cannot be confused across missions."""

    if generation < 1 or episode < 1:
        raise ValueError('generation and episode must be positive')
    return f'target-g{generation}-e{episode}'


def parse_target_detection_id(value: str) -> tuple[int, int] | None:
    """Decode a target ID, returning None for legacy or malformed data."""

    match = TARGET_ID_PATTERN.fullmatch(value)
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2))


def arm_response(generation: int, target_class: str, coat_color: str) -> str:
    """Encode the detector-arm acknowledgement carried by Trigger."""

    return json.dumps(
        {
            'mission_generation': generation,
            'target_class': target_class,
            'target_coat_color': coat_color,
        },
        sort_keys=True,
    )


def parse_arm_response(value: str) -> dict:
    """Validate and decode a detector-arm acknowledgement."""

    payload = json.loads(value)
    if not isinstance(payload, dict):
        raise ValueError('detector arm response is not an object')
    generation = payload.get('mission_generation')
    if not isinstance(generation, int) or generation < 1:
        raise ValueError('detector arm response has no valid generation')
    if payload.get('target_class') != 'person':
        raise ValueError('detector arm response has an unsupported class')
    if payload.get('target_coat_color') not in {'white', 'red', 'any'}:
        raise ValueError('detector arm response has an unsupported coat colour')
    return payload
