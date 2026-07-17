"""Packaged JSON Schemas for mission contracts."""

import json
from importlib.resources import files
from typing import Any, Dict


def load_mission_v1_schema() -> Dict[str, Any]:
    """Load a fresh copy of the strict Task 1 mission schema."""

    schema_text = files(__package__).joinpath('mission_v1.schema.json').read_text(
        encoding='utf-8'
    )
    return json.loads(schema_text)
