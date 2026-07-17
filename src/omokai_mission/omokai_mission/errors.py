"""Stable error codes for mission validation.

These identifiers are part of the audit contract. They are consumed by audit
logs, operator feedback, and tests, so their string values must remain stable
across releases even if internal messages change. Codes are grouped by the
validation stage that can emit them.
"""

# Stage identifiers.
STAGE_STRUCTURAL = 'structural'
STAGE_SEMANTIC = 'semantic'
STAGE_ACCEPTED = 'accepted'

# Structural (JSON parse + JSON Schema) error codes.
MALFORMED_JSON = 'malformed_json'
MISSING_FIELD = 'missing_field'
UNEXPECTED_FIELD = 'unexpected_field'
WRONG_TYPE = 'wrong_type'
UNSUPPORTED_VALUE = 'unsupported_value'
INVALID_FORMAT = 'invalid_format'
OUT_OF_RANGE = 'out_of_range'
SCHEMA_VIOLATION = 'schema_violation'

# Semantic (policy) error codes.
UNSUPPORTED_ACTION = 'unsupported_action'
UNKNOWN_ROUTE = 'unknown_route'
UNSUPPORTED_DIRECTION = 'unsupported_direction'
REPETITIONS_OUT_OF_RANGE = 'repetitions_out_of_range'
SPEED_OUT_OF_RANGE = 'speed_out_of_range'

# Mapping from JSON Schema (Draft 7) validator keywords to stable structural
# codes. Any keyword not listed falls back to SCHEMA_VIOLATION so new schema
# constructs still produce an auditable, non-crashing result.
SCHEMA_KEYWORD_CODES = {
    'required': MISSING_FIELD,
    'additionalProperties': UNEXPECTED_FIELD,
    'type': WRONG_TYPE,
    'enum': UNSUPPORTED_VALUE,
    'const': UNSUPPORTED_VALUE,
    'pattern': INVALID_FORMAT,
    'minimum': OUT_OF_RANGE,
    'maximum': OUT_OF_RANGE,
    'minItems': OUT_OF_RANGE,
    'maxItems': OUT_OF_RANGE,
    'exclusiveMinimum': OUT_OF_RANGE,
    'exclusiveMaximum': OUT_OF_RANGE,
    'multipleOf': OUT_OF_RANGE,
}
