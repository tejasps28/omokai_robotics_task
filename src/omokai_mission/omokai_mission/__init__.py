"""Mission planning for the Omokai core pipeline.

This package turns an untrusted planner proposal into an executable route:

    proposal -> validate_proposal -> ValidatedMission -> compile_mission -> goals

The LLM/planner only proposes JSON; nothing here publishes motion commands or
calls Nav2. Only a semantically accepted mission can be compiled into poses.
"""

from .catalog import (
    CatalogError,
    Pose2D,
    Route,
    RouteCatalog,
    Waypoint,
    load_catalog,
    parse_catalog,
)
from .compiler import CompiledRoute, NavGoal, compile_mission
from .errors import STAGE_ACCEPTED, STAGE_SEMANTIC, STAGE_STRUCTURAL
from .gemini_planner import (
    DEFAULT_GEMINI_MODEL,
    GeminiPlanner,
    GeminiPlannerError,
    UrllibGeminiTransport,
)
from .openai_planner import (
    DEFAULT_OPENAI_MODEL,
    OpenAIPlanner,
    OpenAIPlannerError,
    UrllibOpenAITransport,
)
from .planner import DEFAULT_SPEED_MPS, FakePlanner, Planner, PlannerError
from .validation import (
    KNOWN_ROUTE_DIRECTIONS,
    KNOWN_ROUTE_IDS,
    MAX_REPETITIONS,
    MAX_SPEED_MPS,
    POLICY_VERSION,
    SUPPORTED_ACTIONS,
    ValidationIssue,
    ValidationResult,
    validate_proposal,
)

__all__ = [
    'STAGE_ACCEPTED',
    'STAGE_SEMANTIC',
    'STAGE_STRUCTURAL',
    'KNOWN_ROUTE_IDS',
    'KNOWN_ROUTE_DIRECTIONS',
    'MAX_REPETITIONS',
    'MAX_SPEED_MPS',
    'POLICY_VERSION',
    'SUPPORTED_ACTIONS',
    'ValidationIssue',
    'ValidationResult',
    'validate_proposal',
    'CatalogError',
    'Pose2D',
    'Route',
    'RouteCatalog',
    'Waypoint',
    'load_catalog',
    'parse_catalog',
    'CompiledRoute',
    'NavGoal',
    'compile_mission',
    'DEFAULT_GEMINI_MODEL',
    'GeminiPlanner',
    'GeminiPlannerError',
    'UrllibGeminiTransport',
    'DEFAULT_OPENAI_MODEL',
    'OpenAIPlanner',
    'OpenAIPlannerError',
    'UrllibOpenAITransport',
    'DEFAULT_SPEED_MPS',
    'FakePlanner',
    'Planner',
    'PlannerError',
]
