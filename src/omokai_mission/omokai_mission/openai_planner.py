"""OpenAI-backed planner adapter for the mission pipeline.

This module keeps the LLM boundary narrow: OpenAI proposes one JSON object that
matches the mission schema, then the existing local validator decides whether it
is safe and executable. The adapter never publishes ROS messages and never talks
to Nav2.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Protocol

from omokai_interfaces import MissionProposal, PlanRequest, load_mission_v1_schema


OPENAI_RESPONSES_URL = 'https://api.openai.com/v1/responses'
DEFAULT_OPENAI_MODEL = 'gpt-5.6'
DEFAULT_TIMEOUT_SEC = 30.0
DEFAULT_MAX_OUTPUT_TOKENS = 500


class OpenAIPlannerError(RuntimeError):
    """Raised when the OpenAI planner cannot produce a proposal."""


class OpenAITransport(Protocol):
    """Minimal HTTP transport seam used by tests and the production adapter."""

    def create_response(
        self,
        *,
        api_key: str,
        payload: Mapping[str, Any],
        timeout_sec: float,
    ) -> Mapping[str, Any]:
        """Submit a Responses API request and return the decoded JSON body."""
        ...


@dataclass(frozen=True)
class UrllibOpenAITransport:
    """Small dependency-free transport for the OpenAI Responses API."""

    url: str = OPENAI_RESPONSES_URL

    def create_response(
        self,
        *,
        api_key: str,
        payload: Mapping[str, Any],
        timeout_sec: float,
    ) -> Mapping[str, Any]:
        data = json.dumps(payload, sort_keys=True).encode('utf-8')
        request = urllib.request.Request(
            self.url,
            data=data,
            method='POST',
            headers={
                'Authorization': f'Bearer {api_key}',
                'Content-Type': 'application/json',
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout_sec) as response:
                body = response.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise OpenAIPlannerError(
                f'OpenAI request failed with HTTP {exc.code}: '
                f'{_format_api_error(detail)}'
            ) from exc
        except urllib.error.URLError as exc:
            raise OpenAIPlannerError(f'OpenAI request failed: {exc.reason}') from exc
        except TimeoutError as exc:
            raise OpenAIPlannerError('OpenAI request timed out') from exc

        try:
            decoded = json.loads(body)
        except json.JSONDecodeError as exc:
            raise OpenAIPlannerError('OpenAI response body was not valid JSON') from exc
        if not isinstance(decoded, dict):
            raise OpenAIPlannerError('OpenAI response body was not a JSON object')
        return decoded


class OpenAIPlanner:
    """Planner implementation backed by OpenAI Structured Outputs.

    Credentials are read from an explicit ``api_key`` argument or from
    ``OPENAI_API_KEY``. The model defaults to ``OMOKAI_OPENAI_MODEL`` when set,
    otherwise to :data:`DEFAULT_OPENAI_MODEL`.
    """

    provider = 'openai'

    def __init__(
        self,
        *,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        transport: Optional[OpenAITransport] = None,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
        max_output_tokens: int = DEFAULT_MAX_OUTPUT_TOKENS,
    ) -> None:
        self._api_key = api_key if api_key is not None else os.getenv('OPENAI_API_KEY')
        self._model = model or os.getenv('OMOKAI_OPENAI_MODEL') or DEFAULT_OPENAI_MODEL
        self._transport = transport or UrllibOpenAITransport()
        self._timeout_sec = timeout_sec
        self._max_output_tokens = max_output_tokens

        if not self._model.strip():
            raise OpenAIPlannerError('OpenAI model must not be blank')
        if self._timeout_sec <= 0:
            raise OpenAIPlannerError('OpenAI timeout must be positive')
        if self._max_output_tokens <= 0:
            raise OpenAIPlannerError('OpenAI max output tokens must be positive')

    @property
    def model(self) -> str:
        """Return the configured model identifier."""

        return self._model

    def propose(self, request: PlanRequest) -> MissionProposal:
        """Return an untrusted mission proposal from the configured provider."""

        api_key = self._require_api_key()
        payload = self._build_payload(request)
        response = self._transport.create_response(
            api_key=api_key,
            payload=payload,
            timeout_sec=self._timeout_sec,
        )
        content = _extract_output_text(response)
        # Normalize whitespace while preserving the provider's JSON values. The
        # downstream validator remains the authority for schema and policy.
        try:
            normalized = json.dumps(json.loads(content), sort_keys=True)
        except json.JSONDecodeError as exc:
            raise OpenAIPlannerError('OpenAI structured output was not JSON') from exc
        return MissionProposal(
            request_id=request.request_id,
            provider=f'{self.provider}:{self._model}',
            content=normalized,
        )

    def _require_api_key(self) -> str:
        if self._api_key is None or not self._api_key.strip():
            raise OpenAIPlannerError(
                'OPENAI_API_KEY is required for the OpenAI planner'
            )
        return self._api_key

    def _build_payload(self, request: PlanRequest) -> Mapping[str, Any]:
        return {
            'model': self._model,
            'store': False,
            'max_output_tokens': self._max_output_tokens,
            'instructions': _SYSTEM_INSTRUCTIONS,
            'input': request.prompt,
            'text': {
                'format': {
                    'type': 'json_schema',
                    'name': 'omokai_mission_v1',
                    'strict': True,
                    'schema': _structured_output_schema(),
                }
            },
        }


def _structured_output_schema() -> Mapping[str, Any]:
    """Return the mission schema in the form sent to OpenAI."""

    schema = deepcopy(load_mission_v1_schema())
    # Provider requests do not need repository-specific identifiers.
    schema.pop('$schema', None)
    schema.pop('$id', None)
    properties = schema.get('properties', {})
    if isinstance(properties, dict):
        schema_version = properties.get('schema_version')
        if isinstance(schema_version, dict) and 'const' in schema_version:
            schema_version['type'] = 'string'
            schema_version['enum'] = [schema_version.pop('const')]
        action = properties.get('action')
        if isinstance(action, dict):
            action.setdefault('type', 'string')
        segments = properties.get('segments')
        if isinstance(segments, dict):
            items = segments.get('items')
            if isinstance(items, dict):
                item_properties = items.get('properties')
                if isinstance(item_properties, dict):
                    direction = item_properties.get('direction')
                    if isinstance(direction, dict):
                        direction.setdefault('type', 'string')
    return schema


def _extract_output_text(response: Mapping[str, Any]) -> str:
    """Extract text from common Responses API response shapes."""

    status = response.get('status')
    if isinstance(status, str) and status != 'completed':
        detail = _response_failure_detail(response)
        raise OpenAIPlannerError(f'OpenAI response status was {status}: {detail}')

    if response.get('refusal') is not None:
        raise OpenAIPlannerError('OpenAI model refused the mission request')

    direct = response.get('output_text')
    if isinstance(direct, str) and direct.strip():
        return direct

    output = response.get('output')
    if isinstance(output, list):
        for item in output:
            if not isinstance(item, dict):
                continue
            content = item.get('content')
            if not isinstance(content, list):
                continue
            for part in content:
                if not isinstance(part, dict):
                    continue
                if part.get('type') == 'refusal' or part.get('refusal') is not None:
                    raise OpenAIPlannerError(
                        'OpenAI model refused the mission request'
                    )
                text = part.get('text')
                if isinstance(text, str) and text.strip():
                    return text

    raise OpenAIPlannerError('OpenAI response did not contain output text')


def _response_failure_detail(response: Mapping[str, Any]) -> str:
    error = response.get('error')
    if isinstance(error, Mapping):
        message = error.get('message')
        code = error.get('code')
        if isinstance(message, str) and message.strip():
            return f'{code}: {message}' if code else message
    incomplete = response.get('incomplete_details')
    if isinstance(incomplete, Mapping):
        reason = incomplete.get('reason')
        if isinstance(reason, str) and reason.strip():
            return reason
    return 'provider did not complete the structured response'


def _format_api_error(detail: str) -> str:
    """Return a concise provider error without echoing an arbitrary response body."""

    try:
        payload = json.loads(detail)
    except json.JSONDecodeError:
        payload = None
    if isinstance(payload, Mapping):
        error = payload.get('error')
        if isinstance(error, Mapping):
            message = error.get('message')
            code = error.get('code') or error.get('type')
            if isinstance(message, str) and message.strip():
                formatted = f'{code}: {message}' if code else message
                return formatted[:500]
    compact = ' '.join(detail.split())
    return compact[:500] if compact else 'no error detail returned'


_SYSTEM_INSTRUCTIONS = (
    'You convert a ground-robot operator request into exactly one JSON mission '
    'proposal. Output only fields from the provided schema. Supported routes are '
    'inspection_loop, aisle_sweep, and full_area_sweep. Use action "patrol". '
    'Treat “inspection '
    'route”, “perimeter loop”, “patrol loop”, or a patrol request containing a '
    'clockwise/counterclockwise direction as inspection_loop. For inspection_loop, '
    'segments may use only "clockwise" or "counterclockwise"; preserve the '
    'operator\'s ordered direction changes and repetition count in separate '
    'segments. When loop direction is omitted, use "counterclockwise". For '
    'aisle_sweep for requests limited to the central lanes or aisles. Use '
    'full_area_sweep for the full/whole/entire area or all aisles, lanes, or '
    'corridors. Both sweep routes may use only "forward" or "reverse"; use '
    '"forward" when omitted. The total repetitions across all '
    'segments must not exceed 10. Use speed_mps no greater than 0.26 and set '
    'return_home true for requests to return home or return to start. If the '
    'request is unrelated, ambiguous, unsafe, or cannot be represented by these '
    'bounded routes and directions, produce schema-valid JSON with route_id '
    '"rejected_request" so the local semantic validator rejects it. Local '
    'validation and operator approval decide whether any proposal may execute.'
)
