"""Gemini-backed planner adapter for the mission pipeline.

Gemini is an optional hosted planner provider. It proposes one mission JSON
object, then the local schema and semantic validator decide whether the proposal
can compile or execute. The adapter never publishes ROS messages and never talks
to Nav2.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.parse
import urllib.request
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Protocol

from omokai_interfaces import MissionProposal, PlanRequest

from .openai_planner import _SYSTEM_INSTRUCTIONS, _structured_output_schema


GEMINI_API_BASE_URL = 'https://generativelanguage.googleapis.com/v1beta'
DEFAULT_GEMINI_MODEL = 'gemini-2.5-flash'
DEFAULT_TIMEOUT_SEC = 30.0
DEFAULT_MAX_OUTPUT_TOKENS = 1024


class GeminiPlannerError(RuntimeError):
    """Raised when the Gemini planner cannot produce a proposal."""


class GeminiTransport(Protocol):
    """Minimal HTTP transport seam used by tests and the production adapter."""

    def generate_content(
        self,
        *,
        api_key: str,
        model: str,
        payload: Mapping[str, Any],
        timeout_sec: float,
    ) -> Mapping[str, Any]:
        """Submit a generateContent request and return the decoded JSON body."""
        ...


@dataclass(frozen=True)
class UrllibGeminiTransport:
    """Small dependency-free transport for the Gemini generateContent API."""

    base_url: str = GEMINI_API_BASE_URL

    def generate_content(
        self,
        *,
        api_key: str,
        model: str,
        payload: Mapping[str, Any],
        timeout_sec: float,
    ) -> Mapping[str, Any]:
        data = json.dumps(payload, sort_keys=True).encode('utf-8')
        escaped_model = urllib.parse.quote(model, safe='')
        request = urllib.request.Request(
            f'{self.base_url}/models/{escaped_model}:generateContent',
            data=data,
            method='POST',
            headers={
                'x-goog-api-key': api_key,
                'Content-Type': 'application/json',
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout_sec) as response:
                body = response.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise GeminiPlannerError(
                f'Gemini request failed with HTTP {exc.code}: '
                f'{_format_api_error(detail)}'
            ) from exc
        except urllib.error.URLError as exc:
            raise GeminiPlannerError(f'Gemini request failed: {exc.reason}') from exc
        except TimeoutError as exc:
            raise GeminiPlannerError('Gemini request timed out') from exc

        try:
            decoded = json.loads(body)
        except json.JSONDecodeError as exc:
            raise GeminiPlannerError('Gemini response body was not valid JSON') from exc
        if not isinstance(decoded, dict):
            raise GeminiPlannerError('Gemini response body was not a JSON object')
        return decoded


class GeminiPlanner:
    """Planner implementation backed by Gemini structured JSON responses.

    Credentials are read from an explicit ``api_key`` argument or from
    ``GEMINI_API_KEY``. The model defaults to ``OMOKAI_GEMINI_MODEL`` when set,
    otherwise to :data:`DEFAULT_GEMINI_MODEL`.
    """

    provider = 'gemini'

    def __init__(
        self,
        *,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        transport: Optional[GeminiTransport] = None,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
        max_output_tokens: int = DEFAULT_MAX_OUTPUT_TOKENS,
    ) -> None:
        self._api_key = api_key if api_key is not None else os.getenv('GEMINI_API_KEY')
        self._model = model or os.getenv('OMOKAI_GEMINI_MODEL') or DEFAULT_GEMINI_MODEL
        self._transport = transport or UrllibGeminiTransport()
        self._timeout_sec = timeout_sec
        self._max_output_tokens = max_output_tokens

        if not self._model.strip():
            raise GeminiPlannerError('Gemini model must not be blank')
        if self._timeout_sec <= 0:
            raise GeminiPlannerError('Gemini timeout must be positive')
        if self._max_output_tokens <= 0:
            raise GeminiPlannerError('Gemini max output tokens must be positive')

    @property
    def model(self) -> str:
        """Return the configured model identifier."""

        return self._model

    def propose(self, request: PlanRequest) -> MissionProposal:
        """Return an untrusted mission proposal from the configured provider."""

        api_key = self._require_api_key()
        payload = self._build_payload(request)
        response = self._transport.generate_content(
            api_key=api_key,
            model=self._model,
            payload=payload,
            timeout_sec=self._timeout_sec,
        )
        content = _extract_output_text(response)
        try:
            normalized = json.dumps(json.loads(content), sort_keys=True)
        except json.JSONDecodeError as exc:
            raise GeminiPlannerError('Gemini structured output was not JSON') from exc
        return MissionProposal(
            request_id=request.request_id,
            provider=f'{self.provider}:{self._model}',
            content=normalized,
        )

    def _require_api_key(self) -> str:
        if self._api_key is None or not self._api_key.strip():
            raise GeminiPlannerError('GEMINI_API_KEY is required for the Gemini planner')
        return self._api_key

    def _build_payload(self, request: PlanRequest) -> Mapping[str, Any]:
        return {
            'contents': [
                {
                    'parts': [
                        {
                            'text': (
                                f'{_SYSTEM_INSTRUCTIONS}\n\n'
                                f'Operator request: {request.prompt}'
                            )
                        }
                    ]
                }
            ],
            'generationConfig': {
                'maxOutputTokens': self._max_output_tokens,
                'responseMimeType': 'application/json',
                'responseSchema': _gemini_response_schema(),
            },
        }


def _gemini_response_schema() -> Mapping[str, Any]:
    """Return the mission schema projected onto Gemini's supported subset."""

    schema = deepcopy(_structured_output_schema())
    _drop_unsupported_schema_keywords(schema)
    return schema


def _drop_unsupported_schema_keywords(value: Any) -> None:
    if isinstance(value, dict):
        value.pop('additionalProperties', None)
        value.pop('exclusiveMinimum', None)
        for child in value.values():
            _drop_unsupported_schema_keywords(child)
    elif isinstance(value, list):
        for child in value:
            _drop_unsupported_schema_keywords(child)


def _extract_output_text(response: Mapping[str, Any]) -> str:
    """Extract text from common Gemini response shapes."""

    error = response.get('error')
    if isinstance(error, Mapping):
        message = error.get('message')
        detail = message if isinstance(message, str) else 'provider error'
        raise GeminiPlannerError(f'Gemini response failed: {detail}')

    prompt_feedback = response.get('promptFeedback')
    if isinstance(prompt_feedback, Mapping):
        block_reason = prompt_feedback.get('blockReason')
        if isinstance(block_reason, str) and block_reason.strip():
            raise GeminiPlannerError(
                f'Gemini blocked the mission request: {block_reason}'
            )

    direct = response.get('output_text')
    if isinstance(direct, str) and direct.strip():
        return direct

    candidates = response.get('candidates')
    if isinstance(candidates, list):
        finish_reasons = []
        for candidate in candidates:
            if not isinstance(candidate, dict):
                continue
            finish_reason = candidate.get('finishReason')
            if isinstance(finish_reason, str) and finish_reason.strip():
                finish_reasons.append(finish_reason)
            content = candidate.get('content')
            if not isinstance(content, dict):
                continue
            parts = content.get('parts')
            if not isinstance(parts, list):
                continue
            for part in parts:
                if not isinstance(part, dict):
                    continue
                text = part.get('text')
                if isinstance(text, str) and text.strip():
                    return text
        if finish_reasons:
            raise GeminiPlannerError(
                'Gemini response contained no mission JSON '
                f'(finish reason: {finish_reasons[0]})'
            )

    raise GeminiPlannerError('Gemini response did not contain output text')


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
            status = error.get('status')
            if isinstance(message, str) and message.strip():
                formatted = f'{status}: {message}' if status else message
                return formatted[:500]
    compact = ' '.join(detail.split())
    return compact[:500] if compact else 'no error detail returned'
