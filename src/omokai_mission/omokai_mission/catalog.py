"""Versioned route catalog for the Task 1 core pipeline.

The catalog is the audited source of truth that maps a symbolic ``route_id`` to
concrete 2D map-frame poses. A planner never supplies coordinates; the compiler
resolves route IDs against this catalog only. The catalog is stored as JSON under
packaged data directory so it is portable, human-auditable, and version controlled.

Loading validates the catalog structure strictly and raises :class:`CatalogError`
on any malformed entry, so a corrupt catalog fails fast instead of producing
undefined navigation goals.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Dict, Mapping, Tuple

from omokai_interfaces import TraversalDirection

CATALOG_FILENAME = 'catalog.v1.json'
_SYMBOLIC_ID = re.compile(r'^[a-z][a-z0-9_]{0,63}$')


class CatalogError(ValueError):
    """Raised when catalog data is missing required structure or valid poses."""


@dataclass(frozen=True)
class Pose2D:
    """A 2D goal pose in a named frame (metres, radians)."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Waypoint:
    """A named pose along a route."""

    name: str
    pose: Pose2D


@dataclass(frozen=True)
class Route:
    """An ordered sequence of waypoints identified by a symbolic ID."""

    route_id: str
    description: str
    closed: bool
    allowed_directions: Tuple[TraversalDirection, ...]
    waypoints: Tuple[Waypoint, ...]


@dataclass(frozen=True)
class RouteCatalog:
    """A validated, immutable set of routes plus the shared home pose."""

    catalog_version: str
    frame_id: str
    home: Pose2D
    routes: Mapping[str, Route]

    def route_ids(self) -> Tuple[str, ...]:
        """Return known route IDs in sorted, deterministic order."""

        return tuple(sorted(self.routes))

    def get(self, route_id: str) -> Route:
        """Return a route by ID or raise :class:`CatalogError` if unknown."""

        try:
            return self.routes[route_id]
        except KeyError as exc:
            raise CatalogError(f'route {route_id!r} is not in the catalog') from exc

    def direction_policy(self) -> Mapping[str, Tuple[TraversalDirection, ...]]:
        """Return the immutable allowed-direction policy for every route."""

        return MappingProxyType(
            {
                route_id: route.allowed_directions
                for route_id, route in self.routes.items()
            }
        )


def _require(mapping: Any, key: str, where: str) -> Any:
    if not isinstance(mapping, Mapping):
        raise CatalogError(f'{where} must be an object')
    if key not in mapping:
        raise CatalogError(f'{where} is missing required key {key!r}')
    return mapping[key]


def _require_exact_keys(
    mapping: Any,
    required: set[str],
    optional: set[str],
    where: str,
) -> None:
    if not isinstance(mapping, Mapping):
        raise CatalogError(f'{where} must be an object')
    missing = required - set(mapping)
    extra = set(mapping) - required - optional
    if missing:
        raise CatalogError(f'{where} is missing keys: {sorted(missing)}')
    if extra:
        raise CatalogError(f'{where} has unsupported keys: {sorted(extra)}')


def _parse_pose(raw: Any, where: str, *, allow_name: bool = False) -> Pose2D:
    _require_exact_keys(
        raw,
        {'x', 'y', 'yaw'},
        {'name'} if allow_name else set(),
        where,
    )
    if allow_name:
        name = raw.get('name')
        if name is not None and (not isinstance(name, str) or not name.strip()):
            raise CatalogError(f'{where}.name must be a non-empty string')
    x = _require(raw, 'x', where)
    y = _require(raw, 'y', where)
    yaw = _require(raw, 'yaw', where)
    for name, value in (('x', x), ('y', y), ('yaw', yaw)):
        # Reject booleans explicitly: bool is a subclass of int in Python.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise CatalogError(f'{where}.{name} must be a number')
        if not math.isfinite(value):
            raise CatalogError(f'{where}.{name} must be finite')
    return Pose2D(x=float(x), y=float(y), yaw=float(yaw))


def _parse_route(route_id: str, raw: Any) -> Route:
    where = f'route {route_id!r}'
    if not _SYMBOLIC_ID.fullmatch(route_id):
        raise CatalogError(f'{where} is not a valid symbolic route ID')
    _require_exact_keys(
        raw,
        {'description', 'closed', 'allowed_directions', 'waypoints'},
        set(),
        where,
    )
    description = _require(raw, 'description', where)
    if not isinstance(description, str) or not description.strip():
        raise CatalogError(f'{where}.description must be a non-empty string')
    closed = _require(raw, 'closed', where)
    if not isinstance(closed, bool):
        raise CatalogError(f'{where}.closed must be a boolean')
    raw_directions = _require(raw, 'allowed_directions', where)
    if not isinstance(raw_directions, list) or not raw_directions:
        raise CatalogError(f'{where}.allowed_directions must be a non-empty list')
    try:
        allowed_directions = tuple(
            TraversalDirection(direction) for direction in raw_directions
        )
    except (TypeError, ValueError) as exc:
        raise CatalogError(
            f'{where}.allowed_directions contains an unsupported direction'
        ) from exc
    if len(set(allowed_directions)) != len(allowed_directions):
        raise CatalogError(f'{where}.allowed_directions contains duplicates')
    loop_directions = {
        TraversalDirection.CLOCKWISE,
        TraversalDirection.COUNTERCLOCKWISE,
    }
    path_directions = {
        TraversalDirection.FORWARD,
        TraversalDirection.REVERSE,
    }
    expected_directions = loop_directions if closed else path_directions
    if set(allowed_directions) != expected_directions:
        route_kind = 'closed route' if closed else 'open route'
        raise CatalogError(
            f'{where}.allowed_directions does not match its {route_kind} policy'
        )
    raw_waypoints = _require(raw, 'waypoints', where)
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise CatalogError(f'{where}.waypoints must be a non-empty list')
    waypoints = []
    waypoint_names = set()
    for index, raw_wp in enumerate(raw_waypoints):
        wp_where = f'{where}.waypoints[{index}]'
        _require_exact_keys(raw_wp, {'name', 'x', 'y', 'yaw'}, set(), wp_where)
        name = _require(raw_wp, 'name', wp_where)
        if not isinstance(name, str) or not name.strip():
            raise CatalogError(f'{wp_where}.name must be a non-empty string')
        if name in waypoint_names:
            raise CatalogError(f'{where} contains duplicate waypoint name {name!r}')
        waypoint_names.add(name)
        waypoints.append(
            Waypoint(name=name, pose=_parse_pose(raw_wp, wp_where, allow_name=True))
        )
    return Route(
        route_id=route_id,
        description=str(description),
        closed=closed,
        allowed_directions=allowed_directions,
        waypoints=tuple(waypoints),
    )


def parse_catalog(data: Mapping[str, Any]) -> RouteCatalog:
    """Validate a catalog mapping and return an immutable :class:`RouteCatalog`."""

    _require_exact_keys(
        data,
        {'catalog_version', 'frame_id', 'home', 'routes'},
        {'description'},
        'catalog',
    )
    catalog_version = _require(data, 'catalog_version', 'catalog')
    if catalog_version != '1.0':
        raise CatalogError(f'unsupported catalog version {catalog_version!r}')
    frame_id = _require(data, 'frame_id', 'catalog')
    if frame_id != 'map':
        raise CatalogError("catalog.frame_id must be 'map' for Task 1")
    home = _parse_pose(
        _require(data, 'home', 'catalog'), 'catalog.home', allow_name=True
    )

    raw_routes = _require(data, 'routes', 'catalog')
    if not isinstance(raw_routes, Mapping) or not raw_routes:
        raise CatalogError('catalog.routes must be a non-empty object')

    routes: Dict[str, Route] = {}
    for route_id, raw_route in raw_routes.items():
        routes[route_id] = _parse_route(route_id, raw_route)

    return RouteCatalog(
        catalog_version=str(catalog_version),
        frame_id=frame_id,
        home=home,
        routes=MappingProxyType(routes),
    )


def default_catalog_path() -> Path:
    """Return the catalog packaged beside the mission implementation."""

    candidate = Path(__file__).resolve().parent / 'data' / CATALOG_FILENAME
    if not candidate.is_file():
        raise CatalogError(f'packaged route catalog is missing at {candidate}')
    return candidate


def load_catalog(path: Path | str | None = None) -> RouteCatalog:
    """Load and validate the route catalog from ``path`` (or the default)."""

    catalog_path = Path(path) if path is not None else default_catalog_path()
    try:
        raw_text = catalog_path.read_text(encoding='utf-8')
    except OSError as exc:
        raise CatalogError(f'cannot read catalog at {catalog_path}: {exc}') from exc
    try:
        data = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        raise CatalogError(f'catalog at {catalog_path} is not valid JSON: {exc}') from exc
    return parse_catalog(data)
