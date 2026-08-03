"""Deterministic frontier extraction and goal selection."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .grid import Cell, OccupancyGrid


@dataclass(frozen=True)
class FrontierConfig:
    """Thresholds and weights used to select exploration goals."""

    free_threshold: int = 20
    occupied_threshold: int = 65
    min_cluster_size: int = 5
    clearance_m: float = 0.25
    blacklist_radius_m: float = 0.5
    visited_radius_m: float = 0.6
    information_gain_weight: float = 1.0
    distance_weight: float = 1.0

    def __post_init__(self) -> None:
        if (
            isinstance(self.free_threshold, bool)
            or not isinstance(self.free_threshold, int)
            or not 0 <= self.free_threshold < 100
        ):
            raise ValueError('free_threshold must be an integer from 0 to 99')
        if (
            isinstance(self.occupied_threshold, bool)
            or not isinstance(self.occupied_threshold, int)
            or not 1 <= self.occupied_threshold <= 100
        ):
            raise ValueError(
                'occupied_threshold must be an integer from 1 to 100'
            )
        if self.free_threshold >= self.occupied_threshold:
            raise ValueError(
                'free_threshold must be lower than occupied_threshold'
            )
        if (
            isinstance(self.min_cluster_size, bool)
            or not isinstance(self.min_cluster_size, int)
            or self.min_cluster_size <= 0
        ):
            raise ValueError('min_cluster_size must be a positive integer')
        for name, value in (
            ('clearance_m', self.clearance_m),
            ('blacklist_radius_m', self.blacklist_radius_m),
            ('visited_radius_m', self.visited_radius_m),
            ('information_gain_weight', self.information_gain_weight),
            ('distance_weight', self.distance_weight),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f'{name} must be a non-negative finite number')


@dataclass(frozen=True)
class FrontierCandidate:
    """A connected frontier and its selected free-space navigation goal."""

    cells: tuple[Cell, ...]
    goal: Cell
    goal_x: float
    goal_y: float
    distance_m: float
    information_gain_m: float
    score: float


def frontier_cells(
    grid: OccupancyGrid,
    config: FrontierConfig = FrontierConfig(),
) -> tuple[Cell, ...]:
    """Return free cells that share a four-connected edge with unknown space."""

    result = []
    for index, value in enumerate(grid.cells):
        if not _is_free(value, config):
            continue
        cell = grid.cell_at_index(index)
        if any(grid.value(neighbor) == -1 for neighbor in grid.neighbors4(cell)):
            result.append(cell)
    return tuple(result)


def frontier_clusters(
    grid: OccupancyGrid,
    cells: Iterable[Cell],
) -> tuple[tuple[Cell, ...], ...]:
    """Group frontier cells using deterministic eight-connectivity."""

    remaining = set(cells)
    clusters = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        pending = [seed]
        cluster = []
        while pending:
            cell = pending.pop()
            cluster.append(cell)
            connected = sorted(
                (
                    neighbor
                    for neighbor in grid.neighbors8(cell)
                    if neighbor in remaining
                ),
                reverse=True,
            )
            for neighbor in connected:
                remaining.remove(neighbor)
                pending.append(neighbor)
        clusters.append(tuple(sorted(cluster)))
    return tuple(clusters)


def rank_frontiers(
    grid: OccupancyGrid,
    robot_x: float,
    robot_y: float,
    config: FrontierConfig = FrontierConfig(),
    blacklist: Iterable[tuple[float, float]] = (),
    visited: Iterable[tuple[float, float]] = (),
) -> tuple[FrontierCandidate, ...]:
    """Extract, filter, and rank frontiers for the current robot position."""

    _require_finite_point('robot', robot_x, robot_y)
    blocked_points = tuple(blacklist)
    for index, (x, y) in enumerate(blocked_points):
        _require_finite_point(f'blacklist[{index}]', x, y)
    visited_points = tuple(visited)
    for index, (x, y) in enumerate(visited_points):
        _require_finite_point(f'visited[{index}]', x, y)

    candidates = []
    for cluster in frontier_clusters(grid, frontier_cells(grid, config)):
        if len(cluster) < config.min_cluster_size:
            continue
        eligible = tuple(
            cell
            for cell in cluster
            if _has_clearance(grid, cell, config)
            and not _is_near_points(
                grid,
                cell,
                blocked_points,
                config.blacklist_radius_m,
            )
            and not _is_near_points(
                grid,
                cell,
                visited_points,
                config.visited_radius_m,
            )
        )
        if not eligible:
            continue

        mean_row = sum(cell.row for cell in cluster) / len(cluster)
        mean_column = sum(cell.column for cell in cluster) / len(cluster)
        goal = min(
            eligible,
            key=lambda cell: (
                (cell.row - mean_row) ** 2
                + (cell.column - mean_column) ** 2,
                grid.index(cell),
            ),
        )
        goal_x, goal_y = grid.cell_center(goal)
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        information_gain = len(cluster) * grid.metadata.resolution
        score = (
            config.information_gain_weight * information_gain
            - config.distance_weight * distance
        )
        candidates.append(
            FrontierCandidate(
                cells=cluster,
                goal=goal,
                goal_x=goal_x,
                goal_y=goal_y,
                distance_m=distance,
                information_gain_m=information_gain,
                score=score,
            )
        )

    return tuple(
        sorted(
            candidates,
            key=lambda candidate: (
                -candidate.score,
                grid.index(candidate.goal),
            ),
        )
    )


def _is_free(value: int, config: FrontierConfig) -> bool:
    return 0 <= value <= config.free_threshold


def _is_occupied(value: int, config: FrontierConfig) -> bool:
    return value >= config.occupied_threshold


def _has_clearance(
    grid: OccupancyGrid,
    cell: Cell,
    config: FrontierConfig,
) -> bool:
    radius_cells = math.ceil(config.clearance_m / grid.metadata.resolution)
    if radius_cells == 0:
        return True
    for row in range(
        max(0, cell.row - radius_cells),
        min(grid.metadata.height, cell.row + radius_cells + 1),
    ):
        for column in range(
            max(0, cell.column - radius_cells),
            min(grid.metadata.width, cell.column + radius_cells + 1),
        ):
            neighbor = Cell(row=row, column=column)
            if _is_occupied(grid.value(neighbor), config):
                return False
    return True


def _is_near_points(
    grid: OccupancyGrid,
    cell: Cell,
    points: tuple[tuple[float, float], ...],
    radius_m: float,
) -> bool:
    x, y = grid.cell_center(cell)
    return any(
        math.hypot(x - blocked_x, y - blocked_y)
        <= radius_m
        for blocked_x, blocked_y in points
    )


def _require_finite_point(name: str, x: float, y: float) -> None:
    for axis, value in (('x', x), ('y', y)):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ValueError(f'{name} {axis} must be a finite number')
