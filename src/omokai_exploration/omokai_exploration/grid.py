"""ROS-independent occupancy-grid primitives."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Iterator


@dataclass(frozen=True, order=True)
class Cell:
    """A zero-based occupancy-grid location."""

    row: int
    column: int

    def __post_init__(self) -> None:
        if (
            isinstance(self.row, bool)
            or not isinstance(self.row, int)
            or self.row < 0
        ):
            raise ValueError('row must be a non-negative integer')
        if (
            isinstance(self.column, bool)
            or not isinstance(self.column, int)
            or self.column < 0
        ):
            raise ValueError('column must be a non-negative integer')


@dataclass(frozen=True)
class GridMetadata:
    """Geometry needed to convert between grid and map coordinates."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.width, bool)
            or not isinstance(self.width, int)
            or self.width <= 0
        ):
            raise ValueError('width must be a positive integer')
        if (
            isinstance(self.height, bool)
            or not isinstance(self.height, int)
            or self.height <= 0
        ):
            raise ValueError('height must be a positive integer')
        if (
            isinstance(self.resolution, bool)
            or not isinstance(self.resolution, (int, float))
            or not math.isfinite(self.resolution)
            or self.resolution <= 0.0
        ):
            raise ValueError('resolution must be a positive finite number')
        for name, value in (
            ('origin_x', self.origin_x),
            ('origin_y', self.origin_y),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f'{name} must be a finite number')

    @property
    def cell_count(self) -> int:
        return self.width * self.height


@dataclass(frozen=True)
class OccupancyGrid:
    """An immutable row-major occupancy grid using ROS occupancy values."""

    metadata: GridMetadata
    cells: tuple[int, ...]

    def __init__(self, metadata: GridMetadata, cells: Iterable[int]) -> None:
        values = tuple(cells)
        if len(values) != metadata.cell_count:
            raise ValueError(
                f'expected {metadata.cell_count} cells, received {len(values)}'
            )
        if any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < -1
            or value > 100
            for value in values
        ):
            raise ValueError('occupancy values must be integers from -1 to 100')
        object.__setattr__(self, 'metadata', metadata)
        object.__setattr__(self, 'cells', values)

    def contains(self, cell: Cell) -> bool:
        return (
            cell.row < self.metadata.height
            and cell.column < self.metadata.width
        )

    def index(self, cell: Cell) -> int:
        if not self.contains(cell):
            raise IndexError(f'cell is outside the grid: {cell}')
        return cell.row * self.metadata.width + cell.column

    def cell_at_index(self, index: int) -> Cell:
        if (
            isinstance(index, bool)
            or not isinstance(index, int)
            or index < 0
            or index >= self.metadata.cell_count
        ):
            raise IndexError(f'index is outside the grid: {index}')
        row, column = divmod(index, self.metadata.width)
        return Cell(row=row, column=column)

    def value(self, cell: Cell) -> int:
        return self.cells[self.index(cell)]

    def neighbors4(self, cell: Cell) -> Iterator[Cell]:
        self.index(cell)
        for row_offset, column_offset in (
            (-1, 0),
            (0, -1),
            (0, 1),
            (1, 0),
        ):
            row = cell.row + row_offset
            column = cell.column + column_offset
            if (
                0 <= row < self.metadata.height
                and 0 <= column < self.metadata.width
            ):
                yield Cell(row=row, column=column)

    def neighbors8(self, cell: Cell) -> Iterator[Cell]:
        self.index(cell)
        for row_offset in (-1, 0, 1):
            for column_offset in (-1, 0, 1):
                if row_offset == 0 and column_offset == 0:
                    continue
                row = cell.row + row_offset
                column = cell.column + column_offset
                if (
                    0 <= row < self.metadata.height
                    and 0 <= column < self.metadata.width
                ):
                    yield Cell(row=row, column=column)

    def cell_center(self, cell: Cell) -> tuple[float, float]:
        self.index(cell)
        return (
            self.metadata.origin_x
            + (cell.column + 0.5) * self.metadata.resolution,
            self.metadata.origin_y
            + (cell.row + 0.5) * self.metadata.resolution,
        )

    def world_to_cell(self, x: float, y: float) -> Cell:
        for name, value in (('x', x), ('y', y)):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f'{name} must be a finite number')
        column = math.floor(
            (x - self.metadata.origin_x) / self.metadata.resolution
        )
        row = math.floor(
            (y - self.metadata.origin_y) / self.metadata.resolution
        )
        if row < 0 or column < 0:
            raise IndexError(f'point is outside the grid: ({x}, {y})')
        cell = Cell(row=row, column=column)
        if not self.contains(cell):
            raise IndexError(f'point is outside the grid: ({x}, {y})')
        return cell


@dataclass(frozen=True)
class MapProgress:
    """Simple map-growth measurements independent of changing map bounds."""

    total_cells: int
    known_cells: int
    unknown_cells: int
    known_area_m2: float

    @property
    def known_fraction(self) -> float:
        return self.known_cells / self.total_cells


def measure_map_progress(grid: OccupancyGrid) -> MapProgress:
    known_cells = sum(value != -1 for value in grid.cells)
    return MapProgress(
        total_cells=grid.metadata.cell_count,
        known_cells=known_cells,
        unknown_cells=grid.metadata.cell_count - known_cells,
        known_area_m2=(
            known_cells * grid.metadata.resolution * grid.metadata.resolution
        ),
    )
