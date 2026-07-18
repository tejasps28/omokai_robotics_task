import math
import unittest

from omokai_exploration import (
    Cell,
    GridMetadata,
    OccupancyGrid,
    measure_map_progress,
)


def make_grid() -> OccupancyGrid:
    return OccupancyGrid(
        GridMetadata(
            width=3,
            height=2,
            resolution=0.5,
            origin_x=-1.0,
            origin_y=2.0,
        ),
        [-1, 0, 100, 10, 20, 30],
    )


class GridMetadataTest(unittest.TestCase):
    def test_reports_cell_count(self) -> None:
        metadata = make_grid().metadata

        self.assertEqual(6, metadata.cell_count)

    def test_rejects_invalid_dimensions_and_geometry(self) -> None:
        values = (
            {'width': 0},
            {'height': -1},
            {'resolution': 0.0},
            {'resolution': math.inf},
            {'origin_x': math.nan},
            {'origin_y': True},
        )
        defaults = {
            'width': 1,
            'height': 1,
            'resolution': 0.1,
            'origin_x': 0.0,
            'origin_y': 0.0,
        }

        for changes in values:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                GridMetadata(**(defaults | changes))


class OccupancyGridTest(unittest.TestCase):
    def test_requires_exact_cell_count(self) -> None:
        metadata = GridMetadata(2, 2, 1.0, 0.0, 0.0)

        with self.assertRaises(ValueError):
            OccupancyGrid(metadata, [0, 0, 0])

    def test_rejects_values_outside_ros_occupancy_range(self) -> None:
        metadata = GridMetadata(1, 1, 1.0, 0.0, 0.0)

        for value in (-2, 101, 0.5, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                OccupancyGrid(metadata, [value])

    def test_uses_ros_row_major_indexing(self) -> None:
        grid = make_grid()

        self.assertEqual(0, grid.index(Cell(0, 0)))
        self.assertEqual(5, grid.index(Cell(1, 2)))
        self.assertEqual(Cell(1, 1), grid.cell_at_index(4))
        self.assertEqual(20, grid.value(Cell(1, 1)))

    def test_rejects_cells_and_indices_outside_grid(self) -> None:
        grid = make_grid()

        with self.assertRaises(IndexError):
            grid.index(Cell(2, 0))
        with self.assertRaises(IndexError):
            grid.cell_at_index(6)
        with self.assertRaises(IndexError):
            grid.cell_at_index(-1)

    def test_four_connected_neighbors_have_stable_order(self) -> None:
        grid = make_grid()

        self.assertEqual(
            [Cell(0, 1), Cell(1, 0)],
            list(grid.neighbors4(Cell(0, 0))),
        )
        self.assertEqual(
            [Cell(0, 1), Cell(1, 0), Cell(1, 2)],
            list(grid.neighbors4(Cell(1, 1))),
        )

    def test_eight_connected_neighbors_respect_edges(self) -> None:
        grid = make_grid()

        self.assertEqual(
            [Cell(0, 1), Cell(1, 0), Cell(1, 1)],
            list(grid.neighbors8(Cell(0, 0))),
        )

    def test_converts_cell_center_to_world_coordinates(self) -> None:
        grid = make_grid()

        self.assertEqual((-0.75, 2.25), grid.cell_center(Cell(0, 0)))
        self.assertEqual((0.25, 2.75), grid.cell_center(Cell(1, 2)))

    def test_world_to_cell_round_trip(self) -> None:
        grid = make_grid()

        for cell in (Cell(0, 0), Cell(0, 2), Cell(1, 1)):
            with self.subTest(cell=cell):
                self.assertEqual(cell, grid.world_to_cell(*grid.cell_center(cell)))

    def test_world_to_cell_uses_half_open_grid_bounds(self) -> None:
        grid = make_grid()

        self.assertEqual(Cell(0, 0), grid.world_to_cell(-1.0, 2.0))
        for point in (
            (-1.0001, 2.0),
            (-1.0, 1.9999),
            (0.5, 2.0),
            (-1.0, 3.0),
        ):
            with self.subTest(point=point), self.assertRaises(IndexError):
                grid.world_to_cell(*point)

    def test_measures_known_map_area_and_fraction(self) -> None:
        progress = measure_map_progress(make_grid())

        self.assertEqual(6, progress.total_cells)
        self.assertEqual(5, progress.known_cells)
        self.assertEqual(1, progress.unknown_cells)
        self.assertAlmostEqual(1.25, progress.known_area_m2)
        self.assertAlmostEqual(5.0 / 6.0, progress.known_fraction)


if __name__ == '__main__':
    unittest.main()
