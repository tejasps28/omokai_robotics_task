import math
import unittest

from omokai_exploration import (
    Cell,
    FrontierConfig,
    GridMetadata,
    OccupancyGrid,
    frontier_cells,
    frontier_clusters,
    rank_frontiers,
)


def grid_from_rows(rows, resolution=1.0) -> OccupancyGrid:
    return OccupancyGrid(
        GridMetadata(
            width=len(rows[0]),
            height=len(rows),
            resolution=resolution,
            origin_x=0.0,
            origin_y=0.0,
        ),
        (value for row in rows for value in row),
    )


def two_frontier_grid() -> OccupancyGrid:
    return grid_from_rows(
        [
            [-1, -1, -1, -1, -1, -1, -1],
            [-1, 0, 0, -1, 0, 0, -1],
            [-1, 0, 0, -1, 0, 0, -1],
            [100, 100, 100, -1, 0, 0, -1],
            [100, 100, 100, 100, 100, 100, 100],
        ]
    )


class FrontierConfigTest(unittest.TestCase):
    def test_rejects_overlapping_thresholds(self) -> None:
        with self.assertRaises(ValueError):
            FrontierConfig(free_threshold=65, occupied_threshold=65)

    def test_rejects_invalid_sizes_and_distances(self) -> None:
        for changes in (
            {'min_cluster_size': 0},
            {'clearance_m': -0.1},
            {'blacklist_radius_m': math.inf},
            {'distance_weight': True},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                FrontierConfig(**changes)


class FrontierExtractionTest(unittest.TestCase):
    def test_finds_only_free_cells_adjacent_to_unknown_space(self) -> None:
        grid = grid_from_rows(
            [
                [-1, -1, -1],
                [100, 0, 50],
                [100, 0, 100],
            ]
        )

        self.assertEqual(
            (Cell(1, 1),),
            frontier_cells(grid, FrontierConfig(free_threshold=20)),
        )

    def test_clusters_with_eight_connectivity_in_stable_order(self) -> None:
        grid = two_frontier_grid()
        clusters = frontier_clusters(grid, frontier_cells(grid))

        self.assertEqual(2, len(clusters))
        self.assertEqual(
            (Cell(1, 1), Cell(1, 2), Cell(2, 1), Cell(2, 2)),
            clusters[0],
        )
        self.assertEqual(6, len(clusters[1]))

    def test_filters_clusters_below_minimum_size(self) -> None:
        candidates = rank_frontiers(
            two_frontier_grid(),
            robot_x=0.0,
            robot_y=0.0,
            config=FrontierConfig(
                min_cluster_size=5,
                clearance_m=0.0,
                information_gain_weight=1.0,
                distance_weight=0.0,
            ),
        )

        self.assertEqual(1, len(candidates))
        self.assertEqual(6, len(candidates[0].cells))

    def test_scores_information_gain_and_distance(self) -> None:
        candidates = rank_frontiers(
            two_frontier_grid(),
            robot_x=0.5,
            robot_y=1.5,
            config=FrontierConfig(
                min_cluster_size=1,
                clearance_m=0.0,
                information_gain_weight=1.0,
                distance_weight=1.0,
            ),
        )

        self.assertEqual(Cell(1, 1), candidates[0].goal)
        self.assertAlmostEqual(1.0, candidates[0].distance_m)
        self.assertAlmostEqual(4.0, candidates[0].information_gain_m)
        self.assertAlmostEqual(3.0, candidates[0].score)

    def test_ties_are_broken_by_row_major_goal_index(self) -> None:
        grid = grid_from_rows(
            [
                [-1, -1, -1, -1, -1],
                [-1, 0, -1, 0, -1],
                [100, 100, 100, 100, 100],
            ]
        )
        candidates = rank_frontiers(
            grid,
            robot_x=2.5,
            robot_y=1.5,
            config=FrontierConfig(
                min_cluster_size=1,
                clearance_m=0.0,
                information_gain_weight=1.0,
                distance_weight=1.0,
            ),
        )

        self.assertEqual(Cell(1, 1), candidates[0].goal)
        self.assertEqual(Cell(1, 3), candidates[1].goal)

    def test_clearance_rejects_goals_near_occupied_cells(self) -> None:
        grid = grid_from_rows(
            [
                [-1, -1, -1],
                [-1, 0, -1],
                [100, 100, 100],
            ]
        )

        candidates = rank_frontiers(
            grid,
            robot_x=0.0,
            robot_y=0.0,
            config=FrontierConfig(
                min_cluster_size=1,
                clearance_m=1.0,
            ),
        )

        self.assertEqual((), candidates)

    def test_blacklist_removes_only_goals_inside_radius(self) -> None:
        grid = two_frontier_grid()
        config = FrontierConfig(
            min_cluster_size=1,
            clearance_m=0.0,
            blacklist_radius_m=2.5,
            information_gain_weight=1.0,
            distance_weight=0.0,
        )

        candidates = rank_frontiers(
            grid,
            robot_x=0.0,
            robot_y=0.0,
            config=config,
            blacklist=((1.5, 1.5),),
        )

        self.assertEqual(1, len(candidates))
        self.assertGreaterEqual(candidates[0].goal.column, 4)

    def test_rejects_non_finite_robot_or_blacklist_points(self) -> None:
        grid = two_frontier_grid()

        with self.assertRaises(ValueError):
            rank_frontiers(grid, robot_x=math.nan, robot_y=0.0)
        with self.assertRaises(ValueError):
            rank_frontiers(
                grid,
                robot_x=0.0,
                robot_y=0.0,
                blacklist=((0.0, math.inf),),
            )


if __name__ == '__main__':
    unittest.main()
