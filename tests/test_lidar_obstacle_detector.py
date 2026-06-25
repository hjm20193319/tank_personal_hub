import unittest
from pathlib import Path

from lidar_obstacle_detector import (
    LidarPoint,
    TerrainMap,
    cluster_points,
    filter_obstacle_points,
    load_lidar_points,
    load_terrain_map,
    summarize_obstacles,
)


class TerrainMapTests(unittest.TestCase):
    def setUp(self):
        self.terrain = TerrainMap(
            {
                (0.0, 0.0): 0.0,
                (1.0, 0.0): 2.0,
                (0.0, 1.0): 4.0,
                (1.0, 1.0): 6.0,
            }
        )

    def test_returns_exact_height_at_grid_coordinate(self):
        self.assertAlmostEqual(self.terrain.height_at(1.0, 0.0), 2.0)

    def test_bilinearly_interpolates_between_four_grid_coordinates(self):
        self.assertAlmostEqual(self.terrain.height_at(0.5, 0.5), 3.0)

    def test_rejects_coordinate_outside_terrain_bounds(self):
        with self.assertRaisesRegex(ValueError, "outside terrain bounds"):
            self.terrain.height_at(-0.1, 0.5)


class PointProcessingTests(unittest.TestCase):
    def setUp(self):
        self.flat_terrain = TerrainMap(
            {
                (0.0, 0.0): 10.0,
                (2.0, 0.0): 10.0,
                (0.0, 2.0): 10.0,
                (2.0, 2.0): 10.0,
            }
        )

    def test_filter_keeps_only_points_above_terrain_threshold(self):
        points = [
            LidarPoint(0.5, 10.2, 0.5),
            LidarPoint(1.0, 10.5, 1.0),
            LidarPoint(1.5, 10.8, 1.5),
        ]

        filtered = filter_obstacle_points(
            points,
            self.flat_terrain,
            minimum_height_above_terrain=0.5,
        )

        self.assertEqual(
            filtered,
            [
                LidarPoint(1.0, 10.5, 1.0),
                LidarPoint(1.5, 10.8, 1.5),
            ],
        )

    def test_cluster_points_connects_horizontal_neighbor_chain(self):
        points = [
            LidarPoint(0.0, 1.0, 0.0),
            LidarPoint(0.4, 2.0, 0.0),
            LidarPoint(0.8, 3.0, 0.0),
        ]

        clusters = cluster_points(points, radius=0.5, minimum_points=3)

        self.assertEqual(clusters, [points])

    def test_cluster_points_drops_clusters_below_minimum_size(self):
        points = [
            LidarPoint(0.0, 1.0, 0.0),
            LidarPoint(0.2, 1.0, 0.0),
            LidarPoint(5.0, 1.0, 5.0),
        ]

        clusters = cluster_points(points, radius=0.5, minimum_points=2)

        self.assertEqual(clusters, [[points[0], points[1]]])


class ObstacleSummaryTests(unittest.TestCase):
    def test_uses_world_height_for_representative_coordinate(self):
        cluster = [
            LidarPoint(63.0, 11.0, 31.0),
            LidarPoint(64.0, 13.0, 31.0),
            LidarPoint(65.0, 15.0, 31.0),
        ]

        obstacles = summarize_obstacles([cluster], tank_x=60.0, tank_z=27.0)

        self.assertEqual(len(obstacles), 1)
        obstacle = obstacles[0]
        self.assertAlmostEqual(obstacle.x, 64.0)
        self.assertAlmostEqual(obstacle.y, 13.0)
        self.assertAlmostEqual(obstacle.z, 31.0)
        self.assertAlmostEqual(obstacle.shortest_distance, 5.0)
        self.assertEqual(obstacle.point_count, 3)


class CsvLoadingTests(unittest.TestCase):
    def test_loads_detected_lidar_points_and_terrain_coordinates(self):
        fixture_directory = Path(__file__).with_name("fixtures")
        points = load_lidar_points(fixture_directory / "lidar.csv")
        terrain = load_terrain_map(fixture_directory / "terrain.csv")

        self.assertEqual(points, [LidarPoint(1.0, 2.0, 3.0)])
        self.assertAlmostEqual(terrain.height_at(0.5, 0.5), 8.5)


if __name__ == "__main__":
    unittest.main()
