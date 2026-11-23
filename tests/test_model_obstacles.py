from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from algorithms.astar_3d import AStar3DPathPlanner
from algorithms.dijkstra_3d import Dijkstra3DPathPlanner
from algorithms.grid_node import create_3d_grid
from obstacles.collision import CollisionChecker, segment_intersects_zone
from obstacles.surface_voxel import SurfaceVoxelDataset
import main
import schemas


OUT2_DATA_ROOT = Path(__file__).resolve().parent.parent / "obstacle_data" / "out2"
RECONSTRUCTION1_DATA_ROOT = Path(__file__).resolve().parent.parent / "obstacle_data" / "reconstruction1"


class ModelObstacleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out2_dataset = SurfaceVoxelDataset(OUT2_DATA_ROOT)
        if not cls.out2_dataset.enabled:
            raise RuntimeError(cls.out2_dataset.error)
        cls.out2_tile = cls.out2_dataset.tiles[0]
        cls.out2_tile.ensure_loaded()

        cls.reconstruction1_dataset = SurfaceVoxelDataset(RECONSTRUCTION1_DATA_ROOT)
        if not cls.reconstruction1_dataset.enabled:
            raise RuntimeError(cls.reconstruction1_dataset.error)
        cls.reconstruction1_tile = cls.reconstruction1_dataset.tiles[0]
        cls.reconstruction1_tile.ensure_loaded()

    @staticmethod
    def occupied_voxel(tile):
        assert tile.occupied is not None
        flat = tile.occupied[len(tile.occupied) // 2]
        level_row, column = divmod(flat, tile.grid["columns"])
        level, row = divmod(level_row, tile.grid["rows"])
        resolution = tile.grid["resolutionMeters"]
        east = tile.grid["originEastMeters"] + (column + 0.5) * resolution
        north = tile.grid["originNorthMeters"] + (row + 0.5) * resolution
        up = tile.grid["originUpMeters"] + (level + 0.5) * resolution
        return east, north, up

    def test_out2_is_an_independent_three_dimensional_surface_dataset(self):
        self.assertEqual(1, len(self.out2_dataset.tiles))
        self.assertEqual(0.5, self.out2_tile.grid["resolutionMeters"])
        self.assertEqual("surface-voxel-v1", self.out2_tile.grid["kind"])
        self.assertEqual("out2", self.out2_tile.metadata["id"])

    def test_out2_surface_voxel_blocked_and_empty_space_free(self):
        tile = self.out2_tile
        east, north, up = self.occupied_voxel(tile)
        resolution = tile.grid["resolutionMeters"]
        surface = tile.enu_to_wgs84(east, north, up)
        high = tile.enu_to_wgs84(
            east,
            north,
            tile.grid["originUpMeters"] + tile.grid["levels"] * resolution + 2,
        )
        checker = CollisionChecker([], self.out2_dataset)
        self.assertTrue(checker.is_position_blocked(*surface))
        self.assertFalse(checker.is_position_blocked(*high))

        start = tile.enu_to_wgs84(east - resolution, north, up)
        end = tile.enu_to_wgs84(east + resolution, north, up)
        self.assertTrue(checker.is_segment_blocked(
            {"lat": start[0], "lng": start[1], "altitude": start[2]},
            {"lat": end[0], "lng": end[1], "altitude": end[2]},
        ))

    def test_reconstruction1_is_used_by_unified_collision_checker(self):
        tile = self.reconstruction1_tile
        east, north, up = self.occupied_voxel(tile)
        surface = tile.enu_to_wgs84(east, north, up)
        checker = CollisionChecker([], None, self.reconstruction1_dataset)
        self.assertEqual(0.5, tile.grid["resolutionMeters"])
        self.assertGreater(len(tile.occupied), 20_000_000)
        self.assertTrue(checker.is_position_blocked(*surface))

        highest = self.reconstruction1_dataset.get_surface_height(surface[0], surface[1])
        self.assertIsNotNone(highest)
        self.assertGreaterEqual(highest, surface[2] - tile.grid["resolutionMeters"])

        response = main.obstacle_surface_height(
            schemas.SurfaceHeightRequest(lat=surface[0], lng=surface[1])
        )
        self.assertAlmostEqual(5.0, response.clearanceMeters)
        self.assertAlmostEqual(response.surfaceAltitude + 5.0, response.flightAltitude)
        self.assertFalse(checker.is_position_blocked(surface[0], surface[1], response.flightAltitude))

    def test_missing_surface_dataset_is_explicitly_disabled(self):
        missing = SurfaceVoxelDataset(OUT2_DATA_ROOT / "definitely-missing")
        self.assertFalse(missing.enabled)
        self.assertIn("tiles.json", missing.status()["error"])

    def test_enu_wgs84_round_trip(self):
        original = (123.5, -98.25, 18.0)
        wgs84 = self.out2_tile.enu_to_wgs84(*original)
        restored = self.out2_tile.wgs84_to_enu(*wgs84)
        self.assertLess(math.dist(original, restored), 1e-5)

    def test_manual_zone_diagonal_segment(self):
        zone = {
            "south_lat": 34.0,
            "north_lat": 34.001,
            "west_lng": 117.0,
            "east_lng": 117.001,
            "min_altitude": 20,
            "max_altitude": 80,
        }
        start = {"lat": 33.9995, "lng": 116.9995, "altitude": 50}
        end = {"lat": 34.0015, "lng": 117.0015, "altitude": 50}
        self.assertTrue(segment_intersects_zone(start, end, zone))
        self.assertTrue(CollisionChecker([zone]).is_segment_blocked(start, end))

    def test_unsafe_smoothing_falls_back(self):
        zone = {
            "south_lat": 0.00018,
            "north_lat": 0.00022,
            "west_lng": 0.00038,
            "east_lng": 0.00042,
            "min_altitude": 0,
            "max_altitude": 100,
        }
        checker = CollisionChecker([zone])
        planner = AStar3DPathPlanner(collision_checker=checker)
        original = [
            {"lat": 0.0, "lng": 0.0, "altitude": 50},
            {"lat": 0.0, "lng": 0.001, "altitude": 50},
            {"lat": 0.001, "lng": 0.001, "altitude": 50},
        ]
        self.assertTrue(checker.path_is_safe(original))
        self.assertEqual(original, planner._smooth_path(original))

    def test_oversized_grid_fails_fast_instead_of_allocating(self):
        planner = AStar3DPathPlanner(
            grid_resolution=10,
            max_iterations=50_000,
            collision_checker=CollisionChecker([]),
        )
        result = planner.plan_path(
            34.0,
            34.2,
            117.0,
            117.2,
            {"lat": 34.01, "lng": 117.01, "altitude": 5},
            {"lat": 34.19, "lng": 117.19, "altitude": 5},
        )
        self.assertFalse(result["success"])
        self.assertIn("规划范围过大", result["error"])

    def test_fixed_altitude_uses_one_grid_layer_and_never_changes_height(self):
        start = {"lat": 34.0, "lng": 117.0, "altitude": 60}
        end = {"lat": 34.001, "lng": 117.001, "altitude": 60}
        fixed_grid = create_3d_grid(
            33.9998,
            34.0012,
            116.9998,
            117.0012,
            start,
            end,
            grid_resolution=20,
            fixed_altitude=60,
        )
        free_grid = create_3d_grid(
            33.9998,
            34.0012,
            116.9998,
            117.0012,
            start,
            end,
            grid_resolution=20,
        )
        self.assertIsNotNone(fixed_grid)
        self.assertIsNotNone(free_grid)
        self.assertEqual(1, fixed_grid.info.gridZ)
        self.assertGreater(free_grid.info.gridZ, 1)

        for planner_class in (AStar3DPathPlanner, Dijkstra3DPathPlanner):
            planner = planner_class(
                grid_resolution=20,
                collision_checker=CollisionChecker([]),
            )
            result = planner.plan_path(
                33.9998,
                34.0012,
                116.9998,
                117.0012,
                start,
                end,
                fixed_altitude=60,
                smooth_path=True,
            )
            self.assertTrue(result["success"], result["error"])
            self.assertTrue(result["path"])
            self.assertTrue(all(point["altitude"] == 60 for point in result["path"]))

    def test_altitude_mode_defaults_to_fixed_and_accepts_free(self):
        base = {
            "bounds": {"minLat": 0, "maxLat": 1, "minLng": 0, "maxLng": 1},
            "start": {"lat": 0.1, "lng": 0.1},
            "end": {"lat": 0.9, "lng": 0.9},
            "altitude": 50,
            "algorithms": ["astar"],
        }
        self.assertEqual("fixed", schemas.PlanRequest(**base).altitudeMode)
        self.assertEqual("free", schemas.PlanRequest(**base, altitudeMode="free").altitudeMode)


if __name__ == "__main__":
    unittest.main()
