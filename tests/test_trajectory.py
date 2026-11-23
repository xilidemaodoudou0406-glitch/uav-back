"""Regression tests for shared curve postprocessing and timed flight trajectories.

Fixtures use metre-sized local scenes and the production manual-zone checker;
the large reconstructed model datasets are deliberately not loaded here.
Run: python -m unittest tests.test_trajectory -v
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from algorithms.geo_utils import haversine_3d_distance
from algorithms.trajectory import build_timed_trajectory, postprocess_leg
from obstacles.collision import CollisionChecker
from obstacles.geodesy import geodetic_to_ecef


REFERENCE_LAT = 34.0
REFERENCE_LNG = 117.0
METRES_PER_DEGREE = math.pi * 6371000.0 / 180.0


def point(east: float, north: float, altitude: float = 50.0) -> dict:
    return {
        "lat": REFERENCE_LAT + north / METRES_PER_DEGREE,
        "lng": REFERENCE_LNG + east / (
            METRES_PER_DEGREE * math.cos(math.radians(REFERENCE_LAT))
        ),
        "altitude": altitude,
    }


def box(west: float, south: float, east: float, north: float) -> dict:
    low, high = point(west, south), point(east, north)
    return {
        "south_lat": low["lat"],
        "north_lat": high["lat"],
        "west_lng": low["lng"],
        "east_lng": high["lng"],
        "min_altitude": 0,
        "max_altitude": 100,
    }


class LocalCurveTests(unittest.TestCase):
    def assert_same_position(self, first, second, tolerance=1e-4):
        self.assertLessEqual(haversine_3d_distance(first, second), tolerance)

    def assert_valid_result(self, result, checker):
        self.assertTrue(result["path"])
        self.assertEqual(len(result["path"]), len(result["curvatures"]))
        self.assertTrue(checker.path_is_safe(result["path"]))
        self.assertTrue(all(math.isfinite(value) and value >= 0 for value in result["curvatures"]))
        self.assertTrue(all(0 <= index < len(result["path"]) for index in result["stopIndices"]))

    def test_clear_ground_removes_grid_staircase(self):
        raw = [point(0, 0), point(10, 0), point(10, 10), point(20, 10), point(30, 20)]
        checker = CollisionChecker([])
        result = postprocess_leg(raw, checker)
        self.assert_valid_result(result, checker)
        self.assert_same_position(raw[0], result["path"][0])
        self.assert_same_position(raw[-1], result["path"][-1])
        for position in result["path"]:
            # A point on the direct segment has x:y = 30:20.
            east = (position["lng"] - REFERENCE_LNG) * METRES_PER_DEGREE * math.cos(math.radians(REFERENCE_LAT))
            north = (position["lat"] - REFERENCE_LAT) * METRES_PER_DEGREE
            self.assertAlmostEqual(north, east * 2 / 3, delta=1e-3)

    def test_none_keeps_original_geometry_and_does_not_mutate_input(self):
        raw = [point(0, 0), point(30, 0), point(30, 30)]
        saved = [dict(position) for position in raw]
        result = postprocess_leg(raw, CollisionChecker([]), method="none")
        self.assertEqual(saved, raw)
        self.assertEqual(saved, result["path"])

    def test_long_clear_route_over_shortcut_window_becomes_one_direct_segment(self):
        raw = [point(index * 0.5, 2 * math.sin(index / 10)) for index in range(601)]
        checker = CollisionChecker([])
        self.assertTrue(checker.path_is_safe(raw))
        result = postprocess_leg(raw, checker)
        self.assertEqual(2, len(result["path"]))
        self.assertEqual(raw[0], result["path"][0])
        self.assertEqual(raw[-1], result["path"][-1])

    def test_local_curve_rounds_an_obstacle_constrained_corner(self):
        raw = [point(0, 0), point(30, 0), point(30, 30)]
        checker = CollisionChecker([box(12, 6, 24, 28)])
        self.assertTrue(checker.path_is_safe(raw))
        self.assertTrue(checker.is_segment_blocked(raw[0], raw[-1]))
        result = postprocess_leg(raw, checker, transition_distance=8)
        self.assert_valid_result(result, checker)
        self.assertGreater(result["report"]["smoothedCorners"], 0)
        self.assertGreater(max(result["curvatures"]), 0)
        self.assert_same_position(raw[0], result["path"][0])
        self.assert_same_position(raw[-1], result["path"][-1])

    def test_inside_corner_collision_reverts_and_requires_stop(self):
        raw = [point(0, 0), point(30, 0), point(30, 30)]
        checker = CollisionChecker([box(-10, 0.001, 29.999, 40)])
        self.assertTrue(checker.path_is_safe(raw))
        result = postprocess_leg(raw, checker, transition_distance=12)
        self.assert_valid_result(result, checker)
        self.assertGreaterEqual(result["report"]["fallbackCorners"], 1)
        retained = [
            index for index, position in enumerate(result["path"])
            if haversine_3d_distance(position, raw[1]) < 1e-4
        ]
        self.assertTrue(retained)
        self.assertTrue(any(index in result["stopIndices"] for index in retained))

    def test_unsafe_input_is_rejected_instead_of_silently_returned(self):
        checker = CollisionChecker([box(10, -5, 20, 5)])
        with self.assertRaises(ValueError):
            postprocess_leg([point(0, 0), point(30, 0)], checker)

    def test_duplicate_vertices_do_not_make_nonfinite_curve(self):
        raw = [point(0, 0), point(0, 0), point(10, 0), point(10, 0), point(20, 20)]
        checker = CollisionChecker([])
        result = postprocess_leg(raw, checker)
        self.assert_valid_result(result, checker)
        for first, second in zip(result["path"], result["path"][1:]):
            self.assertGreater(haversine_3d_distance(first, second), 1e-6)

    def test_fixed_altitude_is_preserved_at_every_returned_sample(self):
        raw = [point(0, 0, 60), point(30, 0, 60), point(30, 30, 60)]
        checker = CollisionChecker([box(12, 6, 24, 28)])
        result = postprocess_leg(raw, checker, fixed_altitude=60)
        self.assert_valid_result(result, checker)
        self.assertTrue(all(position["altitude"] == 60 for position in result["path"]))

    def test_free_altitude_does_not_drop_below_minimum(self):
        raw = [point(0, 0, 20), point(30, 0, 30), point(30, 30, 20)]
        checker = CollisionChecker([box(12, 6, 24, 28)])
        result = postprocess_leg(raw, checker, minimum_altitude=20)
        self.assert_valid_result(result, checker)
        self.assertTrue(all(position["altitude"] >= 20 - 1e-9 for position in result["path"]))

    def test_separate_legs_preserve_ordered_protected_anchor(self):
        start, via, end = point(0, 0), point(30, 0), point(30, 30)
        checker = CollisionChecker([])
        first = postprocess_leg([start, point(15, 5), via], checker)
        second = postprocess_leg([via, point(25, 15), end], checker)
        self.assert_same_position(first["path"][-1], via)
        self.assert_same_position(second["path"][0], via)
        joined = first["path"] + second["path"][1:]
        occurrences = [index for index, position in enumerate(joined) if haversine_3d_distance(position, via) < 1e-4]
        self.assertEqual(1, len(occurrences))
        self.assertLess(0, occurrences[0])
        self.assertLess(occurrences[0], len(joined) - 1)

    def test_moving_average_compatibility_keeps_safe_endpoints(self):
        raw = [point(0, 0), point(30, 0), point(30, 30)]
        checker = CollisionChecker([box(-10, 0.001, 29.999, 40)])
        result = postprocess_leg(raw, checker, method="moving_average")
        self.assert_valid_result(result, checker)
        self.assert_same_position(raw[0], result["path"][0])
        self.assert_same_position(raw[-1], result["path"][-1])


class TimedTrajectoryTests(unittest.TestCase):
    def assert_valid_time(self, result):
        samples = result["samples"]
        self.assertEqual(len(result["path"]), len(samples))
        self.assertTrue(all(math.isfinite(sample["timeSeconds"]) for sample in samples))
        self.assertTrue(all(math.isfinite(sample["speedMetersPerSecond"]) and sample["speedMetersPerSecond"] >= 0 for sample in samples))
        self.assertTrue(all(second["timeSeconds"] > first["timeSeconds"] for first, second in zip(samples, samples[1:])))
        if samples:
            self.assertEqual(0, samples[0]["timeSeconds"])
            self.assertAlmostEqual(samples[-1]["timeSeconds"], result["stats"]["durationSeconds"])

    def test_empty_path_has_empty_samples_and_zero_totals(self):
        result = build_timed_trajectory([])
        self.assertEqual([], result["path"])
        self.assertEqual([], result["samples"])
        self.assertEqual(0, result["stats"]["durationSeconds"])
        self.assertEqual(0, result["stats"]["totalDistanceMeters"])

    def test_stationary_duplicates_do_not_divide_by_zero(self):
        result = build_timed_trajectory([point(0, 0), point(0, 0), point(0, 0)])
        self.assert_valid_time(result)
        self.assertEqual(1, len(result["samples"]))
        self.assertEqual(0, result["stats"]["durationSeconds"])
        self.assertEqual(0, result["samples"][0]["speedMetersPerSecond"])

    def test_pure_vertical_motion_has_nonzero_duration_and_vertical_speed_cap(self):
        result = build_timed_trajectory(
            [point(0, 0, 5), point(0, 0, 35)],
            phases=["takeoff", "takeoff"],
            vertical_speed=3,
        )
        self.assert_valid_time(result)
        self.assertGreaterEqual(result["stats"]["durationSeconds"], 10)
        self.assertTrue(all(sample["speedMetersPerSecond"] <= 3 + 1e-6 for sample in result["samples"]))
        self.assertTrue(all(sample["phase"] == "takeoff" for sample in result["samples"]))

    def test_two_endpoint_segment_is_densified_and_has_finite_duration(self):
        start, end = point(0, 0), point(100, 0)
        result = build_timed_trajectory([start, end], cruise_speed=12)
        self.assert_valid_time(result)
        self.assertGreater(len(result["samples"]), 2)
        self.assertEqual(start, result["path"][0])
        self.assertEqual(end, result["path"][-1])
        self.assertGreater(result["stats"]["durationSeconds"], 0)
        self.assertTrue(all(sample["speedMetersPerSecond"] <= 12 + 1e-6 for sample in result["samples"]))

    def test_curvature_limits_speed_below_cruise_setting(self):
        result = build_timed_trajectory(
            [point(0, 0), point(50, 0), point(100, 0)],
            curvatures=[0.25, 0.25, 0.25],
            cruise_speed=15,
            max_lateral_acceleration=2,
        )
        self.assert_valid_time(result)
        cap = math.sqrt(2 / 0.25)
        self.assertTrue(all(sample["speedMetersPerSecond"] <= cap + 1e-6 for sample in result["samples"]))

    def test_acceleration_and_deceleration_bound_between_samples(self):
        acceleration = 1.5
        result = build_timed_trajectory(
            [point(0, 0), point(50, 0), point(100, 0)],
            cruise_speed=15,
            max_tangential_acceleration=acceleration,
        )
        self.assert_valid_time(result)
        for first, second in zip(result["samples"], result["samples"][1:]):
            # Timing uses WGS84 ECEF chord lengths, as Cesium Cartesian playback
            # does; the older spherical helper is only used for point matching.
            distance = math.dist(
                geodetic_to_ecef(first["lat"], first["lng"], first["altitude"]),
                geodetic_to_ecef(second["lat"], second["lng"], second["altitude"]),
            )
            squared_speed_change = abs(second["speedMetersPerSecond"] ** 2 - first["speedMetersPerSecond"] ** 2)
            self.assertLessEqual(squared_speed_change, 2 * acceleration * distance + 1e-4)

    def test_protected_stop_is_remapped_after_densification(self):
        via = point(50, 0)
        result = build_timed_trajectory(
            [point(0, 0), via, point(50, 50)],
            stop_indices=[0, 1, 2],
        )
        self.assert_valid_time(result)
        matching = [sample for sample in result["samples"] if haversine_3d_distance(sample, via) < 1e-4]
        self.assertTrue(matching)
        self.assertTrue(all(sample["speedMetersPerSecond"] == 0 for sample in matching))

    def test_takeoff_cruise_landing_all_have_real_time(self):
        raw = [point(0, 0, 5), point(0, 0, 35), point(100, 0, 35), point(100, 0, 5)]
        result = build_timed_trajectory(
            raw,
            phases=["takeoff", "cruise", "landing", "landing"],
            stop_indices=[0, 1, 2, 3],
        )
        self.assert_valid_time(result)
        times = []
        for anchor in raw:
            matches = [sample for sample in result["samples"] if haversine_3d_distance(sample, anchor) < 1e-4]
            self.assertTrue(matches)
            times.append(matches[0]["timeSeconds"])
        self.assertTrue(all(second > first for first, second in zip(times, times[1:])))
        self.assertEqual({"takeoff", "cruise", "landing"}, {sample["phase"] for sample in result["samples"]})

    def test_duplicate_anchor_does_not_remove_requested_stop(self):
        via = point(50, 0)
        result = build_timed_trajectory(
            [point(0, 0), via, dict(via), point(100, 0)],
            stop_indices=[2],
        )
        self.assert_valid_time(result)
        matches = [sample for sample in result["samples"] if haversine_3d_distance(sample, via) < 1e-4]
        self.assertTrue(matches)
        self.assertTrue(any(sample["speedMetersPerSecond"] == 0 for sample in matches))

    def test_near_u_turn_with_stop_has_finite_nonzero_timing(self):
        raw = [point(0, 0), point(20, 0), point(0, 0.01)]
        result = build_timed_trajectory(raw, stop_indices=[1])
        self.assert_valid_time(result)
        self.assertGreater(result["stats"]["durationSeconds"], 0)
        self.assertTrue(all(math.isfinite(value) for value in result["stats"].values() if isinstance(value, (int, float))))


class PlanningIntegrationTests(unittest.TestCase):
    """Exercise request/response stitching without loading models or a large grid."""

    @classmethod
    def setUpClass(cls):
        import main
        import schemas

        cls.main = main
        cls.schemas = schemas

    def request(self, **changes):
        body = {
            "bounds": {
                "minLat": REFERENCE_LAT - 0.001,
                "maxLat": REFERENCE_LAT + 0.002,
                "minLng": REFERENCE_LNG - 0.001,
                "maxLng": REFERENCE_LNG + 0.002,
            },
            "start": point(0, 0, 5),
            "end": point(100, 50, 5),
            "altitude": 35,
            "algorithms": ["astar"],
            "options": {"smoothingMethod": "local_quintic_bezier"},
        }
        body.update(changes)
        return self.schemas.PlanRequest(**body)

    @staticmethod
    def fake_leg(*args, **kwargs):
        return {
            "success": True,
            "path": [dict(args[4]), dict(args[5])],
            "elapsedMs": 1.0,
            "iterations": 2,
        }

    def test_both_algorithms_use_shared_postprocessing_and_matching_playback_path(self):
        for algorithm, planner_name in (("astar", "AStar3DPathPlanner"), ("dijkstra", "Dijkstra3DPathPlanner")):
            with self.subTest(algorithm=algorithm), patch.object(self.main, planner_name) as factory:
                factory.return_value.plan_path.side_effect = self.fake_leg
                result = self.main._run_single_algorithm(algorithm, self.request(), [])
                self.assertTrue(result.success, result.message)
                self.assertEqual(len(result.path), len(result.trajectorySamples))
                self.assertGreater(result.trajectoryStats.durationSeconds, 0)
                self.assertEqual("local_quintic_bezier", result.smoothingReport.method)
                for location, sample in zip(result.path, result.trajectorySamples):
                    self.assertEqual(location.model_dump(), {key: getattr(sample, key) for key in ("lat", "lng", "altitude")})
                self.assertTrue(all(call.kwargs["smooth_path"] is False for call in factory.return_value.plan_path.call_args_list))

    def test_fixed_mode_keeps_ordered_cruise_anchors_and_stops(self):
        via_points = [point(30, 0, 80), point(60, 40, 70)]
        with patch.object(self.main, "AStar3DPathPlanner") as factory:
            factory.return_value.plan_path.side_effect = self.fake_leg
            result = self.main._run_single_algorithm("astar", self.request(viaPoints=via_points), [])
        self.assertTrue(result.success, result.message)
        indices = []
        for anchor in via_points:
            cruise_anchor = {**anchor, "altitude": 35}
            matches = [index for index, sample in enumerate(result.trajectorySamples) if haversine_3d_distance(sample.model_dump(), cruise_anchor) < 1e-4]
            self.assertTrue(matches)
            indices.append(matches[0])
            self.assertEqual(0, result.trajectorySamples[matches[0]].speedMetersPerSecond)
        self.assertLess(indices[0], indices[1])
        self.assertTrue(all(sample.altitude == 35 for sample in result.trajectorySamples if sample.phase == "cruise"))

    def test_free_mode_preserves_higher_cruise_anchor(self):
        via = point(50, 0, 60)
        with patch.object(self.main, "AStar3DPathPlanner") as factory:
            factory.return_value.plan_path.side_effect = self.fake_leg
            result = self.main._run_single_algorithm("astar", self.request(viaPoints=[via], altitudeMode="free"), [])
        self.assertTrue(result.success, result.message)
        self.assertTrue(any(haversine_3d_distance(sample.model_dump(), via) < 1e-4 for sample in result.trajectorySamples))
        self.assertTrue(all(sample.altitude >= 35 - 1e-9 for sample in result.trajectorySamples if sample.phase == "cruise"))

    def test_same_start_end_still_times_vertical_takeoff_and_landing(self):
        location = point(0, 0, 5)
        with patch.object(self.main, "AStar3DPathPlanner") as factory:
            result = self.main._run_single_algorithm("astar", self.request(start=location, end=location), [])
        self.assertTrue(result.success, result.message)
        factory.return_value.plan_path.assert_not_called()
        self.assertGreater(result.trajectoryStats.durationSeconds, 20)
        self.assertEqual(location, result.path[0].model_dump())
        self.assertEqual(location, result.path[-1].model_dump())

    def test_unsafe_raw_leg_fails_as_structured_result(self):
        with patch.object(self.main, "AStar3DPathPlanner") as factory:
            factory.return_value.plan_path.side_effect = self.fake_leg
            result = self.main._run_single_algorithm(
                "astar",
                self.request(end=point(100, 0, 5)),
                [box(40, -5, 60, 5)],
            )
        self.assertFalse(result.success)
        self.assertEqual([], result.path)
        self.assertIn("postprocessing", result.message)

    def test_legacy_request_and_disabled_toggle_keep_expected_method(self):
        for options, expected in (({}, "moving_average"), ({"smoothPath": False, "smoothingMethod": "local_quintic_bezier"}, "none")):
            with self.subTest(options=options), patch.object(self.main, "AStar3DPathPlanner") as factory:
                factory.return_value.plan_path.side_effect = self.fake_leg
                result = self.main._run_single_algorithm("astar", self.request(options=options), [])
            self.assertTrue(result.success, result.message)
            self.assertEqual(expected, result.smoothingReport.method)

    def test_successful_single_grid_node_restores_two_distinct_exact_anchors(self):
        request = self.request(end=point(4, 0, 5))
        with patch.object(self.main, "AStar3DPathPlanner") as factory, patch.object(
            self.main, "postprocess_leg", wraps=postprocess_leg,
        ) as postprocess:
            factory.return_value.plan_path.return_value = {
                "success": True,
                "path": [point(2, 0, 35)],
                "elapsedMs": 1.0,
                "iterations": 1,
            }
            result = self.main._run_single_algorithm("astar", request, [])
        self.assertTrue(result.success, result.message)
        restored = postprocess.call_args.args[0]
        self.assertEqual(2, len(restored))
        self.assertEqual({**request.start.model_dump(), "altitude": 35}, restored[0])
        self.assertEqual({**request.end.model_dump(), "altitude": 35}, restored[-1])
        self.assertEqual(request.start.model_dump(), result.path[0].model_dump())
        self.assertEqual(request.end.model_dump(), result.path[-1].model_dump())

    def test_successful_but_empty_search_result_is_structured_failure(self):
        with patch.object(self.main, "AStar3DPathPlanner") as factory:
            factory.return_value.plan_path.return_value = {
                "success": True,
                "path": [],
                "elapsedMs": 1.0,
                "iterations": 1,
            }
            result = self.main._run_single_algorithm("astar", self.request(), [])
        self.assertFalse(result.success)
        self.assertEqual([], result.path)
        self.assertTrue(result.message)

    def test_real_astar_fixed_height_around_manual_box_smoke(self):
        lower, upper = point(-20, -60), point(140, 70)
        start, end = point(0, 0, 5), point(120, 0, 5)
        zones = [box(40, -15, 80, 15)]
        request = self.request(
            start=start,
            end=end,
            altitude=50,
            bounds={
                "minLat": lower["lat"], "maxLat": upper["lat"],
                "minLng": lower["lng"], "maxLng": upper["lng"],
            },
            options={
                "gridResolution": 20,
                "smoothingMethod": "local_quintic_bezier",
                "transitionDistanceMeters": 12,
            },
        )
        result = self.main._run_single_algorithm("astar", request, zones)
        self.assertTrue(result.success, result.message)
        self.assertGreater(result.smoothingReport.smoothedCorners, 0)
        self.assertEqual(start, result.path[0].model_dump())
        self.assertEqual(end, result.path[-1].model_dump())
        final_path = [position.model_dump() for position in result.path]
        self.assertTrue(CollisionChecker(zones).path_is_safe(final_path))
        self.assertTrue(all(sample.altitude == 50 for sample in result.trajectorySamples if sample.phase == "cruise"))


if __name__ == "__main__":
    unittest.main()
