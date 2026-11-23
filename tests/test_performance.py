"""Performance regressions: cache isolation and exact planning semantics."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from algorithms.astar_3d import AStar3DPathPlanner
from algorithms.dijkstra_3d import Dijkstra3DPathPlanner
from algorithms.grid_node import GridNode, GridOccupancyCache, create_3d_grid, mark_blocked_nodes
from obstacles.collision import CollisionChecker


ZONE = dict(south_lat=34.0008, north_lat=34.0012, west_lng=117.0008,
            east_lng=117.0012, min_altitude=0, max_altitude=80)
START = dict(lat=34.0002, lng=117.0002, altitude=50)
END = dict(lat=34.0025, lng=117.0025, altitude=50)


class Counter(CollisionChecker):
    def __init__(self, zones):
        super().__init__(zones)
        self.calls = 0
        self.edges = []

    def is_position_blocked(self, *args):
        self.calls += 1
        return super().is_position_blocked(*args)

    def is_segment_blocked(self, start, end):
        self.edges.append((start, end))
        return super().is_segment_blocked(start, end)


def grid(altitude=50, max_lng=117.003):
    return create_3d_grid(34, 34.003, 117, max_lng, START, END, 10, fixed_altitude=altitude)


def flags(data):
    return [node.walkable for col in data.grid for row in col for node in row]


class PerformanceTests(unittest.TestCase):
    def test_nodes_do_not_allocate_per_instance_attribute_dictionaries(self):
        node = GridNode(0, 0, 0, 34, 117, 50)
        self.assertFalse(hasattr(node, '__dict__'))
        node.parent = GridNode(1, 0, 0, 34, 117.001, 50)
        node.g = 3.0
        node.update_f()
        self.assertEqual(3.0, node.f)

    def test_identical_grids_reuse_mask_without_sharing_search_nodes(self):
        checker, cache = Counter([ZONE]), GridOccupancyCache()
        first, second = grid(), grid()
        count = mark_blocked_nodes(first.grid, first.info, checker, cache=cache)
        calls = checker.calls
        first.start_node.g = 0
        first.end_node.parent = first.start_node
        self.assertEqual(count, mark_blocked_nodes(second.grid, second.info, checker, cache=cache))
        self.assertEqual(calls, checker.calls)
        self.assertEqual(flags(first), flags(second))
        self.assertIsNot(first.start_node, second.start_node)
        self.assertEqual(float('inf'), second.start_node.g)
        self.assertIsNone(second.end_node.parent)
        self.assertEqual((1, 1), (cache.hits, cache.misses))

    def test_cache_key_includes_height_and_boundary(self):
        checker, cache = Counter([ZONE]), GridOccupancyCache()
        for data in (grid(), grid(altitude=100), grid(max_lng=117.0031)):
            calls = checker.calls
            mark_blocked_nodes(data.grid, data.info, checker, cache=cache)
            self.assertGreater(checker.calls, calls)
        self.assertEqual(0, cache.hits)
        self.assertEqual(3, cache.misses)

    def test_replacing_checker_never_reuses_another_obstacle_mask(self):
        cache = GridOccupancyCache()
        blocked, clear = grid(), grid()
        mark_blocked_nodes(blocked.grid, blocked.info, Counter([ZONE]), cache=cache)
        other = Counter([])
        mark_blocked_nodes(clear.grid, clear.info, other, cache=cache)
        self.assertTrue(all(flags(clear)))
        self.assertGreater(other.calls, 0)

    def test_explicit_clear_invalidates_changed_geometry(self):
        checker, cache = Counter([ZONE]), GridOccupancyCache()
        original = grid()
        mark_blocked_nodes(original.grid, original.info, checker, cache=cache)
        checker.manual_zones = []
        cache.clear()
        updated = grid()
        mark_blocked_nodes(updated.grid, updated.info, checker, cache=cache)
        self.assertTrue(all(flags(updated)))

    def test_cached_and_uncached_planners_produce_identical_ordered_legs(self):
        anchors = [START, dict(lat=34.0025, lng=117.0002, altitude=50), END, START]
        for planner_class in (AStar3DPathPlanner, Dijkstra3DPathPlanner):
            with self.subTest(planner=planner_class.__name__):
                cache = GridOccupancyCache()
                plain = planner_class(grid_resolution=10, collision_checker=Counter([ZONE]))
                cached = planner_class(grid_resolution=10, collision_checker=Counter([ZONE]),
                                       occupancy_cache=cache)
                for start, end in zip(anchors, anchors[1:]):
                    args = (34, 34.003, 117, 117.003, start, end)
                    before = plain.plan_path(*args, fixed_altitude=50, smooth_path=False)
                    after = cached.plan_path(*args, fixed_altitude=50, smooth_path=False)
                    self.assertTrue(after['success'], after['error'])
                    self.assertEqual(before['path'], after['path'])
                    self.assertEqual(before['iterations'], after['iterations'])
                self.assertEqual(2, cache.hits)

    def test_planner_setters_clear_occupancy_cache(self):
        for cls in (AStar3DPathPlanner, Dijkstra3DPathPlanner):
            cache, checker = GridOccupancyCache(), Counter([ZONE])
            data = grid()
            planner = cls(collision_checker=checker, occupancy_cache=cache)
            mark_blocked_nodes(data.grid, data.info, checker, cache=cache)
            planner.set_no_fly_zones([])
            self.assertIsNone(cache.get(data.info, checker))
            mark_blocked_nodes(data.grid, data.info, checker, cache=cache)
            planner.set_collision_checker(Counter([]))
            self.assertIsNone(cache.get(data.info, checker))

    def test_closed_and_blocked_neighbors_do_not_trigger_redundant_edge_queries(self):
        for cls in (AStar3DPathPlanner, Dijkstra3DPathPlanner):
            data = grid()
            checker = Counter([])
            center = data.grid[5][5][0]
            data.grid[4][4][0].walkable = False
            closed = {(4, 5, 0), (5, 4, 0)}
            planner = cls(collision_checker=checker)
            neighbors = planner._get_neighbors(data.grid, center, data.info, closed=closed)
            self.assertEqual(len(neighbors), len(checker.edges))
            self.assertTrue(all((n.x, n.y, n.z) not in closed for n in neighbors))
            self.assertTrue(all(n.walkable for n in neighbors))
            self.assertLess(len(neighbors), 8)


if __name__ == '__main__':
    unittest.main()
