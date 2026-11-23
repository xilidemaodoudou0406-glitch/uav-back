"""Small deterministic CPU/memory benchmark; no server, DB, or files are written.

Run from backend: python tools/benchmark_planning.py --repeats 3
Measures real grid/search work, not browser FPS or real-flight safety.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import gc
import hashlib
import inspect
import json
import math
from pathlib import Path
import statistics
import sys
import time
import tracemalloc

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from algorithms.astar_3d import AStar3DPathPlanner
from algorithms.dijkstra_3d import Dijkstra3DPathPlanner
from algorithms import grid_node
from obstacles.collision import CollisionChecker


@dataclass
class ReferenceGridNode:
    """Pre-optimization object layout, for a reproducible reference mode only."""
    x: int
    y: int
    z: int
    lat: float
    lng: float
    altitude: float
    g: float = math.inf
    h: float = 0.0
    f: float = math.inf
    parent: object = field(default=None, repr=False)
    walkable: bool = True

    def update_f(self):
        self.f = self.g + self.h


class ReferenceNeighbors:
    """Original eager edge-query order, deliberately without the new pruning."""
    def _get_neighbors(self, grid, node, info, *, closed=None):
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == dy == dz == 0:
                        continue
                    nx, ny, nz = node.x+dx, node.y+dy, node.z+dz
                    if 0 <= nx < info.gridX and 0 <= ny < info.gridY and 0 <= nz < info.gridZ:
                        neighbor = grid[nx][ny][nz]
                        if self.collision_checker is not None and self.collision_checker.is_segment_blocked(
                            dict(lat=node.lat, lng=node.lng, altitude=node.altitude),
                            dict(lat=neighbor.lat, lng=neighbor.lng, altitude=neighbor.altitude),
                        ):
                            continue
                        neighbors.append(neighbor)
        return neighbors


class CountingChecker(CollisionChecker):
    def __init__(self, zones):
        super().__init__(zones)
        self.point_calls = 0
        self.segment_calls = 0

    def is_position_blocked(self, *args):
        self.point_calls += 1
        return super().is_position_blocked(*args)

    def is_segment_blocked(self, *args):
        self.segment_calls += 1
        return super().is_segment_blocked(*args)


def point(lat, lng):
    return dict(lat=lat, lng=lng, altitude=50.0)


def run_case(planner_class, *, fixed, repeats, reference=False):
    samples = []
    for _ in range(repeats):
        checker = CountingChecker([dict(south_lat=34.004, north_lat=34.008,
            west_lng=117.004, east_lng=117.008, min_altitude=0, max_altitude=80)])
        options = dict(grid_resolution=10, max_iterations=200_000, collision_checker=checker)
        if not reference and 'occupancy_cache' in inspect.signature(planner_class).parameters:
            options['occupancy_cache'] = grid_node.GridOccupancyCache()
        active_class = type(f'Reference{planner_class.__name__}', (ReferenceNeighbors, planner_class), {}) if reference else planner_class
        planner = active_class(**options)
        anchors = ([point(34.001, 117.001), point(34.01, 117.001),
            point(34.01, 117.01), point(34.001, 117.01)] if fixed else
            [point(34.003, 117.003), point(34.009, 117.009)])
        started = time.perf_counter()
        results = [planner.plan_path(34.0, 34.012, 117.0, 117.012, start, end,
            fixed_altitude=50.0 if fixed else None, smooth_path=False)
            for start, end in zip(anchors, anchors[1:])]
        elapsed = (time.perf_counter() - started) * 1000
        signature = hashlib.sha256(json.dumps([r['path'] for r in results],
            sort_keys=True).encode()).hexdigest()[:16]
        samples.append(dict(milliseconds=elapsed, positionChecks=checker.point_calls,
            segmentChecks=checker.segment_calls, pathSignature=signature,
            iterations=sum(r['iterations'] for r in results),
            success=all(r['success'] for r in results)))
    return dict(algorithm=planner_class.__name__, mode='fixed_3legs' if fixed else 'free_1leg',
        medianMs=round(statistics.median(s['milliseconds'] for s in samples), 3),
        **{k: samples[-1][k] for k in samples[-1] if k != 'milliseconds'})


def grid_memory():
    gc.collect()
    tracemalloc.start()
    data = grid_node.create_3d_grid(34, 34.012, 117, 117.012,
        point(34.001, 117.001), point(34.01, 117.01), 10, fixed_altitude=50)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return dict(nodes=data.info.gridX * data.info.gridY * data.info.gridZ,
        peakMiB=round(peak / 1024**2, 3))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--reference', action='store_true',
        help='Reproduce old node layout/eager edge queries; disable occupancy reuse (benchmark process only)')
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('repeats must be positive')
    if args.reference:
        grid_node.GridNode = ReferenceGridNode
    print(json.dumps(dict(reference=args.reference,
        cases=[run_case(AStar3DPathPlanner, fixed=True, repeats=args.repeats, reference=args.reference),
        run_case(AStar3DPathPlanner, fixed=False, repeats=args.repeats, reference=args.reference),
        run_case(Dijkstra3DPathPlanner, fixed=True, repeats=args.repeats, reference=args.reference)],
        gridMemory=grid_memory()), indent=2))
