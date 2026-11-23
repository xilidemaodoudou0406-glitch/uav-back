"""
3D Dijkstra 路径规划

对应前端 src/utils/Dijkstra3DPathPlanner.js。
和 A* 的差异：没有启发式，按 g 值最小展开。
默认网格分辨率 50 米（与前端一致，更粗以避免节点爆炸）。
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from typing import Optional

from .geo_utils import calculate_statistics, force_constant_altitude
from .grid_node import (
    GridInfo,
    GridNode,
    adjust_point_outside_zones,
    create_3d_grid,
    mark_blocked_nodes,
    mark_no_fly_zones,
)


class Dijkstra3DPathPlanner:
    """3D Dijkstra 路径规划器"""

    def __init__(
        self,
        grid_resolution: float = 50.0,
        max_iterations: int = 200000,
        safety_margin: float = 10.0,
        collision_checker=None,
        occupancy_cache=None,
    ) -> None:
        self.grid_resolution = grid_resolution
        self.max_iterations = max_iterations
        self.safety_margin = safety_margin
        self.no_fly_zones: list[dict] = []
        self.collision_checker = collision_checker
        self.occupancy_cache = occupancy_cache

    # -------- 公开 API --------
    def set_no_fly_zones(self, zones: list[dict]) -> None:
        self.no_fly_zones = zones or []
        if self.occupancy_cache is not None:
            self.occupancy_cache.clear()

    def set_collision_checker(self, collision_checker) -> None:
        self.collision_checker = collision_checker
        if self.occupancy_cache is not None:
            self.occupancy_cache.clear()

    def plan_path(
        self,
        min_lat: float,
        max_lat: float,
        min_lng: float,
        max_lng: float,
        start: dict,
        end: dict,
        cruise_altitude: float | None = None,
        fixed_altitude: float | None = None,
        smooth_path: bool = True,
    ) -> dict:
        """规划路径，签名/返回值结构与 AStar3DPathPlanner.plan_path 完全一致"""
        t0 = time.perf_counter()
        try:
            grid_data = create_3d_grid(
                min_lat,
                max_lat,
                min_lng,
                max_lng,
                start,
                end,
                self.grid_resolution,
                fixed_altitude=fixed_altitude,
            )
            if grid_data is None:
                raise RuntimeError("Failed to create 3D grid (start/end not walkable?)")

            blocked = (
                mark_blocked_nodes(grid_data.grid, grid_data.info, self.collision_checker,
                                   cache=self.occupancy_cache)
                if self.collision_checker is not None
                else mark_no_fly_zones(grid_data.grid, grid_data.info, self.no_fly_zones)
            )

            if not grid_data.start_node.walkable or not grid_data.end_node.walkable:
                raise RuntimeError("Start or end point is inside a no-fly zone")

            path_nodes, iterations = self._find_path(
                grid_data.grid, grid_data.start_node, grid_data.end_node, grid_data.info
            )
            reached_end = bool(path_nodes) and (
                path_nodes[-1].x == grid_data.end_node.x
                and path_nodes[-1].y == grid_data.end_node.y
                and path_nodes[-1].z == grid_data.end_node.z
            )
            if not reached_end:
                raise RuntimeError("No complete path found")

            raw_waypoints = [
                {"lat": n.lat, "lng": n.lng, "altitude": n.altitude} for n in path_nodes
            ]
            waypoints = raw_waypoints

            if smooth_path:
                waypoints = self._smooth_path(waypoints)

            if fixed_altitude is not None:
                waypoints = force_constant_altitude(waypoints, fixed_altitude)
                if self.collision_checker is not None and not self.collision_checker.path_is_safe(waypoints):
                    waypoints = raw_waypoints

            statistics = calculate_statistics(waypoints)

            return {
                "success": True,
                "path": waypoints,
                "statistics": statistics,
                "iterations": iterations,
                "blockedNodes": blocked,
                "elapsedMs": (time.perf_counter() - t0) * 1000,
                "error": None,
            }
        except Exception as e:
            return {
                "success": False,
                "path": [],
                "statistics": {},
                "iterations": 0,
                "blockedNodes": 0,
                "elapsedMs": (time.perf_counter() - t0) * 1000,
                "error": str(e),
            }

    # -------- Dijkstra 核心 --------
    def _find_path(
        self,
        grid: list,
        start_node: GridNode,
        end_node: GridNode,
        info: GridInfo,
    ) -> tuple[list[GridNode], int]:
        """
        Dijkstra 搜索。
        没有启发式，所以排序键是 g（起点到当前的代价）。
        与 JS 行为一致：如果迭代用完仍未到达终点，返回到目前发现的最近节点的部分路径。
        """
        start_node.g = 0

        counter = itertools.count()
        open_heap: list[tuple[float, int, GridNode]] = []
        heapq.heappush(open_heap, (start_node.g, next(counter), start_node))

        closed: set[tuple[int, int, int]] = set()
        closest_node = start_node
        closest_dist = math.inf
        iterations = 0

        while open_heap and iterations < self.max_iterations:
            iterations += 1
            _, _, current = heapq.heappop(open_heap)
            key = (current.x, current.y, current.z)
            if key in closed:
                continue
            closed.add(key)

            if current.x == end_node.x and current.y == end_node.y and current.z == end_node.z:
                return self._reconstruct_path(current), iterations

            dist_to_end = self._distance(current, end_node)
            if dist_to_end < closest_dist:
                closest_dist = dist_to_end
                closest_node = current

            for neighbor in self._get_neighbors(grid, current, info, closed=closed):
                n_key = (neighbor.x, neighbor.y, neighbor.z)
                if n_key in closed:
                    continue
                if not neighbor.walkable:
                    closed.add(n_key)
                    continue

                move_cost = self._distance(current, neighbor)
                tentative_g = current.g + move_cost
                if tentative_g < neighbor.g:
                    neighbor.g = tentative_g
                    neighbor.parent = current
                    heapq.heappush(open_heap, (neighbor.g, next(counter), neighbor))

        # 与 JS 一致：返回最接近终点的节点反推出的路径
        return self._reconstruct_path(closest_node), iterations

    # -------- 工具方法 --------
    @staticmethod
    def _distance(a: GridNode, b: GridNode) -> float:
        dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _get_neighbors(self, grid: list, node: GridNode, info: GridInfo, *, closed=None) -> list[GridNode]:
        neighbors: list[GridNode] = []
        x, y, z = node.x, node.y, node.z
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    nx, ny, nz = x + dx, y + dy, z + dz
                    if 0 <= nx < info.gridX and 0 <= ny < info.gridY and 0 <= nz < info.gridZ:
                        neighbor = grid[nx][ny][nz]
                        if not neighbor.walkable or (closed is not None and (nx, ny, nz) in closed):
                            continue
                        if self.collision_checker is not None and self.collision_checker.is_segment_blocked(
                            {"lat": node.lat, "lng": node.lng, "altitude": node.altitude},
                            {"lat": neighbor.lat, "lng": neighbor.lng, "altitude": neighbor.altitude},
                        ):
                            continue
                        neighbors.append(neighbor)
        return neighbors

    @staticmethod
    def _reconstruct_path(end_node: GridNode) -> list[GridNode]:
        path: list[GridNode] = []
        current: Optional[GridNode] = end_node
        while current is not None:
            path.append(current)
            current = current.parent
        path.reverse()
        return path

    def _smooth_path(self, waypoints: list[dict], window_size: int = 3) -> list[dict]:
        if len(waypoints) < 3:
            return waypoints

        smoothed: list[dict] = []
        half_left = window_size // 2
        half_right = window_size - half_left

        for i, p in enumerate(waypoints):
            if i == 0 or i == len(waypoints) - 1:
                smoothed.append(
                    adjust_point_outside_zones(p, self.no_fly_zones, safety_margin=self.safety_margin)
                )
                continue

            start = max(0, i - half_left)
            end_idx = min(len(waypoints), i + half_right)
            window = waypoints[start:end_idx]

            avg_lat = sum(w["lat"] for w in window) / len(window)
            avg_lng = sum(w["lng"] for w in window) / len(window)
            avg_alt = sum(w["altitude"] for w in window) / len(window)

            candidate = {"lat": avg_lat, "lng": avg_lng, "altitude": avg_alt}
            smoothed.append(
                adjust_point_outside_zones(candidate, self.no_fly_zones, safety_margin=self.safety_margin)
            )

        if self.collision_checker is not None and not self.collision_checker.path_is_safe(smoothed):
            return waypoints
        return smoothed
