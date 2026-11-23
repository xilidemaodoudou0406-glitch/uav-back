"""
3D A* 路径规划

对应前端 src/utils/AStar3DPathPlanner.js。
采用 stdlib heapq 替换 JS 中线性扫描的 PriorityQueue，复杂度从 O(n) → O(log n)。
其余逻辑（26 邻居、启发式、平滑、统计）与前端 1:1 对齐。
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from typing import Optional

from .geo_utils import calculate_statistics, force_constant_altitude
from .grid_node import (
    GridData,
    GridInfo,
    GridNode,
    adjust_point_outside_zones,
    create_3d_grid,
    mark_blocked_nodes,
    mark_no_fly_zones,
)


class AStar3DPathPlanner:
    """3D A* 路径规划器"""

    def __init__(
        self,
        grid_resolution: float = 10.0,
        max_iterations: int = 50000,
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
        """
        规划路径，对应 JS planPath。
        返回:
            {
              "success": bool,
              "path": [...waypoint],
              "statistics": {...},
              "iterations": int,
              "elapsedMs": float,
              "error": str | None,
            }
        """
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
                # 平滑后的点可能横向进入障碍物；固定高度模式不能通过抬高修正，
                # 因此退回未经平滑、但已经过网格碰撞检查的等高路径。
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

    # -------- A* 核心 --------
    def _find_path(
        self,
        grid: list,
        start_node: GridNode,
        end_node: GridNode,
        info: GridInfo,
    ) -> tuple[list[GridNode], int]:
        """
        A* 搜索。返回 (path_nodes, iterations)。
        如果没找到完整路径，返回到目前发现的"距离目标最近"的节点的部分路径，
        与 JS 行为一致。
        """
        start_node.g = 0
        start_node.h = self._heuristic(start_node, end_node)
        start_node.update_f()

        # heapq: (f, tiebreak, node)，tiebreak 用 itertools.count 防止节点之间比较
        counter = itertools.count()
        open_heap: list[tuple[float, int, GridNode]] = []
        heapq.heappush(open_heap, (start_node.f, next(counter), start_node))

        # 用 (x,y,z) 作 key 的集合，O(1) 查 in_open / in_closed
        in_open: set[tuple[int, int, int]] = {(start_node.x, start_node.y, start_node.z)}
        closed: set[tuple[int, int, int]] = set()

        closest_node = start_node
        closest_dist = start_node.h
        iterations = 0

        while open_heap and iterations < self.max_iterations:
            iterations += 1
            _, _, current = heapq.heappop(open_heap)
            key = (current.x, current.y, current.z)
            in_open.discard(key)

            # 由于堆里可能存在同一节点的过期记录，跳过已闭合的
            if key in closed:
                continue

            # 到达终点
            if current.x == end_node.x and current.y == end_node.y and current.z == end_node.z:
                return self._reconstruct_path(current), iterations

            closed.add(key)

            dist_to_end = self._heuristic(current, end_node)
            if dist_to_end < closest_dist:
                closest_dist = dist_to_end
                closest_node = current

            for neighbor in self._get_neighbors(grid, current, info, closed=closed):
                n_key = (neighbor.x, neighbor.y, neighbor.z)
                if not neighbor.walkable or n_key in closed:
                    continue

                tentative_g = current.g + self._distance(current, neighbor)
                if n_key not in in_open or tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = self._heuristic(neighbor, end_node)
                    neighbor.update_f()
                    heapq.heappush(open_heap, (neighbor.f, next(counter), neighbor))
                    in_open.add(n_key)

        # 找不到完整路径 → 返回到最近节点的部分路径（与 JS 一致）
        if closest_dist < start_node.h:
            return self._reconstruct_path(closest_node), iterations

        return [], iterations

    # -------- 工具方法 --------
    @staticmethod
    def _heuristic(a: GridNode, b: GridNode) -> float:
        """启发式：网格坐标系下的欧氏距离（与 JS 一致）"""
        dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _distance(a: GridNode, b: GridNode) -> float:
        """节点间代价：同样是网格欧氏距离"""
        dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _get_neighbors(self, grid: list, node: GridNode, info: GridInfo, *, closed=None) -> list[GridNode]:
        """3D 26 邻居"""
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
                        # Exclude nodes that cannot be expanded BEFORE expensive
                        # voxel segment queries. Retained edges keep all checks.
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
        """从终点回溯到起点"""
        path: list[GridNode] = []
        current: Optional[GridNode] = end_node
        while current is not None:
            path.append(current)
            current = current.parent
        path.reverse()
        return path

    def _smooth_path(self, waypoints: list[dict], window_size: int = 3) -> list[dict]:
        """
        移动平均平滑，起终点保留但仍做禁飞区校正。
        对应 JS smoothPath + adjustPoints。
        """
        if len(waypoints) < 3:
            return waypoints

        smoothed: list[dict] = []
        half_left = window_size // 2
        half_right = window_size - half_left  # 等同 JS Math.ceil(windowSize/2)

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
