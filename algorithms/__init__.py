"""
algorithms 包 —— 无人机 3D 路径规划算法

从前端 src/utils/AStar3DPathPlanner.js & Dijkstra3DPathPlanner.js
以及 src/stores/flight.js 中的几何工具函数迁移而来。

使用：
    from algorithms import AStar3DPathPlanner, Dijkstra3DPathPlanner

    planner = AStar3DPathPlanner(grid_resolution=10)
    planner.set_no_fly_zones(zones)
    result = planner.plan_path(
        min_lat, max_lat, min_lng, max_lng,
        start={"lat":..., "lng":..., "altitude": 120},
        end={"lat":..., "lng":..., "altitude": 120},
        smooth_path=True,
    )
    # result["path"] 是巡航段，前端会再拼上 takeoff/landing；
    # 也可在后端直接拼好：
    from algorithms import plan_takeoff_path, plan_landing_path, force_constant_altitude
"""

from .astar_3d import AStar3DPathPlanner
from .dijkstra_3d import Dijkstra3DPathPlanner
from .geo_utils import (
    calculate_bearing,
    calculate_statistics,
    create_direct_path,
    force_constant_altitude,
    haversine_3d_distance,
    plan_landing_path,
    plan_takeoff_path,
)
from .grid_node import (
    GridData,
    GridInfo,
    GridNode,
    adjust_point_outside_zones,
    create_3d_grid,
    get_grid_node,
    is_point_in_any_zone,
    is_point_in_zone,
    mark_no_fly_zones,
    nearest_safe_position,
)

__all__ = [
    # planners
    "AStar3DPathPlanner",
    "Dijkstra3DPathPlanner",
    # geo helpers
    "calculate_bearing",
    "calculate_statistics",
    "create_direct_path",
    "force_constant_altitude",
    "haversine_3d_distance",
    "plan_landing_path",
    "plan_takeoff_path",
    # grid
    "GridData",
    "GridInfo",
    "GridNode",
    "create_3d_grid",
    "get_grid_node",
    "mark_no_fly_zones",
    "is_point_in_zone",
    "is_point_in_any_zone",
    "nearest_safe_position",
    "adjust_point_outside_zones",
]
