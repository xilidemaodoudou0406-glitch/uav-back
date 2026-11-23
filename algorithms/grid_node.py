"""
3D 网格节点 + 网格构建 + 禁飞区标记

对应前端：
- GridNode 类（AStar3DPathPlanner.js / Dijkstra3DPathPlanner.js）
- create3DGrid
- getGridNode
- markNoFlyZones
- isPointInZone

A* 和 Dijkstra 共享这些逻辑，统一抽到这里。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from .geo_utils import meters_per_degree


# 规则三维网格使用 Python 对象保存每个节点，节点数量过大会迅速消耗内存，
# 并让障碍物标记阶段长时间占用 CPU。前端通常会在 25 万节点内自动调整
# 分辨率；此处再设置硬上限，保护 API 不被其他客户端的大范围请求拖死。
MAX_GRID_NODES = 600_000


@dataclass(slots=True)
class GridNode:
    """3D 网格中的单个节点"""
    x: int
    y: int
    z: int
    lat: float
    lng: float
    altitude: float
    g: float = math.inf          # 起点到该节点的代价（Dijkstra/A* 都用）
    h: float = 0.0               # 启发式代价（仅 A* 用）
    f: float = math.inf          # f = g + h（仅 A* 用）
    parent: Optional["GridNode"] = field(default=None, repr=False)
    walkable: bool = True

    def update_f(self) -> None:
        self.f = self.g + self.h


@dataclass
class GridInfo:
    """网格元信息，用于后续坐标反查、禁飞区扫描等"""
    minLat: float
    maxLat: float
    minLng: float
    maxLng: float
    minAlt: float
    maxAlt: float
    gridX: int
    gridY: int
    gridZ: int


@dataclass
class GridData:
    """网格创建结果"""
    grid: list                   # 三维 list: grid[x][y][z] -> GridNode
    start_node: GridNode
    end_node: GridNode
    info: GridInfo


# ============================================================
# 网格创建
# ============================================================
def create_3d_grid(
    min_lat: float,
    max_lat: float,
    min_lng: float,
    max_lng: float,
    start: dict,           # {"lat", "lng", "altitude"}
    end: dict,
    grid_resolution: float = 10.0,
    fixed_altitude: float | None = None,
) -> Optional[GridData]:
    """
    创建 3D 网格，对应 AStar/Dijkstra 中的 create3DGrid。

    - 水平边界：使用调用者传入的工作区域
    - 自由高度：min(start.alt, end.alt) ~ max(start.alt, end.alt)+100
    - 固定高度：只创建 fixed_altitude 所在的一层网格
    - 经纬度→米：使用区域中纬度的 WGS84 局部尺度
    """
    if fixed_altitude is None:
        min_alt = min(start["altitude"], end["altitude"])
        max_alt = max(start["altitude"], end["altitude"]) + 100
    else:
        min_alt = float(fixed_altitude)
        max_alt = float(fixed_altitude)

    lat_range = max_lat - min_lat
    lng_range = max_lng - min_lng
    alt_range = max_alt - min_alt

    lat_scale, lng_scale = meters_per_degree((min_lat+max_lat)/2)
    lat_meters = lat_range * lat_scale
    lng_meters = lng_range * lng_scale
    alt_meters = alt_range

    grid_x = max(1, math.ceil(lng_meters / grid_resolution))
    grid_y = max(1, math.ceil(lat_meters / grid_resolution))
    grid_z = 1 if fixed_altitude is not None else max(1, math.ceil(alt_meters / grid_resolution))

    grid_node_count = grid_x * grid_y * grid_z
    if grid_node_count > MAX_GRID_NODES:
        raise RuntimeError(
            "规划范围过大：预计生成 "
            f"{grid_node_count:,} 个三维网格节点（上限 {MAX_GRID_NODES:,}）。"
            "请缩小工作区域或提高网格分辨率。"
        )

    # 三维 list 填充
    grid: list = [
        [
            [
                GridNode(
                    x=x,
                    y=y,
                    z=z,
                    lat=min_lat + (y / grid_y) * lat_range,
                    lng=min_lng + (x / grid_x) * lng_range,
                    altitude=(
                        float(fixed_altitude)
                        if fixed_altitude is not None
                        else min_alt + (z / grid_z) * alt_range
                    ),
                )
                for z in range(grid_z)
            ]
            for y in range(grid_y)
        ]
        for x in range(grid_x)
    ]

    info = GridInfo(
        minLat=min_lat, maxLat=max_lat,
        minLng=min_lng, maxLng=max_lng,
        minAlt=min_alt, maxAlt=max_alt,
        gridX=grid_x, gridY=grid_y, gridZ=grid_z,
    )

    start_node = get_grid_node(grid, start["lat"], start["lng"], start["altitude"], info)
    end_node = get_grid_node(grid, end["lat"], end["lng"], end["altitude"], info)

    if start_node is None or end_node is None:
        return None
    if not start_node.walkable or not end_node.walkable:
        # 与 JS 行为一致：起点终点不可走时返回 None
        return None

    return GridData(grid=grid, start_node=start_node, end_node=end_node, info=info)


def get_grid_node(
    grid: list,
    lat: float,
    lng: float,
    alt: float,
    info: GridInfo,
) -> Optional[GridNode]:
    """真实坐标 → 网格节点（含 clamp 防越界），对应 JS getGridNode"""
    if not grid:
        return None

    x = int(((lng - info.minLng) / (info.maxLng - info.minLng)) * info.gridX)
    y = int(((lat - info.minLat) / (info.maxLat - info.minLat)) * info.gridY)
    altitude_range = info.maxAlt - info.minAlt
    z = (
        0
        if info.gridZ == 1 or abs(altitude_range) < 1e-12
        else int(((alt - info.minAlt) / altitude_range) * info.gridZ)
    )

    x = max(0, min(info.gridX - 1, x))
    y = max(0, min(info.gridY - 1, y))
    z = max(0, min(info.gridZ - 1, z))

    return grid[x][y][z]


# ============================================================
# 禁飞区
# ============================================================
def _zone_bounds(zone: dict) -> tuple[float, float, float, float, float, float]:
    """
    兼容前端两种字段命名风格：
    - snake_case: south_lat / north_lat / west_lng / east_lng / min_altitude / max_altitude
    - camelCase:  minLat / maxLat / minLng / maxLng / minAlt / maxAlt
    """
    min_lat = zone.get("minLat", zone.get("south_lat"))
    max_lat = zone.get("maxLat", zone.get("north_lat"))
    min_lng = zone.get("minLng", zone.get("west_lng"))
    max_lng = zone.get("maxLng", zone.get("east_lng"))
    min_alt = zone.get("minAlt", zone.get("min_altitude"))
    max_alt = zone.get("maxAlt", zone.get("max_altitude"))
    return min_lat, max_lat, min_lng, max_lng, min_alt, max_alt


def is_point_in_zone(lat: float, lng: float, altitude: float, zone: dict) -> bool:
    """判断点是否在禁飞区盒子内，对应 JS isPointInZone"""
    min_lat, max_lat, min_lng, max_lng, min_alt, max_alt = _zone_bounds(zone)
    return (
        min_lat <= lat <= max_lat
        and min_lng <= lng <= max_lng
        and min_alt <= altitude <= max_alt
    )


def is_point_in_any_zone(
    lat: float, lng: float, altitude: float, zones: list[dict]
) -> Optional[dict]:
    """点在任意禁飞区内则返回该禁飞区，否则返回 None"""
    for zone in zones:
        if is_point_in_zone(lat, lng, altitude, zone):
            return zone
    return None


def mark_no_fly_zones(grid: list, info: GridInfo, zones: list[dict]) -> int:
    """
    扫描整个网格，把落在禁飞区内的节点标 walkable=False。
    对应 JS markNoFlyZones。返回被标记的节点数。
    """
    if not zones:
        return 0

    blocked = 0
    for x in range(info.gridX):
        for y in range(info.gridY):
            for z in range(info.gridZ):
                node = grid[x][y][z]
                for zone in zones:
                    if is_point_in_zone(node.lat, node.lng, node.altitude, zone):
                        node.walkable = False
                        blocked += 1
                        break
    return blocked


class GridOccupancyCache:
    """One immutable occupancy mask, scoped to ONE planning request.

    Search nodes/g/parent are never cached or shared. Geometry must stay unchanged
    during the request; replacing the checker or grid geometry always misses.
    One entry bounds memory to at most MAX_GRID_NODES bytes.
    """
    def __init__(self):
        self._entry = None
        self.hits = 0
        self.misses = 0

    @staticmethod
    def key(info):
        return (info.minLat, info.maxLat, info.minLng, info.maxLng,
                info.minAlt, info.maxAlt, info.gridX, info.gridY, info.gridZ)

    def get(self, info, checker):
        if self._entry is not None:
            owner, key, mask, blocked = self._entry
            if owner is checker and key == self.key(info):
                self.hits += 1
                return mask, blocked
        self.misses += 1
        return None

    def put(self, info, checker, mask, blocked):
        self._entry = (checker, self.key(info), bytes(mask), blocked)

    def clear(self):
        self._entry = None


def mark_blocked_nodes(grid: list, info: GridInfo, collision_checker, *, cache=None) -> int:
    """用统一碰撞器同时标记人工禁飞区和模型建筑障碍。"""
    if collision_checker is None:
        return 0
    cached = cache.get(info, collision_checker) if cache is not None else None
    if cached is not None:
        mask, blocked = cached
        index = 0
        for column in grid:
            for row in column:
                for node in row:
                    node.walkable = not mask[index]
                    index += 1
        return blocked
    mask = bytearray(info.gridX * info.gridY * info.gridZ) if cache is not None else None
    blocked = 0
    index = 0
    for x in range(info.gridX):
        for y in range(info.gridY):
            for z in range(info.gridZ):
                node = grid[x][y][z]
                if collision_checker.is_position_blocked(node.lat, node.lng, node.altitude):
                    node.walkable = False
                    blocked += 1
                    if mask is not None:
                        mask[index] = 1
                index += 1
    if cache is not None:
        cache.put(info, collision_checker, mask, blocked)
    return blocked


# ============================================================
# 把不安全的航点挪到禁飞区外（路径平滑后用）
# ============================================================
def nearest_safe_position(point: dict, zone: dict, safety_margin: float = 10.0) -> dict:
    """
    把一个落在禁飞区内的点，沿"距离最近的那个面"推出去 + safety_margin。
    对应 JS getNearestSafePosition。
    """
    min_lat, max_lat, min_lng, max_lng, min_alt, max_alt = _zone_bounds(zone)
    lat, lng, altitude = point["lat"], point["lng"], point["altitude"]
    lat_scale, lng_scale = meters_per_degree(lat)

    dist_to_south = (lat - min_lat) * lat_scale
    dist_to_north = (max_lat - lat) * lat_scale
    dist_to_west = (lng - min_lng) * lng_scale
    dist_to_east = (max_lng - lng) * lng_scale
    dist_to_bottom = altitude - min_alt
    dist_to_top = max_alt - altitude

    distances = {
        "south": dist_to_south,
        "north": dist_to_north,
        "west": dist_to_west,
        "east": dist_to_east,
        "bottom": dist_to_bottom,
        "top": dist_to_top,
    }
    min_dir = min(distances, key=lambda k: distances[k])

    offset = safety_margin + 1
    new_point = {"lat": lat, "lng": lng, "altitude": altitude}
    if min_dir == "south":
        new_point["lat"] = min_lat - offset / lat_scale
    elif min_dir == "north":
        new_point["lat"] = max_lat + offset / lat_scale
    elif min_dir == "west":
        new_point["lng"] = min_lng - offset / lng_scale
    elif min_dir == "east":
        new_point["lng"] = max_lng + offset / lng_scale
    elif min_dir == "bottom":
        new_point["altitude"] = min_alt - offset
    elif min_dir == "top":
        new_point["altitude"] = max_alt + offset

    new_point["altitude"] = max(0.0, new_point["altitude"])
    return new_point


def adjust_point_outside_zones(
    point: dict, zones: list[dict], max_attempts: int = 5, safety_margin: float = 10.0
) -> dict:
    """
    反复把点挪到所有禁飞区外。若多次后仍卡住，抬高 100 米飞越。
    对应 JS adjustPoints。
    """
    current = dict(point)
    for _ in range(max_attempts):
        zone = is_point_in_any_zone(current["lat"], current["lng"], current["altitude"], zones)
        if zone is None:
            break
        current = nearest_safe_position(current, zone, safety_margin)

    if is_point_in_any_zone(current["lat"], current["lng"], current["altitude"], zones):
        current["altitude"] += 100

    return current
