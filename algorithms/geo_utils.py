"""
地理坐标工具函数

对应前端 src/stores/flight.js 中的：
- calculateBearing
- planTakeoffPath
- planLandingPath
- forceConstantAltitude
- createDirectPath
以及前端 AStar/Dijkstra 中的 calculate3DDistance（Haversine）。

水平长度使用 WGS84 曲率半径随纬度换算；执行预检另外使用 ECEF/ENU。
"""

from __future__ import annotations

import math
from typing import Iterable

# ---- 常量（与前端 JS 保持一致） ----
EARTH_RADIUS_M = 6371000.0


def meters_per_degree(latitude: float) -> tuple[float, float]:
    """Local meridian/parallel scales for grid sizing, not a map datum conversion."""
    phi = math.radians(latitude)
    denominator = 1-0.0066943799901413165*math.sin(phi)**2
    meridian = 6378137*(1-0.0066943799901413165)/denominator**1.5
    prime = 6378137/math.sqrt(denominator)
    return meridian*math.pi/180, max(1e-6, prime*math.cos(phi)*math.pi/180)


# ---- 类型别名 ----
Waypoint = dict  # {"lat": float, "lng": float, "altitude": float}


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算两点之间的方位角（弧度），对应 flight.js calculateBearing"""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = (
        math.cos(lat1_rad) * math.sin(lat2_rad)
        - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    )
    return math.atan2(y, x)


def plan_takeoff_path(start: Waypoint, cruise_altitude: float, next_point: Waypoint) -> list[Waypoint]:
    """
    从地面（altitude=0）以 90° 垂直爬升到巡航高度。
    对应 flight.js planTakeoffPath。

    JS 里 takeoffAngle=90，tan(90°) 在 JS 浮点中是个很大的有限数，
    horizontalDistance 几乎为 0，效果等同纯垂直起飞。这里直接置零。
    """
    steps = 10
    takeoff_angle_deg = 90.0
    if takeoff_angle_deg >= 90.0:
        horizontal_distance = 0.0
    else:
        horizontal_distance = cruise_altitude / math.tan(math.radians(takeoff_angle_deg))

    bearing = calculate_bearing(start["lat"], start["lng"], next_point["lat"], next_point["lng"])
    lat_scale, lng_scale = meters_per_degree(start["lat"])
    lat_offset = (horizontal_distance / lat_scale) * math.cos(bearing)
    lng_offset = (horizontal_distance / lng_scale) * math.sin(bearing)

    start_altitude = float(start.get("altitude", 0.0))
    waypoints: list[Waypoint] = []
    for i in range(steps + 1):
        ratio = i / steps
        waypoints.append(
            {
                "lat": start["lat"] + lat_offset * ratio,
                "lng": start["lng"] + lng_offset * ratio,
                "altitude": start_altitude + (cruise_altitude - start_altitude) * ratio,
            }
        )
    return waypoints


def plan_landing_path(cruise_end: Waypoint, final_end: Waypoint, cruise_altitude: float) -> list[Waypoint]:
    """
    从巡航高度垂直下降到地面，对应 flight.js planLandingPath。
    landingAngle=90 → 纯垂直降落。
    """
    steps = 10
    landing_angle_deg = 90.0
    if landing_angle_deg >= 90.0:
        horizontal_distance = 0.0
    else:
        horizontal_distance = cruise_altitude / math.tan(math.radians(landing_angle_deg))

    bearing = calculate_bearing(
        cruise_end["lat"], cruise_end["lng"], final_end["lat"], final_end["lng"]
    )
    lat_scale, lng_scale = meters_per_degree(final_end["lat"])
    lat_offset = -(horizontal_distance / lat_scale) * math.cos(bearing)
    lng_offset = -(horizontal_distance / lng_scale) * math.sin(bearing)

    landing_start_lat = final_end["lat"] + lat_offset
    landing_start_lng = final_end["lng"] + lng_offset

    waypoints: list[Waypoint] = []
    for i in range(steps + 1):
        ratio = i / steps
        waypoints.append(
            {
                "lat": landing_start_lat + (final_end["lat"] - landing_start_lat) * ratio,
                "lng": landing_start_lng + (final_end["lng"] - landing_start_lng) * ratio,
                "altitude": cruise_altitude + (float(final_end.get("altitude", 0.0)) - cruise_altitude) * ratio,
            }
        )
    return waypoints


def force_constant_altitude(path: Iterable[Waypoint], target_altitude: float) -> list[Waypoint]:
    """把巡航段每个点的高度强制改为 target_altitude，对应 flight.js forceConstantAltitude"""
    return [{"lat": p["lat"], "lng": p["lng"], "altitude": target_altitude} for p in path]


def create_direct_path(start: Waypoint, end: Waypoint, num_points: int = 10) -> list[Waypoint]:
    """无禁飞区时的直线插值路径，对应 flight.js createDirectPath"""
    points: list[Waypoint] = []
    for i in range(num_points + 1):
        ratio = i / num_points
        points.append(
            {
                "lat": start["lat"] + (end["lat"] - start["lat"]) * ratio,
                "lng": start["lng"] + (end["lng"] - start["lng"]) * ratio,
                "altitude": start["altitude"] + (end["altitude"] - start["altitude"]) * ratio,
            }
        )
    return points


def haversine_3d_distance(p1: Waypoint, p2: Waypoint) -> float:
    """两个航点之间的 3D 距离（水平 Haversine + 垂直高差）"""
    d_lat = math.radians(p2["lat"] - p1["lat"])
    d_lng = math.radians(p2["lng"] - p1["lng"])

    a = (
        math.sin(d_lat / 2) ** 2
        + math.cos(math.radians(p1["lat"]))
        * math.cos(math.radians(p2["lat"]))
        * math.sin(d_lng / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    horizontal = EARTH_RADIUS_M * c
    vertical = abs(p2["altitude"] - p1["altitude"])
    return math.sqrt(horizontal * horizontal + vertical * vertical)


def calculate_statistics(waypoints: list[Waypoint], cruise_speed: float = 15.0) -> dict:
    """
    计算路径统计信息（总距离、总时间、最大/最小高度等）。
    对应 AStar3D.calculateStatistics。
    """
    total_distance = 0.0
    total_time = 0.0
    max_altitude = 0.0
    min_altitude = math.inf

    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        seg = haversine_3d_distance(p1, p2)
        total_distance += seg
        total_time += seg / cruise_speed
        max_altitude = max(max_altitude, p1["altitude"])
        min_altitude = min(min_altitude, p1["altitude"])

    if min_altitude is math.inf:
        min_altitude = 0.0

    avg_speed = round((total_distance / total_time) * 10) / 10 if total_time > 0 else 0.0

    return {
        "totalWaypoints": len(waypoints),
        "totalDistance": round(total_distance),
        "totalTime": round(total_time),
        "totalTimeMinutes": round((total_time / 60) * 10) / 10,
        "maxAltitude": round(max_altitude),
        "minAltitude": round(min_altitude),
        "averageSpeed": avg_speed,
    }
