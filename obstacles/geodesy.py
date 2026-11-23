"""WGS84 地理坐标与地心地固坐标（ECEF）转换。"""

from __future__ import annotations

import math

WGS84_A = 6_378_137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def geodetic_to_ecef(
    lat: float,
    lng: float,
    height: float = 0.0,
) -> tuple[float, float, float]:
    latitude = math.radians(lat)
    longitude = math.radians(lng)
    sin_lat = math.sin(latitude)
    cos_lat = math.cos(latitude)
    radius = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (radius + height) * cos_lat * math.cos(longitude),
        (radius + height) * cos_lat * math.sin(longitude),
        (radius * (1 - WGS84_E2) + height) * sin_lat,
    )


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    """迭代反算，精度足以用于体素查询和路径航点。"""
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)
    if horizontal < 1e-8:
        if abs(z) < 1e-8:
            raise ValueError("地心不能转换为有效地理坐标")
        return math.copysign(90.0, z), 0.0, abs(z) - WGS84_A * (1-WGS84_F)
    latitude = math.atan2(z, horizontal * (1 - WGS84_E2))
    height = 0.0
    for _ in range(8):
        sin_lat = math.sin(latitude)
        radius = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
        height = horizontal*math.cos(latitude)+z*sin_lat-WGS84_A*math.sqrt(1-WGS84_E2*sin_lat*sin_lat)
        latitude = math.atan2(
            z,
            horizontal * (1 - WGS84_E2 * radius / (radius + height)),
        )
    sin_lat = math.sin(latitude)
    height = horizontal*math.cos(latitude)+z*sin_lat-WGS84_A*math.sqrt(1-WGS84_E2*sin_lat*sin_lat)
    return math.degrees(latitude), math.degrees(longitude), height
