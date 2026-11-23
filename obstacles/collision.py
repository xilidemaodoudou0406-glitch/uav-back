"""人工禁飞区与实景模型共用的点/航段碰撞入口。"""

from __future__ import annotations

from .surface_voxel import SurfaceVoxelDataset


def _zone_bounds(zone: dict) -> tuple[float, float, float, float, float, float]:
    return (
        zone.get("south_lat", zone.get("minLat")),
        zone.get("north_lat", zone.get("maxLat")),
        zone.get("west_lng", zone.get("minLng")),
        zone.get("east_lng", zone.get("maxLng")),
        zone.get("min_altitude", zone.get("minAlt")),
        zone.get("max_altitude", zone.get("maxAlt")),
    )


def point_in_zone(lat: float, lng: float, altitude: float, zone: dict) -> bool:
    south, north, west, east, bottom, top = _zone_bounds(zone)
    return south <= lat <= north and west <= lng <= east and bottom <= altitude <= top


def segment_intersects_zone(start: dict, end: dict, zone: dict) -> bool:
    """三维 slab 检测，能阻止两个安全端点之间斜穿长方体。"""
    south, north, west, east, bottom, top = _zone_bounds(zone)
    starts = (start["lat"], start["lng"], start["altitude"])
    deltas = (
        end["lat"] - start["lat"],
        end["lng"] - start["lng"],
        end["altitude"] - start["altitude"],
    )
    intervals = ((south, north), (west, east), (bottom, top))
    enter, leave = 0.0, 1.0
    for origin, delta, (minimum, maximum) in zip(starts, deltas, intervals):
        if abs(delta) < 1e-15:
            if origin < minimum or origin > maximum:
                return False
            continue
        first = (minimum - origin) / delta
        second = (maximum - origin) / delta
        enter = max(enter, min(first, second))
        leave = min(leave, max(first, second))
        if enter > leave:
            return False
    return True


class CollisionChecker:
    def __init__(
        self,
        manual_zones: list[dict] | None,
        out2: SurfaceVoxelDataset | None = None,
        reconstruction1: SurfaceVoxelDataset | None = None,
    ):
        self.manual_zones = manual_zones or []
        self.out2 = out2 if out2 and out2.enabled else None
        self.reconstruction1 = (
            reconstruction1 if reconstruction1 and reconstruction1.enabled else None
        )

    @property
    def model_datasets(self) -> tuple[SurfaceVoxelDataset, ...]:
        return tuple(
            dataset
            for dataset in (self.out2, self.reconstruction1)
            if dataset is not None
        )

    def is_position_blocked(self, lat: float, lng: float, altitude: float) -> bool:
        if any(point_in_zone(lat, lng, altitude, zone) for zone in self.manual_zones):
            return True
        return any(dataset.is_position_blocked(lat, lng, altitude) for dataset in self.model_datasets)

    def is_segment_blocked(self, start: dict, end: dict) -> bool:
        if any(segment_intersects_zone(start, end, zone) for zone in self.manual_zones):
            return True
        return any(dataset.is_segment_blocked(start, end) for dataset in self.model_datasets)

    def path_is_safe(self, path: list[dict]) -> bool:
        if not path:
            return False
        if any(self.is_position_blocked(p["lat"], p["lng"], p["altitude"]) for p in path):
            return False
        return all(
            not self.is_segment_blocked(path[index - 1], path[index])
            for index in range(1, len(path))
        )
