"""由原始实景三角网格保守生成的稀疏三维表面体素。"""

from __future__ import annotations

import bisect
import json
import math
import threading
from array import array
from dataclasses import dataclass, field
from pathlib import Path

from .geodesy import ecef_to_geodetic, geodetic_to_ecef
from .npz_reader import read_sparse_voxel_npz


@dataclass
class SurfaceVoxelTile:
    metadata: dict
    data_root: Path
    occupied: array | None = field(default=None, init=False, repr=False)
    _load_lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    def __post_init__(self) -> None:
        lng, lat, height = self.metadata["centerWgs84"]
        self.center_lat = float(lat)
        self.center_lng = float(lng)
        self.center_height = float(height)
        self.center_ecef = geodetic_to_ecef(self.center_lat, self.center_lng, self.center_height)
        lat_radians = math.radians(self.center_lat)
        lng_radians = math.radians(self.center_lng)
        self.sin_lat, self.cos_lat = math.sin(lat_radians), math.cos(lat_radians)
        self.sin_lng, self.cos_lng = math.sin(lng_radians), math.cos(lng_radians)
        self.grid = self.metadata["grid"]
        west, south, east, north = self.metadata["referenceBboxWgs84"]
        padding = 0.00002
        self.bbox = (west - padding, south - padding, east + padding, north + padding)

    @property
    def loaded(self) -> bool:
        return self.occupied is not None

    def ensure_loaded(self) -> None:
        if self.loaded:
            return
        with self._load_lock:
            if not self.loaded:
                self.occupied = read_sparse_voxel_npz(self.data_root / self.metadata["file"])

    def contains_wgs84(self, lat: float, lng: float) -> bool:
        west, south, east, north = self.bbox
        return west <= lng <= east and south <= lat <= north

    def wgs84_to_enu(self, lat: float, lng: float, height: float = 0.0) -> tuple[float, float, float]:
        x, y, z = geodetic_to_ecef(lat, lng, height)
        dx, dy, dz = x - self.center_ecef[0], y - self.center_ecef[1], z - self.center_ecef[2]
        return (
            -self.sin_lng * dx + self.cos_lng * dy,
            -self.sin_lat * self.cos_lng * dx - self.sin_lat * self.sin_lng * dy + self.cos_lat * dz,
            self.cos_lat * self.cos_lng * dx + self.cos_lat * self.sin_lng * dy + self.sin_lat * dz,
        )

    def enu_to_wgs84(self, east: float, north: float, up: float = 0.0) -> tuple[float, float, float]:
        dx = -self.sin_lng * east - self.sin_lat * self.cos_lng * north + self.cos_lat * self.cos_lng * up
        dy = self.cos_lng * east - self.sin_lat * self.sin_lng * north + self.cos_lat * self.sin_lng * up
        dz = self.cos_lat * north + self.sin_lat * up
        return ecef_to_geodetic(self.center_ecef[0] + dx, self.center_ecef[1] + dy, self.center_ecef[2] + dz)

    def _cell_for_enu(self, east: float, north: float, up: float) -> tuple[int, int, int] | None:
        resolution = self.grid["resolutionMeters"]
        column = math.floor((east - self.grid["originEastMeters"]) / resolution)
        row = math.floor((north - self.grid["originNorthMeters"]) / resolution)
        level = math.floor((up - self.grid["originUpMeters"]) / resolution)
        if not (0 <= column < self.grid["columns"] and 0 <= row < self.grid["rows"] and 0 <= level < self.grid["levels"]):
            return None
        return column, row, level

    def _flat(self, column: int, row: int, level: int) -> int:
        return (level * self.grid["rows"] + row) * self.grid["columns"] + column

    def _is_occupied_cell(self, column: int, row: int, level: int) -> bool:
        if not (0 <= column < self.grid["columns"] and 0 <= row < self.grid["rows"] and 0 <= level < self.grid["levels"]):
            return False
        self.ensure_loaded()
        assert self.occupied is not None
        flat = self._flat(column, row, level)
        index = bisect.bisect_left(self.occupied, flat)
        return index < len(self.occupied) and self.occupied[index] == flat

    def is_position_blocked(self, lat: float, lng: float, altitude: float) -> bool:
        if not self.contains_wgs84(lat, lng):
            return False
        cell = self._cell_for_enu(*self.wgs84_to_enu(lat, lng, altitude))
        return bool(cell and self._is_occupied_cell(*cell))

    def surface_height_at_wgs84(self, lat: float, lng: float) -> float | None:
        """返回该经纬度竖直列中最高表面体素的椭球高。"""
        if not self.contains_wgs84(lat, lng):
            return None
        east, north, _ = self.wgs84_to_enu(lat, lng, 0.0)
        resolution = self.grid["resolutionMeters"]
        column = math.floor((east - self.grid["originEastMeters"]) / resolution)
        row = math.floor((north - self.grid["originNorthMeters"]) / resolution)
        if not (0 <= column < self.grid["columns"] and 0 <= row < self.grid["rows"]):
            return None

        for level in range(self.grid["levels"] - 1, -1, -1):
            if self._is_occupied_cell(column, row, level):
                cell_east = self.grid["originEastMeters"] + (column + 0.5) * resolution
                cell_north = self.grid["originNorthMeters"] + (row + 0.5) * resolution
                cell_up = self.grid["originUpMeters"] + (level + 0.5) * resolution
                return float(self.enu_to_wgs84(cell_east, cell_north, cell_up)[2])
        return None

    def is_segment_blocked(self, start: dict, end: dict) -> bool:
        """使用 3D DDA 精确遍历航段穿过的体素，不按固定距离漏采样。"""
        start_enu = self.wgs84_to_enu(start["lat"], start["lng"], start["altitude"])
        end_enu = self.wgs84_to_enu(end["lat"], end["lng"], end["altitude"])
        resolution = self.grid["resolutionMeters"]
        origins = (self.grid["originEastMeters"], self.grid["originNorthMeters"], self.grid["originUpMeters"])
        sizes = (self.grid["columns"], self.grid["rows"], self.grid["levels"])
        p0 = tuple((start_enu[i] - origins[i]) / resolution for i in range(3))
        p1 = tuple((end_enu[i] - origins[i]) / resolution for i in range(3))
        delta = tuple(p1[i] - p0[i] for i in range(3))

        enter, leave = 0.0, 1.0
        for axis in range(3):
            if abs(delta[axis]) < 1e-15:
                if p0[axis] < 0 or p0[axis] > sizes[axis]:
                    return False
                continue
            first = (0 - p0[axis]) / delta[axis]
            second = (sizes[axis] - p0[axis]) / delta[axis]
            enter, leave = max(enter, min(first, second)), min(leave, max(first, second))
            if enter > leave:
                return False

        point = [p0[i] + delta[i] * enter for i in range(3)]
        cells = [min(sizes[i] - 1, max(0, math.floor(point[i]))) for i in range(3)]
        steps, next_times, increments = [], [], []
        for axis in range(3):
            if delta[axis] > 0:
                steps.append(1)
                next_times.append((cells[axis] + 1 - p0[axis]) / delta[axis])
                increments.append(1 / delta[axis])
            elif delta[axis] < 0:
                steps.append(-1)
                next_times.append((cells[axis] - p0[axis]) / delta[axis])
                increments.append(-1 / delta[axis])
            else:
                steps.append(0)
                next_times.append(math.inf)
                increments.append(math.inf)

        while all(0 <= cells[i] < sizes[i] for i in range(3)):
            if self._is_occupied_cell(cells[0], cells[1], cells[2]):
                return True
            next_time = min(next_times)
            if next_time > leave + 1e-12:
                break
            for axis in range(3):
                if next_times[axis] <= next_time + 1e-12:
                    cells[axis] += steps[axis]
                    next_times[axis] += increments[axis]
        return False


class SurfaceVoxelDataset:
    bucket_degrees = 0.02

    def __init__(self, data_root: str | Path):
        self.data_root = Path(data_root)
        self.manifest_path = self.data_root / "tiles.json"
        self.error: str | None = None
        self.tiles: list[SurfaceVoxelTile] = []
        self._spatial_index: dict[tuple[int, int], list[SurfaceVoxelTile]] = {}
        try:
            manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
            if manifest.get("schemaVersion") != 2:
                raise ValueError("精确表面体素清单版本必须为 2")
            self.tiles = [SurfaceVoxelTile(item, self.data_root) for item in manifest.get("tiles", [])]
            if not self.tiles:
                raise ValueError("精确表面体素清单没有分块")
            self._build_spatial_index()
            self.minimum_resolution = min(tile.grid["resolutionMeters"] for tile in self.tiles)
            # Cached occupancy and geometry must describe the same files as execution validation.
            self.source_stamp = self.current_source_stamp()
        except Exception as exc:
            self.error = str(exc)

    @property
    def enabled(self) -> bool:
        return self.error is None and bool(self.tiles)

    def current_source_stamp(self):
        files = [self.manifest_path] + [self.data_root/t.metadata["file"] for t in self.tiles]
        return tuple((p.name,p.stat().st_size,p.stat().st_mtime_ns) for p in files)

    def _bucket_key(self, lat: float, lng: float) -> tuple[int, int]:
        return math.floor(lat / self.bucket_degrees), math.floor(lng / self.bucket_degrees)

    def _build_spatial_index(self) -> None:
        for tile in self.tiles:
            west, south, east, north = tile.bbox
            min_key, max_key = self._bucket_key(south, west), self._bucket_key(north, east)
            for lat_key in range(min_key[0], max_key[0] + 1):
                for lng_key in range(min_key[1], max_key[1] + 1):
                    self._spatial_index.setdefault((lat_key, lng_key), []).append(tile)

    def candidate_tiles(self, lat: float, lng: float) -> list[SurfaceVoxelTile]:
        return [tile for tile in self._spatial_index.get(self._bucket_key(lat, lng), []) if tile.contains_wgs84(lat, lng)]

    def is_position_blocked(self, lat: float, lng: float, altitude: float) -> bool:
        return any(tile.is_position_blocked(lat, lng, altitude) for tile in self.candidate_tiles(lat, lng))

    def get_surface_height(self, lat: float, lng: float) -> float | None:
        heights = (
            tile.surface_height_at_wgs84(lat, lng)
            for tile in self.candidate_tiles(lat, lng)
        )
        return max((height for height in heights if height is not None), default=None)

    def is_segment_blocked(self, start: dict, end: dict) -> bool:
        candidates = {id(tile): tile for point in (start, end) for tile in self.candidate_tiles(point["lat"], point["lng"])}
        if not candidates:
            # 长航段的两个端点可能都在数据集外，但中间穿过模型；单数据集只有一个瓦片，
            # 此处也检查航段经纬度包围盒与瓦片包围盒重叠。
            for tile in self.tiles:
                west, south, east, north = tile.bbox
                if max(start["lng"], end["lng"]) >= west and min(start["lng"], end["lng"]) <= east and max(start["lat"], end["lat"]) >= south and min(start["lat"], end["lat"]) <= north:
                    candidates[id(tile)] = tile
        return any(tile.is_segment_blocked(start, end) for tile in candidates.values())

    def status(self) -> dict:
        return {
            "enabled": self.enabled,
            "tileCount": len(self.tiles),
            "loadedTileCount": sum(tile.loaded for tile in self.tiles),
            "error": self.error,
            "resolutionMeters": self.minimum_resolution if self.enabled else None,
        }
