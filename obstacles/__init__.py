"""实景三维模型障碍物服务。"""

from pathlib import Path
from threading import Lock

from .collision import CollisionChecker
from .surface_voxel import SurfaceVoxelDataset

_out2_dataset: SurfaceVoxelDataset | None = None
_out2_dataset_lock = Lock()
_reconstruction1_dataset: SurfaceVoxelDataset | None = None
_reconstruction1_dataset_lock = Lock()


def get_out2_dataset() -> SurfaceVoxelDataset:
    """学校 out2 原始三角表面的 0.5 米三维体素数据集。"""
    global _out2_dataset
    if _out2_dataset is None:
        with _out2_dataset_lock:
            if _out2_dataset is None:
                data_root = Path(__file__).resolve().parent.parent / "obstacle_data" / "out2"
                _out2_dataset = SurfaceVoxelDataset(data_root)
    return _out2_dataset


def get_reconstruction1_dataset() -> SurfaceVoxelDataset:
    """重建任务 1 实景模型原始三角表面的 0.5 米三维体素数据集。"""
    global _reconstruction1_dataset
    if _reconstruction1_dataset is None:
        with _reconstruction1_dataset_lock:
            if _reconstruction1_dataset is None:
                data_root = (
                    Path(__file__).resolve().parent.parent
                    / "obstacle_data"
                    / "reconstruction1"
                )
                _reconstruction1_dataset = SurfaceVoxelDataset(data_root)
    return _reconstruction1_dataset


__all__ = [
    "SurfaceVoxelDataset",
    "CollisionChecker",
    "get_out2_dataset",
    "get_reconstruction1_dataset",
]
