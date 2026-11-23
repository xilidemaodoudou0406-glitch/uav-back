"""只读取本项目生成的简单一维 NPY/NPZ，避免后端必须安装 NumPy。"""

from __future__ import annotations

import ast
import sys
import zipfile
from array import array
from pathlib import Path


def _read_npy(content: bytes, expected_descriptor: str) -> array:
    if content[:6] != b"\x93NUMPY" or content[6:8] != b"\x01\x00":
        raise ValueError("仅支持 NPY v1.0")
    header_length = int.from_bytes(content[8:10], "little")
    header = ast.literal_eval(content[10 : 10 + header_length].decode("ascii").strip())
    if header.get("descr") != expected_descriptor:
        raise ValueError(f"NPY 类型错误: {header.get('descr')} != {expected_descriptor}")
    if header.get("fortran_order"):
        raise ValueError("不支持 Fortran-order NPY")
    shape = header.get("shape")
    if not isinstance(shape, tuple) or len(shape) != 1:
        raise ValueError(f"NPY shape 必须是一维: {shape}")

    type_code = "f" if expected_descriptor == "<f4" else "I"
    values = array(type_code)
    values.frombytes(content[10 + header_length :])
    if sys.byteorder != "little":
        values.byteswap()
    if len(values) != shape[0]:
        raise ValueError("NPY 数据长度与 shape 不一致")
    return values


def read_sparse_grid_npz(file_path: str | Path) -> tuple[array, array, array]:
    with zipfile.ZipFile(file_path) as archive:
        rows = _read_npy(archive.read("rows.npy"), "<u4")
        columns = _read_npy(archive.read("columns.npy"), "<u4")
        heights = _read_npy(archive.read("safe_heights.npy"), "<f4")
    if not (len(rows) == len(columns) == len(heights)):
        raise ValueError("NPZ 稀疏数组长度不一致")
    return rows, columns, heights


def read_sparse_voxel_npz(file_path: str | Path) -> array:
    """读取按 level/row/column 排序的稀疏三维体素线性索引。"""
    with zipfile.ZipFile(file_path) as archive:
        return _read_npy(archive.read("occupied_indices.npy"), "<u4")
