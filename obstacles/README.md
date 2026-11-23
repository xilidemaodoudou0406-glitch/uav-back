# 实景模型障碍物模块

该目录负责将实景三维模型的离线体素数据接入统一碰撞检测。

- `surface_voxel.py`：读取稀疏三维表面体素，执行点、线段和竖直表面高度查询。
- `collision.py`：统一检查人工禁飞区、学校 out2 模型与“重建任务 1”模型。
- `geodesy.py`：提供 WGS84、ECEF 与模型局部坐标转换所需的基础函数。
- `npz_reader.py`：使用标准库读取离线生成的 NPZ 数据。

现有模型数据分别位于 `obstacle_data/out2` 和
`obstacle_data/reconstruction1`。数据在进程内共享，并在首次查询时惰性加载。

前端通过 `/api/obstacles/surface-height` 查询点击位置的最高模型表面，路径规划
通过 `CollisionChecker` 对搜索节点、航段及平滑后的完整路径进行复检。
