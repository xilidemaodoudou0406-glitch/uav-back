"""
schemas.py — 请求/响应数据模型（Pydantic Schemas）
===================================================
这个文件定义了前端和后端之间传递数据的"格式"。

用 Pydantic 的好处：
1. 自动校验：数据不对就返回 422 错误，不用手写 if 判断
2. 自动文档：FastAPI 会根据 schema 生成 Swagger 文档（访问 /docs）
3. 类型安全：IDE 能提供自动补全和类型检查

分层关系（从外到内）：
  前端 HTTP JSON → schemas（校验/转换） → main.py 路由 → 算法 / 数据库
  前端 HTTP JSON ← schemas（序列化）   ← main.py 路由 ← 算法 / 数据库

为什么分 models.py 和 schemas.py？
- models.py 描述的是"数据库表结构"（和 SQL 对应）
- schemas.py 描述的是"API 接口格式"（和 HTTP 请求/响应对应）
- 两者分开是为了：接口格式可以独立于数据库结构变化
"""

from typing import Literal, Optional
from pydantic import BaseModel, Field, model_validator


# ============================================================
# Todo 相关（示例 CRUD，路径规划不依赖这些）
# ============================================================

class TodoCreate(BaseModel):
    """
    创建/更新 Todo 时的请求体格式。
    前端只需发送这两个字段，不需要传 id（id 由数据库自动生成）。
    """
    title: str                     # 待办标题
    done: bool = False             # 是否完成，不传默认为 False


class TodoOut(TodoCreate):
    """
    查询 Todo 时的响应体格式。
    继承了 TodoCreate 的所有字段，额外加上 id。
    前端收到时能拿到完整的 { id, title, done }。
    """
    id: int                        # 数据库主键

    class Config:
        # from_attributes = True 告诉 Pydantic：
        # "如果给我一个 ORM 对象（比如 models.Todo 实例），
        #  直接从它的属性（.id, .title, .done）读取数据，不用 .dict()"
        from_attributes = True


# ============================================================
# 身份认证
# ============================================================

class LoginRequest(BaseModel):
    username: str = Field(..., min_length=1, max_length=64)
    password: str = Field(..., min_length=1, max_length=128)


class UserOut(BaseModel):
    id: int
    username: str
    displayName: str


class LoginResponse(BaseModel):
    accessToken: str
    tokenType: str = "bearer"
    expiresIn: int
    user: UserOut


# ============================================================
# 路径规划相关（核心功能的所有请求/响应格式）
# ============================================================

class LatLng(BaseModel):
    """
    经纬度坐标点
    - ge=-90, le=90  → 纬度范围：-90° 到 90°
    - ge=-180, le=180 → 经度范围：-180° 到 180°
    Field(..., ...) 中的 ... 表示必填（没有默认值）
    """
    lat: float = Field(..., ge=-90, le=90)     # 纬度
    lng: float = Field(..., ge=-180, le=180)   # 经度
    altitude: Optional[float] = Field(default=None, ge=-1000, le=20000)
    # ↑ 航点椭球高；新前端传“最高表面 + 5m”，旧客户端可不传


class SurfaceHeightRequest(BaseModel):
    lat: float = Field(..., ge=-90, le=90)
    lng: float = Field(..., ge=-180, le=180)


class SurfaceHeightResponse(BaseModel):
    surfaceAltitude: float
    clearanceMeters: float
    flightAltitude: float
    source: str


class Bounds(BaseModel):
    """
    地图边界矩形框。
    前端在 Cesium 地图上框选区域后，把这个矩形四个角发过来。
    """
    minLat: float = Field(..., ge=-90, le=90)
    maxLat: float = Field(..., ge=-90, le=90)
    minLng: float = Field(..., ge=-180, le=180)
    maxLng: float = Field(..., ge=-180, le=180)

    @model_validator(mode="after")
    def _check_order(self):
        """
        自定义校验：确保 min < max。
        Pydantic 会在所有字段验证完之后自动调用这个方法。
        如果条件不满足，抛出 ValueError → 前端收到 422 错误。
        """
        if self.minLat >= self.maxLat:
            raise ValueError("bounds.minLat must be < bounds.maxLat")
        if self.minLng >= self.maxLng:
            raise ValueError("bounds.minLng must be < bounds.maxLng")
        return self


class NoFlyZone(BaseModel):
    """
    禁飞区 —— 无人机不能经过的矩形立体区域。

    字段名用 snake_case（如 south_lat 而非 southLat），
    是为了和前端 JavaScript 发送的 JSON 字段名保持一致。
    """
    name: Optional[str] = None                     # 禁飞区名称（可选，纯展示用）
    south_lat: float = Field(..., ge=-90, le=90)   # 南边界纬度
    north_lat: float = Field(..., ge=-90, le=90)   # 北边界纬度
    west_lng: float = Field(..., ge=-180, le=180)  # 西边界经度
    east_lng: float = Field(..., ge=-180, le=180)  # 东边界经度
    min_altitude: float = Field(..., ge=0)          # 禁飞区最低高度（米）
    max_altitude: float = Field(..., gt=0)          # 禁飞区最高高度（米），gt=0 表示必须 >0

    @model_validator(mode="after")
    def _check_order(self):
        """确保禁飞区的经纬度和高度范围有效"""
        if self.south_lat >= self.north_lat:
            raise ValueError("noFlyZone.south_lat must be < north_lat")
        if self.west_lng >= self.east_lng:
            raise ValueError("noFlyZone.west_lng must be < east_lng")
        if self.min_altitude >= self.max_altitude:
            raise ValueError("noFlyZone.min_altitude must be < max_altitude")
        return self


class PlanOptions(BaseModel):
    """
    路径规划的算法参数。
    所有字段都有默认值——前端不传就用默认值。
    """
    gridResolution: int = Field(10, ge=1, le=200)
    # ↑ 网格分辨率（米），值越小路径越精细但计算越慢
    #   默认 10 米。ge=1 最小值 1 米，le=200 最大值 200 米

    smoothPath: bool = True
    # ↑ 是否对路径做平滑处理，默认开启

    # 旧客户端不传方法时维持移动平均；新界面显式选择局部五次曲线。
    smoothingMethod: Optional[Literal["none", "moving_average", "local_quintic_bezier"]] = None
    transitionDistanceMeters: float = Field(12.0, ge=0.5, le=100, allow_inf_nan=False)
    cruiseSpeedMetersPerSecond: float = Field(15.0, ge=0.5, le=50, allow_inf_nan=False)
    maxLateralAcceleration: float = Field(2.0, ge=0.1, le=20, allow_inf_nan=False)
    maxTangentialAcceleration: float = Field(2.0, ge=0.1, le=20, allow_inf_nan=False)
    verticalSpeedMetersPerSecond: float = Field(3.0, ge=0.1, le=15, allow_inf_nan=False)

    maxIterations: int = Field(50000, ge=1000, le=1_000_000)
    # ↑ 算法最大迭代次数，防止在复杂地形上无限循环
    #   默认 50000 次，最小 1000，最大 100 万


# ---------------------------------------------------------------------------
# Literal 类型：限制值只能是这几个字符串之一
# 传 "astar" 和 "dijkstra" 以外的值 → Pydantic 自动拒绝
# ---------------------------------------------------------------------------
AlgoName = Literal["astar", "dijkstra"]
AltitudeMode = Literal["fixed", "free"]


class PlanRequest(BaseModel):
    """
    路径规划请求体 —— POST /api/path/plan 的完整输入格式。

    这是整个 API 的入口数据结构，包含所有必需信息：
    - 地图边界
    - 起止点
    - 巡航高度
    - 禁飞区列表
    - 要使用的算法
    - 算法参数
    """
    bounds: Bounds
    # ↑ 地图边界矩形框（用于限制搜索范围）

    start: LatLng
    # ↑ 起点（经纬度）

    end: LatLng
    # ↑ 终点（经纬度）

    viaPoints: list[LatLng] = Field(default_factory=list, max_length=20)
    # ↑ 有序途经点；数组顺序就是无人机必须依次经过的顺序

    altitude: float = Field(..., gt=0)
    # ↑ 巡航高度（米），必须大于 0

    altitudeMode: AltitudeMode = "fixed"
    # ↑ fixed：除起降段外全程严格保持 altitude；free：允许规划途中改变高度

    noFlyZones: list[NoFlyZone] = Field(default_factory=list, max_length=100)
    # ↑ 禁飞区列表，不传默认为空列表，最多 100 个

    useOut2Obstacles: bool = False
    # ↑ 是否启用学校 out2 实景模型提取出的独立障碍物

    useReconstruction1Obstacles: bool = False
    # ↑ 是否启用“重建任务 1”实景模型提取出的独立障碍物

    algorithms: list[AlgoName] = Field(..., min_length=1)
    # ↑ 要运行的算法列表，至少 1 个
    #   例：["astar"]            → 只跑 A*
    #   例：["astar", "dijkstra"] → 两个都跑，方便对比结果

    options: PlanOptions = Field(default_factory=PlanOptions)
    # ↑ 算法参数，不传就全部用默认值

    @model_validator(mode="after")
    def _check_points_in_bounds(self):
        """
        校验起点和终点是否在边界框内。
        如果起点或终点在框外，说明前端传的数据有问题，直接拒绝。
        """
        b = self.bounds
        ordered_points = [
            ("start", self.start),
            *((f"viaPoints[{index}]", point) for index, point in enumerate(self.viaPoints)),
            ("end", self.end),
        ]
        for label, pt in ordered_points:
            if not (b.minLat <= pt.lat <= b.maxLat):
                raise ValueError(f"{label}.lat must be within bounds")
            if not (b.minLng <= pt.lng <= b.maxLng):
                raise ValueError(f"{label}.lng must be within bounds")
        # 去重：如果前端传了 ["astar", "astar"]，只保留一次
        # dict.fromkeys() 保持顺序且去重
        self.algorithms = list(dict.fromkeys(self.algorithms))
        return self


class Waypoint(BaseModel):
    """
    单个航点 —— 路径上的一个坐标点。
    一条路径就是 Waypoint 的列表。
    """
    lat: float
    lng: float
    altitude: float


class TrajectorySample(Waypoint):
    timeSeconds: float = Field(..., ge=0, allow_inf_nan=False)
    speedMetersPerSecond: float = Field(..., ge=0, allow_inf_nan=False)
    phase: Literal["takeoff", "cruise", "landing"] = "cruise"


class TrajectoryStats(BaseModel):
    totalDistanceMeters: float = 0.0
    durationSeconds: float = 0.0
    maxSpeedMetersPerSecond: float = 0.0
    maxCurvature: float = 0.0
    sampleCount: int = 0
    stopCount: int = 0
    maxTangentialAcceleration: float = 0.0
    maxLateralAcceleration: float = 0.0
    validationMode: str = "sampled_polyline"


class SmoothingReport(BaseModel):
    method: str = "none"
    status: str = "unchanged"
    smoothedCorners: int = 0
    fallbackCorners: int = 0
    removedPoints: int = 0
    geometryToleranceMeters: float = 0.03
    notes: list[str] = Field(default_factory=list)


class AlgoResult(BaseModel):
    """
    单个算法的运行结果。
    无论成功还是失败，都用这个结构返回。

    成功示例：
      { "success": true, "path": [...], "cost": 1234.5, "elapsedMs": 456, ... }
    失败示例：
      { "success": false, "path": [], "cost": 0, "message": "no path found" }
      ↑ 注意：失败时 path 返回空数组而不是 null，前端更安全
    """
    success: bool
    path: list[Waypoint] = Field(default_factory=list)
    # ↑ 航点列表，失败时为空列表（不用 null/None）

    rawPath: list[Waypoint] = Field(default_factory=list)
    trajectorySamples: list[TrajectorySample] = Field(default_factory=list)
    trajectoryStats: Optional[TrajectoryStats] = None
    smoothingReport: Optional[SmoothingReport] = None

    cost: float = 0.0
    # ↑ 路径总距离（米）

    elapsedMs: float = 0.0
    # ↑ 算法耗时（毫秒）

    iterations: int = 0
    # ↑ 算法实际迭代次数

    message: Optional[str] = None
    # ↑ 失败时的错误信息（成功时为 null）


class ObstacleStatus(BaseModel):
    """本次规划使用的实景模型障碍物数据状态。"""
    enabled: bool
    tileCount: int = 0
    loadedTileCount: int = 0
    resolutionMeters: Optional[float] = None
    error: Optional[str] = None
    active: bool = True


class PlanResponse(BaseModel):
    """
    路径规划响应体 —— POST /api/path/plan 的完整输出格式。

    results 是一个字典，key 是算法名，value 是该算法的结果。
    前端可以渲染：
      results = { "astar": AlgoResult, "dijkstra": AlgoResult }
    """
    results: dict[AlgoName, AlgoResult]
    out2Obstacles: ObstacleStatus
    reconstruction1Obstacles: ObstacleStatus
