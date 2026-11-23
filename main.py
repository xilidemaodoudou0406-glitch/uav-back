"""
main.py — FastAPI 应用主入口
============================
这是整个后端的"大脑"。它负责：
1. 启动 Web 服务器
2. 接收前端发来的 HTTP 请求
3. 调用算法进行路径规划
4. 将结果返回给前端

架构总览（数据流向）：
  前端 Vue 应用
      │
      ▼  HTTP POST /api/path/plan (JSON)
  main.py（本文件）
      │
      ├─ schemas.py  → 校验请求数据格式
      ├─ algorithms/ → 跑 A* / Dijkstra 路径规划
      └─ schemas.py  → 组装响应数据返回

启动方式：在终端执行
  cd my-python-backend
  uvicorn main:app --reload --host 0.0.0.0 --port 8000

启动后访问 http://localhost:8000/docs 可以看到自动生成的 API 文档。
"""

# ---------------------------------------------------------------------------
# 第一部分：import —— 引入依赖
# ---------------------------------------------------------------------------

# FastAPI 核心
from fastapi import FastAPI, Depends, HTTPException
# FastAPI       → Web 框架（类似前端的 Vue，但这是后端的）
# Depends       → 依赖注入（函数 A 的参数由函数 B 自动提供，后面有例子）
# HTTPException → 主动抛出 HTTP 错误（如 404 Not Found）

from fastapi.concurrency import run_in_threadpool
# run_in_threadpool → 把同步函数放到线程池里运行，避免阻塞异步事件循环
#                      路径规划算法是同步的（会长时间占用 CPU），
#                      用这个包裹后就不会卡住其他请求

from fastapi.middleware.cors import CORSMiddleware
# CORSMiddleware → 处理跨域请求。
#                  前端跑在 localhost:5173，后端跑在 localhost:8000，
#                  浏览器默认禁止这种跨域请求，加了这个中间件才能互通

from sqlalchemy.orm import Session
# Session → 类型提示用，告诉函数参数是一个数据库会话

import time
import asyncio
from contextlib import asynccontextmanager, suppress

import models, schemas
# models   → 数据库表结构（Todo 等）
# schemas  → 请求/响应格式定义

from database import engine, get_db
# engine  → 数据库引擎，用于建表
# get_db  → 依赖注入函数，为每个请求获取数据库会话

# 算法模块（algorithms/ 目录）
from algorithms import (
    AStar3DPathPlanner,      # A* 三维路径规划器
    Dijkstra3DPathPlanner,   # Dijkstra 三维路径规划器
    force_constant_altitude, # 固定巡航段高度
    plan_landing_path,       # 生成降落段路径
    plan_takeoff_path,       # 生成起飞段路径
)
from obstacles import (
    CollisionChecker,
    get_out2_dataset,
    get_reconstruction1_dataset,
)
from auth import authenticate_user, create_access_token, ensure_default_admin, get_current_user
from algorithms.trajectory import postprocess_leg, build_timed_trajectory
from algorithms.grid_node import GridOccupancyCache
from routers.drones import router as drones_router
from services.drone_registry import DroneRegistry
from routers.missions import router as missions_router
from services.mission_service import MissionService
from routers.navigation import router as navigation_router


# ---------------------------------------------------------------------------
# 第二部分：应用初始化
# ---------------------------------------------------------------------------

# 启动时自动建表（简单项目够用，大项目应该用 Alembic 做数据库迁移）
# Base.metadata.create_all() 会扫描所有继承 Base 的 ORM 类，
# 如果表还不存在就自动创建。已存在的表不会被修改。
models.Base.metadata.create_all(bind=engine)
models.ensure_mission_query_indexes(engine)
ensure_default_admin()

# 创建 FastAPI 应用实例
# title 参数会显示在自动生成的 API 文档标题栏
@asynccontextmanager
async def lifespan(app):
    app.state.drone_registry = DroneRegistry()
    app.state.missions = MissionService(app.state.drone_registry)
    app.state.drone_registry.on_lost = app.state.missions.lost
    app.state.drone_registry.on_telemetry = app.state.missions.record
    app.state.drone_registry.on_tick = app.state.missions.tick
    watchdog = asyncio.create_task(app.state.drone_registry.watchdog())
    try:
        yield
    finally:
        watchdog.cancel()
        with suppress(asyncio.CancelledError):
            await watchdog


app = FastAPI(title="UAV Backend", lifespan=lifespan)
app.include_router(drones_router)
app.include_router(missions_router)
app.include_router(navigation_router)

# 添加 CORS 跨域中间件
# allow_origins=["*"] → 允许任何来源的前端访问（开发阶段方便）
#   生产环境应该改为具体的域名，如：allow_origins=["https://你的网站.com"]
# allow_methods=["*"] → 允许所有 HTTP 方法（GET、POST、DELETE 等）
# allow_headers=["*"] → 允许所有请求头
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ============================================================
# 身份认证
# ============================================================

def _user_response(user: models.User) -> schemas.UserOut:
    return schemas.UserOut(
        id=user.id,
        username=user.username,
        displayName=user.display_name,
    )


@app.post("/api/auth/login", response_model=schemas.LoginResponse)
def login(req: schemas.LoginRequest, db: Session = Depends(get_db)):
    user = authenticate_user(db, req.username.strip(), req.password)
    if not user:
        raise HTTPException(status_code=401, detail="用户名或密码错误")
    token, expires_in = create_access_token(user)
    return schemas.LoginResponse(
        accessToken=token,
        expiresIn=expires_in,
        user=_user_response(user),
    )


@app.get("/api/auth/me", response_model=schemas.UserOut)
def current_user(user: models.User = Depends(get_current_user)):
    return _user_response(user)


# ============================================================
# 第三部分：Todo CRUD 路由（示例，路径规划不依赖这些）
# ============================================================
# 这些是后端自带的一个"待办事项"迷你功能，用来演示最基础的增删改查。
# 你可以通过这些接口理解 FastAPI 路由的基本写法。
# ============================================================

@app.get("/todos", response_model=list[schemas.TodoOut])
def list_todos(db: Session = Depends(get_db)):
    """
    查询所有 Todo。
    GET /todos → 返回 Todo 列表（JSON 数组）

    参数 db 是怎么来的？
      Depends(get_db) 告诉 FastAPI："每次请求进来，先调用 get_db() 生成一个
      数据库会话，作为 db 参数传给这个函数。请求结束后自动关闭会话。"
      这就是 FastAPI 的"依赖注入"机制。
    """
    return db.query(models.Todo).all()
    # db.query(models.Todo) → SELECT * FROM todos
    # .all() → 获取所有行


@app.post("/todos", response_model=schemas.TodoOut)
def create_todo(todo: schemas.TodoCreate, db: Session = Depends(get_db)):
    """
    创建新 Todo。
    POST /todos + JSON body {"title": "...", "done": false}

    参数 todo 的类型是 schemas.TodoCreate（Pydantic 模型）。
    FastAPI 会自动：
    1. 读取 HTTP 请求的 JSON body
    2. 按 TodoCreate 的定义校验字段
    3. 如果格式不对，自动返回 422 错误（不用手写校验代码）
    """
    # **todo.model_dump() 把 Pydantic 对象转成普通字典，
    # 然后传给 ORM 构造器，创建一行新记录
    db_todo = models.Todo(**todo.model_dump())
    db.add(db_todo)       # 相当于 INSERT INTO todos ... （暂存在内存）
    db.commit()           # 真正写入数据库
    db.refresh(db_todo)   # 刷新对象，获取数据库自动生成的 id 等字段
    return db_todo        # FastAPI 自动转成 JSON 返回给前端


@app.delete("/todos/{todo_id}")
def delete_todo(todo_id: int, db: Session = Depends(get_db)):
    """
    删除指定 Todo。
    DELETE /todos/3 → 删除 id=3 的 Todo

    {todo_id} 是路径参数。URL 里的 3 会被自动提取为 todo_id=3。
    """
    # 先查找是否存在
    todo = db.query(models.Todo).filter(models.Todo.id == todo_id).first()
    # filter(...)  → WHERE id = todo_id
    # .first()     → 取第一条匹配结果，没有则返回 None

    if not todo:
        # HTTPException 会中断函数执行，返回 HTTP 错误给前端
        raise HTTPException(status_code=404, detail="Todo not found")

    db.delete(todo)   # 相当于 DELETE FROM todos WHERE id = ...
    db.commit()       # 确认删除
    return {"ok": True}


@app.patch("/todos/{todo_id}", response_model=schemas.TodoOut)
def update_todo(todo_id: int, todo: schemas.TodoCreate, db: Session = Depends(get_db)):
    """
    更新指定 Todo。
    PATCH /todos/3 + JSON body {"title": "新标题", "done": true}

    用 PATCH 而非 PUT，表示"部分更新"（只更新传了的字段）。
    """
    db_todo = db.query(models.Todo).filter(models.Todo.id == todo_id).first()
    if not db_todo:
        raise HTTPException(status_code=404, detail="Todo not found")

    # 逐个字段更新
    db_todo.title = todo.title
    db_todo.done = todo.done
    db.commit()           # 确认修改
    db.refresh(db_todo)   # 刷新以获取最新状态
    return db_todo


# ============================================================
# 第四部分：路径规划（核心功能）
# ============================================================
# 以下是整个后端最重要的代码。
# 流程：前端发来区域+起止点 → 校验 → 调用算法 → 返回路径
# ============================================================

def _run_single_algorithm(
    algo_name: str,
    req: schemas.PlanRequest,
    zones: list[dict],
) -> schemas.AlgoResult:
    """
    运行单个路径规划算法，并拼接起飞/巡航/降落三段。

    这个函数的调用被 run_in_threadpool 包裹，运行在线程池中，
    不会阻塞主事件循环上的其他请求。

    参数说明：
    - algo_name      → "astar" 或 "dijkstra"
    - req            → 完整的请求对象（用于读取 bounds、altitude、options）
    - zones          → 禁飞区列表（已转为算法需要的 dict 格式）
    返回结构：
      AlgoResult {
        success: bool        → 是否成功找到路径
        path: list[Waypoint] → 完整航点（起飞+巡航+降落），失败时为空列表
        cost: float          → 路径总距离（米）
        elapsedMs: float     → 算法耗时（毫秒）
        iterations: int      → 算法迭代次数
        message: str|null    → 失败原因（成功时为 null）
      }
    """
    started = time.perf_counter()
    out2_dataset = get_out2_dataset() if req.useOut2Obstacles else None
    reconstruction1_dataset = (
        get_reconstruction1_dataset() if req.useReconstruction1Obstacles else None
    )
    collision_checker = CollisionChecker(
        zones,
        out2_dataset,
        reconstruction1_dataset,
    )
    # Only immutable occupied/free flags are reused between ordered legs.
    # This cache dies with this algorithm request; no cross-user mutable state.
    occupancy_cache = GridOccupancyCache()

    # ----- 步骤 1：根据算法名选择规划器 -----
    if algo_name == "astar":
        # A* 使用请求中的网格分辨率，不做额外限制
        planner = AStar3DPathPlanner(
            grid_resolution=req.options.gridResolution,
            max_iterations=req.options.maxIterations,
            collision_checker=collision_checker,
            occupancy_cache=occupancy_cache,
        )
    else:
        # Dijkstra 默认网格粗一些（≥50），迭代上限大一些（≥20万）
        # 因为 Dijkstra 比 A* 吃更多资源，需要放宽参数来控制运行时间
        planner = Dijkstra3DPathPlanner(
            grid_resolution=max(req.options.gridResolution, 50),
            max_iterations=max(req.options.maxIterations, 200_000),
            collision_checker=collision_checker,
            occupancy_cache=occupancy_cache,
        )

    # ----- 步骤 2：按用户顺序构造必经航点 -----
    planner.set_no_fly_zones(zones)
    fallback_altitude = min(5.0, req.altitude)
    request_points = [req.start, *req.viaPoints, req.end]
    actual_points = [
        {
            "lat": point.lat,
            "lng": point.lng,
            "altitude": point.altitude if point.altitude is not None else fallback_altitude,
        }
        for point in request_points
    ]
    fixed_height = req.altitudeMode == "fixed"
    if fixed_height:
        # 固定高度的含义是绝对严格：所有巡航点（包括途经点）都使用用户选择值。
        # 起点到该高度、该高度到终点的变化只发生在垂直起降段。
        cruise_points = [
            {**point, "altitude": req.altitude}
            for point in actual_points
        ]
    else:
        # 自由高度模式仍以用户选择值为最低巡航基准，并允许三维搜索继续爬升。
        cruise_points = [
            {**point, "altitude": max(req.altitude, point["altitude"])}
            for point in actual_points
        ]

    labels = ["start", *(f"via point {index + 1}" for index in range(len(req.viaPoints))), "end"]
    for label, point in zip(labels, cruise_points):
        if collision_checker.is_position_blocked(point["lat"], point["lng"], point["altitude"]):
            return schemas.AlgoResult(
                success=False,
                path=[],
                cost=0.0,
                elapsedMs=0.0,
                iterations=0,
                message=f"{label} is inside a no-fly zone or model obstacle",
            )

    # ----- 步骤 3：搜索原始分段，再统一做一次后处理 -----
    # 每个分段端点是保护锚点，不能在圆角化时被跳过。
    raw_legs: list[list[dict]] = []
    processed_legs: list[dict] = []
    smoothing_method = (
        (req.options.smoothingMethod or "moving_average")
        if req.options.smoothPath else "none"
    )
    total_elapsed_ms = 0.0
    total_iterations = 0
    for index in range(len(cruise_points) - 1):
        leg_start, leg_end = cruise_points[index], cruise_points[index + 1]
        if all(abs(leg_start[key] - leg_end[key]) < 1e-10 for key in ("lat", "lng", "altitude")):
            result = {"success": True, "path": [dict(leg_start)], "elapsedMs": 0.0, "iterations": 0}
        else:
            result = planner.plan_path(
                req.bounds.minLat,
                req.bounds.maxLat,
                req.bounds.minLng,
                req.bounds.maxLng,
                leg_start,
                leg_end,
                cruise_altitude=max(leg_start["altitude"], leg_end["altitude"]),
                fixed_altitude=req.altitude if fixed_height else None,
                smooth_path=False,
            )
        total_elapsed_ms += result["elapsedMs"]
        total_iterations += result["iterations"]
        if not result["success"]:
            return schemas.AlgoResult(
                success=False,
                path=[],
                cost=0.0,
                elapsedMs=total_elapsed_ms,
                iterations=total_iterations,
                message=(
                    f"segment {index + 1} ({labels[index]} -> {labels[index + 1]}) failed: "
                    f"{result.get('error') or 'no path found'}"
                ),
            )

        leg_path = [dict(point) for point in result.get("path", [])]
        if not leg_path:
            return schemas.AlgoResult(
                success=False, message=f"segment {index + 1} search returned an empty path",
                elapsedMs=(time.perf_counter()-started)*1000, iterations=total_iterations,
            )
        # 不同端点可能量化到同一个网格节点；不能用末点覆盖唯一的首点。
        # 精确连接仍须经过下面的统一后处理和障碍复检。
        if len(leg_path) == 1:
            leg_path = [dict(leg_start), dict(leg_end)]
        # 网格节点是离散坐标；强制恢复精确的必经点，再由最终航段校验把关。
        leg_path[0] = dict(leg_start)
        leg_path[-1] = dict(leg_end)
        if fixed_height:
            leg_path = force_constant_altitude(leg_path, req.altitude)
        raw_legs.append(leg_path)
        try:
            processed_legs.append(postprocess_leg(
                leg_path, collision_checker, method=smoothing_method,
                transition_distance=req.options.transitionDistanceMeters,
                fixed_altitude=req.altitude if fixed_height else None,
                minimum_altitude=None if fixed_height else req.altitude,
                bounds=req.bounds.model_dump(),
            ))
        except ValueError as exc:
            return schemas.AlgoResult(
                success=False, message=f"segment {index + 1} postprocessing failed: {exc}",
                elapsedMs=(time.perf_counter()-started)*1000, iterations=total_iterations,
            )

    cruise = [p for index, leg in enumerate(processed_legs) for p in (leg["path"] if index == 0 else leg["path"][1:])]

    # ----- 步骤 4：只在整条任务的首尾添加起飞和降落段 -----
    takeoff = plan_takeoff_path(actual_points[0], cruise_points[0]["altitude"], cruise[0])
    landing = plan_landing_path(cruise[-1], actual_points[-1], cruise_points[-1]["altitude"])

    def append_without_duplicate(target: list[dict], part: list[dict]) -> None:
        if not part:
            return
        if target and all(
            abs(target[-1][key] - part[0][key]) < 1e-10
            for key in ("lat", "lng", "altitude")
        ):
            target.extend(part[1:])
        else:
            target.extend(part)

    raw_cruise: list[dict] = []
    for leg in raw_legs:
        append_without_duplicate(raw_cruise, leg)
    raw_full_path: list[dict] = []
    for part in (takeoff, raw_cruise, landing):
        append_without_duplicate(raw_full_path, part)

    full_path: list[dict] = []
    full_curvatures: list[float] = []
    phases: list[str] = []
    stop_indices: set[int] = set()

    def append_trajectory_part(part, phase, curvatures=None, stops=None):
        if not part:
            return
        shared = bool(full_path) and all(abs(full_path[-1][key]-part[0][key]) < 1e-10 for key in ("lat","lng","altitude"))
        offset = len(full_path)-1 if shared else len(full_path)
        local_k = curvatures if curvatures is not None else [0.0]*len(part)
        if shared:
            full_curvatures[-1] = max(full_curvatures[-1],local_k[0])
        begin = 1 if shared else 0
        full_path.extend(part[begin:])
        full_curvatures.extend(local_k[begin:])
        phases.extend([phase]*(len(part)-begin))
        stop_indices.update(offset+i for i in (stops if stops is not None else [0,len(part)-1]))

    append_trajectory_part(takeoff,"takeoff")
    for leg in processed_legs:
        append_trajectory_part(leg["path"],"cruise",leg["curvatures"],leg["stopIndices"])
    append_trajectory_part(landing,"landing")

    # 平滑、精确途经点连接和起降段都做统一碰撞校验。
    if not collision_checker.path_is_safe(full_path):
        return schemas.AlgoResult(
            success=False,
            path=[],
            cost=0.0,
            elapsedMs=total_elapsed_ms,
            iterations=total_iterations,
            message="Final path failed unified obstacle collision validation",
        )

    # 时间规划会细分长线段。再验证实际返回、实际播放的精确坐标序列。
    try:
        trajectory = build_timed_trajectory(
            full_path,curvatures=full_curvatures,stop_indices=stop_indices,phases=phases,
            cruise_speed=req.options.cruiseSpeedMetersPerSecond,
            max_lateral_acceleration=req.options.maxLateralAcceleration,
            max_tangential_acceleration=req.options.maxTangentialAcceleration,
            vertical_speed=req.options.verticalSpeedMetersPerSecond,
        )
    except ValueError as exc:
        return schemas.AlgoResult(success=False,message=f"Trajectory generation failed: {exc}",
            elapsedMs=(time.perf_counter()-started)*1000,iterations=total_iterations)
    if not collision_checker.path_is_safe(trajectory["path"]):
        return schemas.AlgoResult(success=False,message="Timed trajectory failed obstacle collision validation",
            elapsedMs=(time.perf_counter()-started)*1000,iterations=total_iterations)

    reports = [leg["report"] for leg in processed_legs]
    smoothed = sum(r["smoothedCorners"] for r in reports)
    fallback = sum(r["fallbackCorners"] for r in reports)
    status = ("disabled" if smoothing_method == "none" else "partial" if fallback else
              "legacy" if smoothing_method == "moving_average" else "smoothed" if smoothed else "unchanged")
    report = dict(method=smoothing_method,status=status,smoothedCorners=smoothed,
                  fallbackCorners=fallback,removedPoints=sum(r["removedPoints"] for r in reports),
                  geometryToleranceMeters=0.03,
                  notes=list(dict.fromkeys(note for r in reports for note in r["notes"])))

    # ----- 步骤 5：同一轨迹用于展示、动画与统计 -----
    return schemas.AlgoResult(
        success=True,
        path=[schemas.Waypoint(**p) for p in trajectory["path"]],
        rawPath=[schemas.Waypoint(**p) for p in raw_full_path],
        trajectorySamples=[schemas.TrajectorySample(**p) for p in trajectory["samples"]],
        trajectoryStats=schemas.TrajectoryStats(**trajectory["stats"]),
        smoothingReport=schemas.SmoothingReport(**report),
        cost=trajectory["stats"]["totalDistanceMeters"],
        elapsedMs=(time.perf_counter()-started)*1000,
        iterations=total_iterations,
        message=None,
    )


@app.post("/api/obstacles/surface-height", response_model=schemas.SurfaceHeightResponse)
def obstacle_surface_height(
    req: schemas.SurfaceHeightRequest,
    _user: models.User = Depends(get_current_user),
) -> schemas.SurfaceHeightResponse:
    """查询经纬度竖直列中的最高模型表面，并预留 5 米起降净空。"""
    candidates: list[tuple[str, float | None]] = [
        ("out2", get_out2_dataset().get_surface_height(req.lat, req.lng)),
        (
            "reconstruction1",
            get_reconstruction1_dataset().get_surface_height(req.lat, req.lng),
        ),
    ]
    valid = [(source, height) for source, height in candidates if height is not None]
    if valid:
        source, surface_altitude = max(valid, key=lambda item: item[1])
    else:
        source, surface_altitude = "ellipsoid", 0.0
    clearance = 5.0
    return schemas.SurfaceHeightResponse(
        surfaceAltitude=float(surface_altitude),
        clearanceMeters=clearance,
        flightAltitude=float(surface_altitude + clearance),
        source=source,
    )


@app.post("/api/path/plan", response_model=schemas.PlanResponse)
async def plan_path(
    req: schemas.PlanRequest,
    _user: models.User = Depends(get_current_user),
) -> schemas.PlanResponse:
    """
    无人机航路规划 API（核心接口）。

    请求：POST /api/path/plan
    请求体格式 → 见 schemas.PlanRequest（包含边界、起止点、高度、禁飞区等）
    响应体格式 → 见 schemas.PlanResponse

    这是 async 函数的原因：
    - FastAPI 的主线程是异步事件循环
    - 路径规划算法是同步的（CPU 密集型），如果直接在主线程跑会卡死
    - 用 run_in_threadpool 把算法丢到线程池，主线程继续处理其他请求
    - async + await 就是这个"等待线程池结果"的机制

    曲线与时间轨迹说明见项目根目录《转弯曲线平滑与轨迹时间规划实现说明.md》。
    """
    # ----- 步骤 1：把 Pydantic 对象转成算法需要的 dict 格式 -----
    # schemas 层用的是 Pydantic 对象（有类型校验），
    # algorithms 层用的是普通 dict（轻量、和算法解耦），
    # 所以这里做一次格式转换。
    zones = [z.model_dump() for z in req.noFlyZones]
    # ----- 步骤 2：逐个运行算法（串行）-----
    # 为什么串行而非并行？
    #   路径规划是纯 CPU 计算，Python 的 GIL（全局解释器锁）导致
    #   多线程无法真正并行跑 CPU 任务，串行反而更快（避免上下文切换开销）。
    #   但放到线程池里是为了不阻塞事件循环上的其他请求。
    results: dict[str, schemas.AlgoResult] = {}
    for algo in req.algorithms:
        results[algo] = await run_in_threadpool(
            _run_single_algorithm, algo, req, zones
        )
        # await 在这里的作用：
        #   "我先把算法任务交给线程池，然后让出控制权给事件循环处理其他请求。
        #    等线程池跑完了再回来继续往下走。"

    # ----- 步骤 3：返回结果 -----
    out2_status = get_out2_dataset().status()
    out2_status["active"] = bool(req.useOut2Obstacles and out2_status["enabled"])
    reconstruction1_status = get_reconstruction1_dataset().status()
    reconstruction1_status["active"] = bool(
        req.useReconstruction1Obstacles and reconstruction1_status["enabled"]
    )
    return schemas.PlanResponse(
        results=results,
        out2Obstacles=schemas.ObstacleStatus(**out2_status),
        reconstruction1Obstacles=schemas.ObstacleStatus(**reconstruction1_status),
    )


@app.get("/api/obstacles/out2/status", response_model=schemas.ObstacleStatus)
def out2_obstacle_status(
    _user: models.User = Depends(get_current_user),
) -> schemas.ObstacleStatus:
    """学校 out2 实景模型障碍数据状态。"""
    status = get_out2_dataset().status()
    status["active"] = False
    return schemas.ObstacleStatus(**status)


@app.get("/api/obstacles/reconstruction1/status", response_model=schemas.ObstacleStatus)
def reconstruction1_obstacle_status(
    _user: models.User = Depends(get_current_user),
) -> schemas.ObstacleStatus:
    """“重建任务 1”实景模型障碍数据状态。"""
    status = get_reconstruction1_dataset().status()
    status["active"] = False
    return schemas.ObstacleStatus(**status)
