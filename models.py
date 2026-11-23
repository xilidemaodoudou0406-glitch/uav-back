"""
models.py — 数据库表模型（ORM 模型）
=====================================
这里定义的每个类，都对应数据库中的一张表。
用的是 SQLAlchemy ORM —— 不需要手写 SQL，用 Python 类就能操作数据库。

简单类比：
- class Todo(Base)        → 定义了一张名叫 "todos" 的表
- Column(Integer, ...)     → 表里的列（字段）
- db.query(models.Todo)    → 相当于 SELECT * FROM todos
- db.add(todo)             → 相当于 INSERT INTO todos ...

温馨提示：
- 这个文件只定义表结构，不做增删改查。
- 实际的增删改查在 main.py 的路由函数里。
"""

from sqlalchemy import Column, Integer, String, Boolean, Float, JSON, ForeignKey, Index
from database import Base    # Base 是所有 ORM 模型的基类（在 database.py 中定义）


class Todo(Base):
    """
    待办事项表（todos）
    这是后端自带的一个示例表，用来演示 CRUD 操作。
    你的路径规划功能不依赖这个表。
    """
    # __tablename__ 指定数据库中的表名
    __tablename__ = "todos"

    # -----------------------------------------------------------------------
    # 字段定义
    # - primary_key=True   → 主键，每条记录的唯一标识
    # - index=True         → 为这个字段建索引，查询更快
    # - nullable=False     → 不允许为空（必填）
    # - default=False      → 默认值为 False
    # -----------------------------------------------------------------------
    id = Column(Integer, primary_key=True, index=True)   # 自动递增的 ID
    title = Column(String, nullable=False)               # 待办标题
    done = Column(Boolean, default=False)                # 是否完成


class User(Base):
    """平台用户；密码字段仅保存加盐哈希。"""

    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    username = Column(String(64), unique=True, nullable=False, index=True)
    display_name = Column(String(64), nullable=False)
    password_hash = Column(String(255), nullable=False)
    active = Column(Boolean, nullable=False, default=True)


class SimulationMission(Base):
    __tablename__ = "simulation_missions"
    id = Column(String(64), primary_key=True)
    owner_id = Column(Integer, ForeignKey("users.id"), nullable=False, index=True)
    device_id = Column(String(64), nullable=False, index=True)
    name = Column(String(80), nullable=False)
    checksum = Column(String(64), nullable=False)
    snapshot = Column(JSON, nullable=False)
    created_at = Column(String(40), nullable=False)


class SimulationRun(Base):
    __tablename__ = "simulation_runs"
    id = Column(String(64), primary_key=True)
    mission_id = Column(String(64), ForeignKey("simulation_missions.id"), nullable=False, index=True)
    owner_id = Column(Integer, nullable=False, index=True)
    device_id = Column(String(64), nullable=False, index=True)
    session_id = Column(String(64), nullable=False)
    state = Column(String(24), nullable=False)
    progress = Column(Float, nullable=False, default=0)
    elapsed_seconds = Column(Float, nullable=False, default=0)
    reason = Column(String(240), nullable=True)
    created_at = Column(String(40), nullable=False)
    updated_at = Column(String(40), nullable=False)


class SimulationCommand(Base):
    __tablename__ = "simulation_commands"
    id = Column(String(64), primary_key=True)
    owner_id = Column(Integer, nullable=False)
    mission_id = Column(String(64), nullable=False)
    run_id = Column(String(64), ForeignKey("simulation_runs.id"), nullable=False, index=True)
    action = Column(String(16), nullable=False)
    status = Column(String(24), nullable=False)
    issued_at = Column(String(40), nullable=False)
    expires_at = Column(Float, nullable=False)
    reason = Column(String(240), nullable=True)


class SimulationEvent(Base):
    __tablename__ = "simulation_events"
    id = Column(Integer, primary_key=True)
    run_id = Column(String(64), ForeignKey("simulation_runs.id"), nullable=False, index=True)
    at = Column(String(40), nullable=False)
    kind = Column(String(40), nullable=False)
    detail = Column(String(240), nullable=False)


class SimulationSample(Base):
    __tablename__ = "simulation_samples"
    id = Column(Integer, primary_key=True)
    run_id = Column(String(64), ForeignKey("simulation_runs.id"), nullable=False, index=True)
    received_at = Column(String(40), nullable=False)
    telemetry = Column(JSON, nullable=False)


class NavigationCalibration(Base):
    __tablename__ = "navigation_calibrations"
    id = Column(String(64), primary_key=True)
    owner_id = Column(Integer, ForeignKey("users.id"), nullable=False, index=True)
    name = Column(String(80), nullable=False)
    payload = Column(JSON, nullable=False)
    created_at = Column(String(40), nullable=False)


class MissionValidation(Base):
    __tablename__ = "mission_validations"
    id = Column(Integer, primary_key=True)
    mission_id = Column(String(64), ForeignKey("simulation_missions.id"), nullable=False, index=True)
    report = Column(JSON, nullable=False)


# Additive indexes also need explicit creation for databases whose tables already exist.
MISSION_QUERY_INDEXES = (
    Index("ix_simulation_commands_status_expiry", SimulationCommand.status, SimulationCommand.expires_at),
    Index("ix_simulation_runs_mission_created", SimulationRun.mission_id, SimulationRun.created_at, SimulationRun.id),
)


def ensure_mission_query_indexes(bind):
    for index in MISSION_QUERY_INDEXES:
        index.create(bind=bind,checkfirst=True)
