"""
database.py — 数据库连接与配置
===============================
这个文件负责：
1. 创建数据库引擎（连接到 SQLite 文件 app.db）
2. 定义"会话工厂"（SessionLocal），用于创建数据库会话
3. 提供 get_db() 依赖函数，供 FastAPI 路由使用

简单类比：
- engine    → 电话线（连接数据库）
- Session   → 一次通话（一次数据库操作对话）
- Base      → 模具（所有 ORM 模型的基类，models.py 用它来建表）
"""

import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base

# ---------------------------------------------------------------------------
# 数据库连接地址
# sqlite:///./app.db 表示使用 SQLite 数据库，文件保存在当前目录的 app.db
# 如果你的数据库换成 PostgreSQL，这里改为类似：
#   postgresql://用户名:密码@主机:端口/数据库名
# ---------------------------------------------------------------------------
SQLALCHEMY_DATABASE_URL = os.getenv("UAV_DATABASE_URL", "sqlite:///./app.db")

# ---------------------------------------------------------------------------
# 创建数据库引擎（engine）
# - echo=False：不打印执行的 SQL 语句（调试时可改为 True）
# - connect_args={"check_same_thread": False}：SQLite 专属设置
#   因为 FastAPI 是异步框架，多个请求可能从不同线程访问 SQLite，
#   默认的 check_same_thread=True 会阻止这种行为，所以必须关闭
# ---------------------------------------------------------------------------
engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    connect_args={"check_same_thread": False}  # SQLite 专属：允许多线程访问
)

# ---------------------------------------------------------------------------
# 创建"会话工厂"（SessionLocal）
# - 把它想象成一个"对话创建器"：每次调用 SessionLocal() 就开一次新的数据库对话
# - autocommit=False：不自动提交，需要手动 db.commit() 才会真正写入
# - autoflush=False：不自动刷新，避免不必要的数据库同步
# - bind=engine：告诉它连哪个数据库
# ---------------------------------------------------------------------------
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# ---------------------------------------------------------------------------
# 声明式基类（Base）
# models.py 中的每个 ORM 模型都要继承这个 Base。
# Base.metadata.create_all() 会根据所有继承它的类自动创建数据库表。
# ---------------------------------------------------------------------------
Base = declarative_base()


# ---------------------------------------------------------------------------
# FastAPI 依赖函数：获取数据库会话
# 使用方式：在路由函数参数里写 db: Session = Depends(get_db)
#
# 工作流程：
#   1. 请求进来 → 创建一次新的数据库会话
#   2. 路由函数用这个会话操作数据库
#   3. 请求结束 → 自动关闭会话（无论成功还是出错）
#
# yield 是 Python 的生成器语法：
#   - yield 之前的代码在请求开始时执行（创建会话）
#   - yield 值交给路由函数使用
#   - yield 之后的代码在请求结束时执行（关闭会话）
# ---------------------------------------------------------------------------
def get_db():
    """每个 HTTP 请求获取一个独立的数据库会话，用完自动关闭"""
    db = SessionLocal()
    try:
        yield db       # 把会话交给路由函数
    finally:
        db.close()     # 无论成功还是失败，都确保关闭会话
