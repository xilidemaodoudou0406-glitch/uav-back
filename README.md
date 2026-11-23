# 无人机平台后端

本项目使用 FastAPI、SQLAlchemy 和 SQLite，提供航线规划、障碍预检、设备遥测和模拟任务接口。两套后端障碍物数据随仓库提供。

## 在另一台 Windows 电脑运行

使用 Python 3.14，在后端目录执行：

```powershell
py -3.14 -m venv venv
.\venv\Scripts\python.exe -m pip install -r requirements.txt
.\venv\Scripts\python.exe -m uvicorn main:app --host 127.0.0.1 --port 8000
```

前端开发服务器默认连接 `127.0.0.1:8000`。如需保留旧电脑上的账号、任务和执行记录，先停止旧后端，再把 `app.db` 单独复制到新电脑的后端目录。首次运行时，如该文件不存在，后端会创建本地数据库。

安装 `requirements-dev.txt` 后运行 `.\venv\Scripts\python.exe -m unittest discover -s tests`，可执行后端测试。默认账号和签名密钥供本地开发使用。部署到共享环境前，请配置 `UAV_AUTH_SECRET` 和 `UAV_ADMIN_PASSWORD` 等环境变量；已有数据库中的账号需单独更新密码。
