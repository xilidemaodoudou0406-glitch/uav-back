"""Real-network acceptance check. Start backend + Vite first; no real drone required."""
from __future__ import annotations

import argparse
import asyncio
import getpass
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from urllib.request import Request, urlopen

from websockets.asyncio.client import connect
from mock_drone_gateway import login


async def verify(args, token):
    device_id = "sim-acceptance-001"
    process = None
    env = {**os.environ, "UAV_GATEWAY_TOKEN": token, "PYTHONIOENCODING": "utf-8"}
    script = Path(__file__).with_name("mock_drone_gateway.py")
    latest = None

    def start(scenario):
        return subprocess.Popen([sys.executable, str(script), "--backend", args.backend,
            "--scenario", scenario, "--device-id", device_id], env=env, stdout=subprocess.DEVNULL)

    def stop():
        if process and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)

    async with connect(args.stream, proxy=None, open_timeout=5) as viewer:
        await viewer.send(json.dumps({"schemaVersion": 1, "type": "authenticate", "token": token}))
        assert json.loads(await viewer.recv())["type"] == "authenticated"
        assert json.loads(await viewer.recv())["type"] == "snapshot"

        async def until(predicate, timeout):
            nonlocal latest
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                event = json.loads(await asyncio.wait_for(viewer.recv(), max(0.1, deadline - time.monotonic())))
                if event.get("type") == "device_updated" and event["device"]["deviceId"] == device_id:
                    latest = event["device"]
                    if predicate(latest):
                        return latest
            raise AssertionError("等待预期设备状态超时")

        try:
            process = start("normal")
            await until(lambda d: d["gatewayConnected"] and not d["telemetryStale"], 5)
            print("PASS 正式接口连接与首条遥测（包含 Vite WebSocket 代理）", flush=True)
            first = latest["telemetry"]["sequence"]
            deadline = time.monotonic() + args.observe
            while time.monotonic() < deadline:
                await until(lambda d: not d["telemetryStale"], 3)
            assert latest["telemetry"]["sequence"] > first
            print(f"PASS 连续观察 {args.observe:g} 秒，位置和电量持续更新", flush=True)
            stop()
            await until(lambda d: not d["gatewayConnected"], 7)
            print("PASS 关闭网关后离线，保留最后位置", flush=True)

            process = start("stale")
            await until(lambda d: not d["telemetryStale"] and d["gatewayConnected"], 5)
            await until(lambda d: d["telemetryStale"] and d["gatewayConnected"], 15)
            assert 3000 <= latest["telemetryAgeMs"] <= 4000
            print(f"PASS 心跳在线但位置过期：{latest['telemetryAgeMs']} ms", flush=True)
            await until(lambda d: not d["telemetryStale"] and d["gatewayConnected"], 10)
            print("PASS 恢复遥测后自动更新", flush=True)
            stop()
            await until(lambda d: not d["gatewayConnected"], 7)

            process = start("reconnect")
            initial = await until(lambda d: not d["telemetryStale"] and d["gatewayConnected"], 5)
            session = initial["gatewaySessionId"]
            await until(lambda d: not d["gatewayConnected"], 15)
            restored = await until(lambda d: d["gatewayConnected"] and not d["telemetryStale"], 12)
            assert restored["gatewaySessionId"] != session
            request = Request(args.backend + "/api/drones", headers={"Authorization": "Bearer " + token})
            with urlopen(request) as response:
                devices = json.load(response)["devices"]
            assert len([d for d in devices if d["deviceId"] == device_id]) == 1
            print("PASS 重连更换会话，同一设备无重复条目", flush=True)
        finally:
            stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", default="http://127.0.0.1:8000")
    parser.add_argument("--stream", default="ws://127.0.0.1:5173/api/drones/stream")
    parser.add_argument("--observe", type=float, default=60)
    args = parser.parse_args()
    token = os.getenv("UAV_GATEWAY_TOKEN")
    if not token:
        token = login(args.backend, input("平台用户名：").strip(), getpass.getpass("平台密码（不显示）："))
    asyncio.run(verify(args, token))
