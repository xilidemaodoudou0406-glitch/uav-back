"""Independent Stage 1 gateway. Uses the same HTTP/WS boundary as future devices."""
from __future__ import annotations

import argparse
import asyncio
import getpass
import json
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit, urlunsplit
from urllib.request import Request, urlopen

from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed
if __package__:
    from .simulated_mission import SimulatedMission
else:
    from simulated_mission import SimulatedMission


def login(base_url, username, password):
    request = Request(base_url + "/api/auth/login",
                      data=json.dumps({"username": username, "password": password}).encode(),
                      headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urlopen(request, timeout=10) as response:
            return json.load(response)["accessToken"]
    except HTTPError as exc:
        raise RuntimeError(f"登录失败（HTTP {exc.code}），请检查平台账号。") from None
    except URLError:
        raise RuntimeError("无法连接后端，请先启动 Python 服务并核对后端地址。") from None


def sample(elapsed, session_id, sequence, center):
    angle = elapsed * 2 * math.pi / 90
    east, north = 50 * math.cos(angle), 50 * math.sin(angle)
    lng, lat = center
    return {
        "schemaVersion": 1, "type": "telemetry", "gatewaySessionId": session_id,
        "sequence": sequence, "capturedAt": datetime.now(timezone.utc).isoformat(), "sourceAgeMs": 0,
        "horizontalDatum": "WGS84", "positionValid": True,
        "lat": lat + north / 111_320,
        "lng": lng + east / (111_320 * math.cos(math.radians(lat))),
        "heightEllipsoidMeters": 80, "heightRelativeTakeoffMeters": 80,
        "heightSource": "simulation", "speedMps": 50 * 2 * math.pi / 90,
        "headingDegrees": math.degrees(math.atan2(-math.sin(angle), math.cos(angle))) % 360,
        "batteryPercent": round(max(20, 100 - elapsed / 60), 1),
        "positioningQuality": "simulated", "flightMode": "SIMULATED",
    }


async def run(args, token):
    url = urlsplit(args.backend)
    uri = urlunsplit(("wss" if url.scheme == "https" else "ws", url.netloc,
                     url.path.rstrip("/") + "/api/gateway/session", "", ""))
    metadata = Path(__file__).resolve().parents[1] / "obstacle_data/out2/tiles.json"
    center = (117.13414206756654, 34.214004843214234)
    if metadata.exists():
        center = tuple(json.loads(metadata.read_text(encoding="utf-8"))["tiles"][0]["centerWgs84"][:2])
    started = time.monotonic()
    scenario_done = False
    controller = SimulatedMission()
    print(f"模拟设备 {args.device_id}，场景 {args.scenario}；按 Ctrl+C 退出。", flush=True)
    print(f"演示中心：经度 {center[0]:.8f}，纬度 {center[1]:.8f}；椭球高 80 m。", flush=True)
    while not args.duration or time.monotonic() - started < args.duration:
        try:
            # Disable environment proxy discovery for the local gateway connection.
            controller.disconnect()
            async with connect(uri, proxy=None, open_timeout=5, max_size=8_000_000) as socket:
                await socket.send(json.dumps({"schemaVersion": 1, "type": "authenticate", "token": token}))
                if json.loads(await socket.recv())["type"] != "authenticated":
                    raise RuntimeError("网关身份验证失败。")
                await socket.send(json.dumps({"schemaVersion": 1, "type": "register",
                    "deviceId": args.device_id, "name": "Mini 3 接入模拟器", "source": "simulated",
                    "capabilities": ["mission_simulation"]}))
                registered = json.loads(await socket.recv())
                session_id = registered["gatewaySessionId"]
                print("网关已连接，正在上传演示遥测。", flush=True)
                connected_at = time.monotonic()
                next_heartbeat = 0
                sequence = 0
                pending_telemetry = []
                was_paused = False
                send_lock = asyncio.Lock()

                async def send_packet(packet):
                    nonlocal sequence
                    async with send_lock:
                        sequence += 1
                        packet.update(schemaVersion=1, gatewaySessionId=session_id, sequence=sequence)
                        await socket.send(json.dumps(packet))

                async def read_errors():
                    async for raw in socket:
                        event = json.loads(raw)
                        if event.get("type") == "error":
                            print(f"后端拒绝消息：{event.get('code')}", flush=True)
                        elif event.get("type") == "mission_command":
                            try:
                                result = controller.handle(event, session_id)
                            except (ValueError, KeyError, TypeError):
                                result = {"type": "command_result", "commandId": event["commandId"],
                                    "runId": event["runId"], "checksum": event["checksum"],
                                    "accepted": False, "state": "failed", "reason": "任务数据校验失败"}
                            await send_packet(result)

                reader = asyncio.create_task(read_errors())
                try:
                    while not args.duration or time.monotonic() - started < args.duration:
                        now = time.monotonic()
                        elapsed = now - connected_at
                        if now >= next_heartbeat:
                            await send_packet({"type": "heartbeat", "aircraftConnected": True})
                            next_heartbeat = now + 1
                        paused = args.scenario == "stale" and not scenario_done and 10 <= elapsed < 10 + args.pause_seconds
                        if paused and not was_paused:
                            print(f"暂停遥测 {args.pause_seconds:g} 秒，心跳继续：页面应显示位置过期。", flush=True)
                        if was_paused and not paused:
                            scenario_done = True
                            print("恢复遥测。", flush=True)
                        was_paused = paused
                        if args.scenario == "reconnect" and not scenario_done and elapsed >= 10:
                            scenario_done = True
                            print("断开网关 8 秒，随后自动重新登记同一设备。", flush=True)
                            break
                        if not paused:
                            packet = controller.telemetry(sample(now - started, session_id, sequence, center))
                            delay = (controller.snapshot or {}).get("errorScenario",{}).get("telemetryDelayMs",0)/1000
                            pending_telemetry.append((now+delay,now,packet))
                        while pending_telemetry and pending_telemetry[0][0] <= now:
                            _, captured, packet = pending_telemetry.pop(0)
                            packet["sourceAgeMs"] = max(0,(now-captured)*1000)
                            await send_packet(packet)
                        if reader.done():
                            reader.result()
                            raise ConnectionError("网关连接已关闭")
                        await asyncio.sleep(0.2)
                finally:
                    controller.disconnect()
                    reader.cancel()
                    await asyncio.gather(reader, return_exceptions=True)
            if args.scenario == "reconnect" and scenario_done and (not args.duration or time.monotonic() - started < args.duration):
                await asyncio.sleep(8)
        except ConnectionClosed as exc:
            if exc.code in (4400, 4401):
                raise RuntimeError("登录或会话无效，请重新运行并登录；不要同时启动同一设备 ID。") from None
            print("连接中断，2 秒后重连。", flush=True)
            await asyncio.sleep(2)
        except (OSError, TimeoutError, ConnectionError):
            print("后端暂时不可用，2 秒后重连。", flush=True)
            await asyncio.sleep(2)


def main():
    parser = argparse.ArgumentParser(description="阶段一模拟网关：不连接或控制真实无人机。")
    parser.add_argument("--backend", default="http://127.0.0.1:8000")
    parser.add_argument("--scenario", choices=["normal", "stale", "reconnect"], default="normal")
    parser.add_argument("--device-id", default="sim-mini3-001")
    parser.add_argument("--username")
    parser.add_argument("--duration", type=float, default=0, help="运行秒数；默认持续运行")
    parser.add_argument("--pause-seconds", type=float, default=8, help="stale 场景的位置暂停秒数")
    args = parser.parse_args()
    args.backend = args.backend.rstrip("/")
    url = urlsplit(args.backend)
    if url.scheme not in ("http", "https") or not url.netloc or url.username or url.password or url.query or url.fragment:
        parser.error("后端地址必须是有效 HTTP(S) 地址，不能带账号、密码、查询参数或片段。")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("duration 必须为有限非负数")
    if not math.isfinite(args.pause_seconds) or args.pause_seconds < 4:
        parser.error("pause-seconds 必须为不小于 4 的有限数")
    token = os.getenv("UAV_GATEWAY_TOKEN")
    if not token:
        username = args.username or input("平台用户名：").strip()
        token = login(args.backend, username, getpass.getpass("平台密码（不显示）："))
    asyncio.run(run(args, token))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n模拟网关已停止。")
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
