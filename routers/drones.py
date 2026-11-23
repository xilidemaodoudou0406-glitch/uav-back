"""Authenticated, user-scoped monitoring. No flight control endpoints."""
from __future__ import annotations

import asyncio
import contextlib
import json
import time

import anyio

from fastapi import APIRouter, Depends, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.concurrency import run_in_threadpool
from pydantic import ValidationError

from auth import decode_access_token, get_current_user, get_user_from_token
from database import SessionLocal
from drone_schemas import Authenticate, Heartbeat, RegisterDevice, Telemetry
from mission_schemas import CommandResult
from services.drone_registry import RegistryError

router = APIRouter()


def socket_identity(token):
    with SessionLocal() as db:
        user = get_user_from_token(token, db)
        return user.id, int(decode_access_token(token)["exp"])


async def receive_message(socket, timeout=None):
    try:
        text = await asyncio.wait_for(socket.receive_text(), timeout) if timeout else await socket.receive_text()
    except KeyError as exc:
        raise ValueError("text_messages_only") from exc
    if len(text) > 16_384:
        raise ValueError("message_too_large")
    message = json.loads(text)
    if not isinstance(message, dict):
        raise ValueError("invalid_message")
    return message


async def authenticate(socket):
    await socket.accept()
    try:
        message = Authenticate.model_validate(await receive_message(socket, 5))
        identity = await run_in_threadpool(socket_identity, message.token)
        await socket.send_json({"schemaVersion": 1, "type": "authenticated", "serverEpochSeconds":time.time()})
        return identity
    except (ValueError, HTTPException, asyncio.TimeoutError):
        await socket.close(code=4401, reason="身份验证失败，请重新登录")
        return None


@router.get("/api/drones")
async def devices(request: Request, user=Depends(get_current_user)):
    registry = request.app.state.drone_registry
    registry.tick()
    return registry.snapshot(user.id)


@router.get("/api/drones/{device_id}/status")
async def device_status(device_id: str, request: Request, user=Depends(get_current_user)):
    registry = request.app.state.drone_registry
    registry.tick()
    device = registry.devices.get((user.id, device_id))
    if device is None:
        raise HTTPException(404, "设备不存在")
    return registry.serialize(device)


@router.websocket("/api/gateway/session")
async def gateway(socket: WebSocket):
    registry = socket.app.state.drone_registry
    owner_id = device_id = session_id = None
    try:
        identity = await authenticate(socket)
        if identity is None:
            return
        owner_id, expires_at = identity
        registration = RegisterDevice.model_validate(await receive_message(socket, 5))
        device_id = registration.deviceId
        session_id = registry.register(owner_id, registration)
        await socket.send_json({"schemaVersion": 1, "type": "registered", "gatewaySessionId": session_id,
                                "staleAfterMs": round(registry.stale_after * 1000)})
        async def receive_gateway():
            while True:
                if time.time() >= expires_at:
                    await socket.close(code=4401, reason="登录已过期")
                    break
                registry.current(owner_id, device_id, session_id)
                try:
                    raw = await receive_message(socket, 1)
                    if raw.get("type") == "heartbeat":
                        message = Heartbeat.model_validate(raw)
                        registry.heartbeat(owner_id, device_id, message)
                        await socket.send_json({"schemaVersion":1,"type":"heartbeat_ack","sequence":message.sequence})
                    elif raw.get("type") == "telemetry":
                        message = Telemetry.model_validate(raw)
                        registry.update_telemetry(owner_id, device_id, message)
                    elif raw.get("type") == "command_result":
                        socket.app.state.missions.acknowledge(owner_id, device_id, CommandResult.model_validate(raw))
                    else:
                        raise ValueError("unsupported_message")
                except asyncio.TimeoutError:
                    continue
                except (ValueError, ValidationError) as exc:
                    if isinstance(exc, RegistryError) and str(exc) == "session_expired":
                        raise
                    await socket.send_json({"schemaVersion": 1, "type": "error",
                        "code": str(exc) if isinstance(exc, RegistryError) else "invalid_message"})

        command_queue = registry.command_queues[(owner_id, device_id, session_id)]
        async with anyio.create_task_group() as group:
            async def send_commands():
                try:
                    while True:
                        packet = await command_queue.get()
                        device = registry.current(owner_id, device_id, session_id)
                        if device.aircraft_connected and packet["expiresAt"] > time.time():
                            await socket.send_json(packet)
                except (WebSocketDisconnect, RuntimeError, RegistryError):
                    group.cancel_scope.cancel()

            group.start_soon(send_commands)
            try:
                await receive_gateway()
            except (WebSocketDisconnect, RuntimeError, ValueError):
                with contextlib.suppress(RuntimeError, WebSocketDisconnect):
                    await socket.close(code=4400, reason="会话结束")
            finally:
                group.cancel_scope.cancel()
    except (WebSocketDisconnect, RuntimeError):
        pass
    except (ValueError, asyncio.TimeoutError):
        with contextlib.suppress(RuntimeError, WebSocketDisconnect):
            await socket.close(code=4400, reason="网关消息无效或会话已更换")
    finally:
        if session_id:
            registry.disconnect(owner_id, device_id, session_id)


@router.websocket("/api/drones/stream")
async def stream(socket: WebSocket):
    registry = socket.app.state.drone_registry
    queue = None
    identity = None
    try:
        identity = await authenticate(socket)
        if identity is None:
            return
        owner_id, expires_at = identity
        queue = registry.subscribe(owner_id)
        await socket.send_json(registry.snapshot(owner_id))

        async def send_events():
            while True:
                if time.time() >= expires_at:
                    await socket.close(code=4401, reason="登录已过期")
                    return
                try:
                    event = await asyncio.wait_for(queue.get(), 1)
                except asyncio.TimeoutError:
                    event = {"schemaVersion": 1, "type": "server_heartbeat"}
                await socket.send_json(event)

        async with anyio.create_task_group() as group:
            async def receive_events():
                try:
                    while True:
                        # Detect a closed browser even when no gateway is connected.
                        message = await receive_message(socket)
                        if message.get("type") != "ping":
                            await socket.close(code=4400, reason="监控连接只支持订阅")
                            return
                except (WebSocketDisconnect, RuntimeError):
                    pass
                except ValueError:
                    with contextlib.suppress(RuntimeError, WebSocketDisconnect):
                        await socket.close(code=4400, reason="监控消息无效")
                finally:
                    group.cancel_scope.cancel()

            group.start_soon(receive_events)
            try:
                await send_events()
            except (WebSocketDisconnect, RuntimeError):
                pass
            finally:
                group.cancel_scope.cancel()
    except (WebSocketDisconnect, RuntimeError):
        pass
    except (ValueError, RegistryError):
        with contextlib.suppress(RuntimeError, WebSocketDisconnect):
            await socket.close(code=4400, reason="订阅消息无效或连接过多")
    finally:
        if queue is not None:
            registry.unsubscribe(identity[0], queue)
