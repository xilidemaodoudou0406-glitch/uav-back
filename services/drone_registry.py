"""Single-event-loop registry. No database writes or unbounded telemetry history."""
from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from uuid import uuid4

from drone_schemas import Heartbeat, RegisterDevice, Telemetry


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class RegistryError(ValueError):
    pass


@dataclass
class Device:
    owner_id: int
    registration: RegisterDevice
    session_id: str
    heartbeat_at: float
    gateway_connected: bool = True
    aircraft_connected: bool = True
    last_sequence: int = -1
    telemetry: dict | None = None
    telemetry_at: float | None = None
    received_at: str | None = None
    captured_at: datetime | None = None
    reported_stale: bool = True
    position_available: bool = False
    revision: int = 0


class DroneRegistry:
    def __init__(self, *, stale_after=3.0, offline_after=6.0, clock=time.monotonic):
        self.stale_after = stale_after
        self.offline_after = offline_after
        self.clock = clock
        self.devices: dict[tuple[int, str], Device] = {}
        self.subscribers: dict[int, set[asyncio.Queue]] = {}
        self.revision = 0
        self.command_queues = {}
        self.on_lost = self.on_telemetry = self.on_tick = None

    def is_stale(self, device):
        return not device.position_available or device.telemetry_at is None or self.clock() - device.telemetry_at >= self.stale_after

    def serialize(self, device):
        age = None if device.telemetry_at is None else max(0, (self.clock() - device.telemetry_at) * 1000)
        return {
            "deviceId": device.registration.deviceId,
            "name": device.registration.name,
            "model": device.registration.model,
            "source": device.registration.source,
            "capabilities": device.registration.capabilities,
            "gatewaySessionId": device.session_id,
            "gatewayConnected": device.gateway_connected,
            "aircraftConnected": device.aircraft_connected and device.gateway_connected,
            "telemetryStale": self.is_stale(device),
            "telemetryAgeMs": None if age is None else round(age),
            "receivedAt": device.received_at,
            "lastHeartbeatAgeMs": round(max(0, self.clock() - device.heartbeat_at) * 1000),
            "telemetry": device.telemetry,
            "revision": device.revision,
        }

    def snapshot(self, owner_id):
        return {
            "schemaVersion": 1, "type": "snapshot", "revision": self.revision,
            "serverTime": utc_now(),
            "staleAfterMs": round(self.stale_after * 1000),
            "offlineAfterMs": round(self.offline_after * 1000),
            "devices": [self.serialize(d) for (owner, _), d in self.devices.items() if owner == owner_id],
        }

    def publish(self, device):
        self.revision += 1
        device.revision = self.revision
        device.reported_stale = self.is_stale(device)
        event = {"schemaVersion": 1, "type": "device_updated", "revision": self.revision,
                 "serverTime": utc_now(), "device": self.serialize(device)}
        for queue in self.subscribers.get(device.owner_id, ()):
            if queue.full():
                # A slow viewer gets a fresh snapshot, never an ever-growing backlog.
                while not queue.empty():
                    queue.get_nowait()
                queue.put_nowait(self.snapshot(device.owner_id))
            else:
                queue.put_nowait(event)

    def register(self, owner_id, registration):
        key = (owner_id, registration.deviceId)
        previous = self.devices.get(key)
        if previous:
            if self.on_lost:
                self.on_lost(owner_id, registration.deviceId, previous.session_id)
            self.command_queues.pop((owner_id, registration.deviceId, previous.session_id), None)
        if previous is None and sum(owner == owner_id for owner, _ in self.devices) >= 32:
            raise RegistryError("device_limit")
        device = Device(owner_id, registration, str(uuid4()), self.clock())
        # Preserve the last known sample as stale until this session supplies new data.
        if previous and previous.registration.source == registration.source and previous.registration.model == registration.model:
            device.telemetry = previous.telemetry
            device.received_at = previous.received_at
        self.devices[key] = device
        self.command_queues[(owner_id, registration.deviceId, device.session_id)] = asyncio.Queue(maxsize=16)
        self.publish(device)

        return device.session_id

    def current(self, owner_id, device_id, session_id):
        device = self.devices.get((owner_id, device_id))
        if device is None or device.session_id != session_id or not device.gateway_connected:
            raise RegistryError("session_expired")
        return device

    def ordered(self, owner_id, device_id, message):
        device = self.current(owner_id, device_id, message.gatewaySessionId)
        if message.sequence <= device.last_sequence:
            raise RegistryError("out_of_order")
        return device

    def heartbeat(self, owner_id, device_id, message: Heartbeat):
        device = self.ordered(owner_id, device_id, message)
        changed = device.aircraft_connected != message.aircraftConnected
        device.last_sequence = message.sequence
        device.heartbeat_at = self.clock()
        device.aircraft_connected = message.aircraftConnected
        if changed:
            # A live gateway is not proof that the aircraft or its previous fix is still available.
            device.position_available = False
            if not message.aircraftConnected:
                queue = self.command_queues.get((owner_id,device_id,device.session_id))
                if queue:
                    while not queue.empty():
                        queue.get_nowait()
                if self.on_lost:
                    self.on_lost(owner_id,device_id,device.session_id,"飞机连接断开，任务中断；恢复后必须重新准备")
            self.publish(device)

    def update_telemetry(self, owner_id, device_id, message: Telemetry):
        device = self.ordered(owner_id, device_id, message)
        if not device.aircraft_connected:
            raise RegistryError("aircraft_disconnected")
        if device.registration.source == "dji" and (message.heightSource == "simulation" or message.positioningQuality == "simulated" or message.missionRunId):
            raise RegistryError("source_scope_mismatch")
        if not message.positionValid:
            device.last_sequence = message.sequence
            device.position_available = False
            self.publish(device)
            raise RegistryError("position_unavailable")
        if message.sourceAgeMs >= self.stale_after * 1000:
            raise RegistryError("telemetry_expired")
        if device.captured_at and message.capturedAt <= device.captured_at:
            raise RegistryError("old_capture")
        device.last_sequence = message.sequence
        device.captured_at = message.capturedAt
        device.telemetry = message.model_dump(mode="json")
        device.position_available = True
        device.telemetry_at = self.clock() - message.sourceAgeMs / 1000
        device.received_at = utc_now()
        self.publish(device)
        if self.on_telemetry:
            self.on_telemetry(owner_id, device_id, device.session_id, message)

    def disconnect(self, owner_id, device_id, session_id):
        device = self.devices.get((owner_id, device_id))
        if device and device.session_id == session_id and device.gateway_connected:
            device.gateway_connected = False
            self.publish(device)
            self.command_queues.pop((owner_id, device_id, session_id), None)
            if self.on_lost:
                self.on_lost(owner_id, device_id, session_id)

    def tick(self):
        for device in self.devices.values():
            if device.gateway_connected and self.clock() - device.heartbeat_at >= self.offline_after:
                self.disconnect(device.owner_id, device.registration.deviceId, device.session_id)
            if device.reported_stale != self.is_stale(device):
                self.publish(device)
        if self.on_tick:
            self.on_tick()

    async def watchdog(self):
        while True:
            await asyncio.sleep(0.25)
            self.tick()

    def subscribe(self, owner_id):
        queues = self.subscribers.setdefault(owner_id, set())
        if len(queues) >= 12:
            raise RegistryError("subscriber_limit")
        queue = asyncio.Queue(maxsize=64)
        queues.add(queue)
        return queue

    def unsubscribe(self, owner_id, queue):
        queues = self.subscribers.get(owner_id)
        if queues:
            queues.discard(queue)
            if not queues:
                del self.subscribers[owner_id]
