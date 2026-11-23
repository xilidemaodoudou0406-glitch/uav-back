"""Persistent simulation tasks with session-bound, acknowledged commands."""
from __future__ import annotations

import hashlib
import json
import time
from collections import OrderedDict
from copy import deepcopy
from threading import RLock
from uuid import uuid4

from fastapi import HTTPException
from sqlalchemy import select, func
from sqlalchemy.orm import aliased, defer

import models as m
from mission_schemas import MissionCreate
from pydantic import ValidationError
from database import SessionLocal
from services.drone_registry import RegistryError, utc_now
from services.navigation_service import NavigationService
from services.mission_validation import validate_snapshot
from navigation import convert_height, apply_calibration

ACTIVE = ("preparing", "ready", "executing", "paused")
EXPECTED = {"PREPARE": "ready", "START": "executing", "PAUSE": "paused", "RESUME": "executing", "ABORT": "aborted"}
ALLOWED = {"START": ("ready",), "PAUSE": ("executing",), "RESUME": ("paused",), "ABORT": ("ready", "executing", "paused")}


def content_checksum(snapshot):
    return hashlib.sha256(json.dumps(snapshot, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
                                     allow_nan=False).encode("utf-8")).hexdigest()


def canonical_snapshot(snapshot):
    return json.dumps(snapshot, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


class MissionService:
    def __init__(self, registry, sessions=SessionLocal, clock=time.time):
        self.registry, self.sessions, self.clock = registry, sessions, clock
        self.last_recorded = {}
        self.run_durations = {}
        self.terminal_recorded = OrderedDict()
        self._metadata_cache = OrderedDict()
        self._metadata_lock = RLock()
        self.recover()

    def recover(self):
        with self.sessions() as db:
            for run in db.scalars(select(m.SimulationRun).where(m.SimulationRun.state.in_(ACTIVE))):
                self.fail(db, run, "后端重启，执行中断；必须重新准备任务")
            db.commit()

    def event(self, db, run, kind, detail):
        db.add(m.SimulationEvent(run_id=run.id, at=utc_now(), kind=kind, detail=detail[:240]))

    def fail(self, db, run, reason):
        self.last_recorded.pop(run.id, None)
        self.run_durations.pop(run.id, None)
        run.state, run.reason, run.updated_at = "failed", reason, utc_now()
        for command in db.scalars(select(m.SimulationCommand).where(
                m.SimulationCommand.run_id == run.id, m.SimulationCommand.status == "sent")):
            command.status, command.reason = "failed", reason
        self.event(db, run, "failed", reason)

    def mission(self, db, owner_id, mission_id, include_snapshot=True):
        query = select(m.SimulationMission).where(m.SimulationMission.id == mission_id,
            m.SimulationMission.owner_id == owner_id)
        if not include_snapshot:
            query = query.options(defer(m.SimulationMission.snapshot))
        mission = db.scalar(query)
        if mission is None or mission.owner_id != owner_id:
            raise HTTPException(404, "任务不存在")
        return mission

    @staticmethod
    def _latest_rows(db, model, partition, order, condition):
        ranked = select(model, func.row_number().over(partition_by=partition, order_by=order).label("_rank"))\
            .where(condition).subquery()
        return list(db.scalars(select(aliased(model, ranked)).where(ranked.c._rank == 1)))

    def _commands(self, db, runs):
        ids = [r.id for r in runs]
        if not ids:
            return {}, {}
        latest = self._latest_rows(db, m.SimulationCommand, m.SimulationCommand.run_id,
            [m.SimulationCommand.issued_at.desc(), m.SimulationCommand.id.desc()], m.SimulationCommand.run_id.in_(ids))
        pending = list(db.scalars(select(m.SimulationCommand).where(m.SimulationCommand.run_id.in_(ids),
            m.SimulationCommand.status == "sent")))
        return {c.run_id:c for c in latest}, {c.run_id:c for c in pending}

    def _metadata(self, db, missions):
        """Read large immutable snapshots once, in one batch; keep only small derived metadata."""
        with self._metadata_lock:
            missing = [item for item in missions if (item.id,item.checksum) not in self._metadata_cache]
            def remember(item, snapshot):
                self._metadata_cache[(item.id,item.checksum)] = {
                    "durationSeconds":snapshot["trajectorySamples"][-1]["timeSeconds"],
                    "pointCount":len(snapshot["trajectorySamples"]),"coordinateContext":snapshot.get("coordinateContext")}
            unloaded = {}
            for item in missing:
                if "snapshot" in item.__dict__:
                    remember(item,item.__dict__["snapshot"])
                else:
                    unloaded[item.id] = item
            if unloaded:
                for identity,snapshot in db.execute(select(m.SimulationMission.id,m.SimulationMission.snapshot)
                        .where(m.SimulationMission.id.in_(unloaded))).yield_per(1):
                    remember(unloaded[identity],snapshot)
            result = {}
            for item in missions:
                key = (item.id,item.checksum)
                result[item.id] = deepcopy(self._metadata_cache[key])
                self._metadata_cache.move_to_end(key)
            while len(self._metadata_cache)>256:
                self._metadata_cache.popitem(last=False)
            return result

    def run_dict(self, db, run, commands=None):
        if run is None:
            return None
        latest, sent = commands if commands is not None else self._commands(db,[run])
        pending, last = sent.get(run.id), latest.get(run.id)
        return {"id": run.id, "state": run.state, "progress": run.progress, "elapsedSeconds": run.elapsed_seconds,
                "reason": run.reason, "createdAt": run.created_at, "updatedAt": run.updated_at,
                "pendingCommand": self.command_dict(pending) if pending else None,
                "lastCommand": self.command_dict(last) if last else None}

    @staticmethod
    def command_dict(command):
        return {"commandId": command.id, "runId": command.run_id, "action": command.action,
                "status": command.status, "issuedAt": command.issued_at,
                "expiresAt": command.expires_at, "reason": command.reason}

    def _summary_context(self, db, missions, history=False):
        ids = [item.id for item in missions]
        metadata = self._metadata(db,missions)
        if not ids:
            return metadata, {}, ({},{}), {}, []
        if history:
            runs = list(db.scalars(select(m.SimulationRun).where(m.SimulationRun.mission_id.in_(ids))
                .order_by(m.SimulationRun.created_at.desc(),m.SimulationRun.id.desc()).limit(50)))
        else:
            runs = self._latest_rows(db,m.SimulationRun,m.SimulationRun.mission_id,
                [m.SimulationRun.created_at.desc(),m.SimulationRun.id.desc()],m.SimulationRun.mission_id.in_(ids))
        latest_runs = {}
        for run in runs:
            latest_runs.setdefault(run.mission_id,run)
        validations = self._latest_rows(db,m.MissionValidation,m.MissionValidation.mission_id,
            [m.MissionValidation.id.desc()],m.MissionValidation.mission_id.in_(ids))
        return metadata,latest_runs,self._commands(db,runs),{v.mission_id:v.report for v in validations},runs

    def summary(self, db, mission, detail=False, context=None, include_snapshot=True):
        metadata,runs,commands,validations,history = context if context is not None else self._summary_context(db,[mission],detail)
        run = runs.get(mission.id)
        result = {"id": mission.id, "name": mission.name, "deviceId": mission.device_id,
                  "version": 1, "checksum": mission.checksum, "executionScope": "simulation_only",
                  "heightReference": "model_ellipsoid_display", "calibrationStatus": "unverified",
                  **metadata[mission.id], "createdAt": mission.created_at,
                  "latestRun": self.run_dict(db, run, commands),"validation":validations.get(mission.id)}
        if detail:
            if include_snapshot:
                result["snapshot"] = mission.snapshot
            result["runs"] = [self.run_dict(db,item,commands) for item in history]
        return result

    def create(self, owner_id, request):
        snapshot = request.model_dump(mode="json")
        snapshot["version"] = 1
        snapshot["calibrationStatus"] = "unverified"
        source = request.heightReference
        calibration = NavigationService(self.sessions).get(owner_id, request.calibrationId) if request.calibrationId else None
        if source != "model_ellipsoid_display":
            if not calibration:
                raise HTTPException(422, "真实坐标输入必须选择校准方案，用其逆变换映射到模型；仍仅可模拟执行")
            converted = []
            for p in snapshot["trajectorySamples"]:
                try:
                    ellipsoid = convert_height(p, source, "ellipsoid", request.takeoffEllipsoidHeightMeters, request.geoidUndulationMeters)
                    geo = apply_calibration(ellipsoid, calibration, inverse=True)
                except ValueError as exc:
                    raise HTTPException(422, str(exc)) from None
                converted.append({**p, **geo})
            snapshot["trajectorySamples"] = converted
            snapshot["heightReference"] = "model_ellipsoid_display"
            try:
                MissionCreate.model_validate({k:v for k,v in snapshot.items() if k not in ("version","calibrationStatus")})
            except ValidationError as exc:
                raise HTTPException(422, "换算后轨迹不满足坐标/高度/采样约束："+str(exc)[:500]) from None
        snapshot["coordinateContext"] = {"horizontalDatum":"WGS84", "sourceHeightReference":source,
            "takeoffEllipsoidHeightMeters":request.takeoffEllipsoidHeightMeters,
            "geoidUndulationMeters":request.geoidUndulationMeters, "calibration":calibration,
            "modelDisplayAlignment":"unverified", "realFlightEnabled":False}
        with self.sessions() as db:
            mission = m.SimulationMission(id=str(uuid4()), owner_id=owner_id, device_id=request.deviceId,
                name=request.name.strip(), checksum=content_checksum(snapshot), snapshot=snapshot, created_at=utc_now())
            db.add(mission)
            db.commit()
            return self.summary(db, mission, True)

    def list(self, owner_id):
        with self.sessions() as db:
            missions = list(db.scalars(select(m.SimulationMission).options(defer(m.SimulationMission.snapshot))
                .where(m.SimulationMission.owner_id == owner_id)
                .order_by(m.SimulationMission.created_at.desc(),m.SimulationMission.id.desc()).limit(100)))
            context = self._summary_context(db,missions)
            return [self.summary(db,item,context=context) for item in missions]

    def get(self, owner_id, mission_id, include_snapshot=True):
        with self.sessions() as db:
            return self.summary(db,self.mission(db,owner_id,mission_id,include_snapshot),True,include_snapshot=include_snapshot)

    def validate(self, owner_id, mission_id):
        with self.sessions() as db:
            mission = self.mission(db, owner_id, mission_id)
            report = validate_snapshot(mission.snapshot, mission.checksum)
            db.add(m.MissionValidation(mission_id=mission.id, report=report))
            db.commit()
            return report

    def previous_command(self, owner_id, mission_id, command_id, action, run_id=None):
        """An HTTP retry must not run expensive validation or append another report."""
        with self.sessions() as db:
            self.mission(db,owner_id,mission_id,False)
            previous = db.get(m.SimulationCommand,command_id)
            if previous:
                self._check_command_identity(previous,owner_id,mission_id,action,run_id)
                return self.command_dict(previous)
            return None

    @staticmethod
    def _check_command_identity(previous, owner_id, mission_id, action, run_id):
        if previous.owner_id != owner_id or previous.mission_id != mission_id or previous.action != action or (
                run_id is not None and previous.run_id != run_id):
            raise HTTPException(409, "指令编号已被其他请求使用")

    def command(self, owner_id, mission_id, command_id, action, run_id=None, validation_report=None):
        self.registry.tick()
        with self.sessions() as db:
            mission = self.mission(db, owner_id, mission_id)
            previous = db.get(m.SimulationCommand, command_id)
            if previous:
                self._check_command_identity(previous,owner_id,mission_id,action,run_id)
                return self.command_dict(previous)
            if action == "PREPARE":
                report = validation_report if validation_report is not None else self.validate(owner_id,mission_id)
                if report.get("snapshotChecksum") != mission.checksum or not report.get("passed"):
                    raise HTTPException(409, "航线预检未通过：" + "；".join(report.get("errors",["快照不匹配"])))
            device = self.registry.devices.get((owner_id, mission.device_id))
            if not device or not device.gateway_connected or not device.aircraft_connected:
                raise HTTPException(409, "模拟设备未连接")
            if device.registration.source != "simulated":
                raise HTTPException(409, "DJI 接入当前只允许遥测，真机控制尚未验收")
            if "mission_simulation" not in device.registration.capabilities:
                raise HTTPException(409, "该网关未声明支持航线任务模拟，请更新并重启网关")
            if action in ("PREPARE", "START", "RESUME") and self.registry.is_stale(device):
                raise HTTPException(409, "位置数据过期，等待有效遥测后重试")
            queue = self.registry.command_queues[(owner_id, mission.device_id, device.session_id)]
            if queue.full():
                raise HTTPException(409, "设备指令队列已满")
            if action == "PREPARE":
                occupied = db.scalar(select(m.SimulationRun).where(m.SimulationRun.owner_id == owner_id,
                    m.SimulationRun.device_id == mission.device_id, m.SimulationRun.state.in_(ACTIVE)))
                if occupied:
                    raise HTTPException(409, "该设备已有活动任务，请先完成或终止")
                run = m.SimulationRun(id=str(uuid4()), mission_id=mission.id, owner_id=owner_id,
                    device_id=mission.device_id, session_id=device.session_id, state="preparing", progress=0,
                    elapsed_seconds=0, created_at=utc_now(), updated_at=utc_now())
                db.add(run)
                db.flush()
            else:
                run = db.get(m.SimulationRun, run_id)
                if not run or run.owner_id != owner_id or run.mission_id != mission_id:
                    raise HTTPException(404, "执行记录不存在")
                if run.session_id != device.session_id or run.state not in ALLOWED.get(action, ()):
                    raise HTTPException(409, "当前状态或网关会话不允许该操作，请刷新任务状态")
                if db.scalar(select(m.SimulationCommand.id).where(m.SimulationCommand.run_id == run.id,
                              m.SimulationCommand.status == "sent")):
                    raise HTTPException(409, "上一条指令尚未得到设备确认")
            command = m.SimulationCommand(id=command_id, owner_id=owner_id, mission_id=mission.id,
                run_id=run.id, action=action, status="sent", issued_at=utc_now(), expires_at=self.clock() + 10)
            db.add(command)
            self.event(db, run, "command_sent", action)
            db.commit()
            packet = {"schemaVersion": 1, "type": "mission_command", "commandId": command.id,
                      "missionId": mission.id, "runId": run.id, "action": action,
                      "gatewaySessionId": device.session_id, "checksum": mission.checksum,
                      "expiresAt": command.expires_at}
            if action == "PREPARE":
                packet["snapshot"] = mission.snapshot
                packet["canonicalSnapshot"] = canonical_snapshot(mission.snapshot)
            queue.put_nowait(packet)
            return self.command_dict(command)

    def acknowledge(self, owner_id, device_id, message):
        device = self.registry.ordered(owner_id, device_id, message)
        with self.sessions() as db:
            command = db.get(m.SimulationCommand, message.commandId)
            run = db.get(m.SimulationRun, message.runId)
            if not command or not run or command.owner_id != owner_id or run.owner_id != owner_id or (
                    command.run_id != run.id or run.device_id != device_id or run.session_id != device.session_id):
                raise RegistryError("command_identity_mismatch")
            mission = self.mission(db, owner_id, run.mission_id)
            if message.checksum != mission.checksum:
                raise RegistryError("mission_checksum_mismatch")
            device.last_sequence = message.sequence
            if command.status != "sent":
                return
            if self.clock() >= command.expires_at or run.state not in ACTIVE:
                raise RegistryError("command_expired")
            if message.accepted and message.state != EXPECTED[command.action]:
                raise RegistryError("unexpected_command_state")
            duration = mission.snapshot["trajectorySamples"][-1]["timeSeconds"]
            if message.accepted and message.elapsedSeconds is not None and not (
                    run.elapsed_seconds <= message.elapsedSeconds <= duration):
                raise RegistryError("invalid_command_progress")
            command.status = "succeeded" if message.accepted else "failed"
            command.reason = message.reason or None
            if message.accepted:
                run.state, run.reason, run.updated_at = message.state, None, utc_now()
                if message.elapsedSeconds is not None:
                    run.elapsed_seconds = message.elapsedSeconds
                    run.progress = min(1, message.elapsedSeconds / duration)
                if run.state == "aborted":
                    self.last_recorded.pop(run.id, None)
                    self.run_durations.pop(run.id, None)
                self.event(db, run, "state", message.state)
            else:
                self.event(db, run, "command_failed", message.reason or command.action)
                if command.action in ("PREPARE", "START"):
                    self.fail(db, run, message.reason or "设备拒绝任务")
            db.commit()

    def record(self, owner_id, device_id, session_id, telemetry):
        if not telemetry.missionRunId:
            return
        with self.sessions() as db:
            run = db.get(m.SimulationRun, telemetry.missionRunId)
            if not run or run.owner_id != owner_id or run.device_id != device_id or run.session_id != session_id:
                return
            if run.state not in ACTIVE:
                # Preserve the final acknowledged stop position once, without logging an endless hover.
                if run.state not in ("aborted", "completed") or telemetry.missionState != run.state:
                    return
                if self.terminal_recorded.get(run.id) == run.state:
                    return
                last = db.scalar(select(m.SimulationSample).where(m.SimulationSample.run_id == run.id)
                                 .order_by(m.SimulationSample.id.desc()).limit(1))
                if last and last.telemetry.get("missionState") == run.state:
                    self._terminal_seen(run.id,run.state)
                    return
                db.add(m.SimulationSample(run_id=run.id, received_at=utc_now(), telemetry=telemetry.model_dump(mode="json")))
                db.commit()
                self._terminal_seen(run.id,run.state)
                return
            if telemetry.missionState in ("failed","aborted"):
                self.fail(db,run,"执行端报告任务异常终止："+telemetry.missionState)
                db.add(m.SimulationSample(run_id=run.id,received_at=utc_now(),telemetry=telemetry.model_dump(mode="json")))
                db.commit()
                return
            if telemetry.missionState != run.state and not (run.state == "executing" and telemetry.missionState == "completed"):
                return
            elapsed = telemetry.missionElapsedSeconds
            if elapsed is None or elapsed < run.elapsed_seconds:
                return
            sample_key = (telemetry.missionState, round(elapsed * 2))
            if self.last_recorded.get(run.id) == sample_key:
                return
            duration = self.run_durations.get(run.id)
            if duration is None:
                mission = self.mission(db,owner_id,run.mission_id,False)
                duration = self._metadata(db,[mission])[mission.id]["durationSeconds"]
                self.run_durations[run.id] = duration
            if elapsed > duration + .01 or (telemetry.missionState == "completed" and elapsed < duration - 1e-6):
                return
            if run.state in ("ready","paused") and abs(elapsed-run.elapsed_seconds)>1e-6:
                return
            run.elapsed_seconds = elapsed
            run.progress = min(1, elapsed / duration)
            run.updated_at = utc_now()
            if telemetry.missionState == "completed" and run.state == "executing" and run.progress >= 0.999:
                run.state, run.progress = "completed", 1
                self.event(db, run, "state", "completed")
            db.add(m.SimulationSample(run_id=run.id, received_at=utc_now(), telemetry=telemetry.model_dump(mode="json")))
            self.last_recorded[run.id] = sample_key
            db.commit()
            if run.state == "completed":
                self.last_recorded.pop(run.id, None)
                self.run_durations.pop(run.id, None)
                self._terminal_seen(run.id,run.state)

    def _terminal_seen(self, run_id, state):
        self.terminal_recorded[run_id] = state
        self.terminal_recorded.move_to_end(run_id)
        while len(self.terminal_recorded)>1024:
            self.terminal_recorded.popitem(last=False)

    def lost(self, owner_id, device_id, session_id, reason="网关断开，执行中断；必须重新准备任务"):
        with self.sessions() as db:
            for run in db.scalars(select(m.SimulationRun).where(m.SimulationRun.owner_id == owner_id,
                    m.SimulationRun.device_id == device_id, m.SimulationRun.session_id == session_id,
                    m.SimulationRun.state.in_(ACTIVE))):
                self.fail(db, run, reason)
                self.last_recorded.pop(run.id, None)
            db.commit()

    def tick(self):
        lost = []
        with self.sessions() as db:
            expired = list(db.scalars(select(m.SimulationCommand).where(m.SimulationCommand.status == "sent",
                m.SimulationCommand.expires_at <= self.clock())))
            for command in expired:
                run = db.get(m.SimulationRun, command.run_id)
                if run and run.state in ACTIVE:
                    self.fail(db, run, "设备确认超时，会话已关闭，禁止自动续行")
                    lost.append((run.owner_id, run.device_id, run.session_id))
                else:
                    command.status, command.reason = "failed", "指令过期"
            db.commit()
        for identity in lost:
            self.registry.disconnect(*identity)

    def record_detail(self, owner_id, mission_id, run_id):
        with self.sessions() as db:
            mission = self.mission(db, owner_id, mission_id)
            run = db.get(m.SimulationRun, run_id)
            if not run or run.mission_id != mission.id:
                raise HTTPException(404, "执行记录不存在")
            samples = list(db.scalars(select(m.SimulationSample).where(m.SimulationSample.run_id == run.id)
                .order_by(m.SimulationSample.id.desc()).limit(10001)))
            truncated = len(samples) > 10000
            samples = list(reversed(samples[:10000]))
            return {"mission": self.summary(db, mission), "run": self.run_dict(db, run),
                "commands": [self.command_dict(c) for c in db.scalars(select(m.SimulationCommand).where(m.SimulationCommand.run_id == run.id))],
                "events": [{"at": e.at, "kind": e.kind, "detail": e.detail} for e in db.scalars(select(m.SimulationEvent)
                    .where(m.SimulationEvent.run_id == run.id).order_by(m.SimulationEvent.id).limit(1000))],
                "samples": [{"receivedAt": s.received_at, "telemetry": s.telemetry} for s in samples],
                "truncated": truncated}
