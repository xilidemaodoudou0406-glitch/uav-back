"""Deterministic local playback for mock telemetry, not a vehicle dynamics model."""
from __future__ import annotations

import bisect
import hashlib
import json
import math
import time


def checksum(snapshot):
    return hashlib.sha256(json.dumps(snapshot, sort_keys=True, separators=(",", ":"),
        ensure_ascii=False, allow_nan=False).encode("utf-8")).hexdigest()


class SimulatedMission:
    def __init__(self, clock=time.monotonic, wall=time.time):
        self.clock, self.wall = clock, wall
        self.snapshot = None
        self.run_id = self.digest = None
        self.state = "idle"
        self.elapsed = 0
        self.anchor = 0
        self.results = {}
        self.times = ()

    def _set_snapshot(self, snapshot):
        self.snapshot = snapshot
        self.times = tuple(p["timeSeconds"] for p in snapshot["trajectorySamples"])

    def current_time(self):
        if self.snapshot and self.state == "executing":
            end = self.snapshot["trajectorySamples"][-1]["timeSeconds"]
            self.elapsed = min(end, self.clock() - self.anchor)
            if self.elapsed >= end:
                self.state = "completed"
        return self.elapsed

    def handle(self, packet, session_id):
        identity = (packet["runId"], packet["commandId"])
        if identity in self.results:
            return self.results[identity]
        accepted, reason = False, "当前状态不允许该操作"
        self.current_time()
        if packet["gatewaySessionId"] != session_id or packet["expiresAt"] <= self.wall():
            reason = "指令会话无效或已过期"
        elif packet["action"] == "PREPARE":
            data = packet.get("snapshot")
            if packet.get("canonicalSnapshot"):
                canonical = packet["canonicalSnapshot"]
                if hashlib.sha256(canonical.encode("utf-8")).hexdigest() != packet["checksum"]:
                    data = None
                else:
                    data = json.loads(canonical)
            if self.state in ("ready", "executing", "paused"):
                reason = "已有活动模拟任务"
            elif not data or data.get("executionScope") != "simulation_only" or checksum(data) != packet["checksum"]:
                reason = "任务范围或内容摘要无效"
            else:
                # Validate the received route independently before marking it ready.
                import sys
                from pathlib import Path
                parent = str(Path(__file__).resolve().parents[1])
                if parent not in sys.path:
                    sys.path.insert(0, parent)
                from mission_schemas import MissionCreate
                MissionCreate.model_validate({k: v for k, v in data.items() if k not in ("version", "calibrationStatus", "coordinateContext")})
                self._set_snapshot(data)
                self.run_id, self.digest = packet["runId"], packet["checksum"]
                self.state, self.elapsed = "ready", 0
                accepted, reason = True, ""
        elif packet["runId"] != self.run_id or packet["checksum"] != self.digest:
            reason = "任务版本或执行记录不匹配"
        elif packet["action"] == "START" and self.state == "ready":
            self.anchor = self.clock()
            self.state, accepted, reason = "executing", True, ""
        elif packet["action"] == "PAUSE" and self.state == "executing":
            self.state, accepted, reason = "paused", True, ""
        elif packet["action"] == "RESUME" and self.state == "paused":
            self.anchor = self.clock() - self.elapsed
            self.state, accepted, reason = "executing", True, ""
        elif packet["action"] == "ABORT" and self.state in ("ready", "executing", "paused"):
            self.state, accepted, reason = "aborted", True, ""
        result = {"type": "command_result", "commandId": packet["commandId"], "runId": packet["runId"],
            "checksum": packet["checksum"], "accepted": accepted,
            "state": self.state if self.state != "idle" else "failed", "reason": reason,
            "elapsedSeconds": self.current_time()}
        self.results[identity] = result
        # Dedup cache is bounded; server idempotency remains persistent.
        if len(self.results) > 512:
            del self.results[next(iter(self.results))]
        return result

    def disconnect(self):
        self.current_time()
        if self.state in ("ready", "executing", "paused"):
            self.state = "failed"

    def telemetry(self, base):
        if not self.snapshot:
            return base
        elapsed = self.current_time()
        points = self.snapshot["trajectorySamples"]
        index = max(0, min(len(points) - 2, bisect.bisect_right(self.times, elapsed) - 1))
        a, b = points[index:index + 2]
        fraction = min(1, max(0, (elapsed - a["timeSeconds"]) / (b["timeSeconds"] - a["timeSeconds"])))
        delta_lng = (b["lng"] - a["lng"] + 180) % 360 - 180
        lat = a["lat"] + (b["lat"] - a["lat"]) * fraction
        lng = (a["lng"] + delta_lng * fraction + 180) % 360 - 180
        altitude = a["altitude"] + (b["altitude"] - a["altitude"]) * fraction
        north = (b["lat"] - a["lat"]) * 111320
        east = delta_lng * 111320 * math.cos(math.radians(lat))
        speed = math.sqrt(north*north + east*east + (b["altitude"] - a["altitude"])**2) / (b["timeSeconds"] - a["timeSeconds"])
        base.update(lat=lat, lng=lng, heightEllipsoidMeters=altitude,
            heightRelativeTakeoffMeters=altitude - points[0]["altitude"],
            speedMps=speed if self.state == "executing" else 0,
            headingDegrees=math.degrees(math.atan2(east, north)) % 360,
            missionRunId=self.run_id, missionState=self.state, missionElapsedSeconds=elapsed)
        scenario = self.snapshot.get("errorScenario", {})
        if any(scenario.get(key,0) for key in ("eastBiasMeters","northBiasMeters","upBiasMeters","driftMetersPerSecond","noiseAmplitudeMeters")):
            from navigation import LocalFrame
            noise = scenario.get("noiseAmplitudeMeters",0)
            e = scenario.get("eastBiasMeters",0)+scenario.get("driftMetersPerSecond",0)*elapsed+noise*math.sin(elapsed*1.7)
            n = scenario.get("northBiasMeters",0)+noise*math.sin(elapsed*2.3+.5)
            u = scenario.get("upBiasMeters",0)+noise*math.sin(elapsed*.9+.2)
            perturbed = LocalFrame({"lat":lat,"lng":lng,"altitude":altitude}).from_enu((e,n,u))
            base.update(lat=perturbed["lat"],lng=perturbed["lng"],heightEllipsoidMeters=perturbed["altitude"],
                        heightRelativeTakeoffMeters=perturbed["altitude"]-points[0]["altitude"])
        return base
