"""Immutable, explicitly uncalibrated simulation missions; never hardware commands."""
from __future__ import annotations

from typing import Literal
import math
from pydantic import BaseModel, ConfigDict, Field, model_validator
from drone_schemas import SessionMessage
from obstacles.geodesy import geodetic_to_ecef
from navigation_schemas import ValidationSettings, ErrorScenario


class TimedPoint(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    lat: float = Field(ge=-90, le=90)
    lng: float = Field(ge=-180, le=180)
    altitude: float = Field(ge=-1000, le=20000)
    timeSeconds: float = Field(ge=0, le=3600)
    speedMetersPerSecond: float = Field(default=0, ge=0, le=100)
    phase: Literal["takeoff", "cruise", "landing"] = "cruise"


class MissionCreate(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = Field(min_length=1, max_length=80)
    deviceId: str = Field(pattern=r"^[a-zA-Z0-9_-]{1,64}$")
    executionScope: Literal["simulation_only"] = "simulation_only"
    heightReference: Literal["model_ellipsoid_display", "ellipsoid", "takeoff_relative", "orthometric"] = "model_ellipsoid_display"
    calibrationId: str | None = Field(default=None, max_length=64)
    takeoffEllipsoidHeightMeters: float | None = Field(default=None, ge=-1000, le=20000)
    geoidUndulationMeters: float | None = Field(default=None, ge=-200, le=200)
    validationSettings: ValidationSettings = Field(default_factory=ValidationSettings)
    errorScenario: ErrorScenario = Field(default_factory=ErrorScenario)
    trajectorySamples: list[TimedPoint] = Field(min_length=2, max_length=20000)

    @model_validator(mode="after")
    def ordered_time(self):
        if not self.name.strip():
            raise ValueError("任务名称不能为空")
        if self.trajectorySamples[0].timeSeconds != 0:
            raise ValueError("轨迹时间必须从 0 开始")
        if any(b.timeSeconds <= a.timeSeconds for a, b in zip(self.trajectorySamples, self.trajectorySamples[1:])):
            raise ValueError("轨迹时间必须严格递增")
        for a, b in zip(self.trajectorySamples, self.trajectorySamples[1:]):
            distance = math.dist(geodetic_to_ecef(a.lat, a.lng, a.altitude), geodetic_to_ecef(b.lat, b.lng, b.altitude))
            if distance / (b.timeSeconds - a.timeSeconds) > 90:
                raise ValueError("相邻采样的位移速度超出模拟器范围")
        return self


class PrepareRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    commandId: str = Field(pattern=r"^[a-zA-Z0-9_-]{1,64}$")


class CommandRequest(PrepareRequest):
    runId: str = Field(min_length=1, max_length=64)
    action: Literal["START", "PAUSE", "RESUME", "ABORT"]


class CommandResult(SessionMessage):
    type: Literal["command_result"]
    commandId: str = Field(min_length=1, max_length=64)
    runId: str = Field(min_length=1, max_length=64)
    checksum: str = Field(min_length=64, max_length=64)
    accepted: bool
    state: Literal["ready", "executing", "paused", "aborted", "completed", "failed"]
    reason: str = Field(default="", max_length=240)
    elapsedSeconds: float | None = Field(default=None, ge=0, le=3600)
