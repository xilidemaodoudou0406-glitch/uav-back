"""Version 1: simulated mission gateways and read-only DJI telemetry gateways."""
from __future__ import annotations

from datetime import datetime
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class Message(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    schemaVersion: Literal[1] = 1


class Authenticate(Message):
    type: Literal["authenticate"]
    token: str = Field(min_length=1, max_length=4096)


class RegisterDevice(Message):
    type: Literal["register"]
    deviceId: str = Field(pattern=r"^[a-zA-Z0-9_-]{1,64}$")
    name: str = Field(min_length=1, max_length=64)
    source: Literal["simulated", "dji"]
    model: Literal["Mini 3 接入模拟器", "DJI Mini 3"] = "Mini 3 接入模拟器"
    capabilities: list[Literal["mission_simulation"]] = Field(default_factory=list, max_length=1)

    @model_validator(mode="after")
    def scope(self):
        if self.source == "dji" and (self.model != "DJI Mini 3" or self.capabilities):
            raise ValueError("DJI 网关当前只允许 Mini 3 遥测，不得声明模拟控制能力")
        return self


class SessionMessage(Message):
    gatewaySessionId: str = Field(min_length=1, max_length=64)
    sequence: int = Field(ge=0, strict=True)


class Heartbeat(SessionMessage):
    type: Literal["heartbeat"]
    aircraftConnected: bool = True


class Telemetry(SessionMessage):
    type: Literal["telemetry"]
    capturedAt: datetime
    sourceAgeMs: float = Field(ge=0, le=60_000)
    horizontalDatum: Literal["WGS84"] = "WGS84"
    positionValid: bool
    lat: float | None = Field(default=None, ge=-90, le=90)
    lng: float | None = Field(default=None, ge=-180, le=180)
    heightEllipsoidMeters: float | None = Field(default=None, ge=-1000, le=20_000)
    heightRelativeTakeoffMeters: float | None = Field(default=None, ge=-1000, le=20_000)
    heightSource: Literal["simulation", "dji_relative", "unavailable"]
    speedMps: float | None = Field(default=None, ge=0, le=100)
    headingDegrees: float | None = Field(default=None, ge=0, lt=360)
    batteryPercent: float | None = Field(default=None, ge=0, le=100)
    positioningQuality: Literal["simulated", "gnss", "unavailable"]
    flightMode: str = Field(default="SIMULATED", max_length=64)
    missionRunId: str | None = Field(default=None, max_length=64)
    missionState: Literal["ready", "executing", "paused", "completed", "aborted", "failed"] | None = None
    missionElapsedSeconds: float | None = Field(default=None, ge=0, le=3600)

    @model_validator(mode="after")
    def validate_position(self):
        if self.capturedAt.tzinfo is None:
            raise ValueError("capturedAt must include a timezone")
        if self.positionValid and (self.lat is None or self.lng is None):
            raise ValueError("valid position requires latitude and longitude")
        if self.heightSource == "unavailable" and (
            self.heightEllipsoidMeters is not None or self.heightRelativeTakeoffMeters is not None
        ):
            raise ValueError("unavailable height must be null")
        if self.heightSource == "dji_relative" and self.heightEllipsoidMeters is not None:
            raise ValueError("未核对基准的 DJI 相对高度不能冒充椭球高")
        return self
