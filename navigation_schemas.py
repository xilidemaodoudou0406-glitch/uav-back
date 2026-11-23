"""Coordinate provenance, offline calibration and simulation validation contracts."""
from typing import Literal
from pydantic import BaseModel, ConfigDict, Field, model_validator


class FiniteModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class GeoPoint(FiniteModel):
    lat: float = Field(ge=-90, le=90)
    lng: float = Field(ge=-180, le=180)
    altitude: float = Field(ge=-1000, le=20000)


class HeightConversion(FiniteModel):
    point: GeoPoint
    source: Literal["ellipsoid", "takeoff_relative", "orthometric"]
    target: Literal["ellipsoid", "takeoff_relative", "orthometric"]
    takeoffEllipsoidHeightMeters: float | None = Field(default=None, ge=-1000, le=20000)
    geoidUndulationMeters: float | None = Field(default=None, ge=-200, le=200)


class ControlPoint(FiniteModel):
    name: str = Field(min_length=1, max_length=64)
    model: GeoPoint
    reference: GeoPoint


class CalibrationCreate(FiniteModel):
    name: str = Field(min_length=1, max_length=80)
    origin: GeoPoint
    dataSource: Literal["simulation", "survey_unverified"] = "simulation"
    controlPoints: list[ControlPoint] = Field(min_length=3, max_length=100)
    checkPoints: list[ControlPoint] = Field(min_length=2, max_length=100)

    @model_validator(mode="after")
    def independent_points(self):
        controls = {p.name for p in self.controlPoints}
        if len(controls) != len(self.controlPoints) or len({p.name for p in self.checkPoints}) != len(self.checkPoints):
            raise ValueError("点名不能重复")
        if controls & {p.name for p in self.checkPoints}:
            raise ValueError("检查点必须与拟合点独立")
        positions = {(p.model.lat, p.model.lng, p.model.altitude) for p in self.controlPoints}
        if positions & {(p.model.lat, p.model.lng, p.model.altitude) for p in self.checkPoints}:
            raise ValueError("检查点坐标不能重复使用拟合点")
        return self


class ErrorScenario(FiniteModel):
    eastBiasMeters: float = Field(default=0, ge=-100, le=100)
    northBiasMeters: float = Field(default=0, ge=-100, le=100)
    upBiasMeters: float = Field(default=0, ge=-100, le=100)
    driftMetersPerSecond: float = Field(default=0, ge=0, le=2)
    noiseAmplitudeMeters: float = Field(default=0, ge=0, le=10)
    telemetryDelayMs: int = Field(default=0, ge=0, le=10000)


class NoFlyBox(FiniteModel):
    minLat: float = Field(ge=-90, le=90)
    maxLat: float = Field(ge=-90, le=90)
    minLng: float = Field(ge=-180, le=180)
    maxLng: float = Field(ge=-180, le=180)
    minAlt: float = Field(ge=-1000, le=20000)
    maxAlt: float = Field(ge=-1000, le=20000)

    @model_validator(mode="after")
    def ordered(self):
        if self.minLat > self.maxLat or self.minLng > self.maxLng or self.minAlt > self.maxAlt:
            raise ValueError("禁飞区下界必须小于等于上界")
        return self


class ValidationSettings(FiniteModel):
    profileId: Literal["simulation", "mini3_lab"] = "simulation"
    horizontalMarginMeters: float = Field(default=2, ge=0, le=100)
    verticalMarginMeters: float = Field(default=2, ge=0, le=100)
    bodyRadiusMeters: float = Field(default=0.25, ge=0, le=5)
    modelErrorMeters: float = Field(default=0, ge=0, le=100)
    positionErrorMeters: float = Field(default=0, ge=0, le=100)
    trackingErrorMeters: float = Field(default=0, ge=0, le=100)
    latencySeconds: float = Field(default=0, ge=0, le=10)
    includeBrakingDistance: bool = True
    requireCoverage: bool = False
    obstacleDatasets: list[Literal["out2", "reconstruction1"]] = Field(default_factory=list, max_length=2)
    noFlyZones: list[NoFlyBox] = Field(default_factory=list, max_length=100)
