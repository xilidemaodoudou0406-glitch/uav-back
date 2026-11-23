"""Datum ambiguity, independent checkpoints, swept clearance and preparation gate."""
import copy
import json
import math
import tempfile
import unittest
from array import array
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from fastapi import HTTPException
from pydantic import ValidationError
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
import models
from navigation import LocalFrame, convert_height, fit_calibration, apply_calibration
from navigation_schemas import CalibrationCreate, ValidationSettings
from mission_schemas import MissionCreate
from drone_schemas import RegisterDevice, Telemetry
from obstacles.surface_voxel import SurfaceVoxelTile, SurfaceVoxelDataset
from services.navigation_service import calibration_example, NavigationService
from services.mission_validation import validate_snapshot
from services.mission_service import MissionService
from services.drone_registry import DroneRegistry, RegistryError
from tests.test_drone_monitor import telemetry

ORIGIN={"lat":34.214,"lng":117.134,"altitude":80}


def route(start=(-5,2,0),end=(5,2,0),seconds=10,**settings):
    frame=LocalFrame(ORIGIN)
    payload=MissionCreate(name="预检路线",deviceId="sim-test",trajectorySamples=[
        {**frame.from_enu(start),"timeSeconds":0}, {**frame.from_enu(end),"timeSeconds":seconds}],
        validationSettings={"profileId":"mini3_lab","horizontalMarginMeters":0,"verticalMarginMeters":0,
                            "includeBrakingDistance":False,**settings})
    return payload.model_dump(mode="json")


class CoordinateTests(unittest.TestCase):
    def test_enu_roundtrip_at_high_latitude_and_dateline(self):
        for lat,lng in ((34,117),(80,179.999),(-65,-179.999),(90,0)):
            frame=LocalFrame({"lat":lat,"lng":lng,"altitude":123})
            p=frame.from_enu((20,-30,7))
            for actual,expected in zip(frame.to_enu(p),(20,-30,7)):
                self.assertAlmostEqual(actual,expected,places=5)

    def test_height_requires_explicit_takeoff_and_geoid(self):
        with self.assertRaises(ValueError):convert_height(ORIGIN,"takeoff_relative","ellipsoid")
        with self.assertRaises(ValueError):convert_height(ORIGIN,"orthometric","ellipsoid")
        self.assertEqual(convert_height(ORIGIN,"orthometric","ellipsoid",geoid=25)["altitude"],105)
        self.assertEqual(convert_height(ORIGIN,"takeoff_relative","orthometric",takeoff=100,geoid=25)["altitude"],155)

    def test_simulated_calibration_is_never_field_verified(self):
        fitted=fit_calibration(CalibrationCreate.model_validate(calibration_example()))
        self.assertEqual(fitted["report"]["status"],"computed_not_field_verified")
        self.assertLess(fitted["report"]["horizontalRmseMeters"],1e-5)
        self.assertAlmostEqual(fitted["transform"]["upMeters"],2.5,places=5)
        p=apply_calibration(ORIGIN,fitted)
        restored=apply_calibration(p,fitted,inverse=True)
        self.assertLess(math.dist(LocalFrame(ORIGIN).to_enu(restored),(0,0,0)),1e-5)

    def test_independent_checkpoints_and_collinear_controls(self):
        data=calibration_example();data["checkPoints"][0]=copy.deepcopy(data["controlPoints"][0])
        with self.assertRaises(ValidationError):CalibrationCreate.model_validate(data)
        data=calibration_example();f=LocalFrame(ORIGIN)
        for i,p in enumerate(data["controlPoints"]):p["model"]=f.from_enu((i*20,0,0))
        with self.assertRaises(ValueError):fit_calibration(CalibrationCreate.model_validate(data))

    def test_nan_and_inverted_zones_are_rejected(self):
        with self.assertRaises(ValidationError):ValidationSettings(modelErrorMeters=float("nan"))
        with self.assertRaises(ValidationError):ValidationSettings(noFlyZones=[dict(minLat=2,maxLat=1,minLng=1,maxLng=2,minAlt=0,maxAlt=1)])

    def test_grid_scale_tracks_wgs84_latitude(self):
        from algorithms.geo_utils import meters_per_degree
        equator=meters_per_degree(0);high=meters_per_degree(60)
        self.assertAlmostEqual(equator[1],111319.4908,places=3)
        self.assertTrue(.49<high[1]/equator[1]<.51)


class ValidationTests(unittest.TestCase):
    def test_limits_use_derived_velocity_not_claimed_speed(self):
        data=route(end=(50,2,0),seconds=2)
        report=validate_snapshot(data,"digest")
        self.assertFalse(report["passed"])
        self.assertTrue(any("水平速度" in e for e in report["errors"]))

    def test_turn_acceleration_and_vertical_limits(self):
        data=route(end=(0,0,10),seconds=2)
        self.assertTrue(any("垂直速度" in e for e in validate_snapshot(data,"digest")["errors"]))
        data=route(start=(-4,0,0),end=(0,0,0),seconds=1)
        data["trajectorySamples"].append({**LocalFrame(ORIGIN).from_enu((-4,0,0)),"timeSeconds":2})
        self.assertTrue(any("加速度" in e for e in validate_snapshot(data,"digest")["errors"]))

    def test_manual_zone_between_safe_endpoints_and_margin_near_miss(self):
        frame=LocalFrame(ORIGIN);a,b=frame.from_enu((-1,-1,-1)),frame.from_enu((1,1,1))
        zone=dict(minLat=a["lat"],maxLat=b["lat"],minLng=a["lng"],maxLng=b["lng"],minAlt=79,maxAlt=81)
        self.assertTrue(validate_snapshot(route(noFlyZones=[zone]),"digest")["passed"])
        report=validate_snapshot(route(noFlyZones=[zone],horizontalMarginMeters=2),"digest")
        self.assertFalse(report["passed"])
        self.assertTrue(any("包络" in e for e in report["errors"]))

    def test_injected_error_and_latency_expand_budget(self):
        data=route();data["errorScenario"]={"eastBiasMeters":3,"northBiasMeters":4,"upBiasMeters":2,
            "driftMetersPerSecond":.1,"noiseAmplitudeMeters":1,"telemetryDelayMs":1000}
        report=validate_snapshot(data,"digest")
        self.assertGreater(report["margins"]["horizontalMeters"],8)
        self.assertGreaterEqual(report["margins"]["verticalMeters"],3.25)
        self.assertFalse(report["realFlightEnabled"])

    def test_missing_selected_data_is_not_silently_ignored(self):
        fake=SimpleNamespace(enabled=False)
        report=validate_snapshot(route(obstacleDatasets=["out2"]),"digest",{"out2":lambda:fake})
        self.assertFalse(report["passed"])

    def test_coverage_requires_a_dataset(self):
        self.assertFalse(validate_snapshot(route(requireCoverage=True),"digest")["passed"])

    def test_corrupt_voxel_archive_returns_a_failed_report_instead_of_http_500(self):
        with tempfile.TemporaryDirectory() as root:
            root=Path(root);(root/"tile.npz").write_bytes(b"not a zip archive")
            metadata={"centerWgs84":[117.134,34.214,80],"referenceBboxWgs84":[117.133,34.213,117.135,34.215],"file":"tile.npz",
                "grid":{"resolutionMeters":1,"originEastMeters":-10,"originNorthMeters":-10,"originUpMeters":-10,"columns":20,"rows":20,"levels":20}}
            (root/"tiles.json").write_text(json.dumps({"schemaVersion":2,"tiles":[metadata]}))
            data=SurfaceVoxelDataset(root)
            report=validate_snapshot(route(obstacleDatasets=["out2"]),"digest",{"out2":lambda:data})
            self.assertFalse(report["passed"])
            self.assertTrue(any("障碍检查未完成" in e for e in report["errors"]))

    def test_changed_model_files_cannot_validate_against_cached_old_occupancy(self):
        with tempfile.TemporaryDirectory() as root:
            root=Path(root);(root/"tile.npz").write_bytes(b"initial")
            metadata={"centerWgs84":[117.134,34.214,80],"referenceBboxWgs84":[117.133,34.213,117.135,34.215],"file":"tile.npz",
                "grid":{"resolutionMeters":1,"originEastMeters":-10,"originNorthMeters":-10,"originUpMeters":-10,"columns":20,"rows":20,"levels":20}}
            (root/"tiles.json").write_text(json.dumps({"schemaVersion":2,"tiles":[metadata]}))
            data=SurfaceVoxelDataset(root)
            self.assertTrue(data.enabled)
            (root/"tile.npz").write_bytes(b"changed and differently sized")
            report=validate_snapshot(route(obstacleDatasets=["out2"]),"digest",{"out2":lambda:data})
            self.assertFalse(report["passed"])
            self.assertTrue(any("重新加载" in e for e in report["errors"]))

    def test_swept_voxel_margin_and_interior_collision(self):
        with tempfile.TemporaryDirectory() as root:
            root=Path(root);(root/"tile.npz").write_bytes(b"test");(root/"tiles.json").write_text("{}")
            metadata={"centerWgs84":[117.134,34.214,80],"referenceBboxWgs84":[117.133,34.213,117.135,34.215],"file":"tile.npz",
                "grid":{"resolutionMeters":1,"originEastMeters":-10,"originNorthMeters":-10,"originUpMeters":-10,"columns":20,"rows":20,"levels":20}}
            tile=SurfaceVoxelTile(metadata,root);tile.occupied=array("Q",[(10*20+10)*20+10])
            data=SimpleNamespace(enabled=True,tiles=[tile],manifest_path=root/"tiles.json",data_root=root)
            check=lambda draft:validate_snapshot(draft,"digest",{"out2":lambda:data})
            self.assertTrue(check(route(obstacleDatasets=["out2"]))["passed"])
            self.assertFalse(check(route(obstacleDatasets=["out2"],horizontalMarginMeters=2))["passed"])
            self.assertFalse(check(route(start=(-5,.5,.5),end=(5,.5,.5),obstacleDatasets=["out2"]))["passed"])

    def test_coverage_rejects_a_gap_between_two_covered_endpoints(self):
        with tempfile.TemporaryDirectory() as root:
            root=Path(root);(root/"tile.npz").write_bytes(b"test");(root/"tiles.json").write_text("{}")
            tiles=[]
            for west,east in ((117.133,117.1339),(117.1341,117.135)):
                tile=SurfaceVoxelTile({"centerWgs84":[117.134,34.214,80],"referenceBboxWgs84":[west,34.213,east,34.215],"file":"tile.npz",
                    "grid":{"resolutionMeters":1,"originEastMeters":-10,"originNorthMeters":-10,"originUpMeters":-10,"columns":20,"rows":20,"levels":20}},root)
                tile.occupied=array("Q");tiles.append(tile)
            data=SimpleNamespace(enabled=True,tiles=tiles,manifest_path=root/"tiles.json",data_root=root)
            report=validate_snapshot(route(start=(-50,0,5),end=(50,0,5),seconds=50,obstacleDatasets=["out2"],requireCoverage=True),"digest",{"out2":lambda:data})
            self.assertFalse(report["passed"])
            self.assertTrue(any("范围" in e for e in report["errors"]))


class PersistenceTests(unittest.TestCase):
    def setUp(self):
        self.engine=create_engine("sqlite://",connect_args={"check_same_thread":False},poolclass=StaticPool)
        models.Base.metadata.create_all(self.engine);self.sessions=sessionmaker(bind=self.engine)
        self.registry=DroneRegistry();self.service=MissionService(self.registry,self.sessions)
    def tearDown(self):self.engine.dispose()

    def test_prepare_gate_runs_even_when_client_skips_validate_endpoint(self):
        saved=self.service.create(1,MissionCreate.model_validate(route(end=(50,2,0),seconds=2)))
        with self.assertRaises(HTTPException) as result:self.service.command(1,saved["id"],"c1","PREPARE")
        self.assertIn("预检未通过",result.exception.detail)
        self.assertFalse(self.service.get(1,saved["id"])["validation"]["passed"])

    def test_real_coordinate_conversion_uses_immutable_owner_scoped_calibration(self):
        calibration=NavigationService(self.sessions).create(1,CalibrationCreate.model_validate(calibration_example()))
        data=route();data.update(heightReference="takeoff_relative",takeoffEllipsoidHeightMeters=50,calibrationId=calibration["id"])
        saved=self.service.create(1,MissionCreate.model_validate(data))
        self.assertEqual(saved["snapshot"]["heightReference"],"model_ellipsoid_display")
        self.assertAlmostEqual(saved["snapshot"]["trajectorySamples"][0]["altitude"],127.5,places=3)
        self.assertEqual(saved["coordinateContext"]["calibration"]["dataSource"],"simulation")
        with self.assertRaises(HTTPException):self.service.create(2,MissionCreate.model_validate(data))
        data["calibrationId"]=None
        with self.assertRaises(HTTPException):self.service.create(1,MissionCreate.model_validate(data))
        data.update(calibrationId=calibration["id"],takeoffEllipsoidHeightMeters=20000)
        with self.assertRaises(HTTPException) as result:self.service.create(1,MissionCreate.model_validate(data))
        self.assertIn("换算后轨迹",result.exception.detail)

    def test_dji_telemetry_is_read_only_and_relative_height_is_not_ellipsoid(self):
        with self.assertRaises(ValidationError):RegisterDevice(type="register",deviceId="dji-1",name="DJI",source="dji",model="DJI Mini 3",capabilities=["mission_simulation"])
        session=self.registry.register(1,RegisterDevice(type="register",deviceId="dji-1",name="DJI",source="dji",model="DJI Mini 3"))
        sample=telemetry(session,heightEllipsoidMeters=None,heightRelativeTakeoffMeters=30,heightSource="dji_relative",positioningQuality="gnss",flightMode="P_GPS")
        self.registry.update_telemetry(1,"dji-1",Telemetry.model_validate(sample))
        self.assertEqual(self.registry.snapshot(1)["devices"][0]["source"],"dji")
        with self.assertRaises(ValidationError):Telemetry.model_validate({**sample,"heightEllipsoidMeters":30})
        with self.assertRaises(RegistryError):self.registry.update_telemetry(1,"dji-1",Telemetry.model_validate(telemetry(session,sequence=2)))
