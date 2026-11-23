"""User isolation, invalid/stale samples and session races, with an isolated test DB."""
from __future__ import annotations

import asyncio
import unittest
from contextlib import asynccontextmanager
from datetime import datetime, timedelta, timezone
from unittest.mock import patch

from fastapi import FastAPI
from fastapi.testclient import TestClient
from pydantic import ValidationError
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
from starlette.websockets import WebSocketDisconnect

import models
from auth import create_access_token
from database import get_db
from drone_schemas import Heartbeat, RegisterDevice, Telemetry
from routers.drones import router
from services.drone_registry import DroneRegistry, RegistryError


def registration(device_id="sim-test"):
    return RegisterDevice(type="register", deviceId=device_id, name="测试模拟器", source="simulated")


def telemetry(session, sequence=1, **changes):
    data = dict(schemaVersion=1, type="telemetry", gatewaySessionId=session, sequence=sequence,
        capturedAt=datetime.now(timezone.utc).isoformat(), sourceAgeMs=0, positionValid=True,
        horizontalDatum="WGS84", lat=34.214, lng=117.134, heightEllipsoidMeters=80,
        heightRelativeTakeoffMeters=80, heightSource="simulation", speedMps=3.5,
        headingDegrees=10, batteryPercent=98, positioningQuality="simulated")
    data.update(changes)
    return data


class RegistryTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.registry = DroneRegistry(clock=lambda: self.now)
        self.session = self.registry.register(1, registration())

    def send(self, sequence=1, **changes):
        self.registry.update_telemetry(1, "sim-test", Telemetry.model_validate(telemetry(self.session, sequence, **changes)))

    def state(self):
        return self.registry.snapshot(1)["devices"][0]

    def test_stale_positions_and_offline_heartbeats_are_distinct(self):
        self.send()
        self.assertFalse(self.state()["telemetryStale"])
        self.now += 3.1
        self.registry.heartbeat(1, "sim-test", Heartbeat(type="heartbeat", gatewaySessionId=self.session, sequence=2))
        self.registry.tick()
        self.assertTrue(self.state()["telemetryStale"])
        self.assertTrue(self.state()["gatewayConnected"])
        self.send(3)
        self.assertFalse(self.state()["telemetryStale"])
        self.now += 6.1
        self.registry.tick()
        self.assertFalse(self.state()["gatewayConnected"])

    def test_explicit_invalid_fix_immediately_marks_last_position_stale(self):
        self.send()
        with self.assertRaises(RegistryError):self.send(2,positionValid=False,lat=None,lng=None)
        self.assertTrue(self.state()["telemetryStale"])
        self.assertEqual(self.state()["telemetry"]["lat"],34.214)
        self.send(3)
        self.assertFalse(self.state()["telemetryStale"])

    def test_switching_source_does_not_keep_simulated_coordinates_on_a_dji_device(self):
        self.send()
        self.registry.register(1,RegisterDevice(type="register",deviceId="sim-test",name="DJI",source="dji",model="DJI Mini 3"))
        self.assertIsNone(self.state()["telemetry"])
        self.assertTrue(self.state()["telemetryStale"])

    def test_duplicate_and_old_samples_never_refresh_valid_position(self):
        self.send()
        before = self.state()["receivedAt"]
        self.now += 1
        for raw in [telemetry(self.session, 1), telemetry(self.session, 2, positionValid=False),
                    telemetry(self.session, 3, sourceAgeMs=3100),
                    telemetry(self.session, 4, capturedAt=(datetime.now(timezone.utc) - timedelta(days=1)).isoformat())]:
            with self.assertRaises(RegistryError):
                self.registry.update_telemetry(1, "sim-test", Telemetry.model_validate(raw))
        self.assertEqual(self.state()["receivedAt"], before)
        self.assertGreaterEqual(self.state()["telemetryAgeMs"], 1000)

    def test_invalid_floats_and_coordinates_are_rejected(self):
        for changes in [{"lat": 91}, {"lng": float("nan")}, {"heightEllipsoidMeters": float("inf")},
                        {"batteryPercent": 101}, {"sequence": True}, {"lat": None},
                        {"capturedAt": "2026-11-12T01:00:00"}]:
            with self.subTest(changes=changes), self.assertRaises(ValidationError):
                Telemetry.model_validate(telemetry(self.session, **changes))

    def test_replaced_session_cannot_disconnect_or_update_current(self):
        self.send()
        new_session = self.registry.register(1, registration())
        self.registry.disconnect(1, "sim-test", self.session)
        self.assertTrue(self.state()["gatewayConnected"])
        self.assertTrue(self.state()["telemetryStale"])
        with self.assertRaises(RegistryError):
            self.send(2)
        self.registry.update_telemetry(1, "sim-test", Telemetry.model_validate(telemetry(new_session, 0)))
        self.assertFalse(self.state()["telemetryStale"])
        self.assertEqual(len(self.registry.snapshot(1)["devices"]), 1)

    def test_user_isolation_and_bounded_slow_subscribers(self):
        other = self.registry.subscribe(2)
        slow = self.registry.subscribe(1)
        for _ in range(100):
            self.registry.publish(self.registry.devices[(1, "sim-test")])
        self.assertTrue(other.empty())
        self.assertLessEqual(slow.qsize(), slow.maxsize)
        self.assertEqual(self.registry.snapshot(2)["devices"], [])
        self.registry.unsubscribe(1, slow)
        self.assertNotIn(1, self.registry.subscribers)


class WebSocketTests(unittest.TestCase):
    def setUp(self):
        self.engine = create_engine("sqlite://", connect_args={"check_same_thread": False}, poolclass=StaticPool)
        models.Base.metadata.create_all(self.engine)
        self.sessions = sessionmaker(bind=self.engine)
        with self.sessions() as db:
            for i in (1, 2):
                db.add(models.User(id=i, username=f"test{i}", display_name=f"测试{i}", password_hash="unused", active=True))
            db.commit()
            self.tokens = {i: create_access_token(db.get(models.User, i))[0] for i in (1, 2)}
        self.registry = DroneRegistry()

        @asynccontextmanager
        async def lifespan(app):
            app.state.drone_registry = self.registry
            yield

        self.app = FastAPI(lifespan=lifespan)
        self.app.include_router(router)

        def db_override():
            with self.sessions() as db:
                yield db

        self.app.dependency_overrides[get_db] = db_override
        self.identity_patch = patch("routers.drones.SessionLocal", self.sessions)
        self.identity_patch.start()
        self.client_context = TestClient(self.app)
        self.client = self.client_context.__enter__()

    def tearDown(self):
        self.client_context.__exit__(None, None, None)
        self.identity_patch.stop()
        self.engine.dispose()

    def auth(self, socket, user=1):
        socket.send_json({"schemaVersion": 1, "type": "authenticate", "token": self.tokens[user]})
        self.assertEqual(socket.receive_json()["type"], "authenticated")

    def register(self, socket):
        socket.send_json(registration().model_dump())
        return socket.receive_json()["gatewaySessionId"]

    def headers(self, user=1):
        return {"Authorization": "Bearer " + self.tokens[user]}

    def test_http_and_stream_require_authentication(self):
        self.assertEqual(self.client.get("/api/drones").status_code, 401)
        for endpoint in ["/api/drones/stream", "/api/gateway/session"]:
            with self.subTest(endpoint=endpoint), self.client.websocket_connect(endpoint) as socket:
                socket.send_json({"schemaVersion": 1, "type": "authenticate", "token": "invalid"})
                with self.assertRaises(WebSocketDisconnect) as closed:
                    socket.receive_json()
                self.assertEqual(closed.exception.code, 4401)

    def test_end_to_end_register_update_disconnect_and_identity_isolation(self):
        with self.client.websocket_connect("/api/drones/stream") as viewer:
            self.auth(viewer)
            self.assertEqual(viewer.receive_json()["devices"], [])
            with self.client.websocket_connect("/api/gateway/session") as gateway:
                self.auth(gateway)
                session = self.register(gateway)
                self.assertEqual(viewer.receive_json()["device"]["source"], "simulated")
                gateway.send_json(telemetry(session))
                event = viewer.receive_json()
                self.assertFalse(event["device"]["telemetryStale"])
                self.assertEqual(event["device"]["telemetry"]["lat"], 34.214)
                self.assertEqual(self.client.get("/api/drones", headers=self.headers(2)).json()["devices"], [])
                self.assertEqual(self.client.get("/api/drones/sim-test/status", headers=self.headers(2)).status_code, 404)
                gateway.send_json(telemetry(session, 2, lat=999))
                self.assertEqual(gateway.receive_json()["code"], "invalid_message")
                self.assertEqual(self.client.get("/api/drones/sim-test/status", headers=self.headers()).json()["telemetry"]["lat"], 34.214)
            self.assertFalse(viewer.receive_json()["device"]["gatewayConnected"])
        self.assertEqual(self.registry.subscribers, {})

    def test_gateway_cannot_register_as_real_hardware(self):
        with self.client.websocket_connect("/api/gateway/session") as socket:
            self.auth(socket)
            socket.send_json({**registration().model_dump(), "source": "real"})
            with self.assertRaises(WebSocketDisconnect):
                socket.receive_json()
        self.assertEqual(self.client.get("/api/drones", headers=self.headers()).json()["devices"], [])

    def test_new_gateway_survives_previous_connection_cleanup(self):
        with self.client.websocket_connect("/api/gateway/session") as old:
            self.auth(old)
            first = self.register(old)
            with self.client.websocket_connect("/api/gateway/session") as new:
                self.auth(new)
                second = self.register(new)
                self.assertNotEqual(first, second)
                old.send_json(telemetry(first))
                with self.assertRaises(WebSocketDisconnect):
                    old.receive_json()
                new.send_json(telemetry(second))
                # The gateway's validation error is an ordering barrier for the previous sample.
                new.send_json(telemetry(second, 2, lat=999))
                new.receive_json()
                state = self.client.get("/api/drones/sim-test/status", headers=self.headers()).json()
                self.assertTrue(state["gatewayConnected"])
                self.assertFalse(state["telemetryStale"])


if __name__ == "__main__":
    unittest.main()
