"""Immutable task snapshots, acknowledged commands, playback and interruption recovery."""
from __future__ import annotations

import unittest
from uuid import uuid4
from fastapi import HTTPException
from sqlalchemy import create_engine, event, select, func, inspect
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
from pydantic import ValidationError

import models
from drone_schemas import RegisterDevice, Telemetry, Heartbeat
from mission_schemas import MissionCreate, CommandResult
from services.drone_registry import DroneRegistry, RegistryError
from services.mission_service import MissionService, content_checksum
from tools.simulated_mission import SimulatedMission
from tests.test_drone_monitor import telemetry
from tests import test_drone_monitor as monitor_tests


def draft(device_id="sim-task"):
    return MissionCreate(name="测试航线", deviceId=device_id, trajectorySamples=[
        {"lat":34.214,"lng":117.134,"altitude":80,"timeSeconds":0},
        {"lat":34.2141,"lng":117.134,"altitude":85,"timeSeconds":5},
        {"lat":34.2141,"lng":117.1341,"altitude":80,"timeSeconds":10},
    ])


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.engine = create_engine("sqlite://", connect_args={"check_same_thread": False}, poolclass=StaticPool)
        models.Base.metadata.create_all(self.engine)
        self.sessions = sessionmaker(bind=self.engine)
        with self.sessions() as db:
            db.add(models.User(id=1, username="test", display_name="测试", password_hash="unused", active=True))
            db.commit()
        self.now = 100.0
        self.registry = DroneRegistry(clock=lambda: self.now)
        self.service = MissionService(self.registry, self.sessions, clock=lambda: self.now)
        self.registry.on_lost = self.service.lost
        self.registry.on_telemetry = self.service.record
        self.registry.on_tick = self.service.tick
        self.registration = RegisterDevice(type="register", deviceId="sim-task", name="任务模拟器",
                                          source="simulated", capabilities=["mission_simulation"])
        self.session = self.registry.register(1, self.registration)
        self.sequence = 0
        self.controller = SimulatedMission(clock=lambda: self.now, wall=lambda: self.now)
        self.frame()
        self.saved = self.service.create(1, draft())

    def tearDown(self):
        self.engine.dispose()

    def frame(self, seconds=0):
        self.now += seconds
        # Keep the independent heartbeat alive for controller-specific tests.
        self.registry.devices[(1, "sim-task")].heartbeat_at = self.now
        self.sequence += 1
        raw = self.controller.telemetry(telemetry(self.session, self.sequence))
        self.registry.update_telemetry(1, "sim-task", Telemetry.model_validate(raw))
        return raw

    def issue(self, action, acknowledge=True, command_id=None):
        latest = self.service.get(1, self.saved["id"])["latestRun"]
        result = self.service.command(1, self.saved["id"], command_id or str(uuid4()), action,
                                      latest["id"] if latest and action != "PREPARE" else None)
        packet = self.registry.command_queues[(1, "sim-task", self.session)].get_nowait()
        if acknowledge:
            response = self.controller.handle(packet, self.session)
            self.sequence += 1
            self.service.acknowledge(1, "sim-task", CommandResult(gatewaySessionId=self.session,
                                                               sequence=self.sequence, **response))
        return result, packet

    def state(self):
        return self.service.get(1, self.saved["id"])["latestRun"]

    def test_snapshot_is_immutable_and_has_explicit_uncalibrated_simulation_scope(self):
        request = draft()
        saved = self.service.create(1, request)
        request.trajectorySamples[0].altitude = 999
        restored = self.service.get(1, saved["id"])
        self.assertEqual(restored["snapshot"]["trajectorySamples"][0]["altitude"], 80)
        self.assertEqual(restored["checksum"], content_checksum(restored["snapshot"]))
        self.assertEqual(restored["calibrationStatus"], "unverified")
        self.assertEqual(restored["executionScope"], "simulation_only")

    def test_prepare_start_pause_resume_complete_and_persistent_record(self):
        self.issue("PREPARE")
        self.assertEqual(self.state()["state"], "ready")
        ready = self.frame(2)
        self.assertEqual(ready["missionElapsedSeconds"], 0)
        self.issue("START")
        moving = self.frame(2.5)
        self.issue("PAUSE")
        self.assertEqual(self.state()["elapsedSeconds"], 2.5)
        frozen = self.frame(3)
        self.assertEqual(moving["lat"], frozen["lat"])
        self.assertEqual(frozen["missionElapsedSeconds"], 2.5)
        self.assertEqual(self.state()["state"], "paused")
        self.issue("RESUME")
        final = self.frame(7.5)
        self.assertEqual(self.state()["state"], "completed")
        self.assertEqual(final["lat"], 34.2141)
        self.assertAlmostEqual(final["lng"], 117.1341)
        self.assertEqual(final["speedMps"], 0)
        record = self.service.record_detail(1, self.saved["id"], self.state()["id"])
        self.assertTrue(record["samples"])
        self.assertIn("paused", [e["detail"] for e in record["events"]])
        self.assertEqual(record["run"]["progress"], 1)
        restored_service = MissionService(self.registry, self.sessions, clock=lambda: self.now)
        self.assertEqual(restored_service.get(1, self.saved["id"])["latestRun"]["state"], "completed")

    def test_duplicate_command_is_not_sent_twice_and_active_task_conflicts(self):
        command_id = str(uuid4())
        first, _ = self.issue("PREPARE", command_id=command_id)
        duplicate = self.service.command(1, self.saved["id"], command_id, "PREPARE")
        self.assertEqual(first["commandId"], duplicate["commandId"])
        self.assertTrue(self.registry.command_queues[(1, "sim-task", self.session)].empty())
        other = self.service.create(1, draft())
        with self.assertRaises(HTTPException) as failure:
            self.service.command(1, other["id"], str(uuid4()), "PREPARE")
        self.assertEqual(failure.exception.status_code, 409)

    def test_abort_freezes_position_and_can_prepare_new_execution(self):
        self.issue("PREPARE"); self.issue("START")
        self.frame(2)
        self.issue("ABORT")
        a, b = self.frame(), self.frame(2)
        self.assertEqual(a["lat"], b["lat"])
        self.assertEqual(self.state()["state"], "aborted")
        self.assertNotIn(self.state()["id"], self.service.last_recorded)
        previous_run = self.state()["id"]
        self.issue("PREPARE")
        self.assertNotEqual(previous_run, self.state()["id"])

    def test_disconnect_and_session_replacement_do_not_resume(self):
        self.issue("PREPARE"); self.issue("START"); self.frame(1)
        previous = self.state()["id"]
        self.registry.disconnect(1, "sim-task", self.session)
        self.controller.disconnect()
        self.session = self.registry.register(1, self.registration)
        self.sequence = 0
        frame = self.frame(1)
        self.assertEqual(frame["missionState"], "failed")
        self.assertEqual(self.state()["state"], "failed")
        with self.assertRaises(HTTPException):
            self.service.command(1, self.saved["id"], str(uuid4()), "RESUME", previous)
        self.assertTrue(self.registry.command_queues[(1, "sim-task", self.session)].empty())

    def test_backend_restart_marks_active_runs_failed(self):
        self.issue("PREPARE")
        other = MissionService(self.registry, self.sessions, clock=lambda: self.now)
        self.assertEqual(other.get(1, self.saved["id"])["latestRun"]["state"], "failed")

    def test_command_timeout_closes_session_and_rejects_late_ack(self):
        _, packet = self.issue("PREPARE", acknowledge=False)
        self.assertEqual(self.state()["state"], "preparing")
        self.now += 11
        self.service.tick()
        self.assertEqual(self.state()["state"], "failed")
        self.assertFalse(self.registry.devices[(1, "sim-task")].gateway_connected)
        self.assertFalse(self.controller.handle(packet, self.session)["accepted"])

    def test_wrong_owner_checksum_and_stale_device_cannot_execute(self):
        with self.assertRaises(HTTPException) as failure:
            self.service.get(2, self.saved["id"])
        self.assertEqual(failure.exception.status_code, 404)
        _, packet = self.issue("PREPARE", acknowledge=False)
        response = self.controller.handle(packet, self.session)
        with self.assertRaises(RegistryError):
            self.service.acknowledge(1, "sim-task", CommandResult(gatewaySessionId=self.session, sequence=2,
                                    **{**response, "checksum":"0" * 64}))
        self.now += 4
        self.frame()
        self.now += 4
        with self.assertRaises(HTTPException):
            self.service.command(1, self.saved["id"], str(uuid4()), "PREPARE")

    def test_bad_time_nonfinite_coordinates_and_hardware_scope_are_rejected(self):
        for change in [{"executionScope":"hardware"}, {"heightReference":"relative_takeoff"},
                       {"trajectorySamples":[{"lat":0,"lng":0,"altitude":0,"timeSeconds":0}]*2}]:
            with self.assertRaises(ValidationError):
                MissionCreate.model_validate({**draft().model_dump(), **change})

    def test_aircraft_disconnect_invalidates_task_and_fix_without_dropping_gateway(self):
        self.issue("PREPARE");self.issue("START")
        self.sequence+=1
        self.registry.heartbeat(1,"sim-task",Heartbeat(type="heartbeat",gatewaySessionId=self.session,
            sequence=self.sequence,aircraftConnected=False))
        self.assertEqual(self.state()["state"],"failed")
        device=self.registry.snapshot(1)["devices"][0]
        self.assertTrue(device["gatewayConnected"]);self.assertTrue(device["telemetryStale"])
        self.sequence+=1
        self.registry.heartbeat(1,"sim-task",Heartbeat(type="heartbeat",gatewaySessionId=self.session,
            sequence=self.sequence,aircraftConnected=True))
        with self.assertRaises(HTTPException):self.service.command(1,self.saved["id"],"old-fix","PREPARE")

    def test_gateway_failure_telemetry_finishes_active_run(self):
        self.issue("PREPARE");self.issue("START");self.frame(1)
        self.controller.disconnect();self.frame()
        self.assertEqual(self.state()["state"],"failed")
        self.assertIn("执行端",self.state()["reason"])

    def test_premature_completion_cannot_round_up_missing_flight_time(self):
        self.issue("PREPARE");self.issue("START")
        self.controller.elapsed=9.995;self.controller.state="completed";self.frame()
        self.assertEqual(self.state()["state"],"executing")
        self.controller.elapsed=10;self.frame()
        self.assertEqual(self.state()["state"],"completed")

    def test_paused_hover_does_not_write_identical_samples_or_progress(self):
        self.issue("PREPARE");self.issue("START");self.frame(1);self.issue("PAUSE");self.frame()
        writes=[]
        def trace(conn,cursor,sql,params,context,many):
            if sql.lstrip().upper().startswith(("INSERT","UPDATE","DELETE")):writes.append(sql)
        event.listen(self.engine,"before_cursor_execute",trace)
        try:
            for _ in range(30):self.frame(.02)
        finally:event.remove(self.engine,"before_cursor_execute",trace)
        self.assertEqual(writes,[])
        self.assertEqual(self.state()["elapsedSeconds"],1)

    def test_status_poll_omits_geometry_but_preserves_run_and_validation(self):
        self.issue("PREPARE")
        full=self.service.get(1,self.saved["id"])
        status=self.service.get(1,self.saved["id"],include_snapshot=False)
        self.assertNotIn("snapshot",status)
        self.assertEqual(full["latestRun"],status["latestRun"])
        self.assertEqual(full["runs"],status["runs"])
        self.assertEqual(full["validation"],status["validation"])

    def test_batched_listing_query_count_does_not_grow_per_mission(self):
        for i in range(15):self.service.create(1,draft(f"other-{i}"))
        statements=[]
        def trace(conn,cursor,sql,params,context,many):statements.append(sql)
        event.listen(self.engine,"before_cursor_execute",trace)
        try:rows=self.service.list(1)
        finally:event.remove(self.engine,"before_cursor_execute",trace)
        self.assertEqual(len(rows),16)
        self.assertLessEqual(len(statements),6)
        self.assertEqual(self.service.list(2),[])

    def test_indexes_can_be_added_to_existing_tables_and_repeated_safely(self):
        for index in models.MISSION_QUERY_INDEXES:index.drop(bind=self.engine)
        models.ensure_mission_query_indexes(self.engine)
        models.ensure_mission_query_indexes(self.engine)
        names={row['name'] for row in inspect(self.engine).get_indexes('simulation_commands')}
        self.assertIn('ix_simulation_commands_status_expiry',names)


class MissionApiTests(unittest.TestCase):
    def setUp(self):
        monitor_tests.WebSocketTests.setUp(self)
        from routers.missions import router
        self.app.include_router(router)
        self.app.state.missions = MissionService(self.registry, self.sessions)
        self.registry.on_lost = self.app.state.missions.lost
        self.registry.on_telemetry = self.app.state.missions.record

    def tearDown(self):
        monitor_tests.WebSocketTests.tearDown(self)

    auth = monitor_tests.WebSocketTests.auth
    headers = monitor_tests.WebSocketTests.headers

    def test_http_task_and_gateway_command_roundtrip(self):
        self.assertEqual(self.client.get('/api/missions').status_code, 401)
        create = self.client.post('/api/missions', headers=self.headers(), json=draft().model_dump(mode='json'))
        self.assertEqual(create.status_code, 201)
        mission = create.json()
        self.assertEqual(self.client.get('/api/missions', headers=self.headers(2)).json(), [])
        with self.client.websocket_connect('/api/gateway/session') as socket:
            self.auth(socket)
            socket.send_json({'type':'register','deviceId':'sim-task','name':'测试','source':'simulated','capabilities':['mission_simulation']})
            session = socket.receive_json()['gatewaySessionId']
            socket.send_json(telemetry(session, 1))
            # Barrier: an invalid sample gets an immediate response after the valid sample.
            socket.send_json(telemetry(session, 2, lat=999)); socket.receive_json()
            result = self.client.post(f"/api/missions/{mission['id']}/prepare", headers=self.headers(), json={'commandId':'prepare-http'})
            self.assertEqual(result.status_code, 202)
            command = socket.receive_json()
            self.assertEqual(command['type'], 'mission_command')
            self.assertEqual(command['checksum'], content_checksum(command['snapshot']))
            socket.send_json({'type':'command_result', 'gatewaySessionId':session,'sequence':3,
                'runId':command['runId'],'commandId':command['commandId'],'checksum':command['checksum'],
                'accepted':True,'state':'ready'})
            socket.send_json(telemetry(session, 4, lat=999)); socket.receive_json()
            restored = self.client.get(f"/api/missions/{mission['id']}", headers=self.headers()).json()
            self.assertEqual(restored['latestRun']['state'], 'ready')
            compact = self.client.get(f"/api/missions/{mission['id']}?include_snapshot=false",headers=self.headers()).json()
            self.assertNotIn('snapshot',compact)
            retry = self.client.post(f"/api/missions/{mission['id']}/prepare",headers=self.headers(),json={'commandId':'prepare-http'})
            self.assertEqual(retry.status_code,202)
            with self.sessions() as db:self.assertEqual(db.scalar(select(func.count()).select_from(models.MissionValidation)),1)


if __name__ == '__main__':
    unittest.main()
