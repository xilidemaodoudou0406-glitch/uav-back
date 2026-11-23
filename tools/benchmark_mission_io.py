"""Repeatable, isolated mission I/O benchmark; never opens the user's app.db."""
import argparse
from datetime import datetime, timezone
import inspect
import json
from pathlib import Path
import statistics
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sqlalchemy import create_engine,event
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
import models as m
from drone_schemas import Telemetry
from services.drone_registry import DroneRegistry
from services.mission_service import MissionService,content_checksum
from tools.simulated_mission import SimulatedMission


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    engine=create_engine('sqlite://',poolclass=StaticPool,connect_args={'check_same_thread':False})
    sessions=sessionmaker(bind=engine);m.Base.metadata.create_all(engine)
    points=[dict(lat=34.214+i*1e-8,lng=117.134,altitude=80,timeSeconds=float(i)) for i in range(1800)]
    snapshot=dict(name='benchmark',deviceId='bench',executionScope='simulation_only',heightReference='model_ellipsoid_display',trajectorySamples=points)
    now=datetime.now(timezone.utc).isoformat()
    with sessions() as db:
        for i in range(100):
            identity=f'm{i:03}'
            db.add(m.SimulationMission(id=identity,owner_id=1,device_id='bench',name=identity,checksum=content_checksum(snapshot),snapshot=snapshot,created_at=now))
            db.add(m.SimulationRun(id=f'r{i:03}',mission_id=identity,owner_id=1,device_id='bench',session_id='s',state='completed',progress=1,elapsed_seconds=1799,created_at=now,updated_at=now))
            db.add(m.SimulationCommand(id=f'c{i:03}',owner_id=1,mission_id=identity,run_id=f'r{i:03}',action='START',status='succeeded',issued_at=now,expires_at=1))
            db.add(m.MissionValidation(mission_id=identity,report={'passed':True,'errors':[],'warnings':[]}))
        db.commit()
    service=MissionService(DroneRegistry(),sessions)
    statements=[]
    def trace(conn,cursor,statement,parameters,context,many):statements.append(statement)
    event.listen(engine,'before_cursor_execute',trace)
    timings=[];counts=[]
    for _ in range(6):
        statements.clear();start=time.perf_counter();service.list(1);timings.append((time.perf_counter()-start)*1000);counts.append(len(statements))
    detail=service.get(1,'m000')
    compact=service.get(1,'m000',include_snapshot=False) if 'include_snapshot' in inspect.signature(service.get).parameters else detail
    with sessions() as db:
        run=db.get(m.SimulationRun,'r000');run.state='paused';run.progress=10/1799;run.elapsed_seconds=10;db.commit()
    telemetry=Telemetry(type='telemetry',gatewaySessionId='s',sequence=1,capturedAt=datetime.now(timezone.utc),sourceAgeMs=0,positionValid=True,
        lat=34.214,lng=117.134,heightEllipsoidMeters=80,heightSource='simulation',positioningQuality='simulated',missionRunId='r000',missionState='paused',missionElapsedSeconds=10)
    statements.clear();start=time.perf_counter()
    for _ in range(100):service.record(1,'bench','s',telemetry)
    paused={'milliseconds':(time.perf_counter()-start)*1000,'sqlStatements':len(statements),
        'writeStatements':sum(s.lstrip().upper().startswith(('INSERT','UPDATE','DELETE')) for s in statements)}
    long_points=[{**points[0],'timeSeconds':i/10,'lat':34.214+i*1e-9} for i in range(20000)]
    controller=SimulatedMission(clock=lambda:1)
    controller.snapshot={**snapshot,'trajectorySamples':long_points};controller.state='ready';controller.run_id='bench';controller.digest='x'
    # New versions cache prepared timestamps; this path intentionally also covers legacy setup.
    if hasattr(controller,'_set_snapshot'):controller._set_snapshot(controller.snapshot)
    start=time.perf_counter()
    for _ in range(1000):controller.telemetry({})
    result={'fixture':{'missions':100,'pointsPerMission':1800,'runsPerMission':1},
        'listColdSql':counts[0],'listWarmSql':counts[-1],'listColdMs':timings[0],'listWarmMedianMs':statistics.median(timings[1:]),
        'detailBytes':len(json.dumps(detail).encode()),'pollResponseBytes':len(json.dumps(compact).encode()),
        'paused100Frames':paused,'simulator20000Points1000FramesMs':(time.perf_counter()-start)*1000}
    Path(args.output).write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2));engine.dispose()


if __name__=='__main__':main()
