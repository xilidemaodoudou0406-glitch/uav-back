from fastapi import APIRouter, Depends, Request
from auth import get_current_user
from mission_schemas import MissionCreate, PrepareRequest, CommandRequest
from fastapi.concurrency import run_in_threadpool

router = APIRouter(prefix="/api/missions", tags=["simulation missions"])


@router.post("", status_code=201)
def create(payload: MissionCreate, request: Request, user=Depends(get_current_user)):
    return request.app.state.missions.create(user.id, payload)


@router.get("")
def list_missions(request: Request, user=Depends(get_current_user)):
    return request.app.state.missions.list(user.id)


@router.get("/{mission_id}")
def get(mission_id: str, request: Request, include_snapshot: bool = True, user=Depends(get_current_user)):
    return request.app.state.missions.get(user.id, mission_id, include_snapshot=include_snapshot)


@router.post("/{mission_id}/prepare", status_code=202)
async def prepare(mission_id: str, payload: PrepareRequest, request: Request, user=Depends(get_current_user)):
    previous = await run_in_threadpool(request.app.state.missions.previous_command,user.id,mission_id,payload.commandId,"PREPARE")
    if previous:
        return previous
    report = await run_in_threadpool(request.app.state.missions.validate, user.id, mission_id)
    return request.app.state.missions.command(user.id, mission_id, payload.commandId, "PREPARE", validation_report=report)


@router.post("/{mission_id}/validate")
async def validate(mission_id: str, request: Request, user=Depends(get_current_user)):
    return await run_in_threadpool(request.app.state.missions.validate, user.id, mission_id)


@router.post("/{mission_id}/commands", status_code=202)
async def command(mission_id: str, payload: CommandRequest, request: Request, user=Depends(get_current_user)):
    return request.app.state.missions.command(user.id, mission_id, payload.commandId, payload.action, payload.runId)


@router.get("/{mission_id}/runs/{run_id}")
def record(mission_id: str, run_id: str, request: Request, user=Depends(get_current_user)):
    return request.app.state.missions.record_detail(user.id, mission_id, run_id)
