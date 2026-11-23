from fastapi import APIRouter, Depends, HTTPException
from auth import get_current_user
from navigation_schemas import CalibrationCreate, HeightConversion
from navigation import convert_height
from services.navigation_service import NavigationService, calibration_example
from services.mission_validation import PROFILES, dataset_catalog

router = APIRouter(prefix="/api/navigation", tags=["navigation assurance"])
service = NavigationService()


@router.get("/catalog")
def catalog(user=Depends(get_current_user)):
    return {"profiles": PROFILES, "datasets": dataset_catalog(), "realFlightEnabled": False}


@router.get("/example")
def example(user=Depends(get_current_user)):
    return calibration_example()


@router.get("/calibrations")
def list_calibrations(user=Depends(get_current_user)):
    return service.list(user.id)


@router.post("/calibrations", status_code=201)
def calibrate(payload: CalibrationCreate, user=Depends(get_current_user)):
    return service.create(user.id, payload)


@router.post("/height/convert")
def height(payload: HeightConversion, user=Depends(get_current_user)):
    try:
        point = convert_height(payload.point.model_dump(), payload.source, payload.target,
            payload.takeoffEllipsoidHeightMeters, payload.geoidUndulationMeters)
        return {"point": point, "heightReference": payload.target, "status": "computed_not_field_verified"}
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from None
