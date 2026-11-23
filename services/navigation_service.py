from uuid import uuid4
from fastapi import HTTPException
from sqlalchemy import select
import models as m
from database import SessionLocal
from navigation import fit_calibration, LocalFrame
from services.drone_registry import utc_now


class NavigationService:
    def __init__(self, sessions=SessionLocal):
        self.sessions = sessions

    def create(self, owner, request):
        try:
            payload = fit_calibration(request)
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from None
        with self.sessions() as db:
            row = m.NavigationCalibration(id=str(uuid4()), owner_id=owner, name=request.name,
                payload=payload, created_at=utc_now())
            db.add(row); db.commit()
            return self.serialize(row)

    @staticmethod
    def serialize(row):
        return {"id": row.id, "createdAt": row.created_at, **row.payload}

    def list(self, owner):
        with self.sessions() as db:
            return [self.serialize(row) for row in db.scalars(select(m.NavigationCalibration)
                .where(m.NavigationCalibration.owner_id == owner).order_by(m.NavigationCalibration.created_at.desc()).limit(100))]

    def get(self, owner, identity):
        with self.sessions() as db:
            row = db.get(m.NavigationCalibration, identity)
            if not row or row.owner_id != owner:
                raise HTTPException(404, "校准方案不存在")
            return self.serialize(row)


def calibration_example():
    origin = {"lat": 34.214, "lng": 117.134, "altitude": 80.0}
    frame = LocalFrame(origin)
    def point(name, e, n, u):
        return {"name": name, "model": frame.from_enu((e, n, u)),
                "reference": frame.from_enu((e+1.2, n-0.8, u+2.5))}
    return {"name": "模拟控制点示例（非实测）", "origin": origin, "dataSource": "simulation",
        "controlPoints": [point("C1", 0, 0, 0), point("C2", 50, 0, 2), point("C3", 0, 50, 4)],
        "checkPoints": [point("K1", 20, 10, 1), point("K2", -15, 30, 3)]}
