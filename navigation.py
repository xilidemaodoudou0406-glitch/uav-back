"""WGS84/ECEF/ENU and a constrained local horizontal similarity + vertical offset fit."""
import math
from obstacles.geodesy import geodetic_to_ecef, ecef_to_geodetic


class LocalFrame:
    def __init__(self, origin):
        self.origin = origin
        self.xyz = geodetic_to_ecef(origin["lat"], origin["lng"], origin["altitude"])
        lat, lng = math.radians(origin["lat"]), math.radians(origin["lng"])
        self.sl, self.cl, self.so, self.co = math.sin(lat), math.cos(lat), math.sin(lng), math.cos(lng)

    def to_enu(self, point):
        xyz = geodetic_to_ecef(point["lat"], point["lng"], point["altitude"])
        x, y, z = (xyz[i] - self.xyz[i] for i in range(3))
        return (-self.so*x + self.co*y, -self.sl*self.co*x - self.sl*self.so*y + self.cl*z,
                self.cl*self.co*x + self.cl*self.so*y + self.sl*z)

    def from_enu(self, value):
        e, n, u = value
        xyz = (self.xyz[0] - self.so*e - self.sl*self.co*n + self.cl*self.co*u,
               self.xyz[1] + self.co*e - self.sl*self.so*n + self.cl*self.so*u,
               self.xyz[2] + self.cl*n + self.sl*u)
        lat, lng, altitude = ecef_to_geodetic(*xyz)
        return {"lat": lat, "lng": lng, "altitude": altitude}


def convert_height(point, source, target, takeoff=None, geoid=None):
    height = point["altitude"]
    if source == target:
        return dict(point)
    if "takeoff_relative" in (source, target) and takeoff is None:
        raise ValueError("相对起飞高度换算必须提供已知的起飞点椭球高")
    if "orthometric" in (source, target) and geoid is None:
        raise ValueError("正高换算必须提供对应位置的大地水准面差 N，不能直接当作椭球高")
    if source == "takeoff_relative": height += takeoff
    elif source == "orthometric": height += geoid
    if target == "takeoff_relative": height -= takeoff
    elif target == "orthometric": height -= geoid
    return {**point, "altitude": height}


def apply_calibration(point, calibration, inverse=False):
    frame = LocalFrame(calibration["origin"])
    e, n, u = frame.to_enu(point)
    t = calibration["transform"]
    a, b = t["a"], t["b"]
    if inverse:
        x, y = e - t["eastMeters"], n - t["northMeters"]
        d = a*a + b*b
        value = ((a*x+b*y)/d, (-b*x+a*y)/d, u-t["upMeters"])
    else:
        value = (a*e-b*n+t["eastMeters"], b*e+a*n+t["northMeters"], u+t["upMeters"])
    return frame.from_enu(value)


def fit_calibration(request):
    data = request.model_dump(mode="json")
    frame = LocalFrame(data["origin"])
    pairs = [(frame.to_enu(p["model"]), frame.to_enu(p["reference"])) for p in data["controlPoints"]]
    count = len(pairs)
    x, y = [sum(p[0][i] for p in pairs)/count for i in (0, 1)]
    u, v = [sum(p[1][i] for p in pairs)/count for i in (0, 1)]
    xx = sum((p[0][0]-x)**2 for p in pairs)
    yy = sum((p[0][1]-y)**2 for p in pairs)
    xy = sum((p[0][0]-x)*(p[0][1]-y) for p in pairs)
    denominator = xx + yy
    if denominator < 1 or xx*yy - xy*xy < denominator*denominator*1e-6:
        raise ValueError("控制点过近或近似共线，不能可靠拟合；请分散布点")
    a = sum((p[0][0]-x)*(p[1][0]-u)+(p[0][1]-y)*(p[1][1]-v) for p in pairs)/denominator
    b = sum((p[0][0]-x)*(p[1][1]-v)-(p[0][1]-y)*(p[1][0]-u) for p in pairs)/denominator
    scale = math.hypot(a, b)
    if not 0.9 <= scale <= 1.1:
        raise ValueError("拟合尺度偏差超过 10%，请先检查坐标系和单位")
    data["transform"] = {"a": a, "b": b, "eastMeters": u-a*x+b*y, "northMeters": v-b*x-a*y,
        "upMeters": sum(p[1][2]-p[0][2] for p in pairs)/count,
        "scale": scale, "yawDegrees": math.degrees(math.atan2(b, a))}
    residuals = []
    for point in data["checkPoints"]:
        predicted = frame.to_enu(apply_calibration(point["model"], data))
        actual = frame.to_enu(point["reference"])
        residuals.append({"name": point["name"], "horizontalMeters": math.hypot(predicted[0]-actual[0], predicted[1]-actual[1]),
                          "verticalMeters": abs(predicted[2]-actual[2])})
    data["report"] = {"checkPoints": residuals,
        "horizontalRmseMeters": math.sqrt(sum(p["horizontalMeters"]**2 for p in residuals)/len(residuals)),
        "verticalRmseMeters": math.sqrt(sum(p["verticalMeters"]**2 for p in residuals)/len(residuals)),
        "status": "computed_not_field_verified", "method": "local_horizontal_similarity_and_vertical_offset"}
    return data
