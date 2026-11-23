"""Shared, bounded path post-processing and simulation time parameterization.

The returned polyline is the geometry the client must display and play. Collision
validation is with respect to the existing obstacle representation, not a flight
certificate. In particular a sampled speed profile is not a full vehicle model.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from obstacles.geodesy import ecef_to_geodetic, geodetic_to_ecef

EPS = 1e-6
CURVE_TOLERANCE = 0.03  # metres in the local frame, not obstacle inflation
CURVE_STEP = 1.5
MAX_POINTS = 20_000
MAX_CURVE_POINTS = 2048
Vec = tuple[float, float, float]


def _point(p: dict) -> dict:
    result = {k: float(p[k]) for k in ("lat", "lng", "altitude")}
    if not all(math.isfinite(v) for v in result.values()):
        raise ValueError("Trajectory contains a non-finite coordinate")
    return result


def _xyz(p: dict) -> Vec:
    return geodetic_to_ecef(p["lat"], p["lng"], p["altitude"])


def _sub(a: Vec, b: Vec) -> Vec:
    return tuple(a[i] - b[i] for i in range(3))


def _add(a: Vec, b: Vec, scale: float = 1.0) -> Vec:
    return tuple(a[i] + scale * b[i] for i in range(3))


def _dot(a: Vec, b: Vec) -> float:
    return sum(a[i] * b[i] for i in range(3))


def _norm(a: Vec) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Vec) -> Vec:
    length = _norm(a)
    return tuple(v / length for v in a) if length > EPS else (0.0, 0.0, 0.0)


def _distance(a: dict, b: dict) -> float:
    return math.dist(_xyz(a), _xyz(b))


def _lerp(a: dict, b: dict, t: float) -> dict:
    return {k: a[k] + t * (b[k] - a[k]) for k in ("lat", "lng", "altitude")}


def _dedup(path: list[dict]) -> list[dict]:
    result: list[dict] = []
    for item in path:
        p = _point(item)
        if not result or _distance(result[-1], p) > EPS:
            result.append(p)
    return result


@dataclass
class LocalFrame:
    origin: dict

    def __post_init__(self):
        self.center = _xyz(self.origin)
        lat, lng = math.radians(self.origin["lat"]), math.radians(self.origin["lng"])
        self.east = (-math.sin(lng), math.cos(lng), 0.0)
        self.north = (-math.sin(lat) * math.cos(lng), -math.sin(lat) * math.sin(lng), math.cos(lat))
        self.up = (math.cos(lat) * math.cos(lng), math.cos(lat) * math.sin(lng), math.sin(lat))

    def to_local(self, p: dict) -> Vec:
        delta = _sub(_xyz(p), self.center)
        return (_dot(delta, self.east), _dot(delta, self.north), _dot(delta, self.up))

    def to_point(self, p: Vec, fixed_altitude=None, minimum_altitude=None) -> dict:
        xyz = tuple(self.center[i] + p[0] * self.east[i] + p[1] * self.north[i] + p[2] * self.up[i] for i in range(3))
        lat, lng, altitude = ecef_to_geodetic(*xyz)
        if fixed_altitude is not None:
            altitude = float(fixed_altitude)
        elif minimum_altitude is not None:
            altitude = max(float(minimum_altitude), altitude)
        return {"lat": lat, "lng": lng, "altitude": altitude}


def _inside(p: dict, bounds: dict | None, fixed_altitude, minimum_altitude) -> bool:
    if fixed_altitude is not None and abs(p["altitude"] - fixed_altitude) > 1e-7:
        return False
    if minimum_altitude is not None and p["altitude"] < minimum_altitude - 1e-7:
        return False
    return not bounds or (
        bounds["minLat"] <= p["lat"] <= bounds["maxLat"]
        and bounds["minLng"] <= p["lng"] <= bounds["maxLng"]
    )


def _safe(path: list[dict], checker, bounds=None, fixed_altitude=None, minimum_altitude=None) -> bool:
    return bool(path) and all(_inside(p, bounds, fixed_altitude, minimum_altitude) for p in path) and (
        checker is None or checker.path_is_safe(path)
    )


def _turn(a: Vec, b: Vec, c: Vec) -> float:
    incoming, outgoing = _sub(b, a), _sub(c, b)
    if min(_norm(incoming), _norm(outgoing)) <= EPS:
        return 0.0
    return math.acos(max(-1.0, min(1.0, _dot(_unit(incoming), _unit(outgoing)))))


def _poly_curvatures(path: list[dict]) -> list[float]:
    xyz = [_xyz(p) for p in path]
    values = [0.0] * len(path)
    for i in range(1, len(path) - 1):
        a, b, c = xyz[i - 1:i + 2]
        ab, bc, ac = _sub(b, a), _sub(c, b), _sub(c, a)
        denom = _norm(ab) * _norm(bc) * _norm(ac)
        cross = (ab[1] * bc[2] - ab[2] * bc[1], ab[2] * bc[0] - ab[0] * bc[2], ab[0] * bc[1] - ab[1] * bc[0])
        if denom > EPS ** 3:
            values[i] = 2 * _norm(cross) / denom
    return values


def _sharp_stops(path: list[dict]) -> set[int]:
    if not path:
        return set()
    xyz = [_xyz(p) for p in path]
    return {0, len(path) - 1} | {
        i for i in range(1, len(path) - 1)
        if _turn(xyz[i - 1], xyz[i], xyz[i + 1]) > math.radians(1.0)
    }


def _shortcut(path: list[dict], checker) -> list[dict]:
    """Bounded farthest-visible attempts; never crosses a leg endpoint."""
    # A clear leg must be a straight connection even when it spans more than
    # the local lookahead budget. Do this once before bounded obstacle detours.
    if checker is None or not checker.is_segment_blocked(path[0], path[-1]):
        return [path[0], path[-1]]
    output = [path[0]]
    cursor = 0
    while cursor < len(path) - 1:
        far = min(cursor + 256, len(path) - 1)
        stride = max(1, math.ceil((far - cursor) / 24))
        candidates = list(range(far, cursor, -stride))
        if cursor + 1 not in candidates:
            candidates.append(cursor + 1)
        selected = cursor + 1
        for target in candidates:
            if checker is None or not checker.is_segment_blocked(path[cursor], path[target]):
                selected = target
                break
        output.append(path[selected])
        cursor = selected
    return output


def _evaluate(controls: list[Vec], u: float) -> Vec:
    current = controls
    while len(current) > 1:
        current = [tuple((1-u) * a[j] + u * b[j] for j in range(3)) for a, b in zip(current, current[1:])]
    return current[0]


def _curve_curvature(controls: list[Vec], u: float) -> float:
    d1 = [tuple(5 * (b[j] - a[j]) for j in range(3)) for a, b in zip(controls, controls[1:])]
    d2 = [tuple(4 * (b[j] - a[j]) for j in range(3)) for a, b in zip(d1, d1[1:])]
    v, a = _evaluate(d1, u), _evaluate(d2, u)
    speed = _norm(v)
    if speed < EPS:
        raise ValueError("Degenerate curve derivative")
    cross = (v[1]*a[2]-v[2]*a[1], v[2]*a[0]-v[0]*a[2], v[0]*a[1]-v[1]*a[0])
    return _norm(cross) / speed ** 3


def _split(controls: list[Vec]) -> tuple[list[Vec], list[Vec]]:
    left, right, row = [controls[0]], [controls[-1]], controls
    while len(row) > 1:
        row = [tuple((a[j]+b[j])/2 for j in range(3)) for a, b in zip(row, row[1:])]
        left.append(row[0])
        right.append(row[-1])
    return left, list(reversed(right))


def _flatness(controls: list[Vec]) -> float:
    a, b = controls[0], controls[-1]
    ab = _sub(b, a)
    denom = _dot(ab, ab)
    if denom < EPS * EPS:
        return max(_norm(_sub(p, a)) for p in controls)
    return max(_norm(_sub(p, _add(a, ab, max(0.0, min(1.0, _dot(_sub(p, a), ab) / denom))))) for p in controls)


def _sample_curve(controls: list[Vec]) -> list[tuple[float, Vec]]:
    output = [(0.0, controls[0])]
    stack = [(controls, 0.0, 1.0, 0)]
    while stack:
        part, begin, end, depth = stack.pop()
        if _flatness(part) <= CURVE_TOLERANCE and _norm(_sub(part[-1], part[0])) <= CURVE_STEP:
            output.append((end, part[-1]))
            if len(output) > MAX_CURVE_POINTS:
                raise ValueError("Curve sample budget exceeded")
        else:
            if depth >= 16:
                raise ValueError("Curve subdivision budget exceeded")
            left, right = _split(part)
            middle = (begin + end) / 2
            stack.extend([(right, middle, end, depth+1), (left, begin, middle, depth+1)])
    return output


def postprocess_leg(
    path: list[dict], checker, *, method: str = "local_quintic_bezier",
    transition_distance: float = 12.0, fixed_altitude: float | None = None,
    minimum_altitude: float | None = None, bounds: dict | None = None,
) -> dict:
    """Process ONE protected-endpoint leg, with no silent relocation of anchors."""
    if method not in {"none", "moving_average", "local_quintic_bezier"}:
        raise ValueError("Unsupported smoothing method")
    if not math.isfinite(transition_distance) or transition_distance <= 0:
        raise ValueError("transition_distance must be finite and positive")
    raw = _dedup(path)
    report = dict(method=method, status="unchanged", smoothedCorners=0, fallbackCorners=0,
                  removedPoints=0, geometryToleranceMeters=CURVE_TOLERANCE, notes=[])
    if not raw:
        return dict(path=[], curvatures=[], stopIndices=[], report=report)
    if not _safe(raw, checker, bounds, fixed_altitude, minimum_altitude):
        raise ValueError("Raw path failed obstacle or task-constraint validation")
    if method == "none" or len(raw) < 3:
        report["status"] = "disabled" if method == "none" else "unchanged"
        return dict(path=raw, curvatures=_poly_curvatures(raw), stopIndices=sorted(_sharp_stops(raw)), report=report)

    frame = LocalFrame(raw[0])
    if method == "moving_average":
        local = [frame.to_local(p) for p in raw]
        candidate = [raw[0]]
        for i in range(1, len(raw)-1):
            averaged = tuple(sum(local[j][axis] for j in range(i-1, i+2))/3 for axis in range(3))
            candidate.append(frame.to_point(averaged, fixed_altitude, minimum_altitude))
        candidate.append(raw[-1])
        if not _safe(candidate, checker, bounds, fixed_altitude, minimum_altitude):
            candidate = raw
            report.update(status="partial", fallbackCorners=1)
            report["notes"].append("移动平均未通过安全复检，已回退原路径。")
        else:
            report["status"] = "legacy"
        return dict(path=candidate, curvatures=_poly_curvatures(candidate), stopIndices=sorted(_sharp_stops(candidate)), report=report)

    simple = _shortcut(raw, checker)
    report["removedPoints"] = len(raw) - len(simple)
    points = [frame.to_local(p) for p in simple]
    result, curvature, stops = [simple[0]], [0.0], {0}

    def append(p, k=0.0, stop=False):
        if _distance(result[-1], p) > EPS:
            result.append(p)
            curvature.append(k)
        else:
            curvature[-1] = max(curvature[-1], k)
        if stop:
            stops.add(len(result)-1)

    for i in range(1, len(simple)-1):
        a, b, c = points[i-1:i+2]
        incoming, outgoing = _sub(b, a), _sub(c, b)
        angle = _turn(a, b, c)
        if angle < math.radians(0.1):
            append(simple[i])
            continue
        # Shared edge trim sums stay below 90% of its length.
        distance = min(transition_distance, 0.45*_norm(incoming), 0.45*_norm(outgoing))
        accepted = False
        if angle < math.radians(170):
            for attempt in range(6):
                d = distance * 0.5 ** attempt
                if d < 0.15:
                    break
                u, v = _unit(incoming), _unit(outgoing)
                p0, p5 = _add(b, u, -d), _add(b, v, d)
                # Collinear equally spaced first/last triples give zero endpoint
                # second derivative. Non-degenerate tangent and curvature checked.
                controls = [p0, _add(p0,u,.4*d), _add(p0,u,.8*d),
                            _add(p5,v,-.8*d), _add(p5,v,-.4*d), p5]
                try:
                    samples = _sample_curve(controls)
                    curve = [frame.to_point(p, fixed_altitude, minimum_altitude) for _, p in samples]
                    ks = [_curve_curvature(controls, t) for t, _ in samples]
                except ValueError:
                    continue
                # Check the incoming stitch, all curve chords, and outgoing stitch.
                validation = [result[-1], *curve, simple[i+1]]
                if len(result)+len(curve) >= MAX_POINTS:
                    break
                if not _safe(validation, checker, bounds, fixed_altitude, minimum_altitude):
                    continue
                for p, k in zip(curve, ks):
                    append(p, k)
                report["smoothedCorners"] += 1
                accepted = True
                break
        if not accepted:
            append(simple[i], stop=True)
            report["fallbackCorners"] += 1

    append(simple[-1], stop=True)
    # After fixed-height conversion and stitching, test the exact returned geometry.
    if not _safe(result, checker, bounds, fixed_altitude, minimum_altitude):
        result, curvature, stops = raw, _poly_curvatures(raw), _sharp_stops(raw)
        report["fallbackCorners"] += report["smoothedCorners"] or 1
        report["smoothedCorners"] = 0
        report["notes"].append("拼接复检未通过，整个分段已回退安全原路径。")
    else:
        discrete = _poly_curvatures(result)
        curvature = [max(a,b) for a,b in zip(curvature, discrete)]
    report["status"] = "partial" if report["fallbackCorners"] else ("smoothed" if report["smoothedCorners"] else "unchanged")
    if report["fallbackCorners"]:
        report["notes"].append("受限拐点保留折线并按停点处理，未移动必经点。")
    return dict(path=result, curvatures=curvature, stopIndices=sorted(stops), report=report)


def build_timed_trajectory(
    path: list[dict], *, curvatures: list[float] | None = None,
    stop_indices: list[int] | set[int] | None = None, phases: list[str] | None = None,
    cruise_speed: float = 15.0, max_lateral_acceleration: float = 2.0,
    max_tangential_acceleration: float = 2.0, vertical_speed: float = 3.0,
) -> dict:
    """Densify exact playback geometry and forward/backward-limit nodal speeds.

    Acceleration values describe this discrete profile. Client linear interpolation
    remains an approximation, not a continuous-acceleration flight controller.
    """
    params = (cruise_speed, max_lateral_acceleration, max_tangential_acceleration, vertical_speed)
    if not all(math.isfinite(v) and v > 0 for v in params):
        raise ValueError("Trajectory speed and acceleration limits must be finite and positive")
    if curvatures is not None and len(curvatures) != len(path):
        raise ValueError("curvatures must match path length")
    if phases is not None and len(phases) != len(path):
        raise ValueError("phases must match path length")
    if curvatures is not None and not all(math.isfinite(k) and k >= 0 for k in curvatures):
        raise ValueError("Curvature must be finite and non-negative")
    if not path:
        return dict(path=[], samples=[], stats=dict(totalDistanceMeters=0.0,durationSeconds=0.0,
                    maxSpeedMetersPerSecond=0.0,maxCurvature=0.0,sampleCount=0,stopCount=0,
                    maxTangentialAcceleration=0.0,maxLateralAcceleration=0.0,validationMode="sampled_polyline"))
    original = [_point(p) for p in path]
    source_k = list(curvatures) if curvatures is not None else _poly_curvatures(original)
    source_phases = phases or ["cruise"] * len(path)
    source_stops = set(stop_indices) if stop_indices is not None else _sharp_stops(original)
    source_stops.update({0,len(path)-1})
    output, ks, tags, stops = [original[0]], [source_k[0]], [source_phases[0]], {0}
    total_length = sum(_distance(a,b) for a,b in zip(original,original[1:]))
    # Avoid unbounded output on very long routes; original curve points stay intact.
    spacing = max(2.0, total_length / max(1, (MAX_POINTS-len(path))*0.8))
    for i in range(1,len(path)):
        a,b = original[i-1],original[i]
        length = _distance(a,b)
        if length <= EPS:
            if i in source_stops:
                stops.add(len(output)-1)
            continue
        # A midpoint makes zero-speed start/end segments physically timeable.
        steps = max(2,math.ceil(length/spacing))
        if len(output)+steps > MAX_POINTS:
            raise ValueError("Trajectory output exceeds point budget")
        for j in range(1,steps+1):
            t=j/steps
            output.append(dict(b) if j==steps else _lerp(a,b,t))
            ks.append((1-t)*source_k[i-1]+t*source_k[i])
            tags.append(source_phases[i])
        if i in source_stops:
            stops.add(len(output)-1)
    # Include the curvature of the exact converted sampled geometry in the limit.
    discrete = _poly_curvatures(output)
    ks = [max(a,b) for a,b in zip(ks,discrete)]
    ds=[_distance(a,b) for a,b in zip(output,output[1:])]
    limits=[min(cruise_speed, math.sqrt(max_lateral_acceleration/k)) if k>1e-10 else cruise_speed for k in ks]
    for i,length in enumerate(ds):
        dz=abs(output[i+1]["altitude"]-output[i]["altitude"])
        cap=min(cruise_speed, vertical_speed*length/dz) if dz>EPS else cruise_speed
        limits[i]=min(limits[i],cap)
        limits[i+1]=min(limits[i+1],cap)
    speeds=list(limits)
    for i in stops:
        speeds[i]=0.0
    for i,length in enumerate(ds):
        speeds[i+1]=min(speeds[i+1],math.sqrt(speeds[i]**2+2*max_tangential_acceleration*length))
    for i in range(len(ds)-1,-1,-1):
        speeds[i]=min(speeds[i],math.sqrt(speeds[i+1]**2+2*max_tangential_acceleration*ds[i]))
    elapsed=0.0
    samples=[]
    for i,p in enumerate(output):
        if i:
            speed_sum=speeds[i-1]+speeds[i]
            if speed_sum<=1e-12:
                raise ValueError("No positive speed available between trajectory stops")
            elapsed+=2*ds[i-1]/speed_sum
        samples.append({**p,"timeSeconds":elapsed,"speedMetersPerSecond":speeds[i],"phase":tags[i]})
    tangential=max((abs(speeds[i+1]**2-speeds[i]**2)/(2*d) for i,d in enumerate(ds) if d>EPS),default=0.0)
    lateral=max((v*v*k for v,k in zip(speeds,ks)),default=0.0)
    stats=dict(totalDistanceMeters=sum(ds),durationSeconds=elapsed,
               maxSpeedMetersPerSecond=max(speeds,default=0.0),maxCurvature=max(ks,default=0.0),
               sampleCount=len(output),stopCount=len(stops),maxTangentialAcceleration=tangential,
               maxLateralAcceleration=lateral,validationMode="sampled_polyline")
    return dict(path=output,samples=samples,stats=stats)
