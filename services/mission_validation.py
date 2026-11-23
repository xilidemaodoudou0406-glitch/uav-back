"""Conservative offline checks. A pass authorizes simulation only, never flight."""
import bisect
import hashlib
import math
from pathlib import Path
from zipfile import BadZipFile
from struct import error as StructError
from navigation import LocalFrame
from navigation_schemas import ValidationSettings
from obstacles import get_out2_dataset, get_reconstruction1_dataset
from services.drone_registry import utc_now

PROFILES = {
    "simulation": {"name": "通用模拟器", "horizontalSpeed": 90, "verticalSpeed": 90,
        "acceleration": 10000, "brakingAcceleration": 3, "relativeHeight": 20000,
        "radius": 100000, "duration": 3600, "status": "software_assumptions"},
    "mini3_lab": {"name": "Mini 3 离线保守配置", "horizontalSpeed": 5, "verticalSpeed": 2,
        "acceleration": 2, "brakingAcceleration": 2, "relativeHeight": 100,
        "radius": 1000, "duration": 1200, "status": "software_assumptions"},
}
LOADERS = {"out2": get_out2_dataset, "reconstruction1": get_reconstruction1_dataset}


def dataset_catalog():
    return [{"id": key, "enabled": data.enabled, "tileCount": len(data.tiles),
             "status": "display_alignment_unverified", "error": data.error}
            for key, loader in LOADERS.items() for data in (loader(),)]


def interval(a, b, lower, upper):
    enter, leave = 0.0, 1.0
    for i in range(len(a)):
        d = b[i]-a[i]
        if abs(d) < 1e-12:
            if a[i] < lower[i] or a[i] > upper[i]: return None
        else:
            x, y = (lower[i]-a[i])/d, (upper[i]-a[i])/d
            enter, leave = max(enter, min(x,y)), min(leave, max(x,y))
            if enter > leave: return None
    return enter, leave


def distance_to_segment(p, a, b):
    d = tuple(b[i]-a[i] for i in range(3))
    length = sum(v*v for v in d)
    t = min(1, max(0, sum((p[i]-a[i])*d[i] for i in range(3))/length)) if length else 0
    return math.dist(p, tuple(a[i]+t*d[i] for i in range(3)))


def geographic_curve_envelope(a, b):
    """Loose second-derivative bound for linear lat/lng/height interpolation vs ECEF chord."""
    angle = math.radians(abs(b["lat"]-a["lat"])+abs(b["lng"]-a["lng"]))
    return ((6378137+max(abs(a["altitude"]),abs(b["altitude"])))*angle*angle
            + abs(b["altitude"]-a["altitude"])*angle)/2


def simplify_indices(points, tolerance=.05):
    """RDP segments enclose the original polyline with a known extra margin."""
    keep = {0, len(points)-1}; stack = [(0,len(points)-1)]; operations = 0
    while stack:
        a,b = stack.pop()
        if b <= a+1: continue
        maximum, index = -1, a
        for i in range(a+1,b):
            operations += 1
            if operations > 2_000_000: return list(range(len(points)))
            d = distance_to_segment(points[i], points[a], points[b])
            if d > maximum: maximum,index = d,i
        if maximum > tolerance:
            keep.add(index); stack.extend(((a,index),(index,b)))
    return sorted(keep)


def voxel_collision(tile, a, b, horizontal, vertical, budget):
    grid = tile.grid; r = grid["resolutionMeters"]
    origins = (grid["originEastMeters"], grid["originNorthMeters"], grid["originUpMeters"])
    sizes = (grid["columns"], grid["rows"], grid["levels"])
    margin = (horizontal,horizontal,vertical)
    lo = [max(0, math.floor((min(a[i],b[i])-margin[i]-origins[i])/r)-1) for i in range(3)]
    hi = [min(sizes[i]-1, math.floor((max(a[i],b[i])+margin[i]-origins[i])/r)) for i in range(3)]
    if any(lo[i]>hi[i] for i in range(3)): return False
    tile.ensure_loaded(); occupied = tile.occupied
    for level in range(lo[2],hi[2]+1):
        for row in range(lo[1],hi[1]+1):
            budget[0] += 1
            if budget[0] > 600_000: raise ValueError("体素检查超出计算预算，请拆分航线或缩小误差范围；本次检查未通过")
            base = (level*sizes[1]+row)*sizes[0]
            first = bisect.bisect_left(occupied,base+lo[0])
            last = bisect.bisect_right(occupied,base+hi[0])
            for index in range(first,last):
                budget[1] += 1
                if budget[1] > 2_000_000: raise ValueError("障碍候选过多，本次检查未通过，请拆分航线")
                cell = (occupied[index]-base,row,level)
                lower = tuple(origins[i]+cell[i]*r-margin[i] for i in range(3))
                upper = tuple(origins[i]+(cell[i]+1)*r+margin[i] for i in range(3))
                if interval(a,b,lower,upper): return True
    return False


def validate_snapshot(snapshot, checksum, loaders=None):
    loaders = LOADERS if loaders is None else loaders
    settings = ValidationSettings.model_validate(snapshot.get("validationSettings",{})).model_dump()
    profile = PROFILES[settings["profileId"]]
    points = snapshot["trajectorySamples"]; frame = LocalFrame(points[0])
    xyz = [frame.to_enu(p) for p in points]
    errors, warnings, versions = [], [], []
    velocities, periods = [], []
    for i in range(len(points)-1):
        dt = points[i+1]["timeSeconds"]-points[i]["timeSeconds"]
        periods.append(dt); velocities.append(tuple((xyz[i+1][j]-xyz[i][j])/dt for j in range(3)))
    horizontal = max(math.hypot(*v[:2]) for v in velocities)
    vertical = max(abs(v[2]) for v in velocities)
    acceleration = max((math.dist(a,b)/((periods[i]+periods[i+1])/2)
        for i,(a,b) in enumerate(zip(velocities,velocities[1:]))),default=0)
    radius = max(math.hypot(*p[:2]) for p in xyz)
    relative = max(abs(p["altitude"]-points[0]["altitude"]) for p in points)
    metrics = {"horizontalSpeed":horizontal,"verticalSpeed":vertical,"acceleration":acceleration,
        "radius":radius,"relativeHeight":relative,"duration":points[-1]["timeSeconds"]}
    labels = {"horizontalSpeed":"水平速度","verticalSpeed":"垂直速度","acceleration":"采样加速度",
              "radius":"距首点距离","relativeHeight":"相对首点高度变化","duration":"计划时长"}
    for key,value in metrics.items():
        if value > profile[key]+1e-6: errors.append(f"{labels[key]} {value:.3f} 超出配置上限 {profile[key]}")
    if any(abs(b["lng"]-a["lng"])>180 for a,b in zip(points,points[1:])):
        errors.append("离线障碍验证不支持跨越日期变更线，请拆分航线")
    common = sum(settings[k] for k in ("bodyRadiusMeters","modelErrorMeters","positionErrorMeters","trackingErrorMeters"))
    scenario = snapshot.get("errorScenario", {})
    duration = metrics["duration"]
    injected_h = math.hypot(scenario.get("eastBiasMeters",0),scenario.get("northBiasMeters",0)) + scenario.get("driftMetersPerSecond",0)*duration + math.sqrt(2)*scenario.get("noiseAmplitudeMeters",0)
    injected_v = abs(scenario.get("upBiasMeters",0)) + scenario.get("noiseAmplitudeMeters",0)
    latency = settings["latencySeconds"]+scenario.get("telemetryDelayMs",0)/1000
    braking_h = horizontal**2/(2*profile["brakingAcceleration"]) if settings["includeBrakingDistance"] else 0
    braking_v = vertical**2/(2*profile["brakingAcceleration"]) if settings["includeBrakingDistance"] else 0
    h = max(settings["horizontalMarginMeters"],common+injected_h+horizontal*latency+braking_h)
    v = max(settings["verticalMarginMeters"],common+injected_v+vertical*latency+braking_v)
    margins = {"horizontalMeters":h,"verticalMeters":v,"brakingHorizontalMeters":braking_h,
               "injectedHorizontalMeters":injected_h,"injectedVerticalMeters":injected_v,"latencySeconds":latency}
    warnings.extend(["通过仅表示离线模拟约束通过；不构成真机放飞许可或定位精度保证",
        "机型参数为平台软件配置，未完成该架 Mini 3 的实测确认",
        "局部 ENU 航段、采样加速度与表面体素无法证明真实飞行跟踪误差及完整障碍覆盖"])
    if snapshot.get("calibrationStatus") != "field_verified": warnings.append("模型高程和水平位置尚未实测校准，真机控制保持关闭")
    datasets = []
    for name in dict.fromkeys(settings["obstacleDatasets"]):
        data = loaders[name]()
        if not data.enabled:
            errors.append(f"所选障碍数据 {name} 不可用，不能跳过检查"); continue
        files = [data.manifest_path]+[data.data_root/t.metadata["file"] for t in data.tiles]
        try:
            if hasattr(data,"source_stamp") and data.current_source_stamp() != data.source_stamp:
                errors.append(f"障碍数据 {name} 在加载后已修改，请重启后端重新加载，再运行预检")
                continue
            digest = hashlib.sha256()
            for file in files:
                stat = file.stat(); digest.update(f"{file.name}:{stat.st_size}:{stat.st_mtime_ns}".encode())
            digest.update(data.manifest_path.read_bytes())
            versions.append({"id":name,"fingerprint":digest.hexdigest(),"kind":"manifest_content_and_file_metadata"})
            datasets.append(data)
        except OSError:
            errors.append(f"障碍数据 {name} 文件缺失或不可读取")
    if settings["requireCoverage"] and not datasets: errors.append("要求覆盖检查时必须选择可用的障碍数据")
    if not settings["obstacleDatasets"]: warnings.append("未选择模型障碍数据；当前报告没有模型避障结论")
    if not settings["requireCoverage"]: warnings.append("未要求数据覆盖；模型范围外不能推定安全")
    simplified = simplify_indices(xyz)
    # Enclose both RDP simplification and the executor's geographic interpolation.
    geometry = .05+max(geographic_curve_envelope(a,b) for a,b in zip(points,points[1:]))
    margins["geometryEnvelopeMeters"] = geometry
    budget = [0,0]; collision = False; uncovered = False
    try:
        boxes = []
        for zone in settings["noFlyZones"]:
            corners = [frame.to_enu({"lat":lat,"lng":lng,"altitude":alt})
                for lat in (zone["minLat"],zone["maxLat"]) for lng in (zone["minLng"],zone["maxLng"])
                for alt in (zone["minAlt"],zone["maxAlt"])]
            box_curve = geographic_curve_envelope({"lat":zone["minLat"],"lng":zone["minLng"],"altitude":zone["minAlt"]},
                {"lat":zone["maxLat"],"lng":zone["maxLng"],"altitude":zone["maxAlt"]})
            boxes.append((tuple(min(c[i] for c in corners)-(h if i<2 else v)-geometry-box_curve for i in range(3)),
                          tuple(max(c[i] for c in corners)+(h if i<2 else v)+geometry+box_curve for i in range(3))))
        coverage = []
        for data in datasets:
            for tile in data.tiles:
                west,south,east,north = tile.metadata["referenceBboxWgs84"]
                corners = [frame.to_enu({"lat":lat,"lng":lng,"altitude":points[0]["altitude"]})
                    for lat,lng in ((south,west),(south,east),(north,west),(north,east))]
                # Inscribed local rectangle; only extent membership, not surveyed completeness.
                extent_curve = geographic_curve_envelope({"lat":south,"lng":west,"altitude":points[0]["altitude"]},
                    {"lat":north,"lng":east,"altitude":points[0]["altitude"]})
                shrink = h+geometry+extent_curve
                coverage.append(((max(corners[0][0],corners[2][0])+shrink,max(corners[0][1],corners[1][1])+shrink),
                                 (min(corners[1][0],corners[3][0])-shrink,min(corners[2][1],corners[3][1])-shrink)))
        for ia,ib in zip(simplified,simplified[1:]):
            a,b = xyz[ia],xyz[ib]
            if any(interval(a,b,lower,upper) for lower,upper in boxes): collision=True; break
            if settings["requireCoverage"]:
                spans = sorted(span for lower,upper in coverage
                    if all(lower[i]<=upper[i] for i in (0,1)) and (span:=interval(a[:2],b[:2],lower,upper)))
                end = 0.0
                for first,last in spans:
                    if first>end+1e-9: break
                    end = max(end,last)
                if end<1-1e-9: uncovered=True
            for data in datasets:
                for tile in data.tiles:
                    aa = tile.wgs84_to_enu(points[ia]["lat"],points[ia]["lng"],points[ia]["altitude"])
                    bb = tile.wgs84_to_enu(points[ib]["lat"],points[ib]["lng"],points[ib]["altitude"])
                    if voxel_collision(tile,aa,bb,h+geometry,v+geometry,budget): collision=True; break
                if collision: break
            if collision: break
    except (OSError,ValueError,KeyError,BadZipFile,StructError) as exc:
        errors.append(f"障碍检查未完成：{str(exc)[:180]}")
    if collision: errors.append("航线或其误差/机体/制动包络进入人工禁飞区或模型表面体素")
    if uncovered: errors.append("航线及水平裕量超出所选模型数据范围；无法给出完整覆盖结论")
    for data in datasets:
        try:
            if hasattr(data,"source_stamp") and data.current_source_stamp() != data.source_stamp:
                errors.append("检查期间模型文件发生变化，需重新加载后再预检")
        except OSError:
            errors.append("检查期间模型文件不可读取，预检未通过")
    return {"schemaVersion":1,"checkedAt":utc_now(),"snapshotChecksum":checksum,"passed":not errors,
        "executionScope":"simulation_only","realFlightEnabled":False,"profile":profile,"profileId":settings["profileId"],
        "metrics":metrics,"margins":margins,"errors":errors,"warnings":warnings,"dataVersions":versions,
        "calibrationStatus":snapshot.get("calibrationStatus","unverified"),"checkedSegments":len(simplified)-1,
        "checks":{"kinematics":True,"manualZones":len(boxes),"modelObstacles":bool(datasets),
                  "coverageRequired":settings["requireCoverage"],"coverageMeaning":"metadata_extent_only"}}
