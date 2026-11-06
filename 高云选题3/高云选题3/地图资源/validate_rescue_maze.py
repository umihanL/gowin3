"""Static 2-D reachability and clearance checks for rescue_maze_config.json.

This is a geometry test only. It does not model wheel kinematics, acceleration,
controller latency, swept vehicle rotation, or dynamic gate physics.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / "rescue_maze_config.json"
REPORT = ROOT / "rescue_maze_validation.json"


def box_distance(p, bounds):
    x, y = p; x0, x1, y0, y1 = bounds
    return math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))


def dist_point_segment(p, a, b):
    px, py = p; ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    t = 0.0 if den == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    q = (ax + t * dx, ay + t * dy)
    return math.hypot(px - q[0], py - q[1])


def dist_segment_segment(a, b, c, d):
    # Segment intersection is distance zero; otherwise endpoint-to-segment is enough in 2-D.
    if orient(a, b, c) * orient(a, b, d) <= 0 and orient(c, d, a) * orient(c, d, b) <= 0:
        return 0.0
    return min(dist_point_segment(a, c, d), dist_point_segment(b, c, d), dist_point_segment(c, a, b), dist_point_segment(d, a, b))


def orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segment_from_wall(wall):
    return tuple(wall["a"]), tuple(wall["b"])


def gate_segment(gate, state):
    if state == "open":
        angle = math.radians(float(gate["open_deg"]))
    else:
        angle = math.radians(float(gate["closed_deg"]))
    px, py, _ = gate["pivot"]
    # The physical arm is along +Y when closed and rotates about the X axis.
    # Its XY projection therefore shortens with cos(angle) as it is raised.
    return (px, py), (px, py + gate["arm_length"] * math.cos(angle))


def add_derived_obstacles(cfg):
    """Return small fixed poles omitted from the visual config's wall list."""
    construction = cfg["construction"]
    items = [
        {"id": "parking_P_pole", "center": [4.98, 6.77], "radius": 0.035, "kind": "parking_pole"},
        {"id": "construction_sign", "center": construction["sign"], "size": [0.45, 0.22], "kind": "construction_sign"},
        {"id": "construction_foot_w", "center": [5.1, 6.55], "size": [0.28, 0.38], "kind": "construction_foot"},
        {"id": "construction_foot_e", "center": [6.9, 6.55], "size": [0.28, 0.38], "kind": "construction_foot"},
    ]
    for light in cfg.get("traffic_lights", []):
        items.append({"id": light["id"] + "_pole", "center": light["position"], "radius": 0.04, "kind": "traffic_pole"})
    return items


def point_clearance(cfg, p, include_gate=None):
    p = tuple(p[:2]); half = float(cfg["wall_thickness"]) / 2
    d = min(p[0] - half, cfg["site"][0] - half - p[0], p[1] - half, cfg["site"][1] - half - p[1])
    for wall in cfg["walls"]:
        d = min(d, dist_point_segment(p, *segment_from_wall(wall)) - half)
    for item in add_derived_obstacles(cfg):
        if "size" in item:
            sx, sy = item["size"]
            d = min(d, box_distance(p, (item["center"][0] - sx / 2, item["center"][0] + sx / 2, item["center"][1] - sy / 2, item["center"][1] + sy / 2)))
        else:
            d = min(d, math.hypot(p[0] - item["center"][0], p[1] - item["center"][1]) - item["radius"])
    if include_gate:
        d = min(d, dist_point_segment(p, *gate_segment(cfg["gate"], include_gate)) - cfg["gate"]["arm_thickness"] / 2)
    construction = cfg["construction"]
    cx, cy = construction["center"]
    d = min(d, dist_point_segment(p, (cx - construction["width"] / 2, cy), (cx + construction["width"] / 2, cy)) - construction["thickness"] / 2)
    for cone in construction["cones"]:
        d = min(d, math.hypot(p[0] - cone[0], p[1] - cone[1]) - 0.12)
    return d


def segment_clearance(cfg, a, b, include_gate=None, step=0.025):
    a, b = tuple(a[:2]), tuple(b[:2])
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    count = max(1, math.ceil(length / step))
    minimum = float("inf"); where = None
    for i in range(count + 1):
        t = i / count
        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        d = point_clearance(cfg, p, include_gate)
        if d < minimum:
            minimum, where = d, p
    return minimum, where, count + 1


def route_check(cfg, points, include_gate=None):
    required = math.hypot(cfg["robot"]["length"], cfg["robot"]["width"]) / 2 + cfg["robot"]["clearance_margin"]
    minimum = float("inf"); where = None; samples = 0
    for a, b in zip(points, points[1:]):
        d, p, n = segment_clearance(cfg, a, b, include_gate)
        samples += n
        if d < minimum:
            minimum, where = d, p
    return {"status": "PASS" if minimum >= required else "FAIL", "min_clearance_m": round(minimum, 4), "required_clearance_m": round(required, 4), "closest_sample": [round(v, 4) for v in where] if where else None, "samples": samples, "gate_state": include_gate or "excluded", "model": "sampled line segments + circular footprint; static geometry only"}


def parking_check(cfg):
    bay_w, bay_d = cfg["parking"]["bay_size"]
    length, width = cfg["robot"]["length"], cfg["robot"]["width"]
    margin = cfg["robot"]["clearance_margin"]
    circle_diameter = 2 * (math.hypot(length, width) / 2 + margin)
    rectangular_ok = bay_w >= length + 2 * margin and bay_d >= width + 2 * margin
    return {
        "bay_size_m": [bay_w, bay_d],
        "robot_size_m": [length, width],
        "circular_envelope_diameter_m": round(circle_diameter, 4),
        "circular_envelope_fits": circle_diameter <= min(bay_w, bay_d),
        "rectangular_footprint_with_margin_fits": rectangular_ok,
        "status": "PASS" if rectangular_ok else "FAIL",
        "note": "Parking uses an axis-aligned rectangular footprint with margin; the conservative rotated circular envelope is reported separately."
    }


def write_png(cfg, route_results):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return False
    sx, sy = cfg["site"]; scale = 90; pad = 30
    image = Image.new("RGB", (int(sx * scale + 2 * pad), int(sy * scale + 2 * pad)), "white")
    draw = ImageDraw.Draw(image)
    def pt(p): return (pad + int(p[0] * scale), pad + int((sy - p[1]) * scale))
    for wall in cfg["walls"]:
        draw.line([pt(wall["a"]), pt(wall["b"])], fill=(45, 55, 65), width=max(2, int(cfg["wall_thickness"] * scale)))
    c = cfg["construction"]
    x0, x1 = c["center"][0] - c["width"] / 2, c["center"][0] + c["width"] / 2
    y0, y1 = c["center"][1] - c["thickness"] / 2, c["center"][1] + c["thickness"] / 2
    draw.polygon([pt((x0, y0)), pt((x1, y0)), pt((x1, y1)), pt((x0, y1))], fill=(225, 130, 35))
    for cone in c["cones"]:
        x, y = pt(cone); rr = int(.12 * scale); draw.ellipse((x-rr, y-rr, x+rr, y+rr), fill=(215, 80, 40))
    for item in add_derived_obstacles(cfg):
        x, y = pt(item["center"])
        if "size" in item:
            sx, sy = item["size"]; x0, y0 = pt((item["center"][0] - sx / 2, item["center"][1] - sy / 2)); x1, y1 = pt((item["center"][0] + sx / 2, item["center"][1] + sy / 2))
            draw.polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], fill=(90, 90, 90) if item["kind"] != "construction_sign" else (180, 100, 30))
        else:
            rr = max(2, int(item["radius"] * scale)); draw.ellipse((x-rr, y-rr, x+rr, y+rr), fill=(90, 90, 90))
    colors = [(30, 120, 220), (30, 160, 80), (155, 70, 180), (220, 80, 70), (220, 140, 20), (60, 150, 150), (90, 90, 90)]
    for i, (rid, points) in enumerate(cfg["routes"].items()):
        color = colors[i % len(colors)]; draw.line([pt(p) for p in points], fill=color, width=3)
        for p in points: x, y = pt(p); draw.ellipse((x-3, y-3, x+3, y+3), fill=color)
    image.save(ROOT / "route_validation.png")
    return True


def main():
    cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
    route_results = {}
    for route_id, points in cfg["routes"].items():
        if route_id == "parking_gate_open":
            # The requested abstraction treats the raised, near-vertical arm as
            # non-blocking. The closed arm remains a horizontal entrance barrier.
            route_results[route_id] = {"open": route_check(cfg, points, None), "closed": route_check(cfg, points, "closed"), "expected": "open_pass_closed_block", "open_gate_model": "raised arm excluded from 2-D route obstacle"}
        elif route_id == "blocked_direct_north":
            result = route_check(cfg, points)
            result["expected_blocked_by_construction"] = True
            result["status"] = "PASS" if result["status"] == "FAIL" else "FAIL"
            result["interpretation"] = "PASS means the route is correctly blocked by static construction geometry."
            route_results[route_id] = result
        else:
            route_results[route_id] = route_check(cfg, points)
    parking = parking_check(cfg)
    report = {
        "source": CONFIG.name,
        "status": "PASS" if all((v.get("status") == "PASS" if "status" in v else v["open"]["status"] == "PASS" and v["closed"]["status"] == "FAIL") for v in route_results.values()) and parking["status"] == "PASS" else "FAIL",
        "official_dimensions": cfg.get("official_dimensions", False),
        "model": {"robot_radius_m": round(math.hypot(cfg["robot"]["length"], cfg["robot"]["width"]) / 2, 4), "clearance_margin_m": cfg["robot"]["clearance_margin"], "required_circle_clearance_m": round(math.hypot(cfg["robot"]["length"], cfg["robot"]["width"]) / 2 + cfg["robot"]["clearance_margin"], 4), "wall_thickness_m": cfg["wall_thickness"], "static_2d_only": True},
        "route_checks": route_results,
        "parking_check": parking,
        "derived_obstacles": add_derived_obstacles(cfg),
        "notes": ["This report checks sampled 2-D line segments against static walls and obstacles.", "It does not validate wheel kinematics, acceleration/braking, controller latency, swept rectangular turns, or animated gate dynamics.", "The blocked_direct_north result is inverted intentionally: PASS means the expected construction blockage was detected.", "The parking gate open route ignores the near-vertical open arm per the requested abstraction; closed state includes the arm."],
    }
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {REPORT.name}; status={report['status']}; route_validation.png={'yes' if write_png(cfg, route_results) else 'no'}")


if __name__ == "__main__":
    main()
