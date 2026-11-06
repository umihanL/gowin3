"""Generate the rear differential LUT from the shared, editable car geometry."""
from pathlib import Path
import json
import math
import re

ROOT = Path(__file__).resolve().parents[1]
header = (ROOT/'src/vehicle/vehicle_config.vh').read_text(encoding='utf-8')
cfg = {k: int(v) for k, v in re.findall(r'^`define (VEHICLE_\w+) (-?\d+)$', header, re.M)}
wheelbase, track = cfg['VEHICLE_WHEELBASE_MM'], cfg['VEHICLE_REAR_TRACK_MM']
limit = cfg['VEHICLE_MAX_STEER_DEG']
radius = wheelbase/math.tan(math.radians(limit))
assert radius >= track/2, 'Operating steering requires a reversing inner rear wheel'
assert cfg['VEHICLE_MIN_RADIUS_MM'] >= radius-0.001, 'Planner permits an untrackable radius'
ratios = []
for angle in range(64):
    r = wheelbase/math.tan(math.radians(angle)) if angle else math.inf
    ratio = (r-track/2)/(r+track/2) if angle else 1
    ratios.append(max(0, min(256, round(ratio*256))))
(ROOT/'src/vehicle/rear_ratio.mem').write_text(''.join(f'{v:03x}\n' for v in ratios), encoding='ascii')
# Measured left wheel angles, equivalent parallel/symmetric front wheels as
# requested. Invert each monotone linkage separately; do not assume equal
# servo offsets produce equal left/right wheel angles.
measured = {
    'left': [(0,90),(8,100),(11,110),(15,120),(20,130),(24,140),(28,150)],
    'right': [(0,90),(4,80),(11,70),(20,60),(26,50),(28,40)],
}
def command(direction, angle):
    points=measured[direction]
    for (a,c),(b,d) in zip(points,points[1:]):
        if angle<=b:
            return round(c+(d-c)*(angle-a)/(b-a))
    return points[-1][1]
(ROOT/'src/vehicle/servo_offset.mem').write_text(''.join(
    f'{abs(command(direction,a)-cfg["VEHICLE_SERVO_CENTER_DEG"]):02x}\n'
    for direction in ('left','right')
    for a in range(64)), encoding='ascii')
report = dict(parameters=cfg, minimum_operating_radius_mm=radius,
              circumference_mm=math.pi*cfg['VEHICLE_WHEEL_DIAMETER_MM'],
              counts_per_mm=cfg['VEHICLE_COUNTS_PER_REV']/(math.pi*cfg['VEHICLE_WHEEL_DIAMETER_MM']),
              measured_servo_table=measured,
              servo_limits=[cfg['VEHICLE_SERVO_MIN_DEG'],cfg['VEHICLE_SERVO_MAX_DEG']],
              front_wheels_assumed_parallel=True,
              parameters_are_estimates=False)
(ROOT/'docs/vehicle_parameters.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
# Update only dimensions/checksum. Preserve all image-derived occupancy words.
nav = ROOT/'src/navigation'
words = [int(v, 16) for v in (nav/'map_metadata.mem').read_text().split()]
words[56:59] = [wheelbase, limit, cfg['VEHICLE_MIN_RADIUS_MM']]
words[59:61] = [cfg['VEHICLE_NOSE_FROM_REAR_MM'], cfg['VEHICLE_REAR_OVERHANG_MM']]
words[62] = 512
words[63] = 4
(nav/'map_metadata.mem').write_text(''.join(f'{v:08x}\n' for v in words), encoding='ascii')
init = (nav/'nav_map_init.vh').read_text(encoding='ascii')
init = re.sub(r"dimensions\[(\d+)\]=32'h[0-9a-f]+;",
              lambda m: f"dimensions[{m[1]}]=32'h{words[int(m[1])]:08x};", init)
(nav/'nav_map_init.vh').write_text(init, encoding='ascii')
signature = 0
for word in words:
    signature ^= word
(nav/'nav_metadata.vh').write_text(f"localparam [31:0] MAP_SIGNATURE=32'h{signature:08x};\n", encoding='ascii')
metadata = json.loads((nav/'map_metadata.json').read_text())
metadata.pop('servo_offset_at_60_wheel_deg',None)
metadata.update(path_reference='nose_midpoint', body_length_mm=cfg['VEHICLE_BODY_LENGTH_MM'],
                front_from_rear_axle_mm=cfg['VEHICLE_NOSE_FROM_REAR_MM'],
                rear_overhang_mm=cfg['VEHICLE_REAR_OVERHANG_MM'],
                front_extent_from_rear_axle_mm=cfg['VEHICLE_NOSE_FROM_REAR_MM'],
                rear_extent_from_rear_axle_mm=cfg['VEHICLE_REAR_OVERHANG_MM'],
                front_overhang_mm=cfg['VEHICLE_FRONT_OVERHANG_MM'],wheelbase_mm=wheelbase, max_front_wheel_angle_deg=limit,
                min_rear_axle_turn_radius_mm=cfg['VEHICLE_MIN_RADIUS_MM'],
                estimated_mechanical_limit_deg=max(cfg['VEHICLE_LEFT_STEER_DEG'],cfg['VEHICLE_RIGHT_STEER_DEG']),
                rear_track_mm=track,front_track_mm=cfg['VEHICLE_FRONT_TRACK_MM'],
                left_front_wheel_limit_deg=cfg['VEHICLE_LEFT_STEER_DEG'],
                right_front_wheel_limit_deg=cfg['VEHICLE_RIGHT_STEER_DEG'],
                servo_center_deg=cfg['VEHICLE_SERVO_CENTER_DEG'],
                servo_limits_deg=[cfg['VEHICLE_SERVO_MIN_DEG'],cfg['VEHICLE_SERVO_MAX_DEG']])
(nav/'map_metadata.json').write_text(json.dumps(metadata, indent=2)+'\n', encoding='ascii')
print(json.dumps(report, indent=2))
