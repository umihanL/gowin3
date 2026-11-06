"""Audit settings/response/geometry build after verify_navigation_build.py."""
from pathlib import Path
import hashlib
import json
import math
import re

ROOT = Path(__file__).resolve().parents[1]


def read(rel):
    return (ROOT / rel).read_text(encoding='utf-8')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


baseline = json.loads(read('impl/settings_response_baseline.json'))
build = json.loads(read('impl/verified_build.json'))
assert digest(ROOT / build['bitstream']) == build['sha256']
assert build['setup_violated_endpoints'] == build['hold_violated_endpoints'] == 0
for rel, expected in build['input_sha256'].items():
    assert digest(ROOT / rel) == expected, 'Changed after build: ' + rel
old_sources = {rel for rel in baseline['sha256'] if rel.startswith('src/')}
current = {p.relative_to(ROOT).as_posix() for p in (ROOT / 'src').rglob('*') if p.is_file()}
assert old_sources <= current, 'Deleted source'
added = sorted(current - old_sources)
assert added == ['src/navigation/vehicle_settings.v'], added
changed = sorted(rel for rel in old_sources if digest(ROOT / rel) != baseline['sha256'][rel])
expected_changes = [
    'src/navigation/map_metadata.json', 'src/navigation/map_metadata.mem',
    'src/navigation/nav_control.v', 'src/navigation/nav_font.vh',
    'src/navigation/nav_map_init.vh', 'src/navigation/nav_metadata.vh',
    'src/navigation/nav_path_refiner.v', 'src/navigation/nav_renderer.v',
    'src/navigation/navigation_ui.v', 'src/ov5640_ddr3_tft.v',
    'src/vehicle/motor/motor_closed_loop.v', 'src/vehicle/motor/motor_pi.v',
    'src/vehicle/motor/motor_target_counts.v', 'src/vehicle/motor/ps2_test_control.v',
    'src/vehicle/rear_ratio.mem', 'src/vehicle/servo_offset.mem',
    'src/vehicle/vehicle_config.vh', 'src/vehicle/vehicle_subsystem.v',
]
assert changed == expected_changes, changed
for rel in ('build.tcl', 'src/ov5640_ddr3_tft.cst', 'src/ov5640_ddr3_tft.sdc',
            'src/imu/imu_firewater_tx.v', 'src/imu/imu_attitude.v', 'src/imu/lsm6dsv16x_spi.v'):
    assert digest(ROOT / rel) == baseline['sha256'][rel], 'Unexpected change: ' + rel
# Occupancy, heading-clearance and route search costs remain untouched.
for name in ('map_raw_1cm.mem', 'map_plan_2cm.mem', 'map_heading_2cm.mem', 'map_central_2cm.mem', 'astar_grid.v'):
    rel = 'src/navigation/' + name
    assert digest(ROOT / rel) == baseline['sha256'][rel], rel

cfg = {k: int(v) for k, v in re.findall(r'^`define (VEHICLE_\w+) (-?\d+)$', read('src/vehicle/vehicle_config.vh'), re.M)}
for name, value in dict(WHEELBASE_MM=138, REAR_TRACK_MM=168, FRONT_TRACK_MM=175,
                        WHEEL_DIAMETER_MM=66, COUNTS_PER_REV=1560,
                        SERVO_CENTER_DEG=90, SERVO_SIGN=1, LEFT_STEER_DEG=36,
                        RIGHT_STEER_DEG=34, MAX_STEER_DEG=34, MIN_RADIUS_MM=205,
                        MAX_SPEED_MM_S=1000, MOTOR1_IS_LEFT=1, ENCODER1_INVERT=0,
                        ENCODER2_INVERT=0).items():
    assert cfg['VEHICLE_' + name] == value, name
ratios = [int(x, 16) for x in read('src/vehicle/rear_ratio.mem').split()]
for angle in range(37):
    radius = 138 / math.tan(math.radians(angle)) if angle else math.inf
    expected = round((radius - 84) / (radius + 84) * 256) if angle else 256
    assert ratios[angle] == expected, angle
servo = [int(x, 16) for x in read('src/vehicle/servo_offset.mem').split()]
assert len(servo) == 128
for offset, end in ((0, 36), (64, 34)):
    assert servo[offset:offset + 64] == [min(70, (a * 70 + end // 2) // end) for a in range(64)]
dimensions = [int(x, 16) for x in read('src/navigation/map_metadata.mem').split()]
assert dimensions[56:59] == [138, 34, 205] and dimensions[62] == 512

cst = read('src/ov5640_ddr3_tft.cst')
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', cst, re.M))
report = read('impl/pnr/ov5640.rpt.txt')
actual = dict(re.findall(r'^([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
assert len(pins) == 116 and all(actual.get(k) == v for k, v in pins.items())
assert pins['tx'] == 'M15'

# The exact seven-channel UART instance is unchanged from the verified baseline.
top = read('src/ov5640_ddr3_tft.v')
old_top = (Path(baseline['backup']) / 'src/ov5640_ddr3_tft.v').read_text(encoding='utf-8')
uart_pattern = r'    imu_firewater_tx #\(.*?\) u_imu_uart\(.*?\);'
assert re.search(uart_pattern, top, re.S)[0] == re.search(uart_pattern, old_top, re.S)[0]
netlist = read('impl/gwsynthesis/ov5640.vg')
instances = {}
for name in ('u_imu_uart', 'u_vehicle', 'u_navigation_ui'):
    match = re.search(r'\b\w+\s+' + name + r'\s*\(.*?\);', netlist, re.S)
    assert match, name
    instances[name] = match[0]
for signal in ('imu_roll', 'imu_pitch', 'imu_yaw', 'motor_target1', 'motor_actual1', 'motor_target2', 'motor_actual2'):
    assert f'.{signal}({signal}[15:0])' in instances['u_imu_uart']
for signal in ('configured_speed', 'touch_steer', 'touch_run', 'reverse_drive', 'control_remote'):
    for name in ('u_vehicle', 'u_navigation_ui'):
        assert signal in instances[name], (signal, name)

logs = {}
checks = {
    'sim/settings_test_results.txt': ('PASS settings:', 'PASS motor response:', '2001 signed speed conversions'),
    'sim/vehicle_test_results.txt': ('PASS gradual stop', 'PASS route execution', 'PASS Ackermann curve tracking',
                                  'PASS obstacle bypass', 'PASS route profile', 'PASS integrated I/O',
                                  '600mm/s beyond old cap', 'manual/remote hardware arbitration'),
    'sim/navigation_test_results.txt': ('PASS A*:', 'PASS Ackermann: 20.5cm', 'PASS map ROM:', 'PASS UI:'),
    'sim/imu_test_results.txt': ('exactly seven TX-wire channels',),
}
for rel, required in checks.items():
    content = read(rel)
    assert not re.search(r'^(?:FAIL|FATAL|ERROR)\b', content, re.I | re.M), rel
    assert all(s in content for s in required), rel
    logs[rel] = dict(sha256=digest(ROOT / rel), output=content)

audit_path = 'impl/pnr/ov5640_settings_audit.json'
audit = {k: build[k] for k in ('project', 'bitstream', 'sha256', 'generated_time_utc8',
         'bitstream_bytes', 'resources', 'setup_violated_endpoints', 'hold_violated_endpoints')}
remaining = {name: values['available'] - values['used'] for name, values in build['resources'].items()}
audit.update(
    feature='Manual/remote settings, shared 0..1000mm/s speed, faster PI, calibrated steering slew and rear differential',
    baseline='impl/settings_response_baseline.json',
    previous_production_sha256=baseline['sha256']['impl/pnr/ov5640.fs'],
    existing_source_changes=changed, added_sources=added,
    unchanged_existing_source_files=len(old_sources) - len(changed),
    cst_sha256=digest(ROOT / 'src/ov5640_ddr3_tft.cst'),
    sdc_sha256=digest(ROOT / 'src/ov5640_ddr3_tft.sdc'),
    pin_constraints_unchanged=True, pins_verified_from_pnr_report=pins,
    geometry=cfg, resources_remaining=remaining,
    defaults=dict(control='manual', speed_mm_s=200, running=False),
    speed_limits_mm_s=[0, 1000], fine_step_mm_s=5, coarse_step_mm_s=50,
    speed_shared_between_touch_and_ps2=True,
    ps2=dict(edit_toggle='circle', increment='R1', decrement='R2', held_key_repeat=False,
             editing_disarms_drive=True, reconnect_requires_fresh_start=True),
    automatic_motion_speed_limit_mm_s=200,
    steering_rate_limit_road_deg_s=100, intermediate_servo_linkage='provisional linear endpoint calibration',
    minimum_planning_radius_mm=205, smoothing_passes=512,
    perfect_no_slip_validated=False, hardware_tested=False,
    pid_auto_identification=False, pi_gain_scheduling=True,
    seven_channel_firewater_unchanged=True,
    output_channels=['Roll', 'Pitch', 'Yaw', 'M1 target', 'M1 actual', 'M2 target', 'M2 actual'],
    regression_logs=logs,
    verification_sha256={rel: digest(ROOT / rel) for rel in (
        'sim/vehicle_settings_tb.sv', 'sim/motor_response_tb.sv', 'sim/motor_pi_before_settings.v',
        'sim/run_settings_tests.py', 'sim/vehicle_io_tb.sv', 'sim/vehicle_motion_tb.sv',
        'tools/verify_settings_build.py', 'tools/generate_vehicle_assets.py')})
(ROOT / audit_path).write_text(json.dumps(audit, indent=2) + '\n', encoding='utf-8')
build.update(current_change=audit['feature'], current_change_audit=audit_path, resources_remaining=remaining)
(ROOT / 'impl/verified_build.json').write_text(json.dumps(build, indent=2) + '\n', encoding='utf-8')
download_path = ROOT / 'impl/pnr/ov5640_download.json'
download = json.loads(download_path.read_text(encoding='utf-8'))
download.update({k: build[k] for k in ('sha256', 'generated_time_utc8', 'bitstream_bytes',
                'setup_violated_endpoints', 'hold_violated_endpoints')})
download.update(current_image=audit['feature'], build_audit=audit_path,
                hardware_tested=False, pin_constraints_unchanged=True,
                output_channels=audit['output_channels'])
download_path.write_text(json.dumps(download, indent=2) + '\n', encoding='utf-8')
resource_doc = ROOT / 'docs/resource_usage.md'
resource_content = resource_doc.read_text(encoding='utf-8').split('\n## Remaining resources\n',1)[0]
resource_content += '\n## Remaining resources\n\n| Resource | Remaining |\n| --- | --- |\n'
resource_content += ''.join(f'| {name} | {count:g} |\n' for name, count in remaining.items())
resource_content += '\nCurrent feature: manual/remote settings, shared speed, faster PI and calibrated differential.\n'
resource_doc.write_text(resource_content, encoding='utf-8')
assert digest(ROOT / build['bitstream']) == build['sha256'] == audit['sha256'] == download['sha256']
print(json.dumps(dict(audit=audit_path, sha256=build['sha256'], pins_checked=len(pins),
                     resources=build['resources'], remaining=remaining, source_changes=changed), indent=2))
