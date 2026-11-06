"""Verify final navigation RTL, hardware outputs and recorded regressions."""
from pathlib import Path
import datetime
import hashlib
import json
import re
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'navigation_followup_20261026'
ALLOWED = {'src/vehicle/vehicle_motion.v', 'src/vehicle/vehicle_route_profile.v',
           'src/vehicle/vehicle_subsystem.v', 'src/vehicle/vehicle_vision_fusion.v'}

def read(name):
    data = (ROOT / name).read_bytes()
    encoding = 'utf-16' if data.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig'
    return data.decode(encoding).replace('\r\n', '\n')

def sha(name):
    return hashlib.sha256((ROOT / name).read_bytes()).hexdigest()

def dump(name, value):
    (ROOT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

def newer(output, inputs):
    assert (ROOT / output).stat().st_mtime >= max((ROOT / n).stat().st_mtime for n in inputs), ('stale', output)

snapshot = f'impl/{LABEL}_inputs.json'
frozen = json.loads(read(snapshot))['files']
hardware = [p for p in (ROOT / 'src').rglob('*') if p.is_file() and p.suffix in
            ('.v', '.vo', '.vh', '.mem', '.cst', '.sdc', '.ipc', '.mod')]
hardware += [ROOT / 'ov5640.gprj', ROOT / 'build.tcl']
actual = {p.relative_to(ROOT).as_posix(): sha(p) for p in hardware}
assert frozen == actual, 'Hardware sources changed after build freeze'
old = json.loads(read('impl/navigation_entry_20261026_inputs.json'))['files']
changed = sorted(n for n, h in actual.items() if old.get(n) != h)
assert set(changed) == ALLOWED, changed
backup = 'impl/baseline_navigation_followup_20261026'
manifest = json.loads(read(backup + '/manifest.json'))
assert all(sha(backup + '/' + n) == h for n, h in manifest.items()), 'Rollback snapshot changed'
metrics = json.loads(read(f'impl/{LABEL}_build_metrics.json'))
assert metrics['exit_code'] == 0
build_name = f'impl/{LABEL}_build.log'
build = read(build_name)
assert 'Bitstream generation completed' in build and 'Placement and routing completed' in build
assert not re.search(r'^\s*ERROR\b', build, re.M)
fs = 'impl/pnr/ov5640.fs'
for output in (fs, 'impl/gwsynthesis/ov5640.vg', 'impl/pnr/ov5640.rpt.txt', 'impl/pnr/ov5640.tr'):
    newer(output, list(actual) + [snapshot])
assert sha(fs) != manifest[fs], 'Bitstream equals the original released version'
timing, report = read('impl/pnr/ov5640.tr'), read('impl/pnr/ov5640.rpt.txt')
violations, slack = {}, {}
for kind in ('Setup', 'Hold'):
    match = re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)
    assert match, kind
    violations[kind] = int(match[1])
for kind in ('Setup', 'Hold', 'Recovery', 'Removal'):
    table = re.search(r'^3\.1\.\d ' + kind + r' Paths Table\n(.*?)(?=\n3\.)', timing, re.M | re.S)
    assert table, kind
    values = [float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s', table[1], re.M)]
    assert values, kind
    slack[kind] = min(values)
assert not any(violations.values()) and min(slack.values()) >= 0, (violations, slack)
prior = json.loads(read('docs/navigation_entry_20261026_validation.json'))
resources = {}
for name in ('Logic', 'Register', 'CLS', 'BSRAM', 'DSP', 'I/O Port'):
    match = re.search(r'^\s*' + re.escape(name) + r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|', report, re.M)
    assert match, name
    used, total = map(float, match.groups())
    assert used <= total
    resources[name] = dict(used=used, total=total, percent=round(100*used/total, 2),
                           delta_from_previous=used-prior['resources'][name]['used'])
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', read('src/ov5640_ddr3_tft.cst'), re.M))
placed = dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
assert pins and len(pins) == len(set(pins.values()))
assert all(placed.get(n) == p for n, p in pins.items()), 'Pin assignment mismatch'
plant = [p.relative_to(ROOT).as_posix() for p in (ROOT / 'src/vehicle').glob('*.v')]
plant += ['src/vehicle/vehicle_config.vh', 'src/imu/imu_math.v']
fusion = ['src/vehicle/vehicle_vision_fusion.v', 'src/vehicle/vehicle_config.vh']
regressions = {}

def check(name, log, count, dependencies, digest=None):
    content = read(log)
    assert content.count('$finish called') == count, (name, content.count('$finish called'))
    assert not re.search(r'^\s*(FATAL|FAIL|ERROR|Traceback)\b', content, re.M), name
    assert 'PASS ' in content
    newer(log, dependencies)
    if digest:
        assert sha(log) == digest, (name, 'log hash mismatch')
    regressions[name] = dict(passed=True, completed_runs=count, log=log, sha256=sha(log))

corners = json.loads(read('sim/navigation_followup_corner_runs.json'))
assert len(corners) == 22
check('corners', 'sim/navigation_followup_corner_results.log', 22, plant)
for case in corners:
    deps = fusion if case['top'] in ('navigation_corner_tb', 'navigation_response_tb', 'vision_fusion_tb', 'vision_blind_turn_tb') else plant
    newer(f'sim/navigation_followup_corner_{case["top"]}_{case["variant"]}.vvp', deps + [f'sim/{case["top"]}.sv'])
routes = json.loads(read('sim/navigation_followup_route_runs.json'))
phases = json.loads(read('sim/navigation_followup_phase_runs.json'))
assert len(routes) == 25 and len(phases) == 8
for case in routes + phases:
    assert case['passed'] and case['max_nose_error_mm'] <= 50
    if case.get('handback'):
        assert case['handbacks'] > 0
    check('route_' + case['name'], case['log'], 1, plant + ['sim/navigation_route_camera_tb.sv'], case['sha256'])
targeted = json.loads(read('sim/navigation_followup_targeted_runs.json'))
extras = json.loads(read('sim/navigation_followup_extras_runs.json'))
assert len(targeted) == 6 and len(extras) == 3
for case in targeted + extras:
    assert case['passed']
    name = case['name']
    deps = plant if name == 'vehicle' else (
        ['src/vehicle/vehicle_route_profile.v', 'src/imu/imu_math.v', 'src/vehicle/vehicle_config.vh'] if name == 'profile' else
        ['src/portrait/vision_zebra.v'] if name == 'zebra' else
        fusion + ['src/portrait/vision_video.v', 'src/portrait/vision_zebra.v', 'src/portrait/vision_camera_config.vh', 'src/portrait/vision_ipm_lut.vh'] if name == 'video' else fusion)
    check('targeted_' + name, case['log'], 1, deps, case['sha256'])
check('synthesized_handoff', 'sim/navigation_followup_handoff_netlist.log', 1, fusion)
check('synthesized_profile', 'sim/navigation_followup_profile_netlist.log', 1,
      ['src/vehicle/vehicle_route_profile.v', 'src/imu/imu_math.v', 'sim/navigation_profile_corner_tb.sv'])
gates = json.loads(read('sim/navigation_followup_gate_runs.json'))
assert len(gates) == 3
for case in gates:
    assert case['passed']
    check('synthesized_fault_' + str(case['case']), case['log'], 1, fusion, case['sha256'])
reproduction = json.loads(read('sim/navigation_followup_20261026_fault_runs.json'))
assert len(reproduction) == 6
for case in reproduction:
    assert sha(case['log']) == case['sha256']
    if case['expected_failure']:
        assert case['exit_code'] != 0 and 'FATAL:' in read(case['log'])
    else:
        assert case['passed'] and case['exit_code'] == 0
        check('reported_fault_' + case['name'], case['log'], 1, fusion)
netlist = read('impl/gwsynthesis/ov5640.vg')
for module in ('vehicle_motion', 'vehicle_route_profile', 'vehicle_vision_fusion', 'vehicle_bypass',
               'vision_video', 'lsm6dsv16x_spi', 'vl53l1x_array_controller'):
    assert re.search(r'^module ' + module + r'\b', netlist, re.M), module
roms = dict(map=audit_map(), font=audit_font(), road=audit_road())
generated = datetime.datetime.fromtimestamp((ROOT / fs).stat().st_mtime, datetime.timezone.utc).isoformat()
result = dict(
    status='digital_verification_passed', recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    bitstream=fs, sha256=sha(fs), bytes=(ROOT / fs).stat().st_size, generated_utc=generated,
    tool='Gowin V1.9.12.03', device='GW5AT-LV60PG484AC1/I0 B', hardware_tested=False, flashed=False,
    synthesis_input_manifest=snapshot, input_count=len(actual), input_sha256=actual, source_inputs_unchanged=True,
    changed_baseline_hardware_inputs=changed,
    unchanged_hardware_source_count=sum(n.startswith('src/') and old.get(n)==h for n,h in actual.items()),
    resources=resources, violated_endpoints=violations, min_slack_ns=slack, timing_passed=True,
    pins_verified=pins, roms=roms, regression_completed_runs=sum(c['completed_runs'] for c in regressions.values()),
    regressions=regressions, actual_route_runs=routes, camera_phase_runs=phases, reproduced_before_fix=reproduction,
    build=dict(**metrics, log=build_name, log_sha256=sha(build_name), preserved_other_module_placement=False),
    rollback=dict(bitstream=backup+'/'+fs, sha256=manifest[fs], clean_complete_snapshot=True),
    navigation=dict(controller_hz=100, startup_stable_frames=3, startup_steer_settle_ms=120,
                    entry_lead_mm=8, corner_cache_bits_per_point=38,
                    route_index_is_address_not_arrival=True, physical_entry_position_latched=True,
                    forward_s_target_countersteer_allowed=True, behind_target_wrong_turn_guard=True,
                    servo_limits_unchanged=[40, 150], wheel_limits_unchanged=[-28, 28]),
    preserved=dict(bypass_source=True, sensor_drivers=True, motor_control=True, servo_calibration=True,
                   zebra_source=True, image_processing=True, planner_geometry=True),
    limitations=['Not flashed; no physical-car test.',
                 'Synthetic camera routes do not reproduce every physical scene; camera intrinsics remain provisional.',
                 'Simulated errors refer to nose-centre position, not whole-body swept clearance.',
                 'Incremental attempt was stopped to add the physical-entry fix; final output uses standard PNR.',
                 'Other modules source code is preserved; their physical placement/routes are not frozen.',
                 'Existing synthesis warnings remain.'])
assert result['regression_completed_runs'] == 72, result['regression_completed_runs']
for name in (f'docs/{LABEL}_validation.json', 'impl/pnr/ov5640_navigation_followup_audit.json', 'impl/pnr/ov5640_current_audit.json'):
    dump(name, result)
dump('impl/pnr/ov5640_download.json', {k: result[k] for k in
     ('bitstream', 'sha256', 'bytes', 'generated_utc', 'tool', 'device', 'timing_passed', 'hardware_tested', 'flashed')})
print(json.dumps({k: result[k] for k in ('status', 'sha256', 'regression_completed_runs', 'resources',
                                      'violated_endpoints', 'min_slack_ns', 'build')}, indent=2))
