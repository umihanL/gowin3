"""Audit the frozen navigation-authority build and its regression evidence."""
from pathlib import Path
import datetime
import hashlib
import json
import re

from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'navigation_authority_20261026'


def read(name):
    data = (ROOT / name).read_bytes()
    encoding = 'utf-16' if data.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig'
    return data.decode(encoding).replace('\r\n', '\n')


def sha(name):
    return hashlib.sha256((ROOT / name).read_bytes()).hexdigest()


def dump(name, data):
    (ROOT / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def newer(output, dependencies):
    assert (ROOT / output).stat().st_mtime >= max((ROOT / n).stat().st_mtime for n in dependencies), ('Stale output', output)


snapshot_name = f'impl/{LABEL}_inputs.json'
snapshot = json.loads(read(snapshot_name))
hardware = [p for p in (ROOT / 'src').rglob('*') if p.is_file() and p.suffix in
            ('.v', '.vo', '.vh', '.mem', '.cst', '.sdc', '.ipc', '.mod')]
hardware += [ROOT / 'ov5640.gprj', ROOT / 'build.tcl']
actual = {p.relative_to(ROOT).as_posix(): sha(p) for p in hardware}
assert actual == snapshot['files'], 'Hardware inputs differ from frozen build'
old = json.loads(read('impl/navigation_ipm_20261026_inputs.json'))['files']
changed = sorted(n for n, h in actual.items() if old.get(n) != h)
expected = {'src/portrait/vision_video.v', 'src/vehicle/vehicle_vision_fusion.v'}
assert set(changed) == expected and not set(old) - set(actual), changed
backup_name = 'impl/baseline_navigation_authority_20261026'
backup = json.loads(read(f'{backup_name}/manifest.json'))
assert all(sha(f'{backup_name}/{n}') == h for n, h in backup.items()), 'Backup integrity failed'

metrics = json.loads(read(f'impl/{LABEL}_build_metrics.json'))
assert metrics['exit_code'] == 0 and metrics['mode'] == 'standard_syn_and_pnr'
assert not metrics['incremental_checkpoint_used']
build_name = f'impl/{LABEL}_build.log'
build = read(build_name)
assert 'Bitstream generation completed' in build
assert not re.search(r'^\s*ERROR\b', build, re.M), 'Build contains errors'
assert 'Prior incremental matching was slower; use standard implementation' in build
command = read('impl/pnr/cmd.do')
assert '-inc_pnr ' not in command or '-inc_pnr 0' in command
fs_name = 'impl/pnr/ov5640.fs'
for name in (fs_name, 'impl/gwsynthesis/ov5640.vg', 'impl/pnr/ov5640.rpt.txt', 'impl/pnr/ov5640.tr'):
    newer(name, list(actual) + [snapshot_name])

report = read('impl/pnr/ov5640.rpt.txt')
timing = read('impl/pnr/ov5640.tr')
resources = {}
prior_audit = json.loads(read('docs/navigation_ipm_20261026_validation.json'))
for name in ('Logic', 'Register', 'CLS', 'BSRAM', 'DSP', 'I/O Port'):
    match = re.search(r'^\s*' + re.escape(name) + r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|', report, re.M)
    assert match, name
    used, total = map(float, match.groups())
    assert used <= total, name
    resources[name] = dict(used=used, total=total, percent=round(100 * used / total, 2),
                           delta_from_previous=used - prior_audit['resources'][name]['used'])
violations, slack = {}, {}
for kind in ('Setup', 'Hold'):
    match = re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)
    assert match, kind
    violations[kind] = int(match[1])
for kind in ('Setup', 'Hold', 'Recovery', 'Removal'):
    match = re.search(r'^3\.1\.\d ' + kind + r' Paths Table\n(.*?)(?=\n3\.)', timing, re.M | re.S)
    assert match, kind
    values = [float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s', match[1], re.M)]
    assert values, kind
    slack[kind] = min(values)
assert not any(violations.values()) and min(slack.values()) >= 0, (violations, slack)
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', read('src/ov5640_ddr3_tft.cst'), re.M))
placed = dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
assert pins and len(pins) == len(set(pins.values()))
assert all(placed.get(n) == p for n, p in pins.items()), 'Pin mismatch'
for name in ('impl/gwsynthesis/ov5640.vg', 'src/ov5640_ddr3_tft.cst', 'src/ov5640_ddr3_tft.sdc'):
    assert str(ROOT / name) in report, name

regressions = {}
vehicle = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]
plant = vehicle + ['src/vehicle/vehicle_config.vh', 'src/imu/imu_math.v']
fusion = ['src/vehicle/vehicle_vision_fusion.v', 'src/vehicle/vehicle_config.vh']
video = ['src/portrait/vision_video.v', 'src/portrait/vision_zebra.v',
         'src/portrait/vision_camera_config.vh', 'src/portrait/vision_ipm_lut.vh']


def check(name, log, count, markers, dependencies):
    content = read(log)
    assert content.count('$finish called') == count, (name, content.count('$finish called'))
    assert not re.search(r'^\s*(FATAL|FAIL|ERROR|Traceback)\b', content, re.M), name
    assert all(marker in content for marker in markers), name
    newer(log, dependencies)
    regressions[name] = dict(passed=True, completed_runs=count, log=log, sha256=sha(log))


corners = json.loads(read('sim/navigation_authority_corner_runs.json'))
assert len(corners) == 22
check('corners', 'sim/navigation_authority_corner_results.log', 22,
      ['PASS committed quarter turn side=-1:', 'PASS committed quarter turn side=1:',
       'PASS terminal arrival:'], plant)
for case in corners:
    top = case['top']
    newer(f'sim/navigation_authority_corner_{top}_{case["variant"]}.vvp',
          plant + [f'sim/{top}.sv'])

routes = json.loads(read('sim/navigation_authority_route_runs.json'))
assert len(routes) == 21
for case in routes:
    assert case['passed'] and case['max_nose_error_mm'] <= 50
    assert sha(case['log']) == case['sha256']
    if case['handback']:
        assert case['handbacks'] > 0
    check('route_'+case['name'], case['log'], 1, ['PASS actual graph navigation'],
          plant + ['sim/navigation_route_camera_tb.sv', f'sim/navigation_robust_{case["route"]}_route.csv'])

targeted = json.loads(read('sim/navigation_authority_targeted_runs.json'))
extras = json.loads(read('sim/navigation_authority_extras_runs.json'))
assert [c['name'] for c in targeted] == ['authority_0', 'authority_1', 'authority_2', 'authority_3', 'handoff', 'video']
assert [c['name'] for c in extras] == ['vehicle', 'profile', 'zebra']
for case in targeted + extras:
    assert case['passed'] and sha(case['log']) == case['sha256']
    check('targeted_'+case['name'], case['log'], 1, ['PASS '],
          plant + video + [f'sim/{case["top"]}.sv'])
check('synthesized_handoff', 'sim/navigation_authority_handoff_netlist.log', 1,
      ['PASS confirmed route turn:', 'PASS S reversal:', 'PASS bounded recovery:'],
      fusion + ['sim/navigation_handoff_tb.sv'])
gate = json.loads(read('sim/navigation_authority_gate_runs.json'))
assert len(gate) == 4
for case in gate:
    assert case['passed'] and sha(case['log']) == case['sha256']
    check('synthesized_fault_'+str(case['case']), case['log'], 1, ['PASS '],
          fusion + ['sim/navigation_authority_tb.sv'])
check('synthesized_video', 'sim/navigation_authority_video_netlist.log', 1,
      ['PASS synthesized vision:', '384000 preview pixels', '6 normalized lane arithmetics'],
      video + ['sim/vision_netlist_tb.sv'])

reference = read('impl/vision_probe/vision_video_reference.v')
source = read('src/portrait/vision_video.v').replace('module vision_video #(', 'module vision_video_reference #(')
source = source.replace('vision_zebra zebra(', 'vision_zebra_reference zebra(')
source = re.sub(r'wire \[6:0\] source_column=scale_lut\[view_u\],source_row=scale_lut\[view_v\];',
                'wire [6:0] source_column=(view_u*13)/60,source_row=(view_v*13)/60;', source)
assert reference.startswith(source), 'Synthesized video reference is stale'
netlist = read('impl/gwsynthesis/ov5640.vg')
for module in ('vision_video', 'vehicle_motion', 'vehicle_vision_fusion', 'vehicle_bypass',
               'lsm6dsv16x_spi', 'vl53l1x_array_controller'):
    assert re.search(r'^module '+module+r'\b', netlist, re.M), module
roms = dict(map=audit_map(), font=audit_font(), road=audit_road())

reproduction = json.loads(read('sim/navigation_authority_reproduction.json'))
assert len(reproduction) == 3 and all(c['exit_code'] != 0 for c in reproduction)
assert all(c['fusion_sha256'] == backup['src/vehicle/vehicle_vision_fusion.v'] for c in reproduction)
assert 'EARLY_MAP_TURN' in read('sim/navigation_authority_before_0.log')
assert 'OFFCENTRE_EXIT_LOCKOUT' in read('sim/navigation_authority_before_1.log')
assert 'CORNER_CORRIDOR_ABORT' in read('sim/navigation_authority_before_2.log')
generated = datetime.datetime.fromtimestamp((ROOT/fs_name).stat().st_mtime, datetime.timezone.utc).isoformat()
result = dict(
    status='digital_verification_passed', recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    bitstream=fs_name, sha256=sha(fs_name), bytes=(ROOT/fs_name).stat().st_size, generated_utc=generated,
    tool='Gowin V1.9.12.03', device='GW5AT-LV60PG484AC1/I0 B', hardware_tested=False, flashed=False,
    synthesis_input_manifest=snapshot_name, input_count=len(actual), input_sha256=actual,
    source_inputs_unchanged=True, changed_baseline_hardware_inputs=changed,
    unchanged_hardware_source_count=sum(n.startswith('src/') and old.get(n) == h for n, h in actual.items()),
    resources=resources, violated_endpoints=violations, min_slack_ns=slack, timing_passed=True,
    pins_verified=pins, roms=roms, camera=json.loads(read('docs/camera_parameters.json')),
    regression_completed_runs=sum(c['completed_runs'] for c in regressions.values()),
    regression_count_note='Completed simulation invocations; includes repeated benches in separate suites.',
    regressions=regressions, actual_route_runs=routes, reproduced_before_fix=reproduction,
    build=dict(**metrics, log=build_name, log_sha256=sha(build_name), preserved_other_module_placement=False),
    rollback=dict(bitstream=f'{backup_name}/impl/pnr/ov5640.fs', sha256=backup['impl/pnr/ov5640.fs'],
                  clean_complete_snapshot=True),
    navigation=dict(visible_straight_overrides_future_map_curve=True,
                    map_curve_needs_matching_near_visual_evidence=True,
                    unconditional_junction_steer_bias_removed=True,
                    offcentre_single_edge_exit_can_reacquire=True,
                    imu_latched_exit_survives_midturn_corridor_gap=True,
                    invalid_corridor_exit_reacquisition_grace_ms=500,
                    countersteer_unwind_deg_per_10ms=1.5,
                    unchanged_actuator_slew_deg_per_10ms=1,
                    near_boundary_scan_rows=12, near_boundary_forward_mm=[172,288],
                    controller_hz=100, pd_gains_unchanged=True, additional_frame_ram=False),
    preserved=dict(bypass_source=True, sensor_drivers=True, motor_control=True,
                   servo_calibration_and_limits=True, zebra_source=True),
    limitations=['Not flashed or tested on the physical car.',
                 'No user camera photos or native calibration pixel coordinates were available locally.',
                 'Camera intrinsics remain provisional; synthetic images do not reproduce every real road/lighting condition.',
                 'Route error is the nose centre, not a guarantee of whole-body boundary clearance.',
                 'Standard PNR was faster in prior measurements; unchanged module placement is not guaranteed.',
                 'Existing synthesis warnings remain.'])
assert result['regression_completed_runs'] == 58, result['regression_completed_runs']
for name in (f'docs/{LABEL}_validation.json', 'impl/pnr/ov5640_navigation_authority_audit.json',
             'impl/pnr/ov5640_current_audit.json'):
    dump(name, result)
dump('impl/pnr/ov5640_download.json', {k: result[k] for k in
     ('bitstream', 'sha256', 'bytes', 'generated_utc', 'tool', 'device', 'timing_passed', 'hardware_tested', 'flashed')})
print(json.dumps({k:result[k] for k in ('status','sha256','regression_completed_runs','resources',
                                     'violated_endpoints','min_slack_ns','build')}, indent=2))

