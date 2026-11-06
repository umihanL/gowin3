"""Audit the frozen IPM build, completed regressions and generated bitstream."""
from pathlib import Path
import datetime
import hashlib
import json
import re

from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'navigation_ipm_20261026'


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
old = json.loads(read('impl/navigation_robust_20261025_inputs.json'))['files']
changed = sorted(n for n, h in actual.items() if old.get(n) != h)
expected = {'src/portrait/vision_video.v', 'src/portrait/vision_camera_config.vh',
            'src/portrait/vision_ipm_lut.vh', 'src/vehicle/vehicle_config.vh',
            'src/vehicle/vehicle_vision_fusion.v', 'src/vehicle/vehicle_bypass.v',
            'src/vehicle/vehicle_motion.v'}
assert set(changed) == expected and not set(old) - set(actual), changed
backup_name = 'impl/baseline_ipm_20261025'
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
prior_audit = json.loads(read('docs/navigation_robust_20261025_validation.json'))
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


def check(name, log, count, markers, dependencies=(), executables=()):
    content = read(log)
    assert content.count('$finish called') == count, (name, content.count('$finish called'), count)
    assert not re.search(r'^\s*(FATAL|FAIL|ERROR|Traceback)\b', content, re.M), name
    assert all(m in content for m in markers), (name, markers)
    if dependencies:
        newer(log, dependencies)
        for executable in executables:
            newer(executable, dependencies)
    regressions[name] = dict(passed=True, completed_runs=count, log=log, sha256=sha(log))


vehicle = [p.relative_to(ROOT).as_posix() for p in (ROOT / 'src/vehicle').glob('*.v')]
motor = [p.relative_to(ROOT).as_posix() for p in (ROOT / 'src/vehicle/motor').glob('*.v')]
config = ['src/vehicle/vehicle_config.vh']
fusion = ['src/vehicle/vehicle_vision_fusion.v'] + config
plant = vehicle + config + ['src/imu/imu_math.v']
video = ['src/portrait/vision_video.v', 'src/portrait/vision_zebra.v',
         'src/portrait/vision_camera_config.vh', 'src/portrait/vision_ipm_lut.vh']

corners = json.loads(read('sim/navigation_corner_20261025_runs.json'))
assert len(corners) == 22
check('navigation_corners', 'sim/navigation_ipm_corner_suite.log', 22,
      ['PASS navigation response suite: 22 runs', 'PASS committed quarter turn side=-1:',
       'PASS committed quarter turn side=1:', 'PASS terminal arrival:'], plant)
for case in corners:
    top = case['top']
    deps = fusion if top in ('navigation_corner_tb', 'navigation_response_tb', 'vision_fusion_tb', 'vision_blind_turn_tb') else plant
    newer(f'sim/navigation_20261025_{top}_{case["variant"]}.vvp', deps + [f'sim/{top}.sv'])

routes = json.loads(read('sim/navigation_robust_20261025_runs.json'))
assert len(routes) == 21
for case in routes:
    assert case['passed'] and case['max_nose_error_mm'] <= 50
    assert sha(case['log']) == case['sha256']
    if case['handback']:
        assert case['handbacks'] > 0
    check('route_' + case['name'], case['log'], 1, ['PASS actual graph navigation'],
          plant + ['sim/navigation_route_camera_tb.sv', f'sim/navigation_robust_{case["route"]}_route.csv'],
          [f'sim/navigation_robust_{case["name"]}.vvp'])
check('physical_bypass_and_followup', 'sim/navigation_ipm_followup.log', 2,
      ['PASS followup straight:', 'PASS physical beams', 'PASS bypass edge case', 'PASS bypass missing side edge:'],
      plant + motor + ['sim/vehicle_followup_tb.sv'],
      ['sim/vehicle_followup_tb.vvp', 'sim/vehicle_close_bypass_tb.vvp'])
regressions['physical_bypass_and_followup']['bench_revision_note'] = (
    'Initial default run preceded the ENABLE_VISION-only bench extension; current default bench is rechecked below.')
check('physical_bypass_final', 'sim/navigation_ipm_bypass_final.log', 1,
      ['PASS physical beams', 'PASS bypass edge case', 'PASS bypass missing side edge:'],
      plant + motor + ['sim/vehicle_close_bypass_tb.sv'], ['sim/navigation_ipm_bypass_final.vvp'])

extras = json.loads(read('sim/navigation_corner_extras_20261025_runs.json'))
assert [c['name'] for c in extras] == ['video', 'vehicle', 'profile']
for name, bench, deps, markers in [
    ('video', 'vision_video_tb', video + fusion, ['PASS video-to-fusion sequence:', 'PASS dense contours:']),
    ('vehicle', 'vehicle_motion_tb', plant + motor, ['PASS smooth low-speed stop:', 'PASS navigation dynamic obstacle:', 'PASS committed bypass:']),
    ('profile', 'vehicle_profile_tb', plant + motor, ['PASS route profile:'])]:
    check('extra_' + name, f'sim/navigation_corner_{name}_20261025.log', 1, markers,
          deps + [f'sim/{bench}.sv'], [f'sim/navigation_corner_{name}.vvp'])

special = json.loads(read('sim/navigation_ipm_special_runs.json'))
assert [c['name'] for c in special] == ['first_frame', 'single_front', 'dual_front', 'handoff_rtl',
                                      'vision_rejoin', 'motion_regression', 'profile']
for case in special:
    assert case['passed'] and sha(case['log']) == case['sha256']
    check('special_' + case['name'], case['log'], 1, ['PASS '], plant,
          [f'sim/navigation_ipm_{case["name"]}.vvp'])
check('vision_remaining', 'sim/navigation_ipm_vision_remaining.log', 14,
      ['PASS video-to-fusion sequence:', 'PASS dense contours:', 'PASS zebra speed:',
       'PASS vision UI:', 'PASS motor brake plant:', 'PASS terminal arrival:'], video + plant + motor)
check('font_and_display', 'sim/navigation_ipm_vision_suite.log', 2,
      ['PASS font rows:', 'PASS vision display timing:'],
      ['src/navigation/nav_renderer.v', 'src/imu/attitude_overlay.v', 'src/vga/disp_driver.v'])
check('synthesized_fusion_handoff', 'sim/navigation_ipm_handoff_netlist.log', 1,
      ['PASS synthesized map/vision handoff', 'PASS persistent fragments mode=2:', 'PASS S reversal:',
       'PASS bounded recovery:'], fusion + ['sim/navigation_handoff_tb.sv'])
check('synthesized_video', 'sim/navigation_ipm_vision_netlist.log', 1,
      ['PASS synthesized vision:', '384000 preview pixels', '6 normalized lane arithmetics'],
      video + ['sim/vision_netlist_tb.sv'], ['sim/vision_netlist_tb.vvp'])
reference = read('impl/vision_probe/vision_video_reference.v')
source = read('src/portrait/vision_video.v').replace('module vision_video #(', 'module vision_video_reference #(')
source = source.replace('vision_zebra zebra(', 'vision_zebra_reference zebra(')
source = re.sub(r'wire \[6:0\] source_column=scale_lut\[view_u\],source_row=scale_lut\[view_v\];',
                'wire [6:0] source_column=(view_u*13)/60,source_row=(view_v*13)/60;', source)
assert reference.startswith(source), 'Synthesized video reference is stale'

netlist = read('impl/gwsynthesis/ov5640.vg')
for module in ('vision_video', 'vehicle_motion', 'vehicle_vision_fusion', 'vehicle_bypass',
               'lsm6dsv16x_spi', 'vl53l1x_array_controller'):
    assert re.search(r'^module ' + module + r'\b', netlist, re.M), module
roms = dict(map=audit_map(), font=audit_font(), road=audit_road())
camera = json.loads(read('docs/camera_parameters.json'))
assert camera['measured_ground_calibration']['middle_three_forward_mm'] == 400
assert not camera['intrinsics_calibrated']
generated = datetime.datetime.fromtimestamp((ROOT / fs_name).stat().st_mtime, datetime.timezone.utc).isoformat()
result = dict(
    status='digital_verification_passed', recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    bitstream=fs_name, sha256=sha(fs_name), bytes=(ROOT / fs_name).stat().st_size, generated_utc=generated,
    tool='Gowin V1.9.12.03', device='GW5AT-LV60PG484AC1/I0 B', hardware_tested=False, flashed=False,
    synthesis_input_manifest=snapshot_name, input_count=len(actual), input_sha256=actual,
    source_inputs_unchanged=True, changed_baseline_hardware_inputs=changed,
    unchanged_hardware_source_count=sum(n.startswith('src/') and old.get(n) == h for n, h in actual.items()),
    resources=resources, violated_endpoints=violations, min_slack_ns=slack, timing_passed=True,
    pins_verified=pins, roms=roms, camera=camera,
    regression_completed_runs=sum(c['completed_runs'] for c in regressions.values()),
    regression_count_note='Completed simulation invocations; includes repeated benches in separate suites.',
    regressions=regressions, actual_route_runs=routes, special_runs=special,
    build=dict(**metrics, log=build_name, log_sha256=sha(build_name), preserved_other_module_placement=False),
    rollback=dict(bitstream=f'{backup_name}/impl/pnr/ov5640.fs', sha256=backup['impl/pnr/ov5640.fs'],
                  source_snapshot_note='This backup includes partial IPM experiments; use the accepted source manifest separately.'),
    navigation=dict(visual_centring_primary=True, imu_map_curve_recovery_retained=True,
                    road_near_units='mm at 210mm ahead of nose', road_heading_units='ground slope x110mm',
                    controller_hz=100, first_valid_frame_acquires=True, dead_zone_mm=1,
                    pd_gains_unchanged=True, additional_frame_ram=False),
    bypass=dict(single_forward_entry_latched=True, single_forward_side_threshold_mm=25,
                distinct_fresh_side_confirmations=2, normal_dual_forward_trend_retained=True,
                normal_min_lateral_offset_mm=45, single_forward_min_lateral_offset_mm=25,
                visual_rejoin_verified=True),
    zebra=dict(retained=True, navigation_speed_cap_mm_s=100, camera_to_tail_clearance_mm=305,
               slowdown_hold_and_restore_verified=True),
    limitations=[
        'Not flashed or tested on the physical car.',
        'No calibration photos or corresponding native pixel coordinates were available in this workspace.',
        'Horizontal FOV 60deg, square pixels and zero distortion are provisional; IPM has not been fitted to the marked points.',
        'Synthetic-image and independent geometric/bicycle tests do not guarantee real lighting, sensor noise or calibration performance.',
        'Route error figures measure the nose centre; full-body clearance was checked separately only in the physical bypass simulations.',
        'Existing encoder/IMU localization drift and camera blind regions are not eliminated.',
        'Standard PNR was used because previously measured incremental matching was slower; unchanged placement is not guaranteed.',
        'Existing synthesis warnings remain; this is not a zero-warning build.'])
assert result['regression_completed_runs'] == 74, result['regression_completed_runs']
for name in (f'docs/{LABEL}_validation.json', 'impl/pnr/ov5640_navigation_ipm_audit.json', 'impl/pnr/ov5640_current_audit.json'):
    dump(name, result)
dump('impl/pnr/ov5640_download.json', {k: result[k] for k in
     ('bitstream', 'sha256', 'bytes', 'generated_utc', 'tool', 'device', 'timing_passed', 'hardware_tested', 'flashed')})
print(json.dumps({k: result[k] for k in ('status', 'sha256', 'regression_completed_runs', 'resources',
                                      'violated_endpoints', 'min_slack_ns', 'build')}, indent=2))
