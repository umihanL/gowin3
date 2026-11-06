"""Audit the bend-boundary, obstacle-trend and terminal-goal build without regenerating RTL or flashing."""
from pathlib import Path
import datetime
import hashlib
import json
import re

from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    data = path.read_bytes()
    return data.decode('utf-16' if data.startswith((b'\xff\xfe', b'\xfe\xff'))
                       else 'utf-8-sig').replace('\r\n', '\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


snapshot_path = ROOT/'impl/turn_edge_goal_20261024_inputs.json'
snapshot = json.loads(read(snapshot_path))
inputs = [p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in
          ('.v', '.vo', '.vh', '.mem', '.cst', '.sdc', '.ipc', '.mod')]
inputs += [ROOT/'ov5640.gprj', ROOT/'build.tcl']
actual = {p.relative_to(ROOT).as_posix(): sha(p) for p in inputs}
assert actual == snapshot['files'], 'Current sources differ from frozen synthesis inputs'
fs = ROOT/'impl/pnr/ov5640.fs'
assert fs.stat().st_mtime >= max([snapshot_path.stat().st_mtime] +
                               [p.stat().st_mtime for p in inputs]), 'Stale bitstream'
report_path, timing_path = ROOT/'impl/pnr/ov5640.rpt.txt', ROOT/'impl/pnr/ov5640.tr'
assert min(report_path.stat().st_mtime, timing_path.stat().st_mtime) >= fs.stat().st_mtime
report, timing = read(report_path), read(timing_path)
assert str(ROOT/'impl/gwsynthesis/ov5640.vg') in report
for name in ('src/ov5640_ddr3_tft.cst', 'src/ov5640_ddr3_tft.sdc'):
    assert str(ROOT/name) in report
build_log = ROOT/'impl/turn_edge_goal_20261024_build.log'
assert 'Bitstream generation completed' in read(build_log)
assert not re.search(r'^\s*ERROR\b', read(build_log), re.M)

resources = {}
for name in ('Logic', 'Register', 'CLS', 'BSRAM', 'DSP', 'I/O Port'):
    m = re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|', report, re.M)
    assert m, name
    used, total = map(float, m.groups())
    assert used <= total, ('Resource overflow', name)
    resources[name] = dict(used=used, total=total, percent=round(100*used/total, 2))
violations, slack = {}, {}
for kind in ('Setup', 'Hold'):
    m = re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)
    assert m, kind
    violations[kind] = int(m[1])
for kind in ('Setup', 'Hold', 'Recovery', 'Removal'):
    m = re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)', timing, re.M | re.S)
    assert m, kind
    values = [float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s', m[1], re.M)]
    assert values, kind
    slack[kind] = min(values)
assert not any(violations.values()) and min(slack.values()) >= 0, (violations, slack)
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',
                       read(ROOT/'src/ov5640_ddr3_tft.cst'), re.M))
actual_pins = dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
assert pins and len(pins) == len(set(pins.values()))
assert all(actual_pins.get(n) == p for n, p in pins.items()), 'Pin mismatch'

vehicle = list((ROOT/'src/vehicle').rglob('*.v')) + [ROOT/'src/vehicle/vehicle_config.vh', ROOT/'src/imu/imu_math.v']
vision = list((ROOT/'src/portrait').glob('vision_*.v')) + [ROOT/'src/portrait/vision_camera_config.vh']
navigation = list((ROOT/'src/navigation').glob('*.v')) + list((ROOT/'src/navigation').glob('nav_centerline_*.vh'))
imu = list((ROOT/'src/imu').glob('*.v'))
specs = {
    'vision': ('vision_test_results.txt', 18, [
        'PASS dense contours:', 'PASS width-normalized 254/286mm lanes:', 'PASS vision display timing:',
        'PASS video-to-fusion sequence:', 'PASS terminal arrival:', 'PASS single-edge approach side=0:', 'PASS frame/goal qualification:', 'PASS vision video:', 'PASS vision fusion:', 'PASS limited-FOV turn side=0:',
        'PASS limited-FOV turn side=1:', 'PASS unseen exit:', 'PASS straight road loss:',
        'PASS integrated limited-FOV turn side=-1:', 'PASS integrated limited-FOV turn side=1:',
        'initialYaw=80 ', 'initialYaw=-80 ', 'PASS vision motion:', 'PASS vision UI:',
        'PASS motor brake plant:'], vehicle+vision+navigation,
        [ROOT/f'sim/{n}_tb.sv' for n in ('font_rows', 'vision_display_timing', 'vision_video',
         'vision_zebra', 'vision_fusion', 'vision_blind_turn', 'vision_turn_motion', 'vision_motion',
         'vision_goal', 'vision_ui', 'motor_brake', 'motor_brake_plant')], 'sim/run_vision_tests.py'),
    'imu': ('imu_test_results.txt', 17, [
        'PASS IMU full restart reference bridge:', 'PASS IMU burst recovery:',
        'PASS IMU local recovery:', 'PASS IMU dead engine:', 'PASS IMU SPI watchdog:',
        'PASS imu_spi bank fault 5:', 'PASS imu_motor_firewater:', 'PASS imu_rotation:'], imu,
        [ROOT/f'sim/imu_{n}_tb.v' for n in ('attitude', 'angle_limits', 'spi', 'ui', 'firewater',
         'motor_firewater', 'rotation')], 'sim/run_imu_tests.py'),
    'vehicle': ('vehicle_test_results.txt', 4, [
        'PASS vertical straight startup:', 'PASS committed bypass:', 'PASS smooth low-speed stop:',
        'PASS navigation dynamic obstacle:', 'PASS route profile:', 'PASS integrated I/O:'], vehicle,
        [ROOT/f'sim/vehicle_{n}_tb.sv' for n in ('math', 'motion', 'profile', 'io')], 'sim/run_vehicle_tests.py'),
    'centerline': ('centerline_test_results.txt', 9, [
        'PASS road planner:', 'PASS road UI:', 'PASS road motion:', 'PASS physical beams',
        'PASS bypass missing side edge:', 'PASS bypass side trend:', '45mm floor',
        'PASS nose navigation:', 'PASS vision motion:'], vehicle+navigation,
        [ROOT/f'sim/{n}_tb.sv' for n in ('road_planner', 'road_ui', 'road_motion', 'nav_barrier_ui',
         'vehicle_close_bypass', 'bypass_edge_trend', 'vehicle_nose', 'vision_fusion', 'vision_motion')],
        'sim/run_centerline_tests.py'),
    'vision_netlist': ('vision_netlist_results.txt', 1, ['PASS synthesized vision:'], vision,
        [ROOT/'sim/vision_netlist_tb.sv'], 'sim/run_vision_netlist_tests.py'),
}
regressions = {}
for name, (filename, count, markers, sources, benches, runner) in specs.items():
    path = ROOT/'sim'/filename
    content = read(path)
    assert not re.search(r'^(?:FATAL|FAIL|ERROR|Traceback)\b', content, re.M), name
    assert all(m in content for m in markers), (name, [m for m in markers if m not in content])
    assert content.count('$finish called') == count, (name, content.count('$finish called'), count)
    dependencies = sources+benches+[ROOT/runner]
    assert path.stat().st_mtime >= max(p.stat().st_mtime for p in dependencies), ('Stale regression', name)
    regressions[name] = dict(passed=True, completed_runs=count, log=path.relative_to(ROOT).as_posix(),
                             sha256=sha(path), dependency_sha256={p.relative_to(ROOT).as_posix(): sha(p) for p in dependencies})

netlist = read(ROOT/'impl/gwsynthesis/ov5640.vg')
for module in ('vision_video', 'vehicle_vision_fusion', 'vehicle_route_profile', 'vehicle_bypass',
               'lsm6dsv16x_spi', 'imu_attitude', 'vl53l1x_array_controller'):
    assert re.search(r'^module '+module+r'\b', netlist, re.M), ('Missing hardware', module)
roms = dict(map=audit_map(), font=audit_font(), road=audit_road())

baseline = json.loads(read(ROOT/'impl/baseline_turn_edge_goal_20261024/manifest.json'))
changed = [name for name, digest in baseline.items() if name.startswith('src/') and sha(ROOT/name) != digest]
baseline_root = ROOT/'impl/baseline_turn_edge_goal_20261024'
backup = baseline_root/'ov5640_before_turn_edge_goal.fs'
backup_metadata = json.loads(read(baseline_root/'ov5640_download.json'))
assert sha(backup) == backup_metadata['sha256'] and backup.stat().st_size == backup_metadata['bytes']
before_bypass = read(baseline_root/'src/vehicle/vehicle_bypass.v')
current_bypass = read(ROOT/'src/vehicle/vehicle_bypass.v')
merge_begin = '    wire merge_done;wire signed [15:0] merge_steer;'
merge_end = '    // A far trigger'
assert (before_bypass[before_bypass.index(merge_begin):before_bypass.index(merge_end)] ==
        current_bypass[current_bypass.index(merge_begin):current_bypass.index(merge_end)])
pass_begin = '                end else if(phase==2)begin'
assert before_bypass[before_bypass.index(pass_begin):] == current_bypass[current_bypass.index(pass_begin):]
references = []
for article in ('132132607', '132647178'):
    files = [ROOT/f'docs/references/{article}.{extension}' for extension in ('html', 'txt')]
    assert all(path.stat().st_size > 1000 for path in files)
    references.append(dict(url=f'https://blog.csdn.net/qq_58114029/article/details/{article}',
                           local_sha256={p.relative_to(ROOT).as_posix(): sha(p) for p in files}))
result = dict(status='digital_verification_passed', recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    bitstream='impl/pnr/ov5640.fs', sha256=sha(fs), bytes=fs.stat().st_size,
    generated_utc=datetime.datetime.fromtimestamp(fs.stat().st_mtime, datetime.timezone.utc).isoformat(),
    tool='Gowin V1.9.12.03', device='GW5AT-LV60PG484AC1/I0 B', hardware_tested=False, flashed=False,
    synthesis_input_manifest=snapshot_path.relative_to(ROOT).as_posix(), input_count=len(actual),
    source_inputs_unchanged=True, input_sha256=actual, changed_baseline_rtl=changed,
    resources=resources, violated_endpoints=violations, min_slack_ns=slack, timing_passed=True,
    pins_verified=pins, roms=roms, regressions=regressions,
    rollback=dict(bitstream=backup.relative_to(ROOT).as_posix(), sha256=sha(backup), verified=True),
    references=references,
    build_log=dict(file=build_log.relative_to(ROOT).as_posix(), sha256=sha(build_log)),
    bypass=dict(minimum_lateral_offset_mm=45, maximum_parameter_offset_mm=50,
                minimum_post_minimum_travel_mm=2, rise_threshold_mm=8, rising_observations=2,
                added_encoder_run_mm=0, accepted_merge_controller_preserved=True, decline_during_settled_diagonal=True, requires_both_front_beams_clear=True, cumulative_small_declines=True),
    vision=dict(method='27 connected scanlines; width-normalized image control', pitch_deg=30,
                control_hz=100, servo_hz=50, visual_authority=True, requires_valid_lane_before_start=True,
                tested_start_yaw_deg=[-8, 8], blind_turn_max_wheel_deg=28, blind_turn_speed_mm_s=100,
                blind_turn_bounds=dict(yaw_deg=100, travel_mm=800, time_ms=8000), photo_calibrated=False, separate_boundary_visibility=True, single_edge_wheel_limit_deg=8, single_edge_frames_before_arming=2, complete_empty_frames_before_commit=2, terminal_goal_plane=True),
    imu=dict(local_retries_before_full_init=2, transaction_watchdog=True,
             full_init_preserves_calibrated_pose=True, zero_triggers=['power-on', 'reset', 'S0'],
             sustained_pose_loss_stop_ms=10000),
    limitations=['No flashing or real-car test performed.',
                 'Image centre references and lens intrinsics remain provisional; referenced photos are unavailable in the workspace.',
                 'Limited-FOV turns use inertial bridging while no road is visible, not visual measurement of an unseen centreline.',
                 'Existing map steering feasibility and full-chassis road-boundary issues remain unresolved.'])
dump(ROOT/'docs/turn_edge_goal_20261024_validation.json', result)
dump(ROOT/'impl/pnr/ov5640_turn_edge_goal_audit.json', result)
dump(ROOT/'impl/pnr/ov5640_current_audit.json', result)
dump(ROOT/'impl/pnr/ov5640_download.json', {k: result[k] for k in (
    'bitstream', 'sha256', 'bytes', 'generated_utc', 'tool', 'device', 'timing_passed', 'hardware_tested', 'flashed')})
resource_doc = ('# 当前构建资源与验证（2026-10-24）\n\n'
    + '位流：`impl/pnr/ov5640.fs`；对应 [本轮说明](turn_edge_goal_20261024.md) 和 '
      '[完整验证记录](turn_edge_goal_20261024_validation.json)。尚未烧录或实车验证。\n\n'
    + '| 资源 | 使用 / 总量 | 使用率 |\n| --- | ---: | ---: |\n'
    + ''.join(f'| {name} | {v["used"]:g} / {v["total"]:g} | {v["percent"]:.2f}% |\n'
              for name, v in resources.items())
    + f'\nSetup/Hold 违例端点：{violations}；最小裕量（ns）：{slack}。\n\n'
    + f'{len(actual)} 个构建输入哈希一致，{len(pins)} 项引脚约束与实际布线一致；道路、地图、字体 ROM 已核对到综合网表。'
    + f'共 {sum(v["completed_runs"] for v in regressions.values())} 个 RTL/集成/综合网表运行通过。\n\n'
    + '现有地图局部转角需求和完整车身边界问题仍未解决；数字验证通过不代表实车全程居中或不压线。\n\n'
    + f'生成时间（UTC）：{result["generated_utc"]}；SHA-256：`{result["sha256"]}`。\n')
(ROOT/'docs/resource_usage.md').write_text(resource_doc, encoding='utf-8')
print(json.dumps({k: result[k] for k in ('status', 'sha256', 'generated_utc', 'input_count',
      'resources', 'violated_endpoints', 'min_slack_ns')}, indent=2))
