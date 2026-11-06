"""Audit the range-dropout change, its regression, and the actual routed image."""
from pathlib import Path
import datetime
import hashlib
import json
import re

from verify_map_netlist import audit as audit_map_rom

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    data = path.read_bytes()
    return data.decode('utf-16' if data.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig').replace('\r\n', '\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


baseline = json.loads(read(ROOT / 'impl/range_memory_baseline.json'))
old = {n: h for n, h in baseline['sha256'].items() if n.startswith('src/')}
current = {p.relative_to(ROOT).as_posix() for p in (ROOT / 'src').rglob('*') if p.is_file()}
assert current == set(old), 'Unexpected added/deleted source file'
changed = sorted(n for n, h in old.items() if sha(ROOT / n) != h)
assert set(changed) == {'src/vehicle/vehicle_motion.v', 'src/vehicle/vehicle_subsystem.v',
                        'src/vehicle/motor/vl53l0x_array_controller.v'}, changed
assert sha(ROOT / 'ov5640.gprj') == baseline['sha256']['ov5640.gprj']
build = read(ROOT / 'build.tcl')
placement_algorithm = int(re.search(r'set_option -place_option (\d+)', build)[1])
routing_algorithm = int(re.search(r'set_option -route_option (\d+)', build)[1])
original_build = (ROOT / 'build.tcl').read_bytes().replace(
    f'set_option -route_option {routing_algorithm}'.encode(), b'set_option -route_option 0').replace(
    f'set_option -place_option {placement_algorithm}'.encode(), b'set_option -place_option 1').replace(
    b'Try placement algorithm 2', b'Use placement algorithm 1')
assert hashlib.sha256(original_build).hexdigest() == baseline['sha256']['build.tcl'], \
    'Unexpected build script change'

log_path = ROOT / 'sim/vehicle_test_results.txt'
log = read(log_path)
required = ('PASS vehicle math:', 'PASS gradual stop:', 'PASS route execution:',
            'PASS Ackermann curve tracking:', 'PASS obstacle bypass:',
            'PASS configurable braking:', 'PASS route profile:', 'PASS integrated I/O:',
            'only one front sensor required', 'all echoes lost before braking',
            'communication loss during braking', 'frozen scan is spent',
            'healthy side scans cannot refresh not-ready front',
            'initial no-echo permits bounded', 'startup bus failure',
            'cached right clearance', 'asynchronous scan pulses confirm tail',
            'missing unobserved obstacle tail', 'exit/re-entry clears prior-run',
            'a real front echo replaces bounded search',
            'newer near reading revokes remembered open right')
assert all(t in log for t in required), 'Missing regression result'
assert not re.search(r'^(?:FAIL|FATAL|ERROR)\b', log, re.M | re.I), 'Regression failed'
tof_log = read(ROOT / 'sim/tof_diagnostic_results.txt')
assert 'PASS ToF sample pulses:' in tof_log and 'PASS ToF diagnostics:' in tof_log
assert not re.search(r'^(?:FAIL|FATAL|ERROR)\b', tof_log, re.M | re.I)
# Removing the newly exported success strobe leaves the protocol RTL identical.
rel = 'src/vehicle/motor/vl53l0x_array_controller.v'
driver = read(ROOT / rel)
driver = re.sub(r'    // One-cycle per-device success pulses;.*?output reg \[3:0\] sample_valid,\n', '', driver, flags=re.S)
driver = driver.replace('; sample_valid<=0;', ';').replace(' sample_valid<=0;', '')
driver = driver.replace('                                    sample_valid[channel]<=1;\n', '')
assert driver == read(Path(baseline['backup']) / rel), 'Unexpected ToF protocol change'

fs = ROOT / 'impl/pnr/ov5640.fs'
inputs = [p for p in (ROOT / 'src').rglob('*') if p.is_file() and p.suffix in
          ('.v', '.vo', '.vh', '.mem', '.cst', '.sdc', '.ipc', '.mod')]
inputs += [ROOT / 'ov5640.gprj', ROOT / 'build.tcl',
           ROOT / 'impl/ov5640_process_config.json']
assert fs.stat().st_mtime >= max(p.stat().st_mtime for p in inputs), 'Old download file'
# This IDE cache is synchronized to the routed options after the command-line
# build. Its values are checked against cmd.do below, not used to date RTL.
inputs += [ROOT / 'impl/project_process_config.json']
assert fs.stat().st_mtime > datetime.datetime.fromisoformat(baseline['created']).timestamp()
report_path = ROOT / 'impl/pnr/ov5640.rpt.txt'
timing_path = ROOT / 'impl/pnr/ov5640.tr'
assert min(report_path.stat().st_mtime, timing_path.stat().st_mtime) >= fs.stat().st_mtime
report, timing = read(report_path), read(timing_path)
pnr_options = read(ROOT / 'impl/pnr/cmd.do')
assert re.search(fr'(?m)^-place_option {placement_algorithm}\s*$', pnr_options)
assert re.search(fr'(?m)^-route_option {routing_algorithm}\s*$', pnr_options)
for config in ('impl/ov5640_process_config.json', 'impl/project_process_config.json'):
    options = json.loads(read(ROOT / config))
    assert options['Route_Option'] == str(routing_algorithm), config
    assert options['Place_Option'] == str(placement_algorithm), config
resources = {}
for name in ('Logic', 'Register', 'CLS', 'BSRAM', 'DSP', 'I/O Port'):
    m = re.search(r'^\s*' + re.escape(name) + r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|', report, re.M)
    assert m, name
    used, total = map(float, m.group(1, 2))
    resources[name] = dict(used=used, total=total, remaining=total-used,
                           percent=round(100*used/total, 2))
timing_counts = {}
for kind in ('Setup', 'Hold'):
    m = re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)
    assert m, 'Missing ' + kind + ' timing summary'
    timing_counts[kind.lower() + '_violated_endpoints'] = int(m[1])
timing_min_slack_ns = {}
for kind in ('Setup', 'Hold', 'Recovery', 'Removal'):
    section = re.search(r'^3\.1\.\d ' + kind + r' Paths Table\n(.*?)(?=\n3\.)', timing, re.M | re.S)
    assert section, 'Missing ' + kind + ' path table'
    slacks = [float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s', section[1], re.M)]
    assert slacks, 'Empty ' + kind + ' path table'
    timing_min_slack_ns[kind.lower()] = min(slacks)
timing_passed = all(n == 0 for n in timing_counts.values()) and all(
    n >= 0 for n in timing_min_slack_ns.values())
cst = read(ROOT / 'src/ov5640_ddr3_tft.cst')
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', cst, re.M))
actual = dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
assert len(pins) == len(set(pins.values()))
assert all(actual.get(n) == loc for n, loc in pins.items()), 'Routed pin mismatch'

generated = datetime.datetime.fromtimestamp(fs.stat().st_mtime,
    datetime.timezone(datetime.timedelta(hours=8))).isoformat()
result = dict(bitstream='impl/pnr/ov5640.fs', sha256=sha(fs), generated_time_utc8=generated,
    tool='Gowin V1.9.12.03', device='GW5AT-LV60PG484AC1/I0 B', resources=resources,
    **timing_counts, timing_passed=timing_passed, changed_sources=changed, added_sources=[],
    timing_min_slack_ns=timing_min_slack_ns,
    build_script_change=f'Placement algorithm 1 -> {placement_algorithm}; routing {routing_algorithm}; clocks and constraints unchanged',
    placement_algorithm=placement_algorithm, routing_algorithm=routing_algorithm,
    unchanged_source_files=len(old)-len(changed), baseline='impl/range_memory_baseline.json',
    backup=baseline['backup'], pins_verified=pins, cst_sdc_unchanged=True,
    tof_protocol_imu_video_motor_pi_uart_unchanged=True,
    limits=dict(initial_no_echo_speed_mm_s=80, initial_search_mm=400,
                side_memory_ms=5000, bypass_mm=3000, post_tail_extra_mm=350),
    hardware_tested=False, regression_log=dict(sha256=sha(log_path), output=log),
    tof_regression_log=tof_log,
    map_rom=audit_map_rom(),
    input_sha256={p.relative_to(ROOT).as_posix(): sha(p) for p in sorted(inputs)},
    warning='Existing PR1014 camera_pclk_d uses generic clock routing')
(ROOT / 'impl/pnr/ov5640_range_memory_audit.json').write_text(
    json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
doc = ('# 当前布局布线资源\n\n'
       f'工具：{result["tool"]}；器件：{result["device"]}。\n\n'
       f'位流生成时间（UTC+8）：{generated}。下载文件：`impl/pnr/ov5640.fs`。\n\n'
       '| 资源 | 使用 / 总量 | 占比 | 剩余 |\n| --- | --- | --- | --- |\n'
       + ''.join(f'| {n} | {v["used"]:g} / {v["total"]:g} | {v["percent"]:.2f}% | {v["remaining"]:g} |\n'
                 for n, v in resources.items())
       + f'\nSetup 违例端点：{timing_counts["setup_violated_endpoints"]}；'
       f'Hold 违例端点：{timing_counts["hold_violated_endpoints"]}。'
       + ('时序检查通过。' if timing_passed else '时序检查未通过，此位流不能作为时序合格版本。')
       + f' Setup/Hold/Recovery/Removal 最小余量（ns）：{timing_min_slack_ns}。'
       + 'DSP 按 Gowin 等效块数统计。\n\n'
       '本轮 RTL 改 `vehicle_motion.v`、`vehicle_subsystem.v` 和 `vl53l0x_array_controller.v`，实现测距丢失后的距离记忆与里程执行；ToF 仅新增逐路有效测量脉冲输出。'
       '摄像头/DDR、IMU、ToF 协议驱动、七通道 FireWater、电机 PI、引脚和时序约束保持不变。'
       f'构建脚本与 Gowin 配置使用布局算法 {placement_algorithm}、布线算法 {routing_algorithm}。'
       '保留已有 PR1014 摄像头像素时钟普通布线提示。\n\n'
       '使用方法及容错边界见 [自动运动说明](auto_motion.md)。仿真和工具验证不代替实车标定。\n\n'
       '构建审计：`impl/pnr/ov5640_range_memory_audit.json`。\n\n'
       f'位流 SHA256：`{sha(fs)}`。\n')
(ROOT / 'docs/resource_usage.md').write_text(doc, encoding='utf-8')
print(json.dumps(dict(resources=resources, **timing_counts,
                     timing_min_slack_ns=timing_min_slack_ns, timing_passed=timing_passed,
                     changed_sources=changed, pins_checked=len(pins), sha256=sha(fs)), indent=2))
if not timing_passed:
    raise SystemExit('Timing failed; resource and audit documents record the actual failed result.')
