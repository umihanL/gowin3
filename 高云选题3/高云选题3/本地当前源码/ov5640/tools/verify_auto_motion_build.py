"""Verify this change against its own backup and audit the routed FPGA build."""
from pathlib import Path
import datetime
import hashlib
import json
import re

from verify_map_netlist import audit as audit_map_rom


ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    data = path.read_bytes()
    return data.decode('utf-16' if data.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig')


baseline_path = ROOT / 'impl/auto_motion_baseline.json'
baseline = json.loads(read(baseline_path))
previous_sources = {name: value for name, value in baseline['sha256'].items()
                    if name.startswith('src/')}
current_sources = {p.relative_to(ROOT).as_posix() for p in (ROOT / 'src').rglob('*')
                   if p.is_file()}
assert set(previous_sources) == current_sources, 'Unexpected added/deleted source'
changed = sorted(name for name, value in previous_sources.items()
                 if digest(ROOT / name) != value)
allowed = {
    'src/ov5640_ddr3_tft.v',
    'src/navigation/navigation_ui.v',
    'src/navigation/nav_control.v',
    'src/navigation/nav_font.vh',
    'src/navigation/nav_renderer.v',
    'src/navigation/vehicle_settings.v',
    'src/vehicle/vehicle_motion.v',
    'src/vehicle/vehicle_subsystem.v',
}
assert set(changed) <= allowed, 'Change outside authorized scope: ' + repr(changed)
for rel in ('ov5640.gprj', 'build.tcl'):
    assert digest(ROOT / rel) == baseline['sha256'][rel], rel

# The seven UART channels and unrelated top-level logic are preserved.
old_top = read(Path(baseline['backup']) / 'src/ov5640_ddr3_tft.v')
new_top = read(ROOT / 'src/ov5640_ddr3_tft.v')
uart_pattern = r'    imu_firewater_tx #\(.*?\) u_imu_uart\(.*?\);'
assert re.search(uart_pattern, old_top, re.S)[0] == re.search(uart_pattern, new_top, re.S)[0]

resources = {}
report_path = ROOT / 'impl/pnr/ov5640.rpt.txt'
report = read(report_path)
assert str(ROOT).replace('\\', '/') in report.replace('\\', '/')
for name in ('Logic', 'Register', 'CLS', 'BSRAM', 'DSP', 'I/O Port'):
    match = re.search(r'^\s*' + re.escape(name) +
                      r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|\s*(\d+)%', report, re.M)
    assert match, 'Missing resource: ' + name
    used, total = map(float, match.group(1, 2))
    resources[name] = dict(used=used, available=total, remaining=total-used,
                           actual_percent=round(100*used/total, 2),
                           report_percent=int(match[3]))

timing_path = ROOT / 'impl/pnr/ov5640.tr'
timing = read(timing_path)
timing_counts = {}
for kind in ('Setup', 'Hold'):
    match = re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)
    assert match and int(match[1]) == 0, kind + ' timing failed'
    timing_counts[kind.lower() + '_violated_endpoints'] = int(match[1])

# Check every explicit pin against the final P&R report, not just the CST text.
cst = read(ROOT / 'src/ov5640_ddr3_tft.cst')
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', cst, re.M))
actual_pins = dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',
                              report, re.M))
assert len(pins) == len(set(pins.values())), 'Duplicate pin'
assert all(actual_pins.get(name) == loc for name, loc in pins.items()), 'Routed pin mismatch'

bitstream = ROOT / 'impl/pnr/ov5640.fs'
inputs = [p for p in (ROOT / 'src').rglob('*') if p.is_file() and p.suffix in
          ('.v', '.vo', '.vh', '.mem', '.cst', '.sdc', '.ipc', '.mod')]
inputs += [ROOT / 'ov5640.gprj', ROOT / 'build.tcl',
           ROOT / 'impl/ov5640_process_config.json', ROOT / 'impl/project_process_config.json']
assert bitstream.exists() and bitstream.stat().st_mtime >= max(p.stat().st_mtime for p in inputs), 'Stale bitstream'
assert report_path.stat().st_mtime >= bitstream.stat().st_mtime
assert timing_path.stat().st_mtime >= bitstream.stat().st_mtime

checks = {
    'sim/tof_diagnostic_results.txt': ('PASS ToF diagnostics:', 'isolation and recovery'),
    'sim/range_limit_results.txt': ('PASS distance limiter:', '110352'),
    'sim/vehicle_test_results.txt': ('PASS gradual stop:', 'PASS route execution:',
        'PASS Ackermann curve tracking:', 'PASS obstacle bypass:', 'PASS right bypass,',
        'PASS configurable braking:', 'reverse-feedback stop', 'swept body collision check',
        'PASS route profile:', 'PASS integrated I/O:', 'manual/remote hardware arbitration'),
    'sim/vehicle_overlay_results.txt': ('PASS telemetry decimal display', 'PASS vehicle overlay raster'),
    'sim/navigation_test_results.txt': ('PASS A*:', 'PASS Ackermann:', 'PASS map ROM:', 'PASS UI:',
                                      'PASS delayed TFT DE/HS/VS and RGB:'),
    'sim/settings_test_results.txt': ('PASS settings:', 'PASS motor response:'),
}
logs = {}
for rel, required in checks.items():
    path = ROOT / rel
    content = read(path)
    assert all(token in content for token in required), 'Missing tests: ' + rel
    assert not re.search(r'^(?:FAIL|FATAL|ERROR)\b', content, re.I | re.M), 'Failed test: ' + rel
    logs[rel] = dict(sha256=digest(path), output=content)

result = dict(
    project='ov5640.gprj', device='GW5AT-LV60PG484AC1/I0 B', tool='Gowin V1.9.12.03',
    bitstream='impl/pnr/ov5640.fs', sha256=digest(bitstream), bitstream_bytes=bitstream.stat().st_size,
    generated_time_utc8=datetime.datetime.fromtimestamp(bitstream.stat().st_mtime,
        datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
    resources=resources, **timing_counts,
    baseline='impl/auto_motion_baseline.json', backup=baseline['backup'],
    changed_existing_sources=changed, added_sources=[],
    unchanged_existing_source_files=len(previous_sources)-len(changed),
    pin_constraints_unchanged=True, timing_constraints_unchanged=True,
    pins_verified_from_pnr_report=pins, seven_channel_firewater_unchanged=True,
    tof_protocol_driver_changed=False, tof_diagnostics_added=True,
    tof_physical_failure_root_cause_confirmed=False,
    motor_closed_loop_and_pi_unchanged=True,
    defaults=dict(slowdown_cm=65, speed_mm_s=200, control='manual', running=False),
    slowdown_setting_cm=dict(minimum=25, maximum=200, step=1),
    automatic_speed_ceiling_mm_s=200, detour_speed_ceiling_mm_s=120,
    map_rom_init_audit=audit_map_rom(), hardware_tested=False,
    warnings=['PR1014: existing camera_pclk_d constraint uses generic clock routing'],
    input_sha256={p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(inputs)},
    regression_logs=logs,
    verification_sha256={p.relative_to(ROOT).as_posix(): digest(p)
                         for p in sorted((ROOT / 'sim').glob('*'))
                         if p.is_file() and p.suffix in ('.v', '.sv', '.py')},
)
output = ROOT / 'impl/pnr/ov5640_auto_motion_audit.json'
output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

resource_doc = (
    '# 最终布局布线资源\n\n'
    f'工具：{result["tool"]}；器件：{result["device"]}。\n\n'
    f'位流生成时间（UTC+8）：{result["generated_time_utc8"]}。\n\n'
    '下载文件：`impl/pnr/ov5640.fs`。\n\n'
    '| 资源 | 使用 / 总量 | 实际占比 | 剩余 |\n| --- | --- | --- | --- |\n'
    + ''.join(f'| {name} | {v["used"]:g} / {v["available"]:g} | '
              f'{v["actual_percent"]:.2f}% | {v["remaining"]:g} |\n'
              for name, v in resources.items())
    + '\nSetup / Hold 违例均为 0。DSP 使用 Gowin 报告的等效块数。'
      'Gowin 报告中的整数占比可能向上取整，上表按使用量/总量计算。\n\n'
      '资源包含摄像头、DDR、IMU、显示、地图 ROM、A*、PS2、四路 ToF、'
      '电机闭环、舵机、里程计、绕障、导航和七通道 FireWater 串口。\n\n'
      '本轮未改动 CST、SDC、摄像头/DDR、IMU、串口以及电机闭环核心。'
      '保留已有 PR1014 摄像头像素时钟普通布线提示；工具验证不等于实车行驶标定。\n\n'
      '使用方法及 ToF 错误码见 [自动运动说明](auto_motion.md)。'
      '源文件范围、实际引脚、回归日志和输入哈希见 '
      '`impl/pnr/ov5640_auto_motion_audit.json`。\n\n'
    f'位流 SHA256：`{result["sha256"]}`。\n'
)
(ROOT / 'docs/resource_usage.md').write_text(resource_doc, encoding='utf-8')
print(json.dumps(dict(resources=resources, **timing_counts,
    changed_sources=changed, constrained_pins_checked=len(pins), sha256=result['sha256']), indent=2))
