"""Audit unchanged legacy RTL and record the final routed build, never an estimate."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import re
import zipfile
from verify_map_netlist import audit

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--backup', default=str(ROOT.parent/'ov5640_before_motor_merge_20261015_230457.zip'))
args = parser.parse_args()
allowed = {
    'src/ov5640_ddr3_tft.v', 'src/portrait/portrait_tiles.v',
    'src/portrait/portrait_ddr_adapter.v', 'src/portrait/portrait_display.v',
    'src/ddr3_ctrl_2port/ddr3_ctrl_2port.v', 'src/vga/disp_driver.v',
    'src/portrait/gaussian_crop.v',
    'src/imu/attitude_overlay.v',
    'src/imu/imu_attitude.v',  # Pitch norm-error correction near +/-90 degrees.
    'src/ov5640_ddr3_tft.cst', 'src/ov5640_ddr3_tft.sdc',
    'src/navigation/nav_control.v', 'src/navigation/nav_path_refiner.v',
    'src/navigation/navigation_ui.v', 'src/navigation/nav_renderer.v',
    'src/navigation/nav_font.vh', 'src/navigation/nav_map_init.vh',
    'src/navigation/nav_metadata.vh', 'src/navigation/map_metadata.mem',
    'src/navigation/map_metadata.json',
}
changed = []
unchanged = 0
with zipfile.ZipFile(args.backup) as archive:
    for item in archive.infolist():
        name = item.filename.replace('\\', '/')
        if not name.startswith('src/') or item.is_dir():
            continue
        local = ROOT/name
        if not local.exists() or local.read_bytes() != archive.read(item):
            changed.append(name)
            assert name in allowed, f'Unexpected legacy source change: {name}'
        else:
            unchanged += 1

timing = (ROOT/'impl/pnr/ov5640.tr').read_text(encoding='utf-8', errors='replace')
resource_text = (ROOT/'impl/pnr/ov5640.rpt.txt').read_text(encoding='utf-8', errors='replace')
counts = {}
for kind in ('Setup', 'Hold'):
    count = int(re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)[1])
    assert count == 0, f'{kind} violations: {count}'
    counts[kind.lower()+'_violated_endpoints'] = count
assert str(ROOT).replace('\\','/') in resource_text.replace('\\','/'), 'Report belongs to another checkout'
resources = {}
for name in ('Logic', 'Register', 'BSRAM', 'DSP', 'I/O Port'):
    match = re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|\s*(\d+)%',
                      resource_text, re.M)
    assert match, f'Missing resource {name}'
    resources[name] = dict(used=float(match[1]), available=float(match[2]), report_percent=int(match[3]))
bitstream = ROOT/'impl/pnr/ov5640.fs'
assert bitstream.exists()
inputs = [p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in
          ('.v', '.vh', '.vo', '.sdc', '.cst', '.mem')]
inputs += [ROOT/'ov5640.gprj', ROOT/'build.tcl', ROOT/'impl/ov5640_process_config.json',
           ROOT/'impl/project_process_config.json']
assert bitstream.stat().st_mtime >= max(p.stat().st_mtime for p in inputs), 'Bitstream is stale'
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
old = ROOT/'impl/verified_build.json'
history = ROOT/'impl/verified_build_before_motor.json'
if old.exists() and not history.exists():
    history.write_bytes(old.read_bytes())
result = dict(project='ov5640.gprj', feature='Landscape RGB565 camera/DDR, map ROM/A*, integrated PS2/ToF/motor/servo, encoder-IMU odometry, closed-loop navigation, obstacle bypass and three-axis IMU FireWater UART',
              tool='Gowin V1.9.12.03', device='GW5AT-LV60PG484AC1/I0 B',
              bitstream='impl/pnr/ov5640.fs', sha256=digest(bitstream),
              generated_time_utc8=datetime.datetime.fromtimestamp(bitstream.stat().st_mtime,
                  datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
              bitstream_bytes=bitstream.stat().st_size, resources=resources, **counts,
              hardware_tested=False, backup=str(args.backup), map_rom_init_audit=audit(),
              changed_legacy_sources=sorted(changed), unchanged_legacy_source_files=unchanged,
              input_sha256={p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(inputs)},
              regression_logs={})
for name in ('navigation', 'vehicle', 'portrait', 'fix', 'imu'):
    log = ROOT/f'sim/{name}_test_results.txt'
    content = log.read_text(encoding='utf-8')
    assert 'PASS' in content and not re.search(r'^(?:FATAL|ERROR|FAIL)\b',content,re.I|re.M), f'Failed or incomplete {name} log'
    if name == 'portrait':
        assert 'full-screen RGB DDR capture/scanout' in content, 'DDR regression not recorded'
        assert content.count('PASS video pipeline:') >= 2, 'Full-size DVP/DDR scanout not recorded'
        assert 'INFO video 640x480:' in content and content.count('matched 307200 pixels') >= 3, 'Three complete VGA scanouts not recorded'
    if name == 'navigation':
        assert 'safe-cell snapping' in content and 'eight-point limit' in content, 'UI regression predates final source'
        assert 'PASS Ackermann' in content and 'PASS map ROM' in content, 'Missing map or vehicle regression'
    if name == 'vehicle':
        for check in ('PASS gradual stop', 'PASS route execution', 'PASS Ackermann curve tracking',
                      'PASS obstacle bypass', 'PASS route profile', 'PASS integrated I/O'):
            assert check in content, f'Missing vehicle regression: {check}'
    if name == 'imu':
        assert 'PASS imu_angle_limits:' in content, 'Missing pitch +/-90 boundary regression'
        assert content.count('PASS imu_firewater:') == 2, 'Missing scaled and production-clock UART wire regressions'
        assert content.count('PASS imu_pose_firewater:') == 2, 'Missing extended pose UART wire regressions'
        assert content.count('PASS imu_motor_firewater:') == 2, 'Missing periodic pose/motor UART wire regressions'
        assert 'PASS imu_rotation: 3264' in content, 'Missing quaternion-vector round-trip regression'
        assert content.count('PASS imu_ui:') == 2, 'Missing Euler/vector display mode regression'
    result['regression_logs'][log.relative_to(ROOT).as_posix()] = dict(sha256=digest(log), output=content)
overlay_log = ROOT/'sim/vehicle_overlay_results.txt'
overlay_content = overlay_log.read_text(encoding='utf-8')
assert 'PASS telemetry decimal display' in overlay_content and 'PASS vehicle overlay raster' in overlay_content
assert not re.search(r'^(?:FATAL|ERROR|FAIL)\b', overlay_content, re.I|re.M)
result['regression_logs'][overlay_log.relative_to(ROOT).as_posix()] = dict(
    sha256=digest(overlay_log), output=overlay_content)
old.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
(ROOT/'docs/resource_usage.md').write_text(
    '# 最终布局布线资源\n\n'
    f"工具：{result['tool']}；器件：{result['device']}。\n\n"
    f"位流生成时间（UTC+8）：{result['generated_time_utc8']}。\n\n"
    '| 资源 | 使用 / 总量 | 报告占比 |\n| --- | --- | --- |\n'+
    ''.join(f'| {name} | {v["used"]:g} / {v["available"]:g} | {v["report_percent"]}% |\n'
            for name,v in resources.items())+
    '\nSetup / Hold 违例均为 0。DSP 数量按 Gowin 报告的等效块计数。\n\n'
    '表中百分比直接采用 Gowin 报告，可能向上取整；117/118 块 BSRAM 实际约 99.2%，仍有 1 块余量。\n\n'
    '资源为整个工程合计，包含摄像头、DDR、IMU、显示、地图/A*、PS2、四路测距、电机闭环、舵机、航位推算、绕障、导航和三轴姿态 FireWater 串口。\n\n'
    '操作、引脚和待标定参数见 [整车整合说明](vehicle_integration.md)。\n\n'
    '原 PR1014 摄像头像素时钟普通布线提示保留。尚未烧录验证实物。\n\n'
    f'位流 SHA256：`{result["sha256"]}`。\n', encoding='utf-8')
print(json.dumps(dict(resources=resources, **counts, changed_legacy_sources=changed,
                     unchanged_legacy_source_files=unchanged), indent=2))
