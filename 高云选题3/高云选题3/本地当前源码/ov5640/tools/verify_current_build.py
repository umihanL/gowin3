"""Audit the current routed design, preserved modules, pins, ROM INIT and tests."""
from pathlib import Path
import datetime
import hashlib
import json
import re
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road

ROOT=Path(__file__).resolve().parents[1]
def read(p):
    b=p.read_bytes()
    return b.decode('utf-16' if b.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig').replace('\r\n','\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

# Compare against the immediate pre-request snapshot, including constraints.
baseline_dir=ROOT/'impl/upgrade_20261022_baseline'
baseline=json.loads(read(baseline_dir/'source_hashes.json'))
allowed={
    'ov5640.gprj','src/ov5640_ddr3_tft.v',
    'src/imu/imu_attitude.v','src/imu/lsm6dsv16x_spi.v',
    'src/portrait/vision_video.v','src/vehicle/vehicle_motion.v',
    'src/vehicle/vehicle_subsystem.v','src/vehicle/vehicle_vision_fusion.v','src/vehicle/vehicle_bypass.v',
    'src/vehicle/import_manifest.json',
}
added={p.relative_to(ROOT).as_posix() for p in (ROOT/'src').rglob('*')
       if p.is_file() and p.relative_to(ROOT).as_posix() not in baseline}
assert added=={'src/portrait/vision_zebra.v','src/vehicle/VL53L1X_LICENSE.txt',
    'src/vehicle/motor/vl53l1x_array_controller.v','src/vehicle/motor/vl53l1x_i2c_master.v'},added
changed=[n for n,h in baseline.items() if not (ROOT/n).exists() or sha(ROOT/n)!=h]
assert set(changed)<=allowed,('Unexpected October 22 request change',changed)
request_baseline=baseline
request_allowed=allowed
request_changed=[n for n,h in request_baseline.items() if not (ROOT/n).exists() or sha(ROOT/n)!=h]
assert set(request_changed)<=request_allowed,('Change outside current request',request_changed)
for n in ('src/ov5640_ddr3_tft.cst','src/ov5640_ddr3_tft.sdc','build.tcl',
          'src/portrait/portrait_display.v',
          'src/vehicle/motor/motor_pi.v','src/vehicle/motor/motor_encoder.v',
          'src/vehicle/motor/ps2_test_control.v','src/imu/imu_firewater_tx.v',
          'src/vehicle/motor/vl53l0x_array_controller.v'):
    assert sha(ROOT/n)==baseline[n],n

fs=ROOT/'impl/pnr/ov5640.fs'
inputs=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in
    ('.v','.vo','.vh','.mem','.cst','.sdc','.ipc','.mod')]+[ROOT/'ov5640.gprj',ROOT/'build.tcl']
build_inputs=json.loads(read(ROOT/'impl/upgrade_synthesis_inputs.json'))
assert {p.relative_to(ROOT).as_posix():sha(p) for p in inputs}==build_inputs,'RTL changed since synthesis began'
assert fs.stat().st_mtime>=(ROOT/'impl/upgrade_synthesis_inputs.json').stat().st_mtime,'Previous build bitstream, current run not complete'
assert fs.stat().st_mtime>=max(p.stat().st_mtime for p in inputs),'Download file older than RTL'
report=read(ROOT/'impl/pnr/ov5640.rpt.txt');timing=read(ROOT/'impl/pnr/ov5640.tr')
assert min((ROOT/'impl/pnr/ov5640.rpt.txt').stat().st_mtime,(ROOT/'impl/pnr/ov5640.tr').stat().st_mtime)>=fs.stat().st_mtime
resources={}
for name in ('Logic','Register','CLS','BSRAM','DSP','I/O Port'):
    m=re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|',report,re.M)
    assert m,name
    used,total=map(float,m.groups());resources[name]=dict(used=used,total=total,remaining=total-used,percent=round(used*100/total,2))
violations={}
for kind in ('Setup','Hold'):
    m=re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)',timing);assert m,kind
    violations[kind]=int(m[1])
slack={}
for kind in ('Setup','Hold','Recovery','Removal'):
    m=re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)',timing,re.M|re.S);assert m,kind
    numbers=[float(x) for x in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s',m[1],re.M)]
    assert numbers,kind;slack[kind]=min(numbers)
assert not any(violations.values()) and all(v>=0 for v in slack.values()),('Timing failed',violations,slack)
cst=read(ROOT/'src/ov5640_ddr3_tft.cst')
pins=dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',cst,re.M))
actual=dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',report,re.M))
assert len(pins)==len(set(pins.values())) and all(actual.get(n)==p for n,p in pins.items()),'Routed pin mismatch'
# Current affected regressions only; earlier route/photo runs remain in the
# pre-request audit and are not presented as newly executed tests.
vehicle_sources=list((ROOT/'src/vehicle').rglob('*.v'))
imu_sources=list((ROOT/'src/imu').glob('*.v'))
vision_sources=list((ROOT/'src/portrait').glob('vision_*.v'))
suites={
    'imu':('imu_test_results.txt',['PASS imu_attitude:','PASS IMU burst recovery:',
        'PASS IMU local recovery:','PASS imu_spi bank fault 5:','PASS imu_motor_firewater:',
        'PASS imu_rotation:'],imu_sources),
    'vision':('vision_test_results.txt',['PASS font rows:','PASS vision display timing:',
        'PASS vision video:','PASS vision zebra:','PASS vision fusion:','PASS zebra speed:',
        'PASS vision motion:','PASS vision UI:','PASS motor brake plant:'],vision_sources+vehicle_sources),
    'vehicle':('vehicle_test_results.txt',['PASS vehicle math:','PASS route profile:',
        'PASS integrated I/O:','PASS L1X motion range:','PASS side clearance:','PASS bypass policy:',
        'PASS navigation dynamic obstacle:'],vehicle_sources),
    'nose':('nose_test_results.txt',['PASS nose fast brake:','PASS nose navigation:',
        'PASS nose IMU watchdog:','PASS motor modulated brake:'],vehicle_sources),
    'oct20':('oct20_test_results.txt',['PASS forbidden barriers:','PASS adaptive brake envelope:',
        'PASS physical beams','PASS navigation with physical beams:','PASS bypass missing side edge:',
        'PASS VL53L1X array:','PASS VL53L1X recovery:'],vehicle_sources),
    'vision_netlist':('vision_netlist_results.txt',['PASS synthesized vision:'],vision_sources),
}
logs={}
for suite,(name,markers,dependencies) in suites.items():
    p=ROOT/'sim'/name;s=read(p)
    assert all(m in s for m in markers) and not re.search(r'^(?:FATAL|FAIL|ERROR)\b',s,re.M),name
    dependencies=dependencies+[ROOT/'sim'/f'run_{suite}_tests.py']
    if suite=='vehicle':dependencies+=list((ROOT/'sim').glob('vehicle_*_tb.sv'))
    if suite=='oct20':dependencies+=[ROOT/'sim/vl53l1x_array_tb.sv',ROOT/'sim/vl53l1x_sensor_model.v']
    assert p.stat().st_mtime>=max(d.stat().st_mtime for d in dependencies),('Stale regression',name)
    logs[name]=dict(sha256=sha(p),output=s)

gprj=read(ROOT/'ov5640.gprj')
for n in re.findall(r'<File path="([^"]+)"',gprj):assert (ROOT/n).exists(),n
netlist=read(ROOT/'impl/gwsynthesis/ov5640.vg')
for name in ('vision_video','vision_zebra','vehicle_vision_fusion','vl53l1x_array_controller'):
    assert re.search(r'^module '+name+r'\b',netlist,re.M),('Module removed by synthesis',name)
assert not re.search(r'^module vl53l0x_array_controller\b',netlist,re.M)
project_sources=re.findall(r'<File path="([^"]+)"',gprj)
assert 'src/vehicle/motor/vl53l1x_array_controller.v' in project_sources
assert 'src/vehicle/motor/vl53l1x_i2c_master.v' in project_sources
assert 'src/vehicle/motor/vl53l0x_array_controller.v' not in project_sources
assert '.recalibrate(imu_rezero)' in read(ROOT/'src/ov5640_ddr3_tft.v')
assert len(re.findall(r'^\s*SDPB bank[01]_bank[01]_\w+\s*\(',netlist,re.M))==6,'Thumbnail BSRAM inference changed'
suffix_ram_count=len(re.findall(r'^\s*SDPB suffix_cost_suffix_cost_\w+\s*\(',netlist,re.M))
assert suffix_ram_count==16,('Waypoint suffix cache did not retain its 16384x16 BSRAM',suffix_ram_count)
generated=datetime.datetime.fromtimestamp(fs.stat().st_mtime,datetime.timezone(datetime.timedelta(hours=8))).isoformat()
result=dict(bitstream='impl/pnr/ov5640.fs',sha256=sha(fs),generated_time_utc8=generated,
    tool='Gowin V1.9.12.03',device='GW5AT-LV60PG484AC1/I0 B',resources=resources,
    violated_endpoints=violations,min_slack_ns=slack,timing_passed=True,hardware_tested=False,
    changed_inputs=changed,current_request_changed=request_changed,
    added_sources=sorted(added),baseline='impl/upgrade_20261022_baseline/source_hashes.json',
    preserved_modules='CST/SDC, camera capture/DDR/FIFO, motor PI/polarity/encoder, PS2 and seven-channel UART match the immediate pre-request snapshot.',
    preview=dict(width=104,height=78,bits_per_pixel=6,buffers=2,full_resolution_upper_restored=False,
        reason='Insufficient spare BSRAM for a full-resolution rotated frame buffer.'),
    imu=dict(zero_triggers=['power-on','reset','S0'],loss_timeout_ms=10000,
        short_gap_yaw='encoder odometry',zero_reference_retained_on_transport_loss=True),
    zebra=dict(speed_cap_mm_s=100,clear_frames=6,camera_to_tail_mm=280),
    pins_verified=pins,map_rom=audit_map(),font_rom=audit_font(),road_rom=audit_road(),
    waypoint_cache=dict(words=16384,bits_per_word=16,bsram=suffix_ram_count),regressions=logs,
    camera=json.loads(read(ROOT/'docs/camera_parameters.json')),
    vehicle=json.loads(read(ROOT/'docs/vehicle_parameters.json')),
    input_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in sorted(inputs)})
(ROOT/'impl/pnr/ov5640_current_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(ROOT/'impl/pnr/ov5640_download.json').write_text(json.dumps({k:result[k] for k in
    ('bitstream','sha256','generated_time_utc8','tool','device','timing_passed','hardware_tested')},indent=2)+'\n',encoding='ascii')
doc=('# 当前布局布线资源\n\n'
    +f'工具：{result["tool"]}；器件：{result["device"]}。\n\n'
    +f'位流生成时间：{generated}。下载文件：`impl/pnr/ov5640.fs`。\n\n'
    +'| 资源 | 使用 / 总量 | 使用率 | 剩余 |\n| --- | --- | --- | --- |\n'
    +''.join(f'| {n} | {v["used"]:g} / {v["total"]:g} | {v["percent"]:.2f}% | {v["remaining"]:g} |\n' for n,v in resources.items())
    +f'\nSetup/Hold 违例端点：{violations}；最小时序裕量（ns）：{slack}。\n\n'
    +f'已核对 {len(pins)} 项引脚与地图、道路和字库 ROM 初始化。输入哈希与本轮综合快照一致。\n\n'
    +'本轮接入四路 VL53L1X，保留 IMU 零位并用编码器跨越短暂断帧，调整视觉主导的道路居中控制，新增斑马线减速。二值页面保留 104×78 双缓冲预览；原始图像页面仍显示原分辨率视频。详情见 [更新说明](upgrade_20261022.md)。\n\n'
    +'已通过 RTL 回归、Gowin 综合后预览验证及完整布局布线；尚未烧录或进行实车验证。\n\n'
    +f'位流 SHA256：`{sha(fs)}`。完整审计：`impl/pnr/ov5640_current_audit.json`。\n')
(ROOT/'docs/resource_usage.md').write_text(doc,encoding='utf-8')
print(json.dumps(dict(resources=resources,violated_endpoints=violations,min_slack_ns=slack,
    pins=len(pins),generated=generated,sha256=sha(fs)),indent=2))
