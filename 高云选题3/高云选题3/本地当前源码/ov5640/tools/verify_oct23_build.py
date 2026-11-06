"""Audit the October 23 sources, routed hardware, ROMs and fresh regressions.

This writes documentation/manifests only. It never regenerates RTL or uploads
the bitstream. Earlier dated audits remain historical evidence.
"""
from pathlib import Path
import datetime
import hashlib
import json
import math
import re
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road

ROOT = Path(__file__).resolve().parents[1]
def read(path):
    data = path.read_bytes()
    return data.decode('utf-16' if data.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig').replace('\r\n','\n')
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def dump(path, data): path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

baseline_path = ROOT/'impl/upgrade_20261023_baseline/source_hashes.json'
baseline = json.loads(read(baseline_path))
allowed = {
    'src/navigation/nav_centerline_bitmap.vh','src/navigation/nav_centerline_config.vh',
    'src/navigation/nav_centerline_init.vh','src/navigation/nav_centerline_reverse.vh',
    'src/navigation/nav_road_planner.v','src/navigation/road_centerlines.json',
    'src/portrait/vision_camera_config.vh','src/portrait/vision_video.v',
    'src/vehicle/vehicle_bypass.v','src/vehicle/vehicle_config.vh',
    'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_vision_fusion.v',
    'tools/generate_centerline.py','tools/generate_camera_assets.py',
    'sim/run_global_road_tests.py','sim/vehicle_close_bypass_tb.sv',
    'sim/vision_motion_tb.sv','sim/vision_fusion_tb.sv','sim/vision_video_tb.sv',
}
changed = [name for name,digest in baseline.items() if not (ROOT/name).exists() or sha(ROOT/name)!=digest]
assert set(changed)<=allowed, ('Unexpected baseline change',changed)
assert not [p for p in (ROOT/'src').rglob('*') if p.is_file() and p.relative_to(ROOT).as_posix() not in baseline], 'Unexpected new RTL'
for name in ('src/ov5640_ddr3_tft.cst','src/ov5640_ddr3_tft.sdc','build.tcl','ov5640.gprj',
             'src/portrait/portrait_display.v','src/vehicle/motor/motor_pi.v',
             'src/vehicle/motor/motor_encoder.v','src/vehicle/motor/ps2_test_control.v',
             'src/imu/imu_attitude.v','src/imu/lsm6dsv16x_spi.v',
             'src/vehicle/motor/vl53l1x_array_controller.v','src/vehicle/motor/vl53l1x_i2c_master.v'):
    assert sha(ROOT/name)==baseline[name], ('Preserved module/constraint changed',name)

snapshot_path = ROOT/'impl/oct23_synthesis_inputs.json'
snapshot = json.loads(read(snapshot_path))
inputs = [p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in
          ('.v','.vo','.vh','.mem','.cst','.sdc','.ipc','.mod')]+[ROOT/'ov5640.gprj',ROOT/'build.tcl']
actual_inputs = {p.relative_to(ROOT).as_posix():sha(p) for p in inputs}
assert actual_inputs==snapshot, 'Sources differ from actual synthesis input snapshot'
fs = ROOT/'impl/pnr/ov5640.fs'
assert fs.stat().st_mtime>=max([snapshot_path.stat().st_mtime]+[p.stat().st_mtime for p in inputs]), 'Stale main bitstream'
report_path,timing_path = ROOT/'impl/pnr/ov5640.rpt.txt',ROOT/'impl/pnr/ov5640.tr'
assert min(report_path.stat().st_mtime,timing_path.stat().st_mtime)>=fs.stat().st_mtime, 'Stale P&R reports'
report,timing = read(report_path),read(timing_path)
assert str(ROOT/'impl/gwsynthesis/ov5640.vg') in report, 'P&R report belongs to another design'
assert str(ROOT/'src/ov5640_ddr3_tft.cst') in report and str(ROOT/'src/ov5640_ddr3_tft.sdc') in report
resources = {}
for name in ('Logic','Register','CLS','BSRAM','DSP','I/O Port'):
    match = re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|',report,re.M)
    assert match, name
    used,total = map(float,match.groups())
    assert used<=total, ('Resource overflow',name)
    resources[name] = dict(used=used,total=total,remaining=total-used,percent=round(100*used/total,2))
violations,slack = {},{}
for kind in ('Setup','Hold'):
    match = re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)',timing)
    assert match, kind
    violations[kind] = int(match[1])
for kind in ('Setup','Hold','Recovery','Removal'):
    match = re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)',timing,re.M|re.S)
    assert match,kind
    values = [float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s',match[1],re.M)]
    assert values,kind
    slack[kind] = min(values)
assert not any(violations.values()) and all(v>=0 for v in slack.values()), ('Timing failure',violations,slack)
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',read(ROOT/'src/ov5640_ddr3_tft.cst'),re.M))
actual_pins = dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',report,re.M))
assert pins and len(pins)==len(set(pins.values())) and all(actual_pins.get(n)==p for n,p in pins.items()), 'Pin mismatch'

vehicle = list((ROOT/'src/vehicle').rglob('*.v'))+[ROOT/'src/vehicle/vehicle_config.vh',ROOT/'src/imu/imu_math.v']
vision = list((ROOT/'src/portrait').glob('vision_*.v'))+[ROOT/'src/portrait/vision_camera_config.vh']
navigation = list((ROOT/'src/navigation').glob('*.v'))+list((ROOT/'src/navigation').glob('nav_centerline_*.vh'))
suite_specs = {
    'vision': ('vision_test_results.txt', ['PASS vision display timing:','PASS vision video:',
        'PASS vision fusion:','PASS vision motion:','PASS vision zebra:','PASS motor brake plant:'],
        vehicle+vision+navigation+[ROOT/f'sim/{n}_tb.sv' for n in (
            'font_rows','vision_display_timing','vision_video','vision_zebra','vision_fusion',
            'vision_motion','vision_ui','motor_brake','motor_brake_plant')]+[ROOT/'sim/run_vision_tests.py']),
    'vision_netlist': ('vision_netlist_results.txt',['PASS synthesized vision:'],
        vision+[ROOT/'sim/vision_netlist_tb.sv',ROOT/'sim/run_vision_netlist_tests.py']),
    'vehicle': ('vehicle_test_results.txt',['PASS vehicle math:','PASS vertical straight startup:',
        'PASS smooth low-speed stop:','PASS navigation dynamic obstacle:','PASS route profile:',
        'PASS integrated I/O:'],vehicle+[ROOT/f'sim/vehicle_{n}_tb.sv' for n in ('math','motion','profile','io')]+[ROOT/'sim/run_vehicle_tests.py']),
    'centerline': ('centerline_test_results.txt',['PASS road planner:','PASS road UI:',
        'PASS road motion:','PASS barrier touch UI:','PASS physical beams',
        'PASS merge travel bound:','PASS bypass edge case','PASS bypass missing side edge:',
        'PASS bypass side trend:','PASS nose navigation:','PASS vision fusion:','PASS vision motion:'],
        vehicle+vision+navigation+[ROOT/f'sim/{n}_tb.sv' for n in (
            'road_planner','road_ui','road_motion','nav_barrier_ui','vehicle_close_bypass',
            'bypass_edge_trend','vehicle_nose','vision_fusion','vision_motion')]+[ROOT/'sim/run_centerline_tests.py']),
    'global_road': ('road_global_test_results.txt',['PASS global road case 10:',
        'PASS global road oracle:'],navigation+[ROOT/'sim/road_global_tb.sv',ROOT/'sim/run_global_road_tests.py']),
    'photo_motion': ('oct23_photo_motion_results.txt',['PASS road motion:'],
        vehicle+[ROOT/'sim/road_motion_tb.sv',ROOT/'sim/run_oct23_photo_motion.py',ROOT/'sim/oct23_photo_route.csv']),
}
logs = {}
for suite,(filename,markers,dependencies) in suite_specs.items():
    path = ROOT/'sim'/filename
    content = read(path)
    assert all(marker in content for marker in markers), ('Missing completion marker',suite,[m for m in markers if m not in content])
    assert not re.search(r'^(?:FATAL|FAIL|ERROR|Traceback)\b',content,re.M), ('Failed regression',suite)
    assert path.stat().st_mtime>=max(p.stat().st_mtime for p in dependencies), ('Stale regression',suite)
    logs[suite] = dict(file=path.relative_to(ROOT).as_posix(),sha256=sha(path),output=content,
                       dependency_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in dependencies})

project_sources = re.findall(r'<File path="([^"]+)"',read(ROOT/'ov5640.gprj'))
assert all((ROOT/p).exists() for p in project_sources)
netlist = read(ROOT/'impl/gwsynthesis/ov5640.vg')
for name in ('vision_video','vision_zebra','vehicle_vision_fusion','vehicle_bypass','vl53l1x_array_controller'):
    assert re.search(r'^module '+name+r'\b',netlist,re.M), ('Synthesized module missing',name)
suffix_count = len(re.findall(r'^\s*SDPB suffix_cost_suffix_cost_\w+\s*\(',netlist,re.M))
assert suffix_count==16, ('Waypoint suffix RAM changed',suffix_count)
assert len(re.findall(r'^\s*SDPB bank[01]_bank[01]_\w+\s*\(',netlist,re.M))==6, 'Thumbnail RAM changed'
roms = dict(map=audit_map(),font=audit_font(),road=audit_road())

# Refresh the human-readable geometry report without touching generated RTL.
cfg = {k:int(v) for k,v in re.findall(r'^`define (VEHICLE_\w+) (-?\d+)$',read(ROOT/'src/vehicle/vehicle_config.vh'),re.M)}
vehicle_report = dict(parameters=cfg,
    minimum_operating_radius_mm=cfg['VEHICLE_WHEELBASE_MM']/math.tan(math.radians(cfg['VEHICLE_MAX_STEER_DEG'])),
    circumference_mm=math.pi*cfg['VEHICLE_WHEEL_DIAMETER_MM'],
    counts_per_mm=cfg['VEHICLE_COUNTS_PER_REV']/(math.pi*cfg['VEHICLE_WHEEL_DIAMETER_MM']),
    servo_operating_offset_deg=70,parameters_are_estimates=True,
    note='Body/camera measurements supplied by user; steering linkage, wheel diameter and encoder scale require real-car calibration.')
dump(ROOT/'docs/vehicle_parameters.json',vehicle_report)
generated = datetime.datetime.fromtimestamp(fs.stat().st_mtime,datetime.timezone(datetime.timedelta(hours=8))).isoformat()
result = dict(bitstream='impl/pnr/ov5640.fs',sha256=sha(fs),generated_time_utc8=generated,
    tool='Gowin V1.9.12.03',device='GW5AT-LV60PG484AC1/I0 B',resources=resources,
    violated_endpoints=violations,min_slack_ns=slack,timing_passed=True,hardware_tested=False,
    baseline=baseline_path.relative_to(ROOT).as_posix(),changed_inputs=changed,
    synthesis_input_snapshot=snapshot_path.relative_to(ROOT).as_posix(),input_sha256=actual_inputs,
    pins_verified=pins,roms=roms,regressions=logs,camera=json.loads(read(ROOT/'docs/camera_parameters.json')),
    vehicle=vehicle_report,
    route=dict(photo_case=10,old_graph_cost_cm=779,new_graph_cost_cm=406,
        terminal_manhattan_tolerance_cm=6,lookahead_mm=60,t_junction_nose_radius_mm=239,
        note='Shortest ordered directed-graph route. Costs round each edge to integer cm. White-road tests cover the nose path, not full chassis road clearance.'),
    bypass=dict(merge_lookahead_mm=140,side_rise_mm=20,required_rise_observations=2,
        physical_collision_test='320x200 mm swept rectangle, independent bicycle and four ray sensors',
        merge_forward_bound_mm=650),
    limitations=['No upload or real-car validation performed.',
        'Intrinsics, optical alignment and steering calibration are provisional.',
        'Map nose-path containment does not prove whole-body clearance on real road corners.'])
dump(ROOT/'impl/pnr/ov5640_oct23_audit.json',result)
current_audit = ROOT/'impl/pnr/ov5640_current_audit.json'
if current_audit.exists():
    previous = json.loads(read(current_audit))
    if previous.get('sha256')!=result['sha256']:
        archive = ROOT/'impl/pnr/ov5640_before_oct23_audit.json'
        if not archive.exists(): archive.write_bytes(current_audit.read_bytes())
dump(current_audit,result)
dump(ROOT/'impl/pnr/ov5640_download.json',{k:result[k] for k in (
    'bitstream','sha256','generated_time_utc8','tool','device','timing_passed','hardware_tested')})
doc = ('# 2026-10-23 全量构建资源\n\n'
    +f'工具：{result["tool"]}；器件：{result["device"]}。\n\n'
    +f'位流：`impl/pnr/ov5640.fs`，生成时间（UTC+8）：{generated}。\n\n'
    +'| 资源 | 使用 / 总量 | 使用率 | 剩余 |\n| --- | ---: | ---: | ---: |\n'
    +''.join(f'| {name} | {v["used"]:g} / {v["total"]:g} | {v["percent"]:.2f}% | {v["remaining"]:g} |\n' for name,v in resources.items())
    +f'\nSetup/Hold 违例：{violations}；所报告路径的最小裕量（ns）：{slack}。\n\n'
    +f'已核对 {len(pins)} 项约束引脚，以及地图、道路、字库 ROM 初始化；{len(actual_inputs)} 项构建输入哈希均与实际综合快照一致。\n\n'
    +'本轮更新摄像头地面投影、终点区域最短路、较晚路口入弯、侧测距边缘趋势和快速回轨。'
    +'六组本轮回归结果和源文件哈希见 `impl/pnr/ov5640_oct23_audit.json`。'
    +'旧日期审计保留为历史记录，不代表当前位流。\n\n'
    +'完整综合、布局布线与回归通过；尚未烧录或进行实车验证。道路检查证明车头轨迹在白色道路内，不替代整车道路边界实测。'
    +'详见 [本轮说明](upgrade_20261023.md)。\n\n'
    +f'位流 SHA-256：`{sha(fs)}`。\n')
(ROOT/'docs/resource_usage.md').write_text(doc,encoding='utf-8')
print(json.dumps(dict(resources=resources,violations=violations,min_slack_ns=slack,
    pins=len(pins),inputs=len(actual_inputs),generated=generated,sha256=sha(fs)),indent=2))
