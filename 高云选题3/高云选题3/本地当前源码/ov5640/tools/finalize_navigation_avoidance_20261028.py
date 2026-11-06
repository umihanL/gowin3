"""Bind the obstacle-switch release to tested inputs, ROMs, timing and resources."""
from pathlib import Path
import datetime, hashlib, json, sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from verify_font_netlist import audit as audit_font
from verify_map_netlist import audit as audit_map
from verify_road_netlist import audit as audit_road

LABEL = 'navigation_avoidance_20261028'
sha = lambda n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
read = lambda n: json.loads((ROOT/n).read_text(encoding='utf-8'))
result = read(f'docs/{LABEL}_validation.json')
assert result['release'] == LABEL and result['timing_passed']
assert result['status'] in ('place_route_timing_passed', 'digital_verification_passed')
assert all(sha(n) == digest for n, digest in result['hardware_inputs'].items())
assert sha(result['bitstream']) == result['sha256'] == sha(result['release_bitstream'])

def verify(row, log_field='log_sha256'):
    assert row['passed'] and sha(row['log']) == row[log_field], row.get('name')
    assert all(sha(n) == digest for n, digest in row['inputs'].items()), row.get('name')
    for key in ('route', 'trace'):
        if key in row: assert sha(row[key]) == row[key+'_sha256']

runs = read('sim/navigation_avoidance_runs.json')
assert {row['name'] for row in runs} == {'modes', 'settings'}
runs += [read('sim/navigation_avoidance_control_run.json')]
for row in runs: verify(row)
control = (ROOT/'sim/navigation_avoidance_control.log').read_text(encoding='utf-8')
assert control.count('PASS OFF stop:') == 4 and 'preserved (8 cases)' in control
assert 'PASS ON side selection, independent avoidance task, OFF emergency and IMU stop protections' in control

routes = read('sim/navigation_avoidance_motion_runs.json')
assert {r['name'] for r in routes} == {'outer_left', 'outer_right', 's_forward', 'photo', 'outer_left_hostile'}
for row in routes:
    verify(row)
    m = row['metrics']
    assert m['status'] == 3 and m['index'] == m['count']-1 and m['target_checks'] >= m['count']
    normal = row['name'].removesuffix('_hostile')
    assert (ROOT/row['trace']).read_bytes() == (ROOT/f'sim/inertial_audit_{normal}_trace.csv').read_bytes()
assert (ROOT/'sim/navigation_avoidance_outer_left_trace.csv').read_bytes() == (
    ROOT/'sim/navigation_avoidance_outer_left_hostile_trace.csv').read_bytes()

# Reuse the audited planner outputs: planner/map sources did not change.
base = read('impl/baseline_navigation_avoidance_20261028/manifest.json')
assert sha('src/navigation/nav_road_planner.v') == base['src/navigation/nav_road_planner.v']
plan = read('sim/inertial_audit_plan_runs.json')
for row in plan:
    assert row['passed'] and row['directed_graph_valid']
    assert sha(row['route']) == row['route_sha256']
    assert sha(row['log']) == row['log_sha256']
assert result['map_files_unchanged']

host = read('sim/navigation_avoidance_host_runs.json')
assert len(host) == 4
for row in host: verify(row, 'sha256')
vehicle = read('sim/navigation_avoidance_vehicle_run.json')
verify(vehicle)
assert 'PASS navigation dynamic obstacle:' in (ROOT/vehicle['log']).read_text(encoding='utf-8')

snapshots = ['docs/settings_navigation_avoidance_on.png', 'docs/settings_navigation_avoidance_off.png']
assert all((ROOT/n).exists() for n in snapshots)
result.update(
    status='digital_verification_passed', digital_regressions=runs,
    inertial_route_regressions=routes, planner_outputs_reused_unchanged=plan,
    host_regressions=host, vehicle_regression=vehicle,
    rom_audits=dict(font=audit_font(), map=audit_map(), road=audit_road()),
    settings_screenshots={n: sha(n) for n in snapshots},
    verified_behavior=dict(avoidance_default_off=True, setting_latched_at_navigation_entry=True,
        navigation_side_tof_steering_disabled=True, off_keeps_planned_target_and_range_stop=True,
        on_preserves_detour_and_route_rejoin=True, independent_avoidance_task_preserved=True,
        inertial_hostile_vision_and_side_tof_trace_identical=True),
    limitations=[
        'Not flashed or tested on the physical vehicle.',
        'Route regressions use an ideal bicycle plant and do not prove real clearance or braking distance.',
        'OFF still uses front ranges for speed limiting/emergency stops and retains IMU/encoder/route protections.',
        'Ordinary obstacle waiting resumes on fresh valid clearance readings; emergency/fault stops require re-entry.',
        'Vision fusion and Gaussian filter are inherited; camera calibration remains provisional.',
    ])

when = datetime.datetime.fromisoformat(result['generated_utc']).astimezone(
    datetime.timezone(datetime.timedelta(hours=8))).isoformat()
table = ('| 资源 | 已用 / 总量 | 使用率 | 剩余 |\n| --- | ---: | ---: | ---: |\n'+
    ''.join(f'| {n} | {v["used"]:g} / {v["total"]:g} | {v["percent"]:.2f}% | {v["remaining"]:g} |\n'
            for n, v in result['resources'].items()))
resource = (
    '# 当前布局布线资源\n\n'
    f'版本：`{LABEL}`。工具：{result["tool"]}；器件：{result["device"]}。\n\n'
    f'位流生成时间：{when}（UTC+8）。默认下载：`{result["bitstream"]}`；'
    f'具名副本：`{result["release_bitstream"]}`。\n\n'+table+
    f'\nSetup/Hold 违规端点：{result["timing_violations"]}；最小时序裕量（ns）：{result["min_slack_ns"]}。\n\n'
    '数据来自本轮完整综合和布局布线。CLS 与 Logic 有重叠，不能相加；'
    '未新增图像帧缓存，原算法选择和高斯滤波继续保留。\n\n'
    '自主避障默认关闭、侧方转角隔离、沿路线停车/继续、设置操作、五组实际规划路线、'
    '开启后的整车绕障回归、串口暂停/继续及综合后地图/道路/字库 ROM 核验通过。未烧录、未实车验证。\n\n'
    f'SHA-256：`{result["sha256"]}`。完整审计：`impl/pnr/ov5640_current_audit.json`。\n\n'
    f'[本轮说明]({LABEL}.md)。\n')
(ROOT/'docs/resource_usage.md').write_text(resource, encoding='utf-8')

note_path = ROOT/f'docs/{LABEL}.md'
note = note_path.read_text(encoding='utf-8').split('\n## 最终构建与下载')[0]
note = note.replace('（完成构建审计后生成）', '')
note += ('\n## 最终构建与下载\n\n'+table+
    f'\nSetup/Hold 违规端点为 0/0，最小时序裕量（ns）：{result["min_slack_ns"]}。\n\n'
    f'生成时间：{when}。位流：`{result["release_bitstream"]}`；默认 `impl/pnr/ov5640.fs` 内容一致。'
    f'SHA-256：`{result["sha256"]}`。未烧录。\n\n'
    '[开启状态截图](settings_navigation_avoidance_on.png) · [关闭状态截图](settings_navigation_avoidance_off.png)。\n')
note_path.write_text(note, encoding='utf-8')

readme_path = ROOT/'README.md'
readme = readme_path.read_text(encoding='utf-8')
first = readme.split('\n', 1)[0]
history = readme[readme.index('## 历史记录'):]
intro = (
    f'\n\n**当前版本：自主导航避障开关（{LABEL}）。** 设置新增“自主避障：关闭 / 开启”，默认关闭。'
    '自主导航关闭侧方 ToF 替换规划转角；仅开启新开关后允许自动绕障，关闭时沿原路线减速停车。'
    'IMU/编码器、前方紧急停车及其他故障保护保留；算法选择和高斯滤波继续保留。\n\n'
    f'完整综合、布局布线及 Setup/Hold 检查通过，违规端点 0/0。下载 `{result["release_bitstream"]}`'
    f'或默认 `impl/pnr/ov5640.fs`，SHA-256 前缀 `{result["sha256"][:16]}`。未烧录、未实车验证。'
    f'详见[本轮修改与测试](docs/{LABEL}.md)及[实际资源](docs/resource_usage.md)；'
    '融合方案与相机测量清单见[前轮说明](docs/navigation_modes_filter_20261027.md)。\n\n')
readme_path.write_text(first+intro+history, encoding='utf-8')

review = ROOT/'docs/inertial_navigation_audit_20261028.md'
prefix = f'> 历史检查：本报告针对修改前的版本；两项 ToF 转向干扰已按[本轮说明]({LABEL}.md)关闭或加入开关。\n\n'
text = review.read_text(encoding='utf-8')
if not text.startswith(prefix):
    old_report_sha = sha(review.relative_to(ROOT).as_posix())
    review.write_text(prefix+text, encoding='utf-8')
    historical_path = ROOT/'docs/inertial_navigation_audit_20261028_validation.json'
    historical = json.loads(historical_path.read_text(encoding='utf-8'))
    assert historical['report_sha256'] == old_report_sha
    historical['report_before_history_notice_sha256'] = old_report_sha
    historical['report_sha256'] = sha(review.relative_to(ROOT).as_posix())
    historical['superseded_by'] = f'docs/{LABEL}_validation.json'
    historical_path.write_text(json.dumps(historical, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

result['documentation'] = {n: sha(n) for n in (f'docs/{LABEL}.md', 'docs/resource_usage.md', 'README.md')}
result['scripts'] = {n: sha(n) for n in (
    'tools/build_navigation_avoidance_20261028.py', 'tools/finalize_navigation_avoidance_20261028.py',
    'sim/run_navigation_avoidance_tests.py', 'sim/run_navigation_avoidance_routes.py',
    'sim/run_navigation_avoidance_vehicle.py', 'sim/run_navigation_avoidance_host.py')}
for name in (f'docs/{LABEL}_validation.json', 'impl/pnr/ov5640_current_audit.json',
             f'impl/pnr/ov5640_{LABEL}_audit.json'):
    (ROOT/name).write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k: result[k] for k in ('status', 'release_bitstream', 'sha256', 'resources')}, indent=2))
