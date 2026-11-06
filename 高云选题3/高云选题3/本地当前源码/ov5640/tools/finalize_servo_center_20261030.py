"""Bind test evidence and user documentation to the generated release."""
from pathlib import Path
import hashlib,json

R=Path(__file__).resolve().parents[1]
LABEL='servo_center_20261030'
sha=lambda p:hashlib.sha256((R/p).read_bytes()).hexdigest()
read=lambda p:json.loads((R/p).read_text(encoding='utf-8'))
validation=read(f'docs/{LABEL}_validation.json')
assert validation['release']==LABEL and validation['timing_passed']
assert validation['sha256']==sha(validation['bitstream'])==sha(validation['release_bitstream'])
assert all(sha(p)==h for p,h in validation['hardware_inputs'].items())
baseline=read(f'impl/{LABEL}_previous_inputs.json')
assert len(validation['changed_hardware_sources'])==10
for p in ['src/vehicle/servo_offset.mem','src/vehicle/rear_ratio.mem',
          'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_tracker.v','src/vehicle/vehicle_odometry.v',
          'src/navigation/nav_road_planner.v','src/navigation/nav_centerline_init.vh','src/navigation/nav_centerline_reverse.vh',
          'src/portrait/vision_zebra.v','src/portrait/vision_video.v']:
    assert sha(p)==baseline[p],p

tests=read('sim/servo_center_20261030_runs.json')
assert len(tests)==4
for test in tests:
    assert test['passed'] and sha(test['log'])==test['log_sha256']
    assert all(sha(p)==h for p,h in test['inputs'].items()),test['top']
logs=[]
for name in [*[test['log'] for test in tests], 'sim/settings_test_results.txt',
             *[f'sim/inertial_zebra_{n}.log' for n in ('modes','avoidance','zebra','camera','uart','host')]]:
    content=(R/name).read_text(encoding='utf-8')
    assert 'PASS ' in content and 'FATAL:' not in content,name
    logs.append(dict(log=name,sha256=sha(name),results=[s.strip() for s in content.splitlines() if 'PASS ' in s]))
motions=[v for v in read('sim/zebra_smooth_motion_runs.json') if v['name'] in
         ('s_forward','s_reverse','outer_left','outer_right','s_forward_zebra')]
assert len(motions)==5
for result in motions:
    assert result['passed'] and result['metrics']['status']==3 and sha(result['log'])==result['log_sha256']
    result['trace_sha256']=sha(result['trace'])

from PIL import Image
images=[]
for name in ('parameters','adjust'):
    image=f'docs/servo_center_{name}.png'
    Image.open(R/f'sim/servo_center_{name}.ppm').save(R/image)
    images.append(dict(path=image,sha256=sha(image)))
for path in (f'docs/{LABEL}_validation.json', f'impl/pnr/ov5640_{LABEL}_audit.json','impl/pnr/ov5640_current_audit.json'):
    data=read(path)
    assert data['sha256']==validation['sha256']
    data['simulation_evidence']=dict(actuator_and_ui=tests,logs=logs,motions=motions,screenshots=images)
    data['known_limitations']=[
        'Not flashed or tested on a physical vehicle.',
        'Centre is a RAM setting; power-on/reset restores 90.0 degrees.',
        'The 50.0..120.0 centre range guarantees 0..180 electrical commands, not measured mechanical clearance.',
        'The original nonlinear linkage table is translated; new physical linkage geometry is not automatically recalibrated.',
        '0.1 degree is the command setting resolution; actual steering accuracy depends on servo deadband and linkage.']
    (R/path).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
resources=validation['resources'];slack=validation['min_slack_ns']
report=[
    '# 本轮 FPGA 资源报告', '',
    f'版本：`{LABEL}`。器件：Gowin `GW5AT-LV60PG484AC1/I0 B`；工具：Gowin V1.9.12.03。', '',
    '已完成综合、布局布线及检查，尚未烧录或实车验证。默认位流：`impl/pnr/ov5640.fs`；'
    f'版本副本：`impl/pnr/ov5640_{LABEL}.fs`。', '',
    '| 资源 | 已用 / 总量 | 使用率 | 剩余 |', '| --- | ---: | ---: | ---: |']
for name,value in resources.items():
    number=lambda x:f'{x:,.1f}'.rstrip('0').rstrip('.')
    report.append(f"| {name} | {number(value['used'])} / {number(value['total'])} | {value['percent']:.2f}% | {number(value['remaining'])} |")
report+=['', 'Setup/Hold 违规均为 0。最小 slack：'+
         '，'.join(f'{name} {value:.3f} ns' for name,value in slack.items())+'。', '',
         f"位流 SHA-256：`{validation['sha256']}`。本次通过检查的完整构建耗时约 {validation['build_metrics']['seconds']:.1f} 秒。", '',
         '中值支持 0.1° 设置、50.0°～120.0° 范围；输出限位随中值平移，原非线性转向表和后轮差速保留。'
         'PWM ROM 从 1 个 BSRAM 扩为 2 个，所有 2048 项已与综合网表逐位核对。', '',
         '4 项专用仿真（33 MHz PWM、设置至整车舵机、完整菜单、整车 I/O）、电机响应回归、'
         '6 项相机/控制回归和 5 条路线运动回放通过。地图、中心线、中文字库、PWM ROM 及引脚约束检查通过。', '',
         f'详细说明见 [`{LABEL}.md`]({LABEL}.md)，输入哈希、日志和资源数据见 '
         f'[`{LABEL}_validation.json`]({LABEL}_validation.json)。', '']
(R/'docs/resource_usage.md').write_text('\n'.join(report),encoding='utf-8')
readme=R/'README.md';raw=readme.read_bytes()
if f'**当前版本：可调舵机中值（`{LABEL}`）'.encode('utf-8') not in raw:
    raw=raw.replace('**当前版本：'.encode('utf-8'),'**2026-10-29 更新：'.encode('utf-8'),1)
    nl=b'\r\n' if b'\r\n' in raw else b'\n';start=raw.index(nl+nl)+len(nl+nl)
    intro=(f'**当前版本：可调舵机中值（`{LABEL}`）。** 设置→参数设置新增“舵机中值”，支持 '
           '±0.1°/±1.0° 调节，范围 50.0°～120.0°；输出上下限随中值平移，原非线性标定与前轮 ±28° '
           '限制保留。设置在本次上电期间有效，复位恢复 90.0°。纯惯导、斑马线限速和原后轮差速继续使用。'
           '综合、布局布线、时序及仿真检查通过；尚未烧录或实车验证。'
           f'详见 [操作与标定说明](docs/{LABEL}.md)、[验证记录](docs/{LABEL}_validation.json) '
           '和 [资源报告](docs/resource_usage.md)。').encode('utf-8')
    readme.write_bytes(raw[:start]+intro+nl+nl+raw[start:])
print('PASS frozen hardware inputs, release hashes, 4 actuator/UI cases, 6 camera/control cases and 5 route replays')
