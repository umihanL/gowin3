"""Publish local release notes only after PnR and source-bound tests pass."""
from pathlib import Path
import datetime, hashlib, json, re, sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from verify_font_netlist import audit as audit_font
from verify_map_netlist import audit as audit_map
from verify_road_netlist import audit as audit_road

LABEL = 'navigation_modes_filter_20261027'
sha = lambda n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
audit_path = ROOT/f'docs/{LABEL}_validation.json'
result = json.loads(audit_path.read_text(encoding='utf-8'))
assert result['timing_passed'] and result['status'] == 'place_route_timing_passed'
assert all(sha(n) == digest for n, digest in result['hardware_inputs'].items())
assert sha(result['bitstream']) == result['sha256'] == sha(result['release_bitstream'])

runs = json.loads((ROOT/'sim/navigation_modes_filter_runs.json').read_text())
assert {row['name'] for row in runs} == {'gaussian', 'modes', 'settings', 'noise_2', 'noise_5'}
for row in runs:
    assert row['passed'] and sha(row['log']) == row['log_sha256']
    assert all(sha(n) == digest for n, digest in row['inputs'].items()), row['name']
host_runs = json.loads((ROOT/'sim/host_link_runs.json').read_text())
for row in host_runs:
    assert row['passed'] and sha(row['log']) == row['sha256']
    assert all(sha(n) == digest for n, digest in row['inputs'].items()), row['name']

before_log = 'sim/navigation_modes_filter_before_retry.log'
assert 'navigation trapped behind prior target attempt=0 state=6' in (ROOT/before_log).read_text()
motion_source = (ROOT/'src/vehicle/vehicle_motion.v').read_text()
begin = motion_source.index('                    // A previous mode can leave a calculation in flight when')
end = motion_source.index('                    if(current_tracker_done', begin)
before_source = 'impl/navigation_modes_filter_motion_before_retry.v'
assert (ROOT/before_source).read_text() == motion_source[:begin]+motion_source[end:]

vision_log = ROOT/'sim/vision_netlist_results.txt'
vision_text = vision_log.read_text(encoding='utf-8')
assert 'PASS synthesized vision:' in vision_text and not re.search(r'^FATAL\b', vision_text, re.M)
assert vision_log.stat().st_mtime >= max((ROOT/n).stat().st_mtime for n in
    ('src/portrait/vision_video.v', 'src/portrait/vision_zebra.v'))
vehicle_run = json.loads((ROOT/'sim/navigation_modes_filter_vehicle_run.json').read_text())
assert vehicle_run['passed'] and sha(vehicle_run['log']) == vehicle_run['log_sha256']
assert all(sha(n) == digest for n, digest in vehicle_run['inputs'].items())
vehicle_log = (ROOT/vehicle_run['log']).read_text(encoding='utf-8')
assert 'PASS navigation dynamic obstacle:' in vehicle_log
motion_log = (ROOT/'sim/vision_test_results.txt').read_text(encoding='utf-8')
assert motion_log.count('PASS immediate startup:') == 3
assert motion_log.count('PASS terminal arrival:') == 4
assert 'FATAL:' not in motion_log

result['rom_audits'] = {'font': audit_font(), 'map': audit_map(), 'road': audit_road()}
result['digital_regressions'] = runs
result['host_regressions'] = host_runs
result['vehicle_regression'] = vehicle_run
result['handoff_before_fix_reproduction'] = dict(
    expected_failure=True, log=before_log, log_sha256=sha(before_log),
    variant=before_source, variant_sha256=sha(before_source),
    testbench='sim/navigation_modes_filter_tb.sv', testbench_sha256=sha('sim/navigation_modes_filter_tb.sv'))
result['other_test_logs'] = {n: sha(n) for n in (
    'sim/vision_netlist_results.txt', 'sim/navigation_modes_filter_vehicle_motion.log', 'sim/vision_test_results.txt')}
result['status'] = 'digital_verification_passed'
result['limitations'] = [
    'Not flashed or tested on the physical vehicle.',
    'Camera intrinsics, distortion and image-ground correspondences remain provisional.',
    'Vision controls lane centring; no runtime visual x/y pose correction or visual odometry is added.',
    'Immediate start assumes the physical vehicle is placed near the selected start and initial route heading.',
    'Gaussian filtering reduces fine noise, not arbitrary shadows, reflections or calibration errors.',
]
for name in (f'docs/{LABEL}_validation.json', 'impl/pnr/ov5640_current_audit.json',
             f'impl/pnr/ov5640_{LABEL}_audit.json'):
    (ROOT/name).write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

when = datetime.datetime.fromisoformat(result['generated_utc']).astimezone(
    datetime.timezone(datetime.timedelta(hours=8))).isoformat()
table = ('| 资源 | 已用 / 总量 | 使用率 | 剩余 |\n| --- | ---: | ---: | ---: |\n'+
         ''.join(f'| {n} | {v["used"]:g} / {v["total"]:g} | {v["percent"]:.2f}% | {v["remaining"]:g} |\n'
                 for n, v in result['resources'].items()))
resource_doc = (
    '# 当前布局布线资源\n\n'
    f'版本：`{LABEL}`。工具：{result["tool"]}；器件：{result["device"]}。\n\n'
    f'位流生成时间：{when}（UTC+8）。默认下载：`{result["bitstream"]}`；'
    f'具名副本：`{result["release_bitstream"]}`。\n\n'+table+
    f'\nSetup/Hold违规端点：{result["timing_violations"]}；最小时序裕量（ns）：{result["min_slack_ns"]}。\n\n'
    '菜单的两种算法与3×3高斯滤波均已保留。资源数据来自本轮完整布局布线；'
    'CLS是逻辑单元的物理打包资源，与Logic有重叠，不能相加。BSRAM仍为113块，未增加图像帧缓存。\n\n'
    '已验证两种算法启动/隔离/失视回退、菜单操作、高斯卷积和读取抢占、2%/5%图像噪点、'
    'IMU/障碍物停车、终点与绕障回归、串口暂停/继续，以及Gowin综合后图像输出和地图/字库ROM。'
    '尚未烧录或实车验证。\n\n'
    f'SHA-256：`{result["sha256"]}`。完整审计：`impl/pnr/ov5640_current_audit.json`。\n\n'
    '[本轮算法说明与相机测量清单](navigation_modes_filter_20261027.md)。\n'
)
(ROOT/'docs/resource_usage.md').write_text(resource_doc, encoding='utf-8')

note_path = ROOT/f'docs/{LABEL}.md'
note = note_path.read_text(encoding='utf-8')
note += ('\n## 本轮最终验证与下载\n\n'+table+
         f'\nSetup/Hold违规：{result["timing_violations"]}；最小时序裕量（ns）：{result["min_slack_ns"]}。\n\n'
         f'位流：`{result["release_bitstream"]}`（默认`impl/pnr/ov5640.fs`内容一致），'
         f'生成于{when}，SHA-256：`{result["sha256"]}`。未烧录。\n\n'
         '5组本轮定向仿真全部通过；高斯2808像素独立卷积及2792次读取抢占一致；'
         '34/37.3 cm道路的2%/5%椒盐像素测试通过；原有直行、90°转弯、绕障、终点、'
         '主机通信暂停/继续等数字回归通过。完整结果及输入哈希见同名validation.json。\n')
note_path.write_text(note, encoding='utf-8')

readme_path = ROOT/'README.md'
readme = readme_path.read_text(encoding='utf-8')
first, rest = readme.split('\n', 1)
intro = (
    f'\n**当前版本：导航算法选择与高斯滤波（{LABEL}）。** 设置页新增“惯性导航 / 惯导加视觉”；'
    '道路识别在二值化前增加3×3高斯滤波。自主导航取消视觉起步等待、静止视觉位姿重置与两秒倒计时，'
    '移除V:TRACK/V:WAIT提示；无可靠视觉时使用地图/编码器/IMU跟踪。\n\n'
    f'完整综合、布局布线及Setup/Hold检查通过，违规端点0/0。下载`{result["release_bitstream"]}`'
    f'或默认`impl/pnr/ov5640.fs`，SHA-256前缀`{result["sha256"][:16]}`。未烧录、未实车验证。'
    f'详见[修改、融合方案与测量清单](docs/{LABEL}.md)及[实际资源](docs/resource_usage.md)。\n\n'
    '## 历史记录（以下说明不代表当前位流）\n'
)
readme_path.write_text(first+'\n'+intro+rest, encoding='utf-8')
for name in ('vision_navigation.md', 'settings_and_drive.md'):
    p = ROOT/'docs'/name
    p.write_text(f'> 当前导航模式、高斯滤波、取消视觉启动等待与相机测量要求以'
                 f'[本轮说明]({LABEL}.md)为准。以下保留历史说明。\n\n'+p.read_text(encoding='utf-8'), encoding='utf-8')
print(json.dumps({k: result[k] for k in ('status', 'release_bitstream', 'sha256', 'resources')}, indent=2))
