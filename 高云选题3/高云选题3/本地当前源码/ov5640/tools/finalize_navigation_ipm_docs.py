"""Publish IPM documentation only for the successfully audited bitstream."""
from pathlib import Path
import datetime
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'navigation_ipm_20261026'
audit = json.loads((ROOT / f'docs/{LABEL}_validation.json').read_text(encoding='utf-8'))
assert audit['status'] == 'digital_verification_passed'
assert audit['timing_passed'] and audit['regression_completed_runs'] == 74
assert hashlib.sha256((ROOT / audit['bitstream']).read_bytes()).hexdigest() == audit['sha256']
generated = datetime.datetime.fromisoformat(audit['generated_utc']).astimezone(
    datetime.timezone(datetime.timedelta(hours=8)))
rows = '\n'.join(f"| {name} | {int(v['used'])} / {int(v['total'])} | {v['percent']:.2f}% | {v['delta_from_previous']:+g} |"
                 for name, v in audit['resources'].items())
timing = audit['min_slack_ns']
resources = f"""# 当前构建资源与时序（2026-10-26）

对应位流：`impl/pnr/ov5640.fs`，生成于 {generated:%Y-%m-%d %H:%M:%S}（UTC+8）。
详见 [逆透视循迹与绕障更新](navigation_ipm_20261026.md) 和 [完整验收记录](navigation_ipm_20261026_validation.json)。

| 资源 | 使用 / 总量 | 占用率 | 相对上一验收版变化 |
| --- | ---: | ---: | ---: |
{rows}

Setup/Hold 违规端点：{audit['violated_endpoints']['Setup']}/{audit['violated_endpoints']['Hold']}。
最小 Setup/Hold 余量：{timing['Setup']:.3f}/{timing['Hold']:.3f} ns；Recovery/Removal：{timing['Recovery']:.3f}/{timing['Removal']:.3f} ns。
本轮综合、布局布线耗时 {audit['build']['seconds']:.3f} 秒。采用普通布线：此前实测增量匹配更慢，本轮没有保留其他模块布局布线的保证。

完成 74 次仿真调用（包含不同套件中重复的回归测试），其中 21 次为实际地图路线闭环测试；图像模块已完成 Gowin 综合后 RTL/网表比较。
引脚及地图、道路、字库 ROM 初始化已核对。相对上一验收源码，本轮改变七个硬件输入文件（包括新增的逆透视查表），其余 {audit['unchanged_hardware_source_count']} 个硬件源码文件保留。
传感器驱动、舵机映射和电机速度环源代码未改。

摄像头内参仍为估计值，未根据用户照片拟合；未烧录，未实车验证。已有综合警告仍保留。

SHA-256：`{audit['sha256']}`
"""
(ROOT / 'docs/resource_usage.md').write_text(resources, encoding='utf-8')
details = f"""# 逆透视视觉循迹、单前方激光绕障和斑马线验收（2026-10-26）

当前下载文件：[ov5640.fs](../impl/pnr/ov5640.fs)。生成时间 {generated:%Y-%m-%d %H:%M:%S}（UTC+8），{audit['bytes']} 字节。
本记录表示数字验证通过；未连接车辆、未烧录、未实车验证。

## 摄像头与逆透视

已写入最新测量：光心高 185mm、车头后方 15mm、距右侧边界 102mm（相对车身中线向左 2mm），画面中心地面点距车头 380mm。
标定地面点按车头前方 200/400/600mm 记录，中间三个点均为 400mm；远端左点向左 100mm，近端右点向右 100mm。
参数记录在 [camera_parameters.json](camera_parameters.json)，生成工具为 `tools/generate_camera_assets.py`。

这些地面位置还没有与原始画面的像素坐标配对：本工作区没有所提照片文件，也没有可用的对应像素坐标。
因此没有声称完成相机标定或使用照片拟合。水平视场角暂按 60°、方形像素、零畸变/偏航/横滚估计。
投影俯角以实测中心地面点约束，约为 25.096°；原机械俯角配置保留 30°，两者不应混同。
后续精标定需要原始 640×480 图像上各地面标记的 `(u,v)`，以及相应的车头前向距离和相对车身中线的左右距离。

`vision_video.v` 对 27 条扫描行使用逐行光线与地面相交查表，将道路中心像素映射到毫米。
近场误差为车头前方 210mm 处的道路中心；航向观测为地面中心斜率乘 110mm。
使用邻近三行稳健中心、有效远近分组及单组退化处理。范围限定在车头前方 150–800mm；不能补出摄像头看不到的近地道路。
运算复用串行移位加法和除法，保留原 104×78 双缓冲与预览布局，没有新增整帧图像 RAM。

## 响应与融合

控制周期为 10ms（100Hz），新的视觉观测按实际画面更新，并不是每 10ms 生成一幅新图像。
首个有效完整观测即可获得视觉控制权，近场与航向死区缩至 ±1mm；小角度输出限速为每周期 6/16°。
原先温和的 PD 增益保留，启动定位去除了前视距离带来的纯航向误差，避免把摆歪误算为整车横向平移。
首帧响应仿真中，左右 5mm 偏差在收到有效观测后 30ms 内产生小幅纠偏；该数字不包含相机成像、传输和扫描等待时间。

可见道路由视觉居中；原有地图/惯导弯道接管、S 弯反向、出口方向约束及有限丢失恢复逻辑保留。
绕障完成后交回视觉居中，再继续原导航终点。两侧方向的物理车身模型测试均观察到视觉重新接管。

## 绕障补充条件

按本次要求增加仅单侧前方激光发现障碍物的分支：在开始绕障时锁存分类，障碍物一侧的侧方测距连续两次新鲜有效观测不大于 25mm 后，可进入车身回正。
该分支还要求离开初始方向的航向已稳定、横向位移至少 25mm、前方测距证据有效；前后通道无需同一时钟更新。
超时、故障和重复保持值不能凑足确认。双前方激光进入的绕障不会因后来一束清空而切换到此捷径。
原双前方激光的侧方极小值/回升逻辑及归轨流程保留。代码中的常规 45mm 是横向位移条件，不是侧方测距阈值。

## 斑马线

原检测链路仍在：图像二值化/道路扫描 → 斑马线检测 → 导航速度仲裁。
导航检测到斑马线时速度上限为 100mm/s；在确认画面已清空并完成相机至车尾 305mm 的通过距离后恢复设定巡航速度。
减速、保持和恢复均已仿真核对。实际识别效果仍受曝光、阈值、光照及条纹尺寸影响。

## 验证与交付

完成 74 次仿真调用，包含套件之间的重复回归，不等于 74 种独立道路场景：

- 22 次导航响应/直角弯/终点回归。
- 21 次实际地图路线闭环：外圈、内侧路口、S 弯正反向、边线缺失、低速及启动观测间隙。
- 3 次惯性/测距后续与完整物理绕障回归（包括更新测试台后的默认绕障复验）；其中绕障采用独立激光束及 320×200mm 车身扫掠碰撞模型。
- 3 次图像、车辆及路线剖面补充回归。
- 7 次首帧响应、单前方激光、双前方激光、视觉/惯导接管、绕障后视觉归轨及车辆/剖面回归。
- 14 次图像、斑马线、视觉控制、视觉菜单、终点及刹车测试；另 2 次字库/显示时序测试。
- 2 次 Gowin 综合后仿真：视觉/地图交接与图像模块。图像模块的 384000 个预览像素和 6 帧控制输出与 RTL 一致。

21 次地图仿真的最大车头中心误差范围为 {min(c['max_nose_error_mm'] for c in audit['actual_route_runs']):.2f}–{max(c['max_nose_error_mm'] for c in audit['actual_route_runs']):.2f}mm。
这些数据来自独立几何/车辆模型和合成图像，不是用户照片回放，也不构成所有道路的整车边界间隙保证。
各日志、哈希、冻结源文件、资源及引脚核对详见 [JSON 验收记录](navigation_ipm_20261026_validation.json)。

整机综合与布线通过，Setup/Hold 违规端点 0/0，最小余量 {timing['Setup']:.3f}/{timing['Hold']:.3f}ns。
普通综合与布线耗时 {audit['build']['seconds']:.3f} 秒。此前实测增量匹配更慢，因此采用较快的普通实现，没有声称保留未修改模块的物理布线。
已有综合警告仍在；定位仍依赖编码器/IMU，长期漂移与道路盲区没有被消除。

旧下载文件保留在 `impl/baseline_ipm_20261025/impl/pnr/ov5640.fs`。
该目录的源码快照包含较早的 IPM 试改，不能把它当成完整干净的上一验收源码；上一验收源文件哈希另见 `impl/navigation_robust_20261025_inputs.json`。

当前位流 SHA-256：`{audit['sha256']}`
"""
(ROOT / f'docs/{LABEL}.md').write_text(details, encoding='utf-8')
readme = ROOT / 'README.md'
original = readme.read_text(encoding='utf-8')
marker = '**2026-10-26 逆透视循迹更新：**'
paragraph = (marker + ' 已按最新相机位置改为逐行地面逆投影，改善首帧小偏差响应，补充单前方激光的 25mm 侧方回正分支，保留绕障后视觉归轨与斑马线减速。'
             '74 次仿真调用及整机综合、布线和时序核验通过。相机内参仍为估计值；未烧录、未实车验证。'
             '详见 [本轮说明](docs/navigation_ipm_20261026.md) 和 [验收记录](docs/navigation_ipm_20261026_validation.json)。')
if marker not in original:
    title, rest = original.split('\n', 1)
    original = title + '\n\n' + paragraph + '\n' + rest
else:
    original = '\n'.join(paragraph if line.startswith(marker) else line for line in original.splitlines()) + '\n'
readme.write_text(original, encoding='utf-8')
print('Published IPM documentation for audited bitstream', audit['sha256'])
