"""Document the verified final navigation follow-up build."""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'navigation_followup_20261026'
audit = json.loads((ROOT / f'docs/{LABEL}_validation.json').read_text(encoding='utf-8'))
assert audit['status'] == 'digital_verification_passed'
assert audit['regression_completed_runs'] == 72
assert hashlib.sha256((ROOT / audit['bitstream']).read_bytes()).hexdigest() == audit['sha256']
assert audit['sha256'] not in {'c53a514240ecfa08383a14fd35187983af91eb15a819b6f061c22941a4328b90', '225803b02809171990a03901a2ed3d3f4124df9c8628750bc68adcd65cfd8681'}
stamp = datetime.fromisoformat(audit['generated_utc']).astimezone(timezone(timedelta(hours=8))).strftime('%Y-%m-%d %H:%M:%S')
digest = audit['sha256']
seconds = audit['build']['seconds']
unchanged = audit['unchanged_hardware_source_count']

detail = f'''# 导航启动、S 弯和路口进入修正

最终新位流：`impl/pnr/ov5640.fs`，生成时间 **{stamp}（UTC+8）**，大小 {audit['bytes']:,} 字节。它包含最后一次物理入口修正，既不是原发布版，也不是本轮中途生成的仅索引门控版本。

SHA-256：`{digest}`。下载清单为 `impl/pnr/ov5640_download.json`。

## 修正行为

1. **启动转向**：静止时确认连续三帧新鲜、有效且一致的道路观测。确认前保持中位；确认后预置视觉纠偏转角，转角稳定 120ms 后允许起步。取消起步时另一套路径转角与视觉输出竞争。摆歪时仍保留视觉纠偏。
2. **S 弯时机和方向**：锁存下一弯的方向、入口和出口航向，新弯立即屏蔽上一弯的放行状态。路线索引只用于取得入口坐标，不再当成车头已经到达入口的依据。坐标取到后固定物理入口截面，使用 8mm 提前量判断到达，避免目标点提前切换约 60mm 直接放行转向。允许前方 S 弯目标所需的反向修正；目标落到车后时仍防止追旧点造成错误反转。
3. **规划路口进入**：到达锁定入口后，将规划转弯作为一次明确事件交给融合控制器；路口画面仍像直道时也可执行指定分支。已确认弯道保留 IMU 出口航向闭环，重新识别出口道路后交回视觉居中。

入口缓存由每点 53 位改为 38 位，保存索引并复用既有路线点读口取得坐标，避免重复保存整套入口坐标。图像处理源码未修改。

## 变更范围

与构建冻结清单逐项核对，只修改以下四个导航相关硬件文件：

- `src/vehicle/vehicle_motion.v`
- `src/vehicle/vehicle_route_profile.v`
- `src/vehicle/vehicle_subsystem.v`
- `src/vehicle/vehicle_vision_fusion.v`

其余 {unchanged} 个硬件源码文件哈希保持原样。绕障、激光/IMU 驱动、电机速度环、舵机标定与 40～150 限幅、图像提取、斑马线识别和地图几何源码均未修改。

## 验证

最终源码共通过 72 次仿真调用；次数包含不同套件间重复场景，不等同于 72 个独立实车场景。

| 套件 | 通过次数 |
| --- | ---: |
| 转弯、入口与融合回归 | 22 |
| 实际地图路线闭环 | 25 |
| 相机帧相位及初始姿态变化 | 8 |
| 定向测试，含视频像素到融合控制 | 6 |
| 整车、路线预处理、斑马线回归 | 3 |
| 预处理及融合交接的综合后仿真 | 2 |
| 三类故障的综合后仿真 | 3 |
| 三类原反馈故障修复后仿真 | 3 |

另外三次旧源码故障复现按预期失败，未计入 72 次通过。实际地图路线与相机相位测试共 33 次，全部满足车头中心误差不超过 50mm 的模型断言；入口测试另有独立位置断言，检查约 8mm 的入口提前量不会变成约 60mm 的目标前视提前量。日志、哈希和依赖新鲜度由 [验收脚本](../tools/verify_navigation_followup_build.py) 核对，详见 [验收记录]({LABEL}_validation.json)。

最终整机综合、布局布线和位流生成成功，耗时 {seconds:.3f} 秒。Setup/Hold 违规端点为 0/0；最小余量分别为 0.047ns / 0.040ns，Recovery/Removal 为 2.786ns / 0.566ns。引脚及地图、道路、字库 ROM 初始化已核对。资源见 [资源报告](resource_usage.md)。

## 布线方式和回退

本轮尝试过从旧 `.pr` 增量实现；补入物理入口修正后，最终交付采用普通综合和布局布线。此前增量匹配超过 920 秒，最终普通实现耗时约 13 分 31 秒。**本次没有锁定其他模块的物理位置与布线**，源码未变不代表物理布线保留。

原发布版完整回退快照位于 `impl/baseline_navigation_followup_20261026/`，含源码、位流、布线检查点与哈希清单；验收脚本已核对快照未被改变。中途版本另存于 `impl/navigation_followup_full_pnr_backup_20261026/`，不作为交付版本。

## 实际验证边界

本次未烧录、未实车测试。模型无法覆盖全部真实光照、边线和路口画面，相机内参仍含估计值。模型车头中心误差不能替代全车转弯包络和碰撞净空核验。既有综合警告保留；数字检查通过不等于实车效果已确认。
'''
(ROOT / f'docs/{LABEL}.md').write_text(detail, encoding='utf-8')
rows = [f"| {name} | {r['used']:g} / {r['total']:g} | {r['percent']:.2f}% | {r['delta_from_previous']:+g} |" for name, r in audit['resources'].items()]
resource = f'''# 当前构建资源与时序（2026-10-27）

对应位流：`impl/pnr/ov5640.fs`，生成于 {stamp}（UTC+8）。
详见 [导航修正说明]({LABEL}.md) 和 [完整验收记录]({LABEL}_validation.json)。

| 资源 | 使用 / 总量 | 占用率 | 相对原发布版变化 |
| --- | ---: | ---: | ---: |
''' + '\n'.join(rows) + f'''

Setup/Hold 违规端点：0/0。
最小 Setup/Hold 余量：0.047/0.040ns；Recovery/Removal：2.786/0.566ns。
最终综合与布局布线耗时 {seconds:.3f} 秒，采用普通实现；其他模块物理布线没有锁定。

最终源码通过 72 次仿真调用，包含套件间重复测试；另在旧源码上完成三次预期故障复现。引脚和地图、道路、字库 ROM 初始化核验通过。

只修改四个导航相关硬件文件，其余 {unchanged} 个源码文件哈希保留。相机内参仍含估计值；既有综合警告保留。未烧录、未实车验证。
原发布版完整回退快照：`impl/baseline_navigation_followup_20261026/`。

SHA-256：`{digest}`
'''
(ROOT / 'docs/resource_usage.md').write_text(resource, encoding='utf-8')
readme = ROOT / 'README.md'
text = readme.read_text(encoding='utf-8-sig')
title, rest = text.split('\n', 1)
marker = '**2026-10-27 导航启动、S 弯与路口修正：**'
entry = f'{marker} 起步前确认三帧稳定道路并预置视觉转角；按锁定物理入口放行转弯，避免前视目标索引提前推进造成早转；允许前方 S 弯目标的必要修正，到达规划路口后执行指定分支。只修改四个导航相关硬件文件，其余 {unchanged} 个源码哈希保留。72 次仿真及最终整机综合布线通过，Setup/Hold 违规 0/0。**新位流生成于 {stamp}（UTC+8）**，文件为 `impl/pnr/ov5640.fs`，SHA-256 前缀 `{digest[:16]}`。本次使用普通布局布线，未冻结其他模块物理布线；未烧录、未实车验证。详见 [本轮说明](docs/{LABEL}.md)、[验收记录](docs/{LABEL}_validation.json) 和 [资源报告](docs/resource_usage.md)。以下为历史记录。'
if marker not in text:
    readme.write_text(title + '\n\n' + entry + '\n\n' + rest.lstrip('\n'), encoding='utf-8')
print(json.dumps(dict(document=f'docs/{LABEL}.md', generated_local=stamp, sha256=digest), ensure_ascii=False))
