"""Render the user-facing build details from the verified current audit."""
from pathlib import Path
import datetime,json
ROOT=Path(__file__).resolve().parents[1]
a=json.loads((ROOT/'docs/navigation_entry_20261026_validation.json').read_text(encoding='utf-8'))
stamp=datetime.datetime.fromisoformat(a['generated_utc']).astimezone(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y-%m-%d %H:%M:%S')
lines=['# 当前构建资源与时序（2026-10-26）','',
       f'对应位流：`{a["bitstream"]}`，生成于 {stamp}（UTC+8）。',
       '详见 [导航入口修复说明](navigation_entry_20261026.md) 和 [完整验收记录](navigation_entry_20261026_validation.json)。','',
       '| 资源 | 使用 / 总量 | 占用率 | 相对上次验收版变化 |',
       '| --- | ---: | ---: | ---: |']
for name,r in a['resources'].items():
    lines.append(f'| {name} | {r["used"]:g} / {r["total"]:g} | {r["percent"]:.2f}% | {r["delta_from_previous"]:+g} |')
s=a['min_slack_ns'];v=a['violated_endpoints']
lines+=['',f'Setup/Hold 违规端点：{v["Setup"]}/{v["Hold"]}。',
        f'最小 Setup/Hold 余量：{s["Setup"]:.3f}/{s["Hold"]:.3f}ns；Recovery/Removal：{s["Recovery"]:.3f}/{s["Removal"]:.3f}ns。',
        f'最终综合与布局布线耗时 {a["build"]["seconds"]} 秒。此前实测增量匹配更慢，因此沿用普通实现；其他模块的物理布线没有锁定。','',
        f'本次完成 {a["regression_completed_runs"]} 次仿真调用，包含套件间重复测试；其中 25 次为实际地图路线闭环，4 次包含有限视野地面投影，9 次为综合后电路仿真。',
        '三类报告故障均在旧源码上复现并在修改后通过；路线入口 ROM 的距离、方向和出口航向也经过综合后核验。',
        '引脚及地图、道路和字库 ROM 初始化已核对。','',
        f'本次只修改四个导航相关硬件文件，其余 {a["unchanged_hardware_source_count"]} 个硬件源码文件保留原哈希。',
        '绕障、IMU/激光驱动、电机速度环、舵机标定及限幅、斑马线、图像提取与地图几何源码保持修改前状态。','',
        '摄像头内参仍含估计值；未烧录、未实车验证。模型车头中心误差不代表全车包络净空。已有综合警告保留。',
        '完整回退快照位于 `impl/baseline_navigation_entry_20261026/`。','',
        f'SHA-256：`{a["sha256"]}`','']
(ROOT/'docs/resource_usage.md').write_text('\n'.join(lines),encoding='utf-8')
p=ROOT/'README.md';s=p.read_text(encoding='utf-8-sig');head,rest=s.split('\n',1)
entry=(f'**2026-10-26 弯道入口与方向修复：** 以路线预告和固定入口判断转向时机，入口前保持来向，入口后沿地图圆弧跟踪；追踪点落到车后不能反转已确认的转弯方向；出弯恢复视觉并补偿残留横移。'
       f'本次只改四个导航相关硬件文件，其余 {a["unchanged_hardware_source_count"]} 个源码哈希保留。'
       f'{a["regression_completed_runs"]} 次仿真调用及整机综合布线通过，Setup/Hold 违规 {v["Setup"]}/{v["Hold"]}。'
       '下载文件为 `impl/pnr/ov5640.fs`；未烧录、未实车验证。'
       '详见 [本轮说明](docs/navigation_entry_20261026.md)、[验收记录](docs/navigation_entry_20261026_validation.json) 和 [资源报告](docs/resource_usage.md)。以下为历史记录。')
if '**2026-10-26 弯道入口与方向修复：**' not in s:
    p.write_text(head+'\n\n'+entry+'\n'+rest,encoding='utf-8')
print('Updated README and resource report from verified audit')
