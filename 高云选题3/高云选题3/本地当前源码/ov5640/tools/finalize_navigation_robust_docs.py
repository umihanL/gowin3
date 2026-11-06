"""Publish documentation only after bitstream audit succeeds."""
from pathlib import Path
import datetime,hashlib,json
ROOT=Path(__file__).resolve().parents[1]
audit=json.loads((ROOT/'docs/navigation_robust_20261025_validation.json').read_text(encoding='utf-8'))
assert audit['status']=='digital_verification_passed' and audit['regression_completed_runs']==53
assert hashlib.sha256((ROOT/audit['bitstream']).read_bytes()).hexdigest()==audit['sha256']
generated=datetime.datetime.fromisoformat(audit['generated_utc']).astimezone(datetime.timezone(datetime.timedelta(hours=8)))
rows='\n'.join(f"| {name} | {int(v['used'])} / {int(v['total'])} | {v['percent']:.2f}% |" for name,v in audit['resources'].items())
text=f"""# 当前构建资源与时序（2026-10-25）

对应位流：`impl/pnr/ov5640.fs`。详见 [自主导航弯道接管修复](navigation_robust_20261025.md) 和 [验证记录](navigation_robust_20261025_validation.json)。

| 资源 | 使用 / 总量 | 占用率 |
| --- | ---: | ---: |
{rows}

位流生成于 {generated:%Y-%m-%d %H:%M:%S}（UTC+8），大小 {audit['bytes']} 字节。53 次完整仿真运行通过，其中 21 次为实际道路路线闭环测试。Setup/Hold 违例为 {audit['violated_endpoints']['Setup']}/{audit['violated_endpoints']['Hold']}，最小时序余量为 {audit['min_slack_ns']['Setup']:.3f}/{audit['min_slack_ns']['Hold']:.3f} ns。最终一次普通综合与布局布线耗时 {audit['build']['seconds']:.3f} 秒。

本轮修改三个自主导航 RTL 文件，其余 {audit['unchanged_hardware_source_count']} 个硬件源码文件与本轮备份一致。绕障、传感器驱动、舵机标定和速度环源码保留。已核对引脚及地图、道路、字体 ROM。首轮时序失败记录单独保存，最终构建验收通过。未烧录，未实车验证。

SHA-256：`{audit['sha256']}`

"""
(ROOT/'docs/resource_usage.md').write_text(text,encoding='utf-8')
readme=ROOT/'README.md';original=readme.read_text(encoding='utf-8');marker='**2026-10-25 自主导航弯道接管修复：**'
paragraph=marker+' 修复中心无效但仍残留边线时不接管、路口视觉抵消规划转弯及弯中误判偏离；S 弯按实际曲线连续转向，出口恢复可靠视觉后交回居中控制。硬件源码仅修改三个导航模块，已认可的绕障及其他模块源码保留。53 次数字仿真通过，最终 Setup/Hold 违例 0/0。详见 [本轮说明](docs/navigation_robust_20261025.md) 与 [验证记录](docs/navigation_robust_20261025_validation.json)。未烧录，未实车验证。'
if marker not in original:
 title,rest=original.split('\n',1);original=title+'\n\n'+paragraph+'\n'+rest
else:
 lines=original.splitlines();lines=[paragraph if line.startswith(marker) else line for line in lines];original='\n'.join(lines)+'\n'
readme.write_text(original,encoding='utf-8')
print('Published README and resource report for audited bitstream',audit['sha256'])
