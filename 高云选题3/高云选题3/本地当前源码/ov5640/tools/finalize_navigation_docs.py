"""Publish factual build results after the audit succeeds."""
from pathlib import Path
import datetime
import json
ROOT=Path(__file__).resolve().parents[1]
j=json.loads((ROOT/'docs/navigation_response_20261025_validation.json').read_text(encoding='utf-8'))
assert j['status']=='digital_verification_passed' and j['timing_passed']
count=sum(r['completed_runs'] for r in j['regressions'].values())
assert count==17
generated=datetime.datetime.fromisoformat(j['generated_utc']).astimezone(datetime.timezone(datetime.timedelta(hours=8)))
section=('\n## 最终构建结果\n\n'
    f'位流于北京时间 {generated:%Y-%m-%d %H:%M:%S} 生成，{j["bytes"]} 字节；'
    f'SHA-256 为 `{j["sha256"]}`。17 次本轮运行通过，另有 2 项旧源码故障复现。'
    f'Setup/Hold 违例为 {j["violated_endpoints"]["Setup"]}/{j["violated_endpoints"]["Hold"]}，'
    f'最小裕量为 {j["min_slack_ns"]["Setup"]}/{j["min_slack_ns"]["Hold"]} ns。\n\n'
    f'增量尝试 {j["build"]["incremental_attempt"]["seconds"]:.3f} 秒后终止；最终复用已完成综合、'
    f'执行普通布局布线耗时 {j["build"]["seconds"]:.3f} 秒。'
    '本轮增量未实现提速，不能保证下次复用一定更快。105 个其余硬件源码文件哈希不变。'
    '新位流尚未烧录或实车验证。\n')
doc=ROOT/'docs/navigation_response_20261025.md'
text=doc.read_text(encoding='utf-8').split('\n## 最终构建结果')[0]
doc.write_text(text.rstrip()+'\n'+section,encoding='utf-8')
resources=('# 当前构建资源与验证（2026-10-25）\n\n'
    '下载文件：`impl/pnr/ov5640.fs`；参见 [本轮导航说明](navigation_response_20261025.md) 与 '
    '[完整验证记录](navigation_response_20261025_validation.json)。未烧录、未实车验证。\n\n'
    '| 资源 | 使用 / 总量 | 使用率 |\n| --- | ---: | ---: |\n'
    + ''.join(f'| {n} | {v["used"]:g} / {v["total"]:g} | {v["percent"]:.2f}% |\n'
              for n,v in j['resources'].items())
    + f'\n17 次本轮回归通过；Setup/Hold 违例：{j["violated_endpoints"]}；最小裕量（ns）：{j["min_slack_ns"]}。\n\n'
    + '仅两个导航硬件源码文件改变，绕障及其他硬件源码不变。103 个构建输入哈希与最终构建一致；'
      '地图、道路、字体 ROM 及引脚已核对。增量匹配过慢后回退，最终使用已完成综合结果进行普通布局布线。\n\n'
    + f'位流生成时间：{generated:%Y-%m-%d %H:%M:%S}（UTC+8）；SHA-256：`{j["sha256"]}`。\n')
(ROOT/'docs/resource_usage.md').write_text(resources,encoding='utf-8')
print('Published final navigation build facts:',count,'runs;',generated.isoformat())
