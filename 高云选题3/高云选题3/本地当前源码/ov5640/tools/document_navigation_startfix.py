"""Write release notes from the completed audit, never from an old bitstream."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[1];LABEL='navigation_startfix_20261027'
r=json.loads((ROOT/f'docs/{LABEL}_validation.json').read_text())
assert r['status']=='digital_verification_passed'
routes=r['regressions']['routes'];reported=[c for c in routes if c['name'].startswith('reported_')]
assert len(reported)==3
worst=max(c['max_nose_error_mm'] for c in reported)
old=reported[0]['previous_release_max_nose_error_mm']
note=f'''# 自主导航启动等待与路口反向修复（2026-10-27）

只修改 `vehicle_motion.v`、`vehicle_vision_fusion.v` 两个硬件源码。其余硬件输入与本轮开始时的快照逐文件核对一致，绕障、测距、舵机标定、地图规划和上位机通信代码保留。

## 复现与修改

1. 静止时，道路误差相邻帧变化超过 12 mm / 8 mm 就会清零三帧启动计数；一帧无效图像也会清零。新的测试向公开端口输入正常的 ±7 mm / ±5 mm 抖动，旧版一直不能起步。现在累计新鲜、完整的有效道路帧，短暂无效帧保留进度；200 ms 没有有效道路时仍撤销启动许可，不能靠超时盲开。
2. 起步预置转角的 120 ms 等待不再因视觉指令普通变化重新计时。短暂未识别道路时暂停计数并保留已预置舵角，过期数据仍复位。起步仍要求当前视觉可用。补测复现了连续三帧无效后舵角被清零、恢复后驱动早于舵机跟上的问题，因此中止第一次构建并加入该修复；对应源码、网表和失败记录保存在 `impl/navigation_startfix_20261027_attempt1/`。
3. 旧版直角分支方向保护在偏转约 10° 后就消失；转到 30° 时输入相反的追踪命令，测试可复现转向反转。现在在接收入弯事件时锁定是否为约 90° 转弯，直到距离出口航向约 15° 内才允许正常反向微调；短角度 S 弯保留原有追踪控制，不强制转够 90°。

屏幕 `V:WAIT` 表示等待可用道路，`V:TRACK` 表示视觉循迹。没有靠修改文字掩盖启动或转向问题。

## 验证及限制

- 两个故障均保存了旧版失败、新版通过的测试记录。
- 8 项针对性 RTL 测试、5 项 Gowin 综合电路网表测试、8 组路线模型测试通过；包含整机运动模块在连续三帧无效后仍预置好舵角才驱动的综合电路测试。路线测试包括歪放起步、摄像头抖动/丢帧、正反 S 弯、起点前后各偏 60 mm，以及本轮照片对应路线。
- 本轮照片路线按起点 `(327,171)`，途经 `(221,293)`、`(25,241)`、`(175,165)`、`(275,127)`，终点 `(327,219)` 重建，281 个采样点、约 953 cm。圈出的右上方弯道在模型中按左转通过。
- **该完整路线仍未通过原有 50 mm 车头中心误差限制**：上一版最坏误差 {old:.3f} mm，本轮三种起点位置最坏 {worst:.3f} mm。完整路线测试在这一项只验证相对上一版未退步，单独道路测试仍保留 50 mm 限制。这不代表整段车身、车轮都在道路内，也不能据此宣称全部导航问题已经解决。
- 仅完成数字验证，未烧录、未实车验证。真实路口的所有图像歧义无法由合成图像覆盖。

![完整路线模型对照；轨迹为车头中心，不能证明车身净空](navigation_startfix_reported_trace.png)

## 下载文件

`{r['release_bitstream']}`

SHA-256：`{r['sha256']}`

Setup/Hold 违规：{r['violated_endpoints']['Setup']}/{r['violated_endpoints']['Hold']}。最小时序裕量、资源、引脚与 ROM 核对结果见 [验收记录]({LABEL}_validation.json)。

本轮使用工程已有的普通实现方式；之前增量匹配较慢，工程中已有 `incremental_pnr_slow.flag`，所以没有冻结其他模块的物理布局布线。最终成功构建耗时 {r['build']['seconds']:.1f} 秒，含第一次中止构建共 {r['build']['total_attempt_seconds']:.1f} 秒。

回退文件：`{r['rollback_bitstream']}`，SHA-256：`{r['rollback_sha256']}`。
'''
(ROOT/f'docs/{LABEL}.md').write_text(note,encoding='utf-8')
readme=ROOT/'README.md';text=readme.read_text(encoding='utf-8-sig');head,rest=text.split('\n',1)
entry=f'''\n**2026-10-27 导航启动等待与路口反向修复：** 修复图像抖动/单帧无效反复清零启动进度，以及约 90° 弯道尚未转完就被相反追踪命令反转。只改两个导航硬件源码。新位流 `{r['release_bitstream']}`，SHA-256 前缀 `{r['sha256'][:16]}`，Setup/Hold 违规 0/0。未烧录、未实车验证；完整新路线的 S 弯最坏车头误差仍约 {worst/10:.1f} cm，尚不满足原有 5 cm 限制。详见 [本轮说明](docs/{LABEL}.md) 和 [验收记录](docs/{LABEL}_validation.json)。以下为历史记录。\n'''
assert LABEL not in rest,'Already documented'
readme.write_text(head+'\n'+entry+rest,encoding='utf-8')
print('Release documentation updated')
