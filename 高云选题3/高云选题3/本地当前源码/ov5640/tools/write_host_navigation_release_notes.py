"""Publish measured build facts after the release audit has passed."""
from pathlib import Path
import datetime,json

ROOT=Path(__file__).resolve().parents[1]
a=json.loads((ROOT/'docs/host_navigation_20261027_validation.json').read_text(encoding='utf-8'))
assert a['status']=='digital_verification_passed'
local=datetime.datetime.fromisoformat(a['generated_utc']).astimezone(datetime.timezone(datetime.timedelta(hours=8)))
generated=local.strftime('%Y-%m-%d %H:%M:%S')+'（UTC+8）'
seconds=a['build']['seconds'];runs=a['regression_completed_runs'];slack=a['min_slack_ns']
text=f'''# 路口提前转向修正与上下位机通信框架

本轮已生成新的 `impl/pnr/ov5640.fs`，时间 **{generated}**，与上一版位流不同。
SHA-256：`{a['sha256']}`。
整机综合、布局布线及数字验收通过；未烧录、未进行实车验证。

## 框选路口的提前打角

这次修正针对中央左侧直路与 S 形道路连接处的控制时机，地图路径几何保持原样。

- 上一弯道退出后，视觉回正状态可能尚未释放，而前视追踪点已经读到下一弯道。旧逻辑会跟随下一目标提前打角；现在在下一个物理入口之前保持来向航向，到达入口才允许下一次转向。
- 原入口判断有 8 mm 提前量，本轮改为 0 mm。路线缓存地址可以提前读取，不能再单凭读取到弯道点就认为车头已到入口。
- 靠近弯道起步时，相机近远采样带可能都落在前方曲线上，曲线斜率会被误当成车身初始偏航。本轮在规划预告入口小于 240 mm 且当前为直道时，抑制这次有歧义的初始航向/横移重定位，并在静止阶段也约束提前预置转角。普通直道起步的视觉校正保留。

0 mm 是逻辑入口提前量，实际转向时机仍受里程误差、100 Hz 控制周期、舵机响应和道路识别影响。短直道前方已被弯道占满画面时，本轮优先使用地图来向，不能保证仅凭该画面恢复任意摆放偏角。

## 新增通信模式

入口：**设置 → 模式选择 → 通信模式**。它与手动模式、遥控模式互斥，复位默认手动且停止。

| 信号 | FPGA 引脚 | 参数 |
| --- | --- | --- |
| TX | E14 | 115200、8N1、3.3V TTL |
| RX | E13 | 115200、8N1、3.3V TTL |

TX 接上位机 RX，RX 接上位机 TX，双方共地。原 M15 调试输出保留。

已接入启动直行、启动已规划导航、设置/降低速度、停车、查询、心跳、继续导航，以及导航完成、故障和掉线事件。导航暂停保留位置、路径进度与已确认转弯；继续不会重新建立起点。进入导航页面不会自动启动，需收到上位机 START。

通信速度由串口命令设置，导航仍遵守原巡航、弯道及斑马线限速。两秒无有效通信会停车，恢复通信后需明确启动/继续；示例客户端每 500 ms 发心跳。

这是一套可扩展框架：已完成的导航自动报告 F0，预留通用业务完成事件接口，具体未来业务仍需接入。当前逐条请求等待回复，只有一个待发事件槽，没有应用层重传或去重。

详细格式、字节示例、命令返回码和操作流程见 [通信协议](host_protocol_20261027.md)。上位机示例为 [host_serial_client.py](../tools/host_serial_client.py)，需要安装 pyserial，COM 口按实际修改。

```powershell
python tools/host_serial_client.py --port COM8 speed 100
python tools/host_serial_client.py --port COM8 start-drive --task 1 --watch
python tools/host_serial_client.py --port COM8 stop
```

## 验证与保留范围

最终源码共 {runs} 次成功数字测试调用，含套件间重复用例；另一次旧逻辑预期失败复现，不计入成功数。

- 框选区域 12 组测试，覆盖四个进出方向、80/160 mm/s、不同相机到帧相位和地面投影采样。
- 25 组实际道路图路线、8 组相机相位测试、22 组转弯/起步/终点测试，以及视觉控制权、视频流、整车运动、路线预处理和斑马线回归。
- 独立车头位置检查验证左右转在入口前超过 2 mm 时不能放行；该 2 mm 是仿真检查容差，不是新增提前量。
- 串口 RX/TX 位级测试、整车控制源仲裁/速度斜坡/暂停恢复测试，及综合后 UART、导航下一入口和视觉交接测试。
- 菜单触摸、三模式互斥与电机响应测试；已渲染检查 [通信模式界面](settings_host.png)。Python 客户端固定报文、分块接收、坏帧恢复和等待 ACK 时的异步事件保留测试通过。
- 实际封装引脚与地图、道路、字库 ROM 初始化均核验通过。

整机构建耗时 {seconds:.3f} 秒，使用普通布局布线。以往增量匹配在本工程更慢，因此沿用已记录的普通流程；没有冻结其他模块物理布线。本轮中途停止的一次构建已归档，最终位流来自后续完整构建。

仅 14 个硬件/工程输入变更，其中包括新增 UART、控制接入、菜单及字库、引脚约束和两处导航模块。本轮有 {a['unchanged_hardware_source_count']} 个 `src` 输入哈希保留。绕障算法、传感器驱动、电机控制、舵机标定、图像提取、斑马线识别及路线几何源码保持原样；共享整车模块增加通信接入和导航暂停。

Setup/Hold 违规端点 0/0，最小余量 {slack['Setup']:.3f}/{slack['Hold']:.3f} ns，Recovery/Removal {slack['Recovery']:.3f}/{slack['Removal']:.3f} ns。资源见 [资源报告](resource_usage.md)，机器可核验结果见 [验收记录](host_navigation_20261027_validation.json)。

仿真中的偏差是车头中心误差，不等于全车扫掠边界；合成道路和相机无法覆盖所有实景，相机内参仍含估计值，既有综合警告保留。未连接实车、未烧录，实车通过情况仍需验证。

本轮前的完整回退快照：`impl/baseline_host_navigation_20261027/`。
'''
(ROOT/'docs/host_navigation_20261027.md').write_text(text,encoding='utf-8')
rows=[]
for name,r in a['resources'].items():
 rows.append(f"| {name} | {r['used']:g} / {r['total']:g} | {r['percent']:.2f}% | {r['delta_from_previous']:+g} |")
resources=f'''# 当前构建资源与时序（2026-10-27）

对应位流：`impl/pnr/ov5640.fs`，生成于 **{generated}**。
详见 [本轮说明](host_navigation_20261027.md) 与 [完整验收记录](host_navigation_20261027_validation.json)。

| 资源 | 使用 / 总量 | 占用率 | 相对上一发布版变化 |
| --- | ---: | ---: | ---: |
'''+ '\n'.join(rows)+f'''

Setup/Hold 违规端点：0/0。
最小 Setup/Hold 余量：{slack['Setup']:.3f}/{slack['Hold']:.3f} ns；Recovery/Removal：{slack['Recovery']:.3f}/{slack['Removal']:.3f} ns。
最终综合与普通布局布线耗时 {seconds:.3f} 秒；未冻结其他模块物理布线。

最终源码 {runs} 次成功数字测试调用，另完成旧逻辑的预期故障复现。实际引脚及地图、道路、字库 ROM 初始化核验通过。新增 E14 TX / E13 RX，115200、8N1。

本轮 14 个硬件/工程输入变更，{a['unchanged_hardware_source_count']} 个 src 输入哈希保留。未烧录、未实车验证；既有综合警告及相机估计内参保留。
完整回退快照：`impl/baseline_host_navigation_20261027/`。

SHA-256：`{a['sha256']}`
'''
(ROOT/'docs/resource_usage.md').write_text(resources,encoding='utf-8')
readme=ROOT/'README.md';old=readme.read_text(encoding='utf-8')
marker='**2026-10-27 路口时机与通信模式：**'
entry=f'''{marker} 修复上一弯道回正期间追踪下一弯道、入口提前量和靠近弯道起步预置转角导致的早转；设置新增通信模式，独立 UART 为 E14 TX / E13 RX、115200 8N1，已接入启动、减速、停车、导航暂停/继续与完成上报。{runs} 次数字测试及整机综合布线通过，Setup/Hold 违规 0/0。**新位流生成于 {generated}**，SHA-256 前缀 `{a['sha256'][:16]}`。绕障等已正常功能源码保留；使用普通布线，没有冻结其他模块物理布线。未烧录、未实车验证。详见 [本轮说明](docs/host_navigation_20261027.md)、[通信协议](docs/host_protocol_20261027.md)、[验收记录](docs/host_navigation_20261027_validation.json) 和 [资源报告](docs/resource_usage.md)。以下为历史记录。

'''
assert marker not in old,'Already added this release'
at=old.index('\n\n')+2
readme.write_text(old[:at]+entry+old[at:],encoding='utf-8')
print('Release notes and README updated from verified build:',generated,a['sha256'])
