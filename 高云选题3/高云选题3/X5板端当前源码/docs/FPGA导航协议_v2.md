# FPGA 自主导航与 X5 语义目标协议

更新日期：2026-10-11。此文替代旧“X5 规划路线、FPGA 执行路段”的实车分工。旧电脑仿真和旧导航代码保留作学习材料，不应与新节点同时启动。

## 分工

- FPGA：存地图、计算路线、编码器与陀螺仪定位、转弯和速度控制、障碍物处理、停车执行。无需视觉循迹。
- X5：把“去三号楼”解析为楼号 3；通过 UART 发送目标、暂停、恢复、取消。识别斑马线和红绿灯、楼牌、火焰、人物与消防员，管理云台、灭火、语音与问答。
- X5 不发送转向轨迹和路段距离。新 `fpga_navigation` 节点没有道路图、Dijkstra、惯导积分或中线计算。
- 目标楼号与实际 OCR 楼号分别保留。给 FPGA 发送三号楼，不代表已经通过摄像头确认在三号楼。

固定起点可以由 FPGA 初始化位置和朝向。陀螺仪负责角度变化，行驶距离仍应使用编码器等实际量测；仅积分陀螺仪不能得到可靠的平面位置。

## ROS 接口

新 launch 启动 `fpga_navigation`，参数 `buildings_config=config/buildings.json`；不要同时启动旧 `navigation`。语音解析、文字命令使用同一套请求。

`/navigation/request`，`std_msgs/String`，内容为 JSON：

```json
{
  "command_id": "voice-go-3-unique",
  "action": "navigate",
  "building_id": "C",
  "mode": "execute",
  "confirmed": true,
  "arrival_policy": "manual",
  "issued_at": 1791670000.0,
  "ttl_s": 5.0
}
```

时间戳须使用当前 X5 的 ROS 时钟，上面数字仅示例。`building_id` 兼容 A～M、01～13、数字和名单中的名称，输出保持原项目 A～M；UART 使用数字楼号。`arrival_policy` 为 `manual`（到达后等待语音）或 `auto`（到达后进入自动找火流程）。`mode=preview` 只展示目标，不写 UART；`execute` 会发目标，但 UART 本身 `dry_run=true` 时仍然没有任何硬件写入。

`action=pause/resume/cancel` 不需要楼号。每次请求使用新的 `command_id`；恢复只允许已被 FPGA 确认为暂停的同一目标。取消后原目标的迟到“已到达”不得触发灭火。

| 话题 | 发布者 → 订阅者 | 用途 |
|---|---|---|
| `/fpga/navigation_command` | fpga_navigation → fpga_bridge | 会话、目标、楼号、动作、到达策略 |
| `/fpga/navigation_status` | fpga_bridge → fpga_navigation | FPGA 实际返回的目标状态 |
| `/navigation/proposal` | fpga_navigation → mission | 是否收到匹配目标状态、是否到达、是否停稳、到达策略 |
| `/navigation/state` | fpga_navigation → 界面/记录 | 同上，加请求结果、UART dry-run 信息；route 始终为空 |
| `/control/motion_permission` | mission → fpga_bridge | 独立的停车/通行许可；唯一来源仍为 mission |

新节点不会发布 `/control/maneuver`。新任务栈还应设置 `fpga_bridge.maneuver_enabled=false`，禁止旧路段命令干扰。旧 launch 需要时可以保留兼容开关。

## UART 帧

沿用已有帧格式及校验，避免破坏已存在的解析器：

```text
A5 5A | version:u8=1 | type:u8 | sequence:u16 | payload_length:u16 | payload | crc16:u16
```

多字节整数为小端。CRC16/CCITT-FALSE，初始值 `FFFF`，多项式 `1021`，覆盖 version 到 payload，CRC 自身不参与。最大 payload 64 字节。本文 v2 表示新增语义导航功能，帧头 version 仍为 1。FPGA 固件必须同时实现这些新增消息；只部署 X5 代码不能让未实现协议的 FPGA 自动导航。

### 0x85 NAV_GOAL：X5 → FPGA，13 字节

Python 格式：`<IIHBH`。

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | uint32 | session_id | X5 导航节点启动时生成的非零会话 ID |
| 4 | uint32 | goal_id | 会话内递增的非零目标 ID |
| 8 | uint16 | building_number | 目的地楼号，当前 1～13 |
| 10 | uint8 | arrival_policy | 0=等待语音，1=自动找火；FPGA 仍只导航，到达策略由 X5 执行 |
| 11 | uint16 | accept_window_ms | 当前 1000；接收后必须在有效窗口内接受或拒绝，不能积压后执行 |

同一 `(session_id, goal_id)` 重发表示同一目标，不得重新初始化定位、重新开始路线或重新触发完成事件。更换目标必须使用新 goal_id。X5 最多重发 3 秒，每 0.4 秒一次，始终保持相同身份；无匹配状态则报超时，迟到状态不能自动恢复运行，需要新指令。

接受目标与允许运动必须分开：收到 NAV_GOAL 时，即使当前为 STOP，也先准备目标并回复 accepted；等合法 GO 许可后才运动。否则会出现 X5 等 accepted、FPGA 等 GO 的相互等待。

### 0x86 NAV_CONTROL：X5 → FPGA，11 字节

Python 格式：`<IIBH`。

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | uint32 | session_id | 必须匹配当前目标 |
| 4 | uint32 | goal_id | 必须匹配当前目标 |
| 8 | uint8 | action | 1=cancel，2=pause，3=resume |
| 9 | uint16 | accept_window_ms | 当前 1000 |

取消目标要记住已取消身份，忽略该身份后来到达的 NAV_GOAL 重传，避免串口队列把旧任务重新启动。暂停/恢复不创建新目标，重复请求只确认当前动作，不重复行驶。任何恢复仍需要 GO 许可。

### 0x21 NAV_STATUS：FPGA → X5，18 字节

Python 格式：`<IIIBBHH`。

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | uint32 | session_id | 原样回传目标会话 |
| 4 | uint32 | goal_id | 原样回传目标 ID |
| 8 | uint32 | status_sequence | 每次发送递增；不是外层 sequence；用于丢弃重复/乱序状态 |
| 12 | uint8 | state | 0=idle、1=accepted、2=navigating、3=paused、4=arrived、5=cancelled、6=failed |
| 13 | uint8 | flags | bit0=实际停稳，bit1=急停；其他位必须为 0 |
| 14 | uint16 | building_number | 与当前目标楼号一致 |
| 16 | uint16 | error_code | 0=无错误；非零原因由 FPGA 固件统一定义并记录 |

建议 10 Hz 周期发送，即使 paused/arrived/cancelled 也继续发送当前状态。X5 只接受匹配会话、目标、楼号和更新序号的消息。超过 0.6 秒没有新状态，导航证据失效。重复发同一序号不能刷新有效期。

ARRIVED 表示 FPGA 判定到达目标观察点。`vehicle_stopped` 单独表示实际停稳：到达但尚未停稳不得假装停稳。喷射控制依然使用实际停稳证据。状态和 CRC 校验用于身份关联与传输检错，并非网络加密认证。

FPGA 重启后不得凭旧目标自行续行；应保持停止，重新接收新目标和许可。若 status_sequence 重置，X5 会拒绝旧身份下的倒退序号，操作员应重新发导航命令。

## 斑马线、红绿灯与导航的关系

1. X5 检出斑马线，立即发 `PERMISSION(0x80, allowed=0)`。
2. X5 本地等待 2 秒后抬头搜索红绿灯；这一步不等待 FPGA 的停车 ACK。
3. 红灯、未知、冲突：持续 STOP。稳定绿灯：发有有效期的 GO，FPGA 继续自己原来的导航任务。
4. FPGA 通过路口后回传 `CROSSING_COMPLETE(0x11)`，event_id 必须与 PERMISSION 中本次斑马线 ID 一致。X5 才结束本次过街并准备下一次识别。

本地等待 2 秒仅决定何时开始观察，不是实际停车的测量结果。GO 租约保持最多 1000 ms；桥接节点默认还要求新鲜 FPGA 状态。FPGA 必须在 GO 到期、急停或 STOP 时停止。没有有效导航目标时，即使收到 GO 也保持停车；GO 只是解除视觉交通暂停，不是“自由直走”。

原 `STATUS(0x12)`、`TELEMETRY(0x20)`、`PERMISSION(0x80)` 保留。新 NAV_STATUS 可刷新通信状态，但云台状态不能凭导航状态推断；X5 本地云台驱动提供自己的控制状态。FPGA 无需提供 line_valid，X5 新导航节点也不检查它。

## 验证与下一步

已实现语义目标节点、UART 编解码、请求过期、同目标重试、匹配到达、重复/乱序拒绝、取消后迟到到达拒绝、模拟串口展示。`tests/test_fpga_navigation.py` 覆盖这些行为；原协议测试保持通过。

尚未把地图规划算法写入你尚未提供的 FPGA 工程，也未在本次操作中打开真实 UART 或驱动底盘。硬件对接顺序：先让 FPGA 打印解析后的目标并回传状态，再架空轮子验证 STOP/GO 与取消，最后在实地图检验编码器、陀螺仪和每个楼栋观察点。
