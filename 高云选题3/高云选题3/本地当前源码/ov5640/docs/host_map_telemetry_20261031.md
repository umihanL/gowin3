# FPGA 实时地图只读遥测 v1

此增量保留原来的十字节帧及全部运动命令语义。现有 FPGA 地图、执行路径 RAM、编码器/IMU 里程计、触屏禁行线是唯一数据源；上位机不模拟车辆位置，也不拿两个编号点连线冒充 FPGA 路径。

当前实现位于本地 RTL 与 X5/上位机源码，已包含在本轮 `impl/pnr/ov5640.fs` 中。2026-10-31 已通过 Gowin 综合、布局布线、现有约束下的时序与资源核验；综合网表确认执行路径与 UART 遥测使用独立读地址，共实例化 8 个 SDPX9B 路径 RAM。完整证据见 [构建审计](../diagnostics/gowin_verified_20261031/build_audit.json)。**尚未烧录，也未验收真实串口和车辆状态。** 旧固件仍会返回 UNKNOWN，上位机应展示同源底图和“固件尚无地图遥测”。

## 帧和读取原则

`A5 5A 01 SEQ CMD D0 D1 D2 D3 XOR(bytes 2..8)`，115200 8N1。
回复 `CMD | 0x80`，SEQ 必须与请求匹配。字段均小端。先查询 `30`，仅接受完整 `00 01 0F 01`；旧固件的 `05 ...` 不是地图数据。

新增地图查询不会刷新运动看门狗，也不会更改速度、运行状态、目标点、模式或路径。控制端必须优先发 STOP/运动/心跳，地图只占串口空闲槽。建议位姿 2 Hz、路径头 2 Hz、禁行头 1 Hz；路径代次变化才下载，并在下载后复核头。最多取 128 个真实 RAM 索引时必须标注“采样路径”，保留首尾，不声称所有点都已传送。

`FF FF FF FF` 是新数据命令的统一无效/过忙标识。token 和 generation 在 `1..FFFE` 循环，保留 0/FFFF。重连后重新能力探测、读取地图 ID 并清空缓存；禁止跨重连复用代次。单次快照必须有超时，不能无限保留旧 token 或用几小时前缓存拼成新数据。

| CMD | 请求 D0..D3 | 回复 D0..D3 |
|---|---|---|
| `30` | 全 0 | `00 01 0F 01`：成功、版本1、功能位、数据格式1 |
| `31` | 全 0 | 锁存位姿：token LE16、flags、host task_id |
| `32` | token LE16、00、00 | 锁存 X mm signed16、Y mm signed16 |
| `33` | token LE16、00、00 | 锁存 heading 0.1° signed16、operation_mode、`phase<<4|status` |
| `34` | 全 0 | 路径 generation LE16、count低8、`valid<<7|planning<<6|count高4` |
| `35` | generation LE16、index LE16 | 该执行路径点 raster X mm uint16、raster Y mm uint16 |
| `36` | 全 0 | 地图身份 CRC32 LE；此版 `0x44BE0006` |
| `38` | 全 0 | 禁行 generation LE16、valid_mask(低4位)、00 |
| `39` | generation LE16、slot(0..3)、endpoint(0/1) | 禁行线端点 raster X/Y mm uint16 |

`37` 保留，不支持。

## 位姿真实含义

`31.flags`：bit0 当前有可画到地图的导航位姿；bit1 IMU 已可用；bit2 路径有效；bit3 UART 主机导航会话。其余为 0。bit0 为 0 时不要读旧数据冒充有效位置。32/33 都必须使用同一个31 token；任一无效就丢弃整组。

位姿取自实际 `vehicle_odometry` 完成一次编码器/IMU更新后的提交值，再由31原子锁存，避免内部先更新 heading 后更新 XY 的阶段混合。仅 `operation_mode=9`、导航有效、路径有效、status为2行驶或3完成时，bit0才为1。手动/扩展距离动作的相对坐标不是全局地图位置，本版不展示为地图小车。

FPGA位置是**车头中点**（工程实例使用 `vehicle_odometry #(.FRONT_REFERENCE(1))`），不是后轴中点；坐标 X 向右、Y 向上、单位mm，地图3520×3200mm，heading 0向右、+900向上、-900向下。heading来自里程计归一化到-1800..1800。上位机SVG为左上原点，转换 `x_cm=x_mm/10`、`y_cm=(3200-y_mm)/10`，图标旋转角 `-heading/10`。这种定位依赖起点/朝向与编码器标定，会有惯导漂移，不能称为外部测量绝对定位。

路径/禁行线已是**左上原点**，Y向下，只除以10变cm，不能再次翻转。35输出只为展示量化到1mm，不修改 FPGA 原 cm Q8 执行路径。

## 路径和禁行一致性

路径 generation 在规划开始、清空、取消、有效性变化、planning变化、禁行线变化时递增。地图正在改变时 valid立即为0。35要求代次相同、路径有效、没有规划、索引小于count；任意失败返回FFFFFFFF。上位机以34头A开始，下载该代次索引，最后34头B验证代次/count/valid/planning都一致才发布。未完成整批、发生代次变化、断线、超时的点集全部丢弃。

禁行generation独立反映4条触屏线寄存器变化。读取38头，按mask取39每条线的两个端点，再读取38确认代次/mask相同。mask=0并通过复核表示“确实没有禁行线”，未读到不是“没有”。这些是触屏设置的禁行线，不代表测距传感器已检测到的全部实物障碍；规划器会在禁行线周围按约11cm扩展避让范围。

## 同源静态底图

X5 `tools/export_fpga_map.py` 从 `nav_map_init.vh` 的全部7040个raw_map字解码352×320的一厘米网格，并与`map_raw_1cm.mem`逐位对比。1=阻塞，0=道路。九个点与`nav_destinations.vh`和X5的`numbered_points.json`逐项校验。输出：

- `ros2_ws/config/fpga_map_catalog.json`：道路逐行游程、9个编号点、源码SHA256、地图CRC。
- `ros2_ws/web/assets/fpga_map.svg`：同源底图。
- `ov5640/src/navigation/nav_host_map_id.vh`：编译进入新RTL的CRC常量。

CRC定义：`b'FPGA_MAP_V1\0' + 所有raw_map字按索引LE16打包 + 每个编号1..9依次的三个单字节(id,x_planner,y_planner)`，使用标准zlib.crc32。ROM或目的地修改后必须运行导出器，同时更新这三份文件。CRC用于发现版本不一致，不是加密身份认证。

只有UART真实返回36且匹配本地catalog，才可叠加位姿/路径/禁行。静态底图离线仍可看，但必须明确离线/未验证。
