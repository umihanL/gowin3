# 恢复截图需求之前的版本

已按用户要求恢复 **host_navigation_20261027**，即“地图指定右转却左转、起点前后偏移影响入弯、S 弯提前转向”这条截图需求提出之前的版本。

- 源码、工程配置、综合网表、布局布线检查点和默认位流均从完整原始快照恢复。
- 共核对 162 个快照文件、110 个源文件；移出后续新增的 `vehicle_visual_localizer.v`，源码树与目标快照一致。
- 恢复包含 E14 TX / E13 RX 通信框架的版本；该需求之后的导航修改不再生效。
- 默认下载文件：`impl/pnr/ov5640.fs`。
- 独立同内容文件：`impl/pnr/ov5640_restored_host_navigation_20261027.fs`。
- SHA-256：`536e8b1d5c724ed80eeb78a268820be2633a9c452d8d64cd0a9259acfe857dfe`。
- 原始生成时间：2026-10-27 01:39:26（UTC+8）；直接恢复原始位流，没有重新编译，没有烧录。
- 原构建 Setup/Hold 违规端点：0/0。当前恢复是文件一致性核验，不表示重新进行了实车测试。
- 恢复前的源码与构建输出已保存至 `impl/before_requested_rollback_20261027_203120/`。

原发布验证记录：`docs/host_navigation_20261027_validation.json`。
恢复核验记录：`docs/navigation_rollback_20261027.json`。
