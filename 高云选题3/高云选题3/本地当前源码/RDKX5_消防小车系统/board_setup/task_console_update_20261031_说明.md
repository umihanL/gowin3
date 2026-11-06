# 任务驾驶台完整更新包

当前修订为 **`continuous_patrol_bidirectional_flame_centering_20261103`**。包名沿用既有安装入口，实际版本以 manifest 和 SHA256 为准。本轮交付及验证见 `../reports/bidirectional_centering_20261103/交付说明.md`。

**正式云台**打开真实双串口并读取实际位置，不因缺失参考而退回 dry-run。已有固定机械零点时，开机根据绝对编码器只读恢复参考，不把当次开机姿态重新定义为0°。上位机保留可重复记录机械零点的入口。ZDT 水平现在一次发送完整剩余角度脉冲，取消0.9°分段等待；机械限位仍为±90°，指令分辨率0.1125°。服务启动和重连不会自动转动、喷水或重新标零。

**区域导航**在出发前按实际位置和朝向一次安排最多四个途经点及出口，FPGA 一次规划完整路线，优先在区域内行驶。途经点只约束路线和记录通过进度，不停车、不等待回正、不额外降速。完成原路线后在区外出口停稳。上位机展示途经顺序、借道原因和出口目标。操作指南见 `../ros2_ws/docs/语义区域搜索与地图导航.md`。

**火焰触发暂停与逐个居中**全程接收 YOLO，仅在置信度至少 90%、连续 3 个新鲜帧确认新火焰后请求停车。停稳后收集画面左右两侧稳定目标，逐个通过 FPGA 前进或后退小段移动，使火焰中心进入画面竖直中线附近（默认半宽±4%）。每个目标必须实际停稳且连续新画面居中至少 1 秒。只搜索记录观察完成；搜索并灭火则在这一秒后进入既有瞄准短喷流程，确认关水后处理下一个目标。整批处理完返回暂停时的原位置与朝向，再恢复同一条路线、任务和路径游标，不重新规划。完成整批后有 3 秒触发冷却，并对已处理跟踪目标去重。

**目标重现和移动边界**局部移动使用原路线上的可验证直线通道，单次20～100毫米、速度20～60毫米/秒；不能确认通道或持续丢失的目标记录原因并跳过，不伪报居中。原目标暂时移出画面时先回原停车位置重找；跟踪编号变化需原视角连续3个新帧唯一匹配，歧义不重绑。相机右半边对应车头前方。配置沿用 `checkpoint_alignment.json` 与优先的 `checkpoint_alignment.local.json`；更新器保留已有本机参数。

**拓展二实时标注**仅拓展二显示竖直中线、容差带、当前/已处理目标编号、前进后退状态和1秒停留进度。框位置必须来自与当前JPEG曝光时间、序号和坐标系一致的目标注册，避免旧框叠到新图。拓展一和普通状态保留原YOLO框，不显示这层标注。

本包同时保留编号导航、规则语音输入、Qwen场景分析与中文播报、火焰统计、选定目标灭火和标定工作台。人工试喷通过FPGA B13输出100～1000毫秒脉冲，开机不自动喷水。自动靠近和自动灭火仍使用实际标定，不从仿真数据生成实测参数。`region_approach.json`仅在缺失时添加，实测参数使用`region_approach.local.json`。

**水枪活动配置须迁移。** 更新器保护板上`*.local.json`，不会自动把旧GPIO配置变成B13。部署时停止任务服务，找到实际`io_config`文件，用包内`tools/migrate_pump_to_fpga.py --config 实际文件`先预览，再加`--apply`备份迁移。脚本只改pump，初次迁移保持禁用，保留电机/标定设置。新固件的extension能力位0x20与1A状态回读用于确认B13协议；详情见《拓展任务与水枪接线_20261031.md》。

**FPGA兼容要求：** 编号导航需要CAPS的0x10位及0x24起始朝向命令；连续区域导航需要地图能力位`0x20|0x40|0x80`及`0x21/0x25/0x26/0x37/0x3A/0x3B`接口。`0x37`提供与位姿同快照的路线版本和前瞻索引，结合实际位置判断途经进度。本次双向居中还要求拓展协议版本2、能力位`0x40`以及`0x3C RETAINED_JOG`/`0x3D RETAINED_JOG_STATUS`，必须配套本次新FPGA固件，旧连续路线位流不具备这一能力。本轮编译状态见相邻`ov5640/diagnostics/retained_jog_20261103/validation.json`；实际刷写版本仍以`ov5640/impl/pnr/ov5640_programming_latest.json`为准，编译并不代表已刷写。本包不刷写FPGA，也不改变模型或活动UART配置。

本包用于更新已有 `/root/rescue/ros2_ws`，不负责安装ROS、模型或首次配置硬件。本轮完成本地修改和验证，尚未部署；此前电机或音响成功记录不代表新版已完成实机验收。之后需以实际部署日志确认版本。

## 包里有什么

- 完整 `rescue_vision` Python 源码、C++ 本地扩展源文件、launch、安装配置，以及 `rescue_interfaces` 源码。
- 上位机 Web 界面、FPGA 地图 SVG、三个只读地图几何文件 `fpga_map_catalog.json`、`numbered_points.json`、`extension_map.json`、ROS 桥接服务器和相关运行/构建脚本。三个几何文件更新前会备份。
- **仅新增的执行器模板** `actuator_io.mixed.template.json`，默认禁用/干运行；不会把它指定成板端活动配置。
- 缺失时才补充的 `extension_uart.json` 与 `traffic.fpga_navigation.json`。UART 默认 `enabled=false`、端口为空，不能自行启用 FPGA。已有源目录配置原样保留；若源目录缺失但安装目录有配置，则先恢复安装目录中的原值，完全没有才使用包内默认值。
- 最终使用指南、完整学习手册。
- 区域目录 `semantic_regions.json`、绑定FPGA ROM身份的 `region_traversability.json` 与三维靠近模板 `region_approach.json`。
- 缺失时添加的视觉配置 `checkpoint_alignment.json`；连续巡检使用其中的火焰阈值和冷却参数，经过途经点不触发旧版停车对齐流程。
- 未标定的独立`fixed_camera_landing.template.json`，`fixed_camera_start_config.py`启动选择器、只读`self_check.py`与仅生成报告的`preview_fixed_camera_calibration.py`。实测表仍由板端`*.local.json`提供，不从测试数据生成。

不覆盖已有模型权重、`.venv`、标定数据、喷嘴落点/零位数据、`*.local.json`、已有 `extension_uart.json` 和红绿灯配置、现有活动执行器/音频/道路配置、控制令牌。已有安装目录中的配置也会在 colcon 构建后原样恢复（三个地图几何文件和新增混合模板除外），避免覆盖仅在安装目录调过的参数。地图目录是 FPGA 源码导出的静态地图标识，实时位置和路径必须由新 FPGA 固件回传。

EXE 是加载 X5 `8090` 服务的桌面外壳，不内嵌网页；电脑侧已重新构建并创建桌面快捷方式。仅复制 HTML/CSS/JS 不够，新增的 `fpga_map` 等 Python 模块及 ROS 节点必须一起更新到 X5 并构建。

按用户最新更正，两轴功能已交换：**水平轴为电机2 ZDT_XS（PIN3 TX/PIN5 RX，ttyS3）；俯仰轴为电机1 PD42（PIN8 TX/PIN10 RX，ttyS1）**。电机编号和协议随物理设备保持不变；新模板、反馈顺序和界面同步此分工。不要把旧轴的已测零位、方向或限位直接当成新安装的标定值。`verify_mixed_gimbal_closed_loop.py` 是电脑工作区的离线验收工具，依赖同工作区 tests，不是板端运行入口。

## 现在：只在电脑重新生成包

在 PowerShell 执行：

```powershell
cd 'D:\工程仿真\FPGA赛道_小车仿真\RDKX5_消防小车系统\board_setup'
.\task_console_update_20261031.ps1 -Python 'D:\工程仿真\FPGA赛道_小车仿真\YOLO11n_model\.venv\Scripts\python.exe'
```

默认只生成本地包，不连接板子。相同输入文件得到相同 tar.gz；源文件逐项 SHA-256 在 `.manifest.json`，整个归档 SHA-256 在 `.sha256`。`task_console_update_20261031_build.py` 是可复现构建器。

## 以后连接 X5 时更新

1. 先让车停止、停止喷水与云台动作，正常退出手动启动的任务。停止已有任务前需支撑可能失去保持力矩的俯仰负载。
2. 在 X5 停止有关服务。摄像头服务和 Qwen 服务无需为这次源码更新关闭：

```bash
for unit in rescue-task-interaction.service rescue-task-modes.service rescue-presentation.service rescue-mipi-vision.service rescue-yolo-preview.service; do
  if systemctl cat "$unit" >/dev/null 2>&1; then sudo systemctl stop "$unit"; fi
done
```

若之前是在终端手动 `ros2 launch`，也需在对应终端正常退出。更新器检测仍运行的有关节点时会退出，不强制杀进程。

3. 在电脑执行：

```powershell
.\task_console_update_20261031.ps1 -Board rdkx5 -Python 'D:\工程仿真\FPGA赛道_小车仿真\YOLO11n_model\.venv\Scripts\python.exe' -Deploy
```

`rdkx5` 是现有 SSH 别名，可替换成已配置密钥的 SSH 主机。脚本先生成包、上传到独立暂存目录、校验整个归档，再在板端验证每个文件 SHA-256、已有配置和进程状态。通过后备份被覆盖源码和已安装的构建产物，更新源码，调用 `tools/build_board.sh` 在 X5 构建。

**部署脚本不会启动/重启服务、切换串口复用、改电机参考角度或发出喷水命令。**后续启动 `run_standalone_tasks.sh` 会开放人工试喷入口，仍需操作者提交单次命令；自动灭火权限与实测标定独立保留。失败时服务保持停止；错误中给出 `/root/rescue/update_backups/task_console_时间_PID`，里面有 `before_update.tar.gz`、`changes.json`、`build.log`。备份用于人工检查/恢复，不自动删除文件或自动启动旧控制程序。

4. 核对原有板端配置，沿用已验收的启动方式。已使用 systemd 的机器可以依次恢复：

```bash
sudo systemctl start rescue-mipi-vision.service rescue-yolo-preview.service
if systemctl cat rescue-task-interaction.service >/dev/null 2>&1; then
  task_unit=rescue-task-interaction.service
elif systemctl cat rescue-task-modes.service >/dev/null 2>&1; then
  task_unit=rescue-task-modes.service
else
  echo '没有已配置的任务服务，请沿用原来的人工启动方式；不要同时启动两套任务。'
  task_unit=''
fi
if [ -n "$task_unit" ]; then
  sudo systemctl start "$task_unit"
  systemctl --no-pager --full status "$task_unit"
  journalctl -u "$task_unit" -n 60 --no-pager
fi
sudo systemctl start rescue-presentation.service
systemctl --no-pager --full status rescue-presentation.service
```

任务服务只启动一个：优先已有的 `rescue-task-interaction.service`，否则沿用旧 `rescue-task-modes.service`，不能两者一起启动。若相机没有运行，先按原有方式启动 `rescue-mipi-camera.service`。首次或已删除原服务文件的机器不应直接使用本更新包安装；应先恢复原本已验收的基础部署。

5. EXE 连接 X5 当前 IP 的 `8090`。检查三个主页、语音转写、摄像头图像、真实状态反馈。设备未连接/数据过期时，界面应显示未知或不可执行。

## 新功能验收边界

- 语音/文字规则入口、参数一致性、朝向UART握手和上位机回执已在本地检查；证据位于 `reports/rule_navigation_20261101/`。FPGA的弯速与途中偏离调整通过7项回归，完整编译状态见 FPGA 新调整说明。本轮未烧录或实车移动；03:42历史烧录记录对应前一版本。
- 场景分析和聊天继续使用现有 `config/edge_dialogue.local.json` 中的Qwen3-1.7B，复用现有loopback接口；导航不调用该接口。本包不下载或更换权重。真人语音的转写与独立核验速度仍需实机验收。
- 最新纯本地验收在系统目录 `reports/bidirectional_centering_20261103/`；本轮覆盖双向居中、同路线恢复、目标冷却、同帧预览和可选灭火流程，没有实际行驶、喷水、真人录音或音响播放。桌面EXE外壳未改，不需要因车载代码修改重复打包EXE；它不会代替本包更新X5页面和ROS节点。
- 两轴闭环/自检做过本地协议与动力学测试，不能代表实机角度精度、传动方向、限位、减速比和安装零位已经验收。新混合模板默认不启用。
- ZDT 现有资料未解释的新增配置位仍需实机核验；不得为了让按钮可用而删除检查。
- 自检依赖真实的新鲜角度反馈、实际车辆停止条件、当前启动周期确认的参考角度及执行器所有权。测试为小角度往返，不进行找零或校准；失败/停止会失能，负载需支撑。
- 瞄准和喷水是独立命令。模板不会自动喷水。

本地使用入口见 `ros2_ws/docs/上位机任务驾驶台使用.md`；完整原理与学习入口见 `ros2_ws/docs/完整学习与验收手册.md`。
