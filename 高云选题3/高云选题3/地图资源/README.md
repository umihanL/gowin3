# 阿克曼消防小车街区地图

**当前地图 V6：一楼贴墙楼牌版。** 打开 [Blender 场景](rescue_maze_v6/community_fire_rescue_v6.blend)，详见 [V6 使用说明](rescue_maze_v6/使用说明.md)。13 栋唯一编号 01–13 与楼名，采用 30×3.8 cm 横向门头贴纸，中心离地约 20.6 cm，已移除屋顶牌架。已同步到 C++ 小车模拟器。楼号可采用低频视觉大模型读取，目前未接入具体模型。

启动器和 `scene-preview.html?view=signs` 均指向 V6；网页“楼牌”按钮可进入离地 13 cm 的观察点。V5 预览保留在 `scene-preview-v5.html`。若网页服务关闭，在本目录运行 `python -m http.server 8765 --bind 127.0.0.1`。以下 V4/V3 文案为历史记录，其“当前版本”、尺寸和入口描述不适用于 V6。

**当前版本 v4：窄街多回路街区**。打开 `rescue_maze_v4/community_fire_rescue_v4.blend`，详见 `rescue_maze_v4/使用说明.md`。场地22×20米，外环主路1.8米、内部街道1.5–1.6米、停车入口连接路1.65米；包含12个公共T路口、7个独立公共回路、1个停车入口和11栋建筑。车载水平视场改为72°、下俯8°，保留双侧白边、停车、施工、单侧绕障及台阶任务。双击 `打开消防街区.cmd` 或 v4 目录的 `打开新版地图.cmd` 打开v4。

浏览器主入口 `http://127.0.0.1:8765/scene-preview.html` 加载 v4 GLB，提供自由、俯视及9个配置中的车载预设视角，镜头参数与画幅从v4配置读取。这是第145帧的静态模型预览，道闸与红绿灯动画请在 Blender 播放。v3预览保留在 `scene-preview-v3.html`，最初的无贴图基准预览保留在 `scene-preview-baseline.html`。若服务未启动，在本目录运行 `python -m http.server 8765 --bind 127.0.0.1`。

以下为历史版本记录，旧配置、旧 SDF/WBT、旧生成命令与旧验证结果不适用于 v4。道路和镜头的调整用于贴近实际车辆尺度，不代表已经验证视觉循迹、双目测距或实车避障算法。

**历史版本 v3：双侧边线感知街区**。直接打开 `rescue_maze_v3/community_fire_rescue_v3.blend`，详见该目录的 `使用说明.md`。该版主路3米、车载HFOV86°、下俯10°，为外环与十字街道布局；浏览器历史入口为 `scene-preview-v3.html`。v3工程与贴图保持保留。

**历史版本 v2：80×60×60厘米小车版**。主文件 `rescue_maze_v2/community_fire_rescue_v2.blend`，中文说明 `rescue_maze_v2/使用说明.md`，双目布置 `rescue_maze_v2/camera_design.md`。如需查看此版，请在 Blender 中直接打开该文件；顶层启动脚本已指向 v4。此版为18×14米、7栋封闭建筑、连续路网、独立三车位数字牌，施工区与道闸分开。

**历史版本 v1：带贴图薄墙街区**。在 Blender 中直接打开 `community_fire_rescue_maze.blend`。该版含居民楼救火壁纸、停车区、动画道闸、红绿灯和施工绕行；操作详见 `消防街区使用说明.md`，贴图重绘交接见 `CLAUDE_TEXTURE_HANDOFF.md`。下面记录的是更早的无贴图基准场景；旧 `.wbt/.sdf` 不适用于新版迷宫布局。

这是面向 2026 嵌入式设计大赛 FPGA 高云赛道的参数化地图基线。官方指南明确要求摄像头/IMU 感知、编码器闭环运动、障碍检测与减速停车、起终点和路径点导航；官方文件没有规定场地长宽、道路宽度或建筑尺寸，因此本工程的 12 m x 8 m 几何是可缩放的工程假设。

## 地图形态

- 米制坐标，右手系，`X=东、Y=北、Z=上`，原点在场地西南角。
- 主路宽 1.8 m，支路宽 1.2--1.3 m，四角为建筑实体，中心为十字街区。
- 追加了 8 段 1.2 m 高、0.08 m 厚的可打印薄墙；墙面可以贴道路、楼体、门窗和标牌图案。
- 追加交通灯、通行杆、路桩、垃圾桶、锥桶和低矮障碍。它们参与占据栅格、SDF 碰撞和路线净空检查。
- 没有居民楼、道路、火焰等图片贴图。`fire_F1/F2` 只是调试语义目标，导航使用其 `approach_pose`。

## 文件

- `scene-config.json`：唯一主配置，尺寸、路网、墙体、障碍、航点和路线。
- `geometry_manifest.json`：由主配置生成的渲染/碰撞几何清单。
- `community_fire_map.blend`：Blender 4.5.10 生成的可编辑完整场景。
- `community_fire_map.glb`：完整物理地图几何，适合导入 Webots、Unity 或其他引擎。
- `community_fire_printable_thin_wall.glb`：地面、边界、薄墙、门面框和交通设施的轻量打印版。
- `community_fire_map.sdf`：Gazebo/SDF 几何世界，不含机器人控制器。
- `community_fire_map.wbt`：由 manifest 生成的 Webots 几何世界，不含机器人控制器。
- `occupancy_raw.pgm/.yaml`：5 cm 分辨率、未膨胀的原始占据栅格；`negate: 0` 时黑色为占据、白色为自由。
- `geometry_validation.json`：语义点、路线净空和官方任务映射检查报告。
- `build_scene_blender.py`：从 manifest 重建完整 `.blend` 和两个 GLB。
- `generate_webots_world.py`：从 manifest 生成 `.wbt`。

## 软件链路

已将 Blender 4.5.10 便携版放在 `D:\仿真工具\Blender`，并实际执行脚本生成 `.blend` 和 GLB。浏览器预览仅用于快速观察几何和车载视角；它不是物理仿真器。

实际跑小车时，推荐把 `community_fire_map.glb` 或 `community_fire_map.wbt` 导入 Webots，加入按实测尺寸建模的麦轮机器人、RGB/双目相机、IMU、编码器和激光/TOF 传感器。Gazebo 更适合 ROS/Ubuntu 链路；当前 Windows 环境未安装 Gazebo。Webots 官方 Windows 安装包下载受当前网络到 GitHub 的连接限制，未能完成自动下载。

## 重新生成

```powershell
python generate_map_artifacts.py
python generate_webots_world.py
& 'D:\仿真工具\Blender\blender.exe' -b --python build_scene_blender.py
```

修改真实小车的长、宽、高、相机高度、视场角和激光高度后，应重新运行以上命令，并以 `geometry_validation.json` 的净空结果为准。薄墙和设施目前是纯色材质，适合后续制作打印贴图或替换为实物材质。
