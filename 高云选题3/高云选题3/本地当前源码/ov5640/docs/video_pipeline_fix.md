# 视频花屏修正

本次替换视频缓存传输部分，保留摄像头初始化、DVP 字节采集、DDR3 IP、IMU、地图、导航菜单及原引脚约束。摄像头输出 640×480 RGB565；800×480 屏幕中间原比例显示，左右各 80 像素黑边。

## 数据路径

`DVP_Capture → Gowin wr_data_fifo（16→128 位）→ DDR 双帧缓存 → Gowin rd_data_fifo（128→16 位）→ 视频/IMU/菜单 → TFT`

- 停用视频分块 mailbox、手工 128 位拼包和分块 DDR 地址计算，改为现有 Gowin FIFO 连续打包；每次实际接受 DDR 写命令后，帧内地址递增 8 个 16 位字。
- 每行必须收到 640 个像素，必须收齐 480 行，且全部 307200 个像素都完成 DDR 写入，才能在 TFT 消隐期切换显示帧。
- 短行、提前出现下一帧、写 FIFO 溢出会丢弃当前采集帧，保留上一幅完整画面；清空不足 8 像素的残留拼包后重新采集。
- 显示预读有占用量限制，预留全部未返回读请求的容量。占用量判断先寄存再参与仲裁，避免显示 FIFO 计数直接穿过仲裁逻辑驱动摄像头 FIFO 的 BSRAM 读使能。
- 写 FIFO 的 128 位输出先进入一级寄存器，再由 DDR 接收；FIFO 取数使能不经过 DDR 读写仲裁组合逻辑。
- DDR 读写仲裁保留显示读请求的服务机会。左右黑边及行消隐不消耗摄像头像素；菜单和 IMU 遮挡视频时，视频 FIFO 仍按屏幕位置消耗。

`portrait_tiles.v` 作为已有源文件保留，但新视频链路不再实例化它。现有 `wr_data_fifo.v` 和 `.vo` 的接口一致；旧 `.ipc` 容量配置记录与生成文件不完全一致，本次直接复用生成文件，不重新生成 IP，也不依据 `.ipc` 的深度常数控制读写。

## 验证

```powershell
python sim/run_portrait_tests.py --full
python sim/run_fix_tests.py
powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1
python tools/verify_navigation_build.py
```

`video_capture_tb.v` 将原 `DVP_Capture`、两个 Gowin FIFO 仿真网表及真实适配器一起仿真。覆盖逐行 HREF、每像素两个字节、58.8 MHz 摄像头像素字节时钟、100 MHz DDR 用户时钟、33 MHz 显示时钟、DDR 延迟与停顿、完整 640×480 帧逐像素比较、并发采集、黑边、短行、提前换帧、写 FIFO 过载、残留拼包清空和三次帧切换。

最终资源、时序、位流时间及 SHA256 由 `tools/verify_navigation_build.py` 写入 `impl/verified_build.json` 和 `docs/resource_usage.md`。位流为 `impl/pnr/ov5640.fs`。

软件仿真和布局布线不代替实物验证。本次未执行板卡烧录，不能据此确认实物花屏已消失。原 `PR1014` 摄像头 PCLK 普通布线提示仍存在，后续实物仍异常时需继续核对摄像头输入时序和板上 DDR 链路。
