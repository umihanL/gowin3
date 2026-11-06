# 当前 GPU 训练状态（自动更新）

训练与测试完成。最后更新：2026-10-09T02:09:50+08:00（北京时间）。

- 运行：`wallpaper_yolo11n_firefighter_v4_20261008`
- GPU：RTX 4060 Laptop / CUDA:0
- 输入：640；batch：8；最多 80 轮；patience 20。
- 已记录完整轮数：42。
- 数据：`D:\工程仿真\FPGA赛道_小车仿真\YOLO11n_model\dataset_firefighter_v4\data.yaml`。
- 初始化：`D:\工程仿真\FPGA赛道_小车仿真\YOLO11n_model\runs\wallpaper_yolo11n_observation_v3_20261008\paused_by_user_20261008_220602\weights\best.pt`。新增数据微调：从旧最佳权重初始化，创建新运行与新优化器；不恢复旧训练轮次。
- 配置：`D:\工程仿真\FPGA赛道_小车仿真\YOLO11n_model\config\yolo11n_people_finetune.yaml`。
- 三类：fire、firefighter、person。
- 训练/验证/测试：4656/252/252 张；不把验证或测试标签用于训练。

[打开进度页](runs/_jobs/wallpaper_yolo11n_firefighter_v4_20261008/progress.html) · [训练日志](runs/_jobs/wallpaper_yolo11n_firefighter_v4_20261008/train.log) · [测试日志](runs/_jobs/wallpaper_yolo11n_firefighter_v4_20261008/test.log) · [详细状态](runs/_jobs/wallpaper_yolo11n_firefighter_v4_20261008/status.json) · [训练曲线数据](runs/wallpaper_yolo11n_firefighter_v4_20261008/results.csv) · [最佳权重](runs/wallpaper_yolo11n_firefighter_v4_20261008/weights/best.pt) · [最终测试指标](runs/wallpaper_yolo11n_firefighter_v4_20261008_test/metrics_summary.json)

最佳权重保存位置：`runs/wallpaper_yolo11n_firefighter_v4_20261008/weights/best.pt`。训练进行中时该文件可能尚不存在，或只是中间版本；以本页状态和详细记录为准。最终自动测试结果位于 `runs/wallpaper_yolo11n_firefighter_v4_20261008_test/metrics_summary.json`。

训练和独立测试已完成；权重与曲线保存在本工程 runs 目录。

初始交付的 NOT_TRAINED 和交付检查记录是训练开始前的历史状态。当前状态以本页、实际日志和权重为准。合成数据指标不代表 GS130W 实拍精度。
