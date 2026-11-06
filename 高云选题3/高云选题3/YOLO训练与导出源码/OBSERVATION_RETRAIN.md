# V8 融合数据再次训练

> 后续更新：用户已要求将96张新消防员场景加入训练并开启V4新微调，见[当前V4说明](FIREFIGHTER_V4_TRAINING.md)。本文及下面暂停记录属于旧V8运行历史。

**最新状态：2026-10-08已按用户要求暂停，等待明确说“开启”。** 完整保存至第25轮，最佳第16轮，第26轮未完成。暂停快照位置见`runs/observation_v3_pause_snapshot.txt`。已核验last.pt可加载且包含优化器与EMA；同一数据与配置下，恢复应从第26轮继续，而非重复运行新微调入口。本次没有进行最终测试。

用户已明确选择：实拍暂未准备，先用现有融合数据重新训练。消防员独立实拍验证仍待真实照片，不将渲染图或训练区回放当作实拍。

后续现场拍摄使用[实拍采集说明](real_capture_validation/采集说明.md)与`10_capture_real_validation.cmd`。当前实拍库存0张，准备了9个机位、24组光照/遮挡条件及6张背景的采集方案；实拍单独验证，不自动进入训练集。

- 运行：`wallpaper_yolo11n_observation_v3_20261008`。
- 数据：`dataset_observation_v3/data.yaml`，共5,064张图；训练4,560、验证252、测试252。
- 初始化：旧任务暂停快照的第30轮最佳权重，SHA256为`24772bb6f8f1125891ad13c9ec79801c6ed6adc50f8eef51d60ea10c91cc22c9`。
- 方法：创建新的YOLO11n Detect微调任务、新优化器、新轮次。旧第34轮任务与快照保留。
- 参数：GPU0 / RTX4060 Laptop，640输入，batch8，workers0，最多80轮，patience20。
- 优化器：AdamW，初始学习率0.0003，预热3轮；沿用上次微调设置，保留全部可见小目标标签。
- 评估：每轮验证，正常训练结束后对本次最佳权重执行一次合成测试集评估。实拍验证单独开展，不能从本轮合成指标推断实物精度。

[实时训练进度](runs/_jobs/wallpaper_yolo11n_observation_v3_20261008/progress.html) · [当前状态](TRAINING_STATUS.md) · [新融合数据](dataset_observation_v3/review.html)

当前后台训练已经停止。训练运行时不依赖进度页面保持打开；`09_train_observation_v3.cmd`是原新建任务入口，使用同一运行名，已有任务会拒绝重复创建，不能用它恢复暂停任务。恢复必须等用户明确说“开启”。

训练输出位于`runs/wallpaper_yolo11n_observation_v3_20261008/`，`weights/best.pt`为本次最佳，`weights/last.pt`为当前最后一轮。旧观察回放页展示旧权重结果；本轮结果必须以新运行路径为准。
