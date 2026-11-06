# 消防员近景补充训练

用户已要求将新消防员场景加入训练集并重新开始训练。这次采用`dataset_firefighter_v4/data.yaml`，在V8融合数据上加入96张新图，共5,160张图、48,240个框；训练4,656张，验证252张，测试252张。没有额外复制增强这96张图，全部可见实例标签均保留。

新增96张包括84张消防员正例和12张负例；负例中8张只有普通人物、4张为空白居民楼壁纸。正常/暗光、正面/左右斜视、三个距离和局部实体遮挡均有覆盖。原壁纸消防员物理尺寸没有放大，4个远端遮挡后的目标低于40px门槛，仍保留标签。

原训练、验证和测试图逐字节保留，val/test清单顺序不变。此次新增内容是合成图，不是GS130W实拍。新增图所在原目录名仍为`source_data/firefighter_holdout_v1`以保留来源，**用途已改为训练，不能再作为本轮及后续模型的独立验证成绩**。原冻结采集计划中的evaluation_only只描述最初计划；用途变更由`CURRENT_USAGE.json`和新数据集`source_role_change.json`记录。

此前12张负例曾出现RGB贴图与mask不一致，现已根据原冻结计划重渲修复：逐张核对实际Base Color图像绑定、源图哈希、RGB整套接触表及所有可见mask标注，通过后才入库。错误版本保存在隔离目录，不会进入训练。

## 本次任务

- 运行名：`wallpaper_yolo11n_firefighter_v4_20261008`。
- 初始化：上一轮暂停快照的第16轮最佳权重，SHA256 `3eef83ce6c6668c68f8ee1560f0a442d6c76d922e9460d046158f475a6585efd`。
- 数据变化，建立新优化器与新轮次；旧任务完整第25轮快照仍保留。
- RTX4060 Laptop / CUDA0，输入640，batch8，workers0；AdamW学习率0.0003、预热3轮，最多80轮、patience20。
- 正常结束后用本轮最佳权重评估一次原合成测试集。停止或失败时不自动测试。

[实时状态](TRAINING_STATUS.md) · [实时进度](runs/_jobs/wallpaper_yolo11n_firefighter_v4_20261008/progress.html) · [融合数据浏览器](dataset_firefighter_v4/review.html) · [完整检查](dataset_firefighter_v4/independent_validation.json)

`11_train_firefighter_v4.cmd`为同一新建任务入口，已启动后请勿重复运行。未来中断恢复需保持本次数据与配置一致，用本次last.pt恢复，不能使用旧任务last.pt更换数据。当前独立消防员近景验证需要另留新场地或实拍样本。
