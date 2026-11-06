# 消防小车 YOLO11n Detect 训练工程

**当前更新：用户已要求把新增96张消防员场景加入训练并开启新微调。** 当前入口为`dataset_firefighter_v4/data.yaml`，见[本次V4训练说明](FIREFIGHTER_V4_TRAINING.md)和[实时状态](TRAINING_STATUS.md)。共5,160张图，训练/验证/测试4,656/252/252张。这96张图不再作为独立验证；实拍仍待准备。旧V8完整25轮检查点保留。观察算法见[使用指南](OBSERVATION_GUIDE.md)。

V8已融合完成：**5,064张图、47,778个框**，train/val/test为4,560/252/252张，新增768张仅进入训练集。[查看融合数据](dataset_observation_v3/review.html) · [数据检查](dataset_observation_v3/OBSERVATION_DATA_REPORT.md) · [旧权重的分组评估与近景验收](reports/observation_v1/README.md)。新增数据与启动训练本身不代表模型精度已经提高，需以本次评估结果为准。

**已暂停的 V7 训练对应 `dataset_people_v2/data.yaml`，其预览使用 `05_review_people_v2.cmd`，原启动脚本为 `06_train_people_v2.cmd`。** 当前轮次和暂停状态，以 [当前训练状态](TRAINING_STATUS.md) 和对应日志为准。以下原训练说明保留作复现；最新 V8 数据与观察功能以页首指南为准。原补充原因见 [人物样本补充与再训练](人物样本补充与再训练.md)。

本工程检测居民楼壁纸上的三类目标：`0 fire` 火焰、`1 firefighter` 消防员、`2 person` 普通人。使用 YOLO11n **Detect**。楼号、楼名和人体关键点不属于这份三类任务；同一个消防员不重复标为普通人。

## 从这里开始

1. 双击 `05_review_people_v2.cmd`，查看当前补充版双目图像、标注与数据来源。彩色框是标注真值，不是模型预测。
2. 在 VS Code 打开整个 `YOLO11n_model`，选择 `.venv` 解释器。本机已安装 PyTorch 2.8.0+cu128、torchvision 0.23.0+cu128、Ultralytics 8.3.253；新机器安装方法见 [训练指南](TRAINING_GUIDE.md)。
3. 先查看 [当前训练状态](TRAINING_STATUS.md)。需要启动当前微调且没有同一任务运行时，使用 `06_train_people_v2.cmd`；它会先检查选定数据。不要重复启动正在运行的任务。
4. 正常训练结束后，任务管理脚本会对最佳权重执行一次独立测试。状态页提供实际权重、训练曲线和测试指标链接。

当前微调设置为最多 **80 epochs、640 输入、batch 8、workers 0、patience 20**；采用 **AdamW、初始学习率 0.0003、预热 3 轮**。从旧最佳权重初始化，但建立新优化器、新轮次与新学习率调度。配置见 [yolo11n_people_finetune.yaml](config/yolo11n_people_finetune.yaml)。

## 当前数据与历史基线

当前 `dataset_people_v2` 融合 V5/V6 与 V7 人物、消防员专属补充视角。V7 只进入训练集；原 val/test 的清单顺序、图片和标签保持不变。**实际张数、框数与目标尺寸分布以 [当前统计](dataset_people_v2/statistics.json) 为准**；检查结果见 [独立检查报告](dataset_people_v2/independent_validation.json)。左右眼和增强图数量不能当作独立场景数。

当前补充版实际共 **4,296 张图、40,786 个框**，训练/验证/测试为 **3,792 / 252 / 252 张**。V7 新增 **1,440 张图**，由 **720 张原图 + 720 张成像增强图**组成，对应 **360 对双目采集、180 个位姿组**，全部进入训练集。

历史基线 `dataset` 保留 **2,856 张图、25,066 个框**，其中训练 2,352 张、验证 252 张、测试 252 张。它包括旧版 600 对、一楼门头版 240 对双目原图，以及仅用于训练的 1,176 张成像增强图。`00_review_dataset.cmd`、`02_check_dataset.cmd`、`03_smoke_cpu.cmd`、`04_train_gpu.cmd` 和未传 `--data` 的默认命令仍指向这份基线。

首次运行 `wallpaper_yolo11n_gpu_20261008` 已按要求停止：完整保存到第 41 轮，最佳权重来自第 34 轮。快照位于 `runs/wallpaper_yolo11n_gpu_20261008/stopped_for_data_expansion/`。当前微调运行名为 `wallpaper_yolo11n_people_v2_20261008`，输出与旧运行分开保存；不要用旧 `04_train_gpu.cmd` 启动当前补充版。

## 文件对应关系

| 路径 | 用途 |
| --- | --- |
| `dataset_people_v2/data.yaml` | 当前补充版训练、验证、测试入口 |
| `dataset_people_v2/images/`、`labels/` | 当前图片与三类 YOLO 检测框 |
| `dataset_people_v2/train.txt`、`val.txt`、`test.txt` | 当前明确划分的清单，val/test 与历史基线冻结一致 |
| `dataset_people_v2/review.html` | 当前双目标注浏览器 |
| `dataset_people_v2/statistics.json`、`independent_validation.json` | 实际统计与全量检查报告 |
| `dataset_people_v2/metadata.jsonl`、`manifest.json` | 来源、相机、增强参数、哈希与冻结基线记录 |
| `dataset/` | V5/V6 历史基线，保留原数据与清单 |
| `source_data/v5_legacy/` | 旧三类原图、mask、源标注、场景与包快照 |
| `source_data/v6_ground_floor/` | 一楼门头地图的原图、mask、源标注与采集计划 |
| `source_data/v7_people_focus/` | 新人物/消防员视角的原图、mask、标注与采集计划，仅用于训练 |
| `train.py`、`tiny_target_training.py` | 训练/评估入口与保留极小框的数据变换 |
| `config/yolo11n_people_finetune.yaml` | 当前微调配置；旧 `yolo11n_detect.yaml` 为基线配置 |
| `tools/run_training_job.py` | 任务管理、动态进度与正常结束后的独立测试 |
| `weights/yolo11n.pt` | COCO 初始化模型；项目自训权重位于 `runs/` |
| `TRAINING_GUIDE.md` | 当前命令、安装、评估与 RDK X5 说明 |
| `数据集说明.md`、`数据复现.md` | 原基线规则与复现记录；V7 更新见补充文档 |

当前操作必须显式选择 `dataset_people_v2/data.yaml`；`train.py` 默认仍为旧 `dataset/data.yaml`。不要将 `source_data/*/packaged` 单独作为最终训练入口，也不要使用历史四类 `wallpaper_gs130w_v2`。

## 数据与训练的依据

V6 修订旧地图背景，去掉重复小号码、加入一楼门头贴纸，保留原火焰、消防员和普通人图案。同一采集位置的新旧版本继承相同划分，左右眼、光照变体和增强图不跨集合。V7 在原训练空间组内增加距离、角度与光照采集，使用新位姿、新立体对编号，不将验证或测试图用于补充训练。

每个新增视角重新渲染可见实例 mask，再从 mask 计算检测框。融合前校验来源，拒绝跨集合相同像素或矛盾标签；仅对完整相同的立体采集及其增强家族去重，部分像素重复保留完整来源并记录。训练使用 LetterBox、HSV 和张量格式化，保留可见目标标签，避免默认几何筛选丢弃极小框。

## 使用边界与历史记录

图像来自 Blender，目标仍是当前壁纸的 2 个火窗、1 个消防员和 8 个普通人图案。合成评估反映已知地图上的视角与光照适应情况；GS130W 的距离、分辨率、畸变、曝光和打印反光仍需实拍验证。保留 1～2 像素目标的标签，不代表这类目标一定可准确识别。

使用 val 选择模型和阈值，冻结设置后评估 test，另留独立实拍测试集。`dataset_people_v2/calibration.txt` 仅提供训练来源的量化校准候选。RDK X5 仍需匹配板端 SDK 的 Detect 导出、量化、预处理与后处理验证；普通 ONNX 不能直接作为 BPU `.bin` 使用。

`NOT_TRAINED` 文件和初始交付检查记录描述训练开始前的历史状态。实际中间权重已保存在首次运行中，当前完成情况始终以 [TRAINING_STATUS.md](TRAINING_STATUS.md) 为准。COCO 初始化已通过 PyTorch 加载与 CUDA 前向检查，记录见 `gpu_preflight.json`。
