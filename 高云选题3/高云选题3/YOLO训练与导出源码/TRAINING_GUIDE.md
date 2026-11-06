# YOLO11n Detect 训练指南

当前选择 **`dataset_firefighter_v4/data.yaml`**，新运行与命令见[本次V4微调说明](FIREFIGHTER_V4_TRAINING.md)，入口为`11_train_firefighter_v4.cmd`。任务是否已启动、当前轮次及完成情况，请看[TRAINING_STATUS.md](TRAINING_STATUS.md)。下文保留V7及历史版本操作方法，旧入口不能用于启动本轮。

本工程检测壁纸上的 `0 fire`、`1 firefighter`、`2 person` 三类矩形框。同一消防员不重复标为普通人。人体关键点、分割掩码和楼牌文字读取不属于本模型任务。

## 1. 当前环境、数据与旧运行

本机项目 `.venv` 已安装 PyTorch 2.8.0+cu128、torchvision 0.23.0+cu128、Ultralytics 8.3.253。GPU 是 NVIDIA RTX 4060 Laptop，约 8GB 显存，检查时驱动为 616.92；以 `tools/check_environment.py` 实际输出为准。真实 CUDA 前向检查见 `gpu_preflight.json`。

| 项目 | 当前人物补充版 | 历史基线 |
| --- | --- | --- |
| 数据入口 | `dataset_people_v2/data.yaml` | `dataset/data.yaml` |
| 来源 | V5 + V6 + 仅训练使用的 V7 补充视角 | V5 + V6 |
| 预览入口 | `05_review_people_v2.cmd` | `00_review_dataset.cmd` |
| 训练入口 | `06_train_people_v2.cmd` | `04_train_gpu.cmd` |
| 配置 | `config/yolo11n_people_finetune.yaml` | `config/yolo11n_detect.yaml` |
| 运行名 | `wallpaper_yolo11n_people_v2_20261008` | `wallpaper_yolo11n_gpu_20261008` |

旧基线共 2,856 张图、25,066 个框，训练/验证/测试为 2,352 / 252 / 252 张。当前补充版实际共 **4,296 张图、40,786 个框**，训练/验证/测试为 **3,792 / 252 / 252 张**。V7 新增 **1,440 张图**，由 **720 张原图 + 720 张成像增强图**组成，对应 **360 对双目采集、180 个位姿组**，全部进入训练集。统计来源见 [statistics.json](dataset_people_v2/statistics.json) 与 [manifest.json](dataset_people_v2/manifest.json)。新增数据只进入训练集；原 val/test 的清单顺序、图片和标签保持一致。左右眼与增强图不是独立场景。

第一次训练已停止，完整保存到第 41 轮，最佳权重来自第 34 轮。快照在 `runs/wallpaper_yolo11n_gpu_20261008/stopped_for_data_expansion/`，含 `best.pt`、`last.pt`、曲线、配置、变换策略和哈希记录。当前微调从该 `best.pt` 初始化，创建新优化器和新轮次。

`NOT_TRAINED` 文件及早期交付记录保留训练前的历史事实，不能用于判断当前状态。已有中间训练权重；新训练及最终测试是否完成，查看动态状态和实际结果。本工程未据此宣称 RDK X5 板端 FPS 或实拍精度。

## 2. 新机器安装；本机通常无需重装

在 VS Code 打开整个工程，选择 `.venv` 解释器。新机器推荐 Python 3.10 或 3.11；安装只写项目环境，不修改系统 Python 的包。

双击 `01_setup_environment.cmd`：

- `Check`：只安装数据检查依赖，不支持训练。
- `CPU`：安装 CPU 版 torch/torchvision，适合软件流程测试。
- `CUDA`：安装 NVIDIA GPU 环境；先在 [PyTorch 官方安装选择器](https://pytorch.org/get-started/locally/)确认驱动兼容的 wheel 标签。本机已验证 `cu128`。
- `Existing`：保留已有 torch，验证导入并安装其余训练依赖。

```powershell
# 仅建立数据检查环境
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\setup_environment.ps1 -TorchMode Check
# 检查当前环境
& .\.venv\Scripts\python.exe .\tools\check_environment.py
```

安装器优先找 Python 3.10，再尝试 3.11、3.12，可用 `-PythonExe "完整路径\python.exe"` 指定。安装后生成 `environment.local.json` 和 `requirements.installed.txt`。更换 CPU/CUDA 构建时按 PyTorch 官方说明操作；安装器不擅自强制降级。正式 GPU 命令使用 `--device 0`，不会悄悄切换 CPU。

## 3. 检查当前补充版数据

双击 `05_review_people_v2.cmd`，查看双目原图与标注；框是数据真值，不是预测。完整检查命令：

```powershell
& .\.venv\Scripts\python.exe .\train.py --check-only --data .\dataset_people_v2\data.yaml --config .\config\yolo11n_people_finetune.yaml
```

它执行 `tools/validate_merged_dataset.py`，检查实际图像、标签、源文件哈希、分组及旧 val/test 冻结关系，再确认类别和配置。不会导入 torch、下载模型、生成运行 YAML 或训练，仅写检查报告。旧 `02_check_dataset.cmd` 检查的是历史 `dataset`。

原 `data.yaml` 的 `path: '.'` 明确相对该 YAML 所在目录。实际训练时，在 `runs/_runtime/<指纹>/` 生成绝对图片/清单路径与 `data.resolved.yaml`，不改原 YAML。移动整个工程后重新运行即可生成正确路径。

## 4. 可选的小样本流程测试

现有环境已完成真实训练通路验证，无需和正式训练同时运行额外测试。在新机器需要检查软件通路时，可执行：

```powershell
& .\.venv\Scripts\python.exe .\train.py --mode train --smoke --from-scratch --device cpu --data .\dataset_people_v2\data.yaml --config .\config\yolo11n_people_finetune.yaml --name people_v2_cpu_check
```

先完整检查数据，再取前 24 张训练图、前 12 张验证图，运行 1 epoch、320 输入、batch 至多 2。`--from-scratch` 使用内置 `yolo11n.yaml` 随机初始化，无需下载权重。它的 mAP 不代表模型能力，也不使用 test 调参。旧 `03_smoke_cpu.cmd` 仍使用基线。

## 5. 当前人物补充版微调

先查看 [当前训练状态](TRAINING_STATUS.md)。需要启动且没有同一任务运行时，双击 `06_train_people_v2.cmd`。等价的任务管理命令为：

```powershell
& .\.venv\Scripts\python.exe .\tools\run_training_job.py --name wallpaper_yolo11n_people_v2_20261008 --data .\dataset_people_v2\data.yaml --model .\runs\wallpaper_yolo11n_gpu_20261008\stopped_for_data_expansion\best.pt --config .\config\yolo11n_people_finetune.yaml --epochs 80 --patience 20 --batch 8
```

最多 80 轮、640 输入、batch 8、workers 0、patience 20。配置显式指定 AdamW、`lr0=0.0003`、`lrf=0.1`、`warmup_epochs=3`、`warmup_bias_lr=0`，避免 `optimizer=auto` 覆盖设定学习率。80 是上限，不保证跑满。旧三类权重用于初始化，优化器、轮次和学习率调度重新建立。

管理脚本先检查选定数据，再训练，正常结束后对最佳权重执行一次独立 test。日志、动态状态与进度页在 `runs/_jobs/<运行名>/`；停止或失败不会自动进入测试。已有任务/训练/测试目录会被拒绝覆盖，重复双击不能恢复同一个已停止任务。

新训练输出在 `runs/wallpaper_yolo11n_people_v2_20261008/`，其中 `weights/best.pt` 用于评估，`weights/last.pt` 用于满足条件的续训。最终测试输出在 `runs/wallpaper_yolo11n_people_v2_20261008_test/metrics_summary.json`。这些文件可能尚未生成，查看状态页出现的真实链接。

旧 `04_train_gpu.cmd` 和未传 `--data` 的 `train.py` 保留原默认数据、100 轮、patience 25，不是当前微调入口。`weights/yolo11n.pt` 是 COCO 初始化模型，不是项目自训火焰模型；来源和哈希见 `weights/PRETRAINED_SOURCE.json`，已完成 PyTorch 加载和 CUDA 前向检查。

可用显存还取决于其他软件。若显存不足，先结束占用 GPU 的渲染，再在新的运行名下把 batch 降到 4 或 2。`workers=0` 是 Windows 稳定默认值。不要在当前任务运行时同时启动另一个 GPU 训练。

### 极小目标的数据变换

训练使用 `tiny_target_training.py` 的 `TinyTargetDetectionTrainer`，固定兼容 Ultralytics 8.3.253。顺序为 **LetterBox 等比例补边 → HSV 颜色/亮度增强 → 张量格式化**，逐图断言目标数量不变。随机几何增强、多尺度训练均关闭，没有全局修改已安装的 Ultralytics。

默认 RandomPerspective 即使几何参数为 0，也会筛掉短边不超过 2 像素的候选框。历史基线训练集 20,534 个框中有 3,742 个此类框，因此直接使用标准 LetterBox 保留原标签；这些是基线数字，当前统计见补充版文件。每次训练保存 `augmentation_policy.json`，记录变换、源码哈希和版本。保留标签不保证一两个像素的目标能够可靠识别。

数据中的渲染光照、双目位姿、训练专属噪声/模糊图继续使用。当前配置在 `config/yolo11n_people_finetune.yaml`，原配置保留在 `config/yolo11n_detect.yaml`。

## 6. 验证、最终测试与预测

待当前最佳权重实际生成后，用明确的新数据入口评估。以下命令在工程根目录运行：

```powershell
# val 分析漏检/误检、选择阈值；使用独立手工诊断名称
& .\.venv\Scripts\python.exe .\train.py --mode val --weights .\runs\wallpaper_yolo11n_people_v2_20261008\weights\best.pt --data .\dataset_people_v2\data.yaml --config .\config\yolo11n_people_finetune.yaml --device 0 --name people_v2_review
# 保存实拍预测叠加图和带置信度的标签
& .\.venv\Scripts\python.exe .\train.py --mode predict --weights .\runs\wallpaper_yolo11n_people_v2_20261008\weights\best.pt --data .\dataset_people_v2\data.yaml --config .\config\yolo11n_people_finetune.yaml --source "自己的实拍目录" --conf 0.25 --device 0 --name people_v2_real
```

管理脚本正常完成时已经执行一次 test，直接读其指标。如果最佳权重已生成，但自动测试因环境问题未完成，修复后可用以下命令补做；不要用 test 反复挑模型或阈值：

```powershell
& .\.venv\Scripts\python.exe .\train.py --mode test --weights .\runs\wallpaper_yolo11n_people_v2_20261008\weights\best.pt --data .\dataset_people_v2\data.yaml --config .\config\yolo11n_people_finetune.yaml --device 0 --name people_v2_final_recovery
```

评估/预测/导出都会核对模型为 Detect、类别名称严格匹配这三类。`--conf` 只控制预测阈值，应按 val 误检/漏检取舍选择。`metrics_summary.json` 记录真实整体指标及每类 precision、recall、AP50、AP50–95；未返回 AP 的类别标为未评估。

合成 test 和 GS130W 实拍 test 分开报告。原壁纸纹理在合成各集合重复出现，不能将合成成绩当成对新楼宇/实拍的泛化精度。实拍需覆盖打印反光、曝光、距离、畸变和运动模糊；按连续采集时段/路段整体划分，不能把相邻帧或左右眼分到不同集合。极小目标应允许 unknown，再通过云台对准或靠近重拍。

## 7. 新微调与中断续训

本次补充数据使用 `--mode train --model 旧best.pt`，建立新优化器和新轮次。**不要对旧第 41 轮 `last.pt` 使用 resume 来更换数据集。**

如果当前人物补充版任务中断，且同一数据、原运行路径和增强策略保持不变，确认没有任务运行后可真正续训：

```powershell
& .\.venv\Scripts\python.exe .\train.py --mode resume --weights .\runs\wallpaper_yolo11n_people_v2_20261008\weights\last.pt --data .\dataset_people_v2\data.yaml --config .\config\yolo11n_people_finetune.yaml --device 0
```

真正续训使用 `resume=True` 恢复 epoch、优化器、学习率状态与原参数，CLI 中的 epochs/batch/imgsz/增强设置不会替换旧设置。检查点必须有可恢复优化器，原运行 YAML 存在，数据清单与来源指纹一致，原 `augmentation_policy.json` 匹配当前 Trainer 的版本及源码哈希。

此直接续训命令不重新启动管理脚本，因此不会自动更新其进度页或执行最终 test；以续训控制台和运行目录为准。改变数据/训练设置应使用新的运行名与新微调。移动工程不会自动修改旧检查点中的绝对路径，精确续训需恢复原路径。

## 8. 参考 ONNX 与 RDK X5

```powershell
& .\.venv\Scripts\python.exe -m pip install -r .\requirements-export.txt
& .\.venv\Scripts\python.exe .\train.py --mode export-reference --weights .\runs\wallpaper_yolo11n_people_v2_20261008\weights\best.pt --data .\dataset_people_v2\data.yaml --config .\config\yolo11n_people_finetune.yaml --device cpu
```

导出保存在 `weights/export_reference/`，固定 batch 1、640 输入、opset 11，无内置 NMS、动态维度或图简化。**这个 ONNX 不是 RDK X5 的 BPU 可执行模型。** 需按实际 RDK OS/工具链的官方 YOLO11 Detect 示例处理输出头、NV12/色彩与缩放预处理、框解码和 NMS，再量化、检查精度及板端时延。不要使用 Pose 后处理解析三类 Detect。

`dataset_people_v2/calibration.txt` 只含训练来源的原图候选，不含 val/test，仍需匹配实际量化前处理。楼号/楼名语义识别独立处理。火焰中心用于云台跟踪，喷水、人员避让、灭火完成由任务状态机结合执行条件判断。

## 9. VS Code 与代码自检

选择项目 `.venv` 解释器。调试面板的“04 新增人物数据检查”显式选择当前数据；原“01 数据检查”“02 CPU 单轮小样本测试”“03 GPU 完整训练”保留默认基线。当前微调使用终端的 `06_train_people_v2.cmd` 或第 5 节命令。

```powershell
& .\.venv\Scripts\python.exe -m unittest discover -s .\tests -p test_training_entry.py -v
& .\.venv\Scripts\python.exe -m unittest discover -s .\tests -p test_training_job.py -v
& .\.venv\Scripts\python.exe .\tools\check_environment.py
```

单元测试检查数据/配置分流、路径可移植性、原 YAML 不改写、禁止 Pose/Seg、续训条件和停止/完成状态；不测识别精度，也不替代真实训练及实拍验证。
