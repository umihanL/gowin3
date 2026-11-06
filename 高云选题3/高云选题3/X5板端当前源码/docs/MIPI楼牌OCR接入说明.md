# MIPI 楼牌 OCR 接入说明

`mipi_building_ocr` 是独立 ROS 2 节点，复用当前官方 MIPI 相机，不会启动第二个驱动。模型为已准备好的 PP-OCRv4／RapidOCR ONNX，使用 X5 CPU，不占用 YOLO 的 BPU 推理实例。

## 工作方式

1. 默认仅在新鲜的 `inspect`／`fire` 任务、车辆没有通行许可、无红绿灯任务且云台已到观察姿态时订阅物理左眼 `/image_right_raw`。
2. 缓存最多 8 帧原始图和 24 份 YOLO 检测元数据，选取原始图与 YOLO 时间戳完全一致的最新一帧。它不会用“最接近的另一个画面”冒充同一帧。
3. 使用和 YOLO 一致的 1280×1088 去畸变几何，再执行 OCR。默认每秒最多提交一次，只有一个工作线程和一个在途任务，不堆积推理。
4. 将识别到的文字与 `config/buildings.json` 的楼号／名称精确匹配；`1` 不会误匹配成 `11`。默认需要 3 个不同时间的稳定位置观测才能确认。
5. 看不到文字、存在不同楼栋的冲突文字、任务／观察姿态变化，都会撤销确认。结果超过原始采集时间 3 秒失效。

`/scene/ocr` 中的 `stamp` 始终是原始采集时间，`processed_stamp` 才是处理完成时间。`matched_detection` 保存与该楼牌同帧的 YOLO 检测。结果中明确设置 `object_building_association_verified=false`：看见某栋楼牌，不足以证明整幅图片里的所有人物、火焰都属于这一栋楼。

由于 OCR 比 YOLO 慢，场景节点应将“最近确认的楼牌观测”与“最新画面的目标数量”分别保留。不能拿旧楼牌的序号替换成最新 YOLO 序号，也不能刷新旧楼牌的采集时间。问答可以说“最近确认看到 3 号楼；当前画面检测到 2 处火焰”，不能在尚未验证楼面归属时说“3 号楼有 2 处火焰”。

## 尚未安装舵机时的桌面联调

`fixed_camera_preview=true` 是明确的固定相机联调选项，允许在 `inspect`／`fire` 且无通行许可时读取楼牌，不需要伪造 `gimbal_ready`。输出同时标记 `unverified_pose=true`，仍不允许把全画面目标归属到楼栋。它不修改任务控制、行驶、云台、喷水的任何权限。

正常实车使用保持 `fixed_camera_preview=false`、`require_settled_pose=true`。如果独立设置 `require_settled_pose=false`，结果同样标记姿态未验证。

## 关键参数

| 参数 | 默认值／路径 |
|---|---|
| `buildings_config` | `config/buildings.json` 的绝对路径 |
| `ocr_models` | `models/ocr` 的绝对路径 |
| `lens_intrinsics` | `config/gs130w_intrinsics_20261010.json` 的绝对路径 |
| `source_topic` | `/image_right_raw`，已确认的物理左眼 |
| `ocr_threads` | 1 |
| `ocr_interval_s` | 1.0 |
| `confirmation_hits` | 3 |
| `max_input_age_s` | 0.5 |
| `max_result_age_s` | 3.0 |
| `lens_enabled` / `lens_alpha` | `true` / `0.0`，需与 YOLO 保持一致 |
| `input_rotation` | 90.0，与当前官方相机一致 |

诊断话题 `/diagnostics/building_ocr` 包含模型是否可用、当前是否启用、原始图订阅是否存在、推理耗时、提交和处理数量、错误数量。`reader_available=true` 只代表模型加载成功；必须看到 `building.confirmed=true` 才代表某个实际楼牌已通过多帧确认。

## 源码和测试

- `rescue_vision/mipi_building_ocr_node.py`：ROS 订阅、单工作线程、去畸变和 OCR 模型调用。
- `rescue_vision/mipi_building_ocr.py`：任务门控、同帧关联、确认与失效策略。
- `rescue_vision/building_ocr.py`：模型哈希校验、OCR 推理、名单匹配。
- `tests/test_mipi_building_ocr.py`：同帧与错帧、缓存上限、结果过期、姿态切换、冲突文字和固定相机联调测试。

这些测试验证软件规则。正式打印楼牌在实际距离、角度和照明下的正确率仍需实拍验收。
