# X5 本地边缘 AI：部署、使用与学习

电脑工程：`D:\工程仿真\FPGA赛道_小车仿真\RDKX5_消防小车系统\ros2_ws`。板端工程：`/root/rescue/ros2_ws`。本页对应 2026-10-11 的本地语言模型接入。

这套系统使用 **ROS 2 节点组织任务**。YOLO 负责从相机画面检测火焰、普通人物和消防员；本地语言模型负责中文问题理解与简短知识解释。语言模型不会直接打开串口、舵机或水泵。

## 1. 选了什么模型，放在哪里

| 部分 | 当前选择 | 运行位置与职责 |
|---|---|---|
| 视觉检测 | 项目已有 YOLO 三类检测模型 | X5 BPU；处理真实相机图像 |
| 中文语言模型 | 官方 `Qwen2.5-0.5B-Instruct`，`Q4_K_M` GGUF 量化 | X5 CPU；读取问题和结构化场景信息 |
| 模型运行器 | `llama.cpp` 的 `llama-server` | 独立进程，只监听 `127.0.0.1:8088` |
| ROS 接入 | `task_scene` | 调用模型、检查输出、组合实时事实、发布答案 |
| 离线语音 | Vosk 中文识别与 espeak-ng 播报 | 已有接口；USB 麦克风和音箱尚未开启 |

“官方”指权重来自 **Qwen 官方模型仓库**。这个 GGUF 版本运行在 CPU 上，不是已经转换成地瓜 X5 BPU 格式的语言模型。YOLO 继续使用 BPU；两者仍共享内存和部分 CPU 资源，需要一起测延迟与视觉帧率。

模型约 491 MB，已经下载、SHA-256 校验并传到板端。源码和许可证一起保留。运行器已用板上 GCC 11.2 原生编译并启动，健康检查通过。预编译程序要求 `glibc 2.38`，不适配本机，因此采用源码编译，未升级操作系统或替换系统 glibc。

| 文件或目录 | 用来做什么 |
|---|---|
| `models/llm/qwen2.5-0.5b-instruct-q4_k_m.gguf` | 语言模型权重 |
| `models/llm/provenance.json` | 官方来源、固定版本、文件大小和 SHA-256 |
| `models/llm/LICENSE`、`UPSTREAM_MODEL_CARD.md` | 许可证与上游说明 |
| `vendor/llama_cpp/llama_cpp_source.tar.gz` | 固定版本的运行器源码 |
| `tools/build_edge_llm.sh` | 校验固定源码包，并在 X5 原生编译运行器 |
| 板端 `/root/rescue/edge_ai/build/bin/llama-server` | 编译后的模型服务程序 |
| `tools/run_edge_llm.sh` | 模型启动参数：CPU 两线程、单并发、本机接口 |
| `tools/systemd/rescue-edge-llm.service` | 模型后台服务定义 |
| `config/edge_dialogue.local.json` | 启用真实本地模型的 ROS 配置 |
| `config/edge_dialogue.template.json` | 默认关闭模型、仅运行确定性规则的配置 |
| `src/rescue_vision/rescue_vision/task_scene.py` | 问句分类、模型请求、结果检查和回答组织 |
| `src/rescue_vision/rescue_vision/task_scene_node.py` | ROS 消息、异步等待和场景证据管理 |

官方来源：[Qwen GGUF 权重](https://huggingface.co/Qwen/Qwen2.5-0.5B-Instruct-GGUF)、[llama.cpp](https://github.com/ggml-org/llama.cpp)。准确版本以 `provenance.json` 为准。

选型时还核对了地瓜官方 [RDK X5 Model Zoo](https://github.com/D-Robotics/rdk_model_zoo/tree/rdk_x5) 和 [hobot_llm](https://github.com/D-Robotics/hobot_llm)。前者适合继续参考 YOLO、OCR 等视觉部署；后者当前说明以 X3 的 BLOOM 演示为主，不能据此认定可直接替换本机 X5 的 BPU 环境。此次使用官方 Qwen 权重与通用 ARM64 CPU 运行器，保留已经运行的 X5 视觉栈。0.5B 是资源与延迟优先的初始选择，复杂推理和长篇对话能力有限；是否升级更大模型应依据实测任务问答质量决定。

## 2. ROS 节点到底怎样连接

ROS 节点可以理解成持续运行的小程序；话题是这些程序交换消息的通道。模型文件本身不是节点：`task_scene` 节点向 `llama-server` 请求计算，再把答案发布给其他节点。

```mermaid
flowchart TD
    CAM[GS130W 相机] --> YOLO[fast_mipi_perception：BPU YOLO]
    YOLO -->|/perception/detections| SCENE[task_scene：当前场景证据]
    MISSION[任务、泵指令状态] --> SCENE
    TEXT[文字命令 / 麦克风转写] --> ROUTER[task_interaction：明确任务语法]
    ROUTER -->|控制请求| CONTROL[mission / navigation / 执行器联锁]
    ROUTER -->|/scene/question：只读问答| SCENE
    SCENE -->|精确事实问题| RULE[读取最新事实，立即回答]
    SCENE -->|双目原理等已审定知识| REF[本地技术参考]
    SCENE -->|分析、解释、聊天| WORKER[独立问答线程]
    WORKER -->|本机 HTTP| LLM[llama-server：Qwen CPU 推理]
    LLM --> CHECK[检查受限 JSON；场景数字重新读取最新事实]
    RULE --> ANSWER[/scene/answer]
    REF --> ANSWER
    CHECK --> ANSWER
    ANSWER --> USER[终端 / 可接入的上位机]
    CHECK --> SPEECH[/speech/say：音频启用后播报]
    RULE --> SPEECH
```

语言模型推理可能耗时数秒甚至更久。等待发生在独立工作线程，`/scene/report` 仍按 0.25 秒周期更新；运动和停止喷水不等待语言模型。最多接受一个正在计算的模型问题，忙时返回 `dialogue_busy`，避免不断积累旧问题。即时事实问题仍可回答。

这不是把摄像头画面直接传给多模态大模型。Qwen 接收的是经过 YOLO 和任务节点整理的少量文字事实。当前没有给它直接看图的能力，也没有把未接通的 OCR 当成已确认楼号。

## 3. 三种回答来源

**实时事实回答**例如“现在有几个人”“有没有消防员”“当前楼号是什么”“灭火完成了吗”。程序直接读取最新有效检测和任务记录，返回 `backend: structured_rules`。即使模型服务关闭，也保留这类回答。

**语言模型回答**例如“什么是机器视觉”“请分析当前场景”“你好”。实际调用本地模型，成功时返回 `backend: edge_llm_generate`。

**本地技术参考**例如“解释双目测距原理”。返回 `backend: domain_reference`、`response_scope: domain_reference`，并显示“本地技术参考”。这里保存了经过审定的校正、匹配、视差与 `Z=fB/d` 解释，关闭模型也可查询。这不是模型自由生成的答案。实测小模型在公式解释上不够可靠，所以确定的专业知识采用明确来源的参考。“当前双目距离是多少”会返回没有已核验读数，参考公式不会冒充实际测距。

模型输出只能包含 `intent`、`reply`、`observation_hint`，并按问句类型分别约束。场景类只允许空 `reply`，程序在模型返回时重新读取最新画面，填写事实，再把受限的观察建议变成文字。通用问答的 `reply` 最多 60 字，触及上限时仅保留完整句，没有完整句就回退。一般知识回答标注“通用知识解释（不代表当前检测或动作结果）”。建议不会自动转动云台或控制底盘。

需要特别理解四个区别：

- 检测到普通人物图案，不等于确认有人受困。
- 当前画面的人数，不等于整栋楼或整个地图的人数。
- 水泵控制指令开启，不等于已有水流传感器证明实际出水。
- 喷水结束或没有检测到火焰，不等于已经验证扑灭；打印火焰喷水后图案也不会自动消失。

入站问题仍要求 3 秒以内的新鲜时间戳；模型计算使用独立回答期限。当前配置 HTTP 超时 75 秒、回答期限 80 秒。模型超时、格式不合法或不可用时回退规则，并附 `model_error`；`general_chat_available` 只在本次真实生成成功时为 true。

目前按单次问题处理，不保证跨多轮代词记忆。问“那个人怎么样”时，系统没有可靠证据把“那个人”长期绑定到某个居民；应明确说明要询问的对象或任务。

## 4. 从终端提问

下面命令都在 **VS Code SSH 连接 X5 后的终端**执行。先准备环境：

```bash
cd /root/rescue/ros2_ws
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
source env.sh
```

相机、视觉和任务栈已运行时，可先切换到观察模式，再逐条提问：

```bash
python tools/task_command.py "开启识别"
python tools/task_command.py "现在有几个人" --wait-answer
python tools/task_command.py "请分析当前场景" --wait-answer --answer-timeout 90
python tools/task_command.py "什么是机器视觉" --wait-answer --answer-timeout 90
python tools/task_command.py "解释双目测距原理" --wait-answer --answer-timeout 90
```

`--wait-answer` 会先打印 `/interaction/result` 的请求回执，再打印相同 `command_id` 的 `/scene/answer`。只有回执里的 `accepted: true`，表示请求被接收；它不是模型的实际答案。看第二份输出的 `text`、`backend`、`model_error` 和 `facts` 才能知道怎么回答、用了什么证据。

“开启场景理解”会切换到观察并请求一份即时报告；要明确触发本地语言模型，使用“请分析当前场景”或知识解释类问句。单独提问不会自行开启 YOLO，观察画面过期时应如实回答未知。

持续查看答案可另开已加载环境的终端：

```bash
ros2 topic echo /scene/answer
```

当前 `http://192.168.127.10:8086/` 是 YOLO 图像预览，暂不是完整聊天页面。模型端口 `8088` 仅监听 X5 本机，电脑不能直接用浏览器打开它。旧通用栈网页也曾使用 8088，不能与当前模型服务同时占用这个端口；现有 `18088` 配置是另一套上位机测试入口。

## 5. 模型服务怎样启动和停止

模型服务已安装并运行。日常操作先看状态，不重复启动多个模型：

```bash
systemctl status rescue-edge-llm --no-pager
curl --max-time 5 http://127.0.0.1:8088/health
```

服务文件若已安装，启动和停止分别是：

```bash
systemctl start rescue-edge-llm
```

```bash
systemctl stop rescue-edge-llm
```

停止语言模型不等于停止相机或任务栈。正在计算的问题可能等待超时后回退；之后精确事实问答仍可工作。查看启动错误使用：

```bash
journalctl -u rescue-edge-llm -n 60 --no-pager
```

从头重建部署时，源码压缩包需要放在板端 `/root/rescue/edge_ai/downloads/llama_cpp_source.tar.gz`。已提供的脚本先核对 SHA-256，再用现有 `cmake`、`g++` 原生编译，不下载或替换系统 glibc：

```bash
cd /root/rescue/ros2_ws
bash tools/build_edge_llm.sh
```

编译可能需要较长时间；已在编译时不要再启动第二份。脚本完成且打印运行器版本后，确认模型权重已校验，再安装项目提供的服务文件：

```bash
cd /root/rescue/ros2_ws
install -m 644 tools/systemd/rescue-edge-llm.service /etc/systemd/system/rescue-edge-llm.service
systemctl daemon-reload
systemctl start rescue-edge-llm
```

此次没有执行 `systemctl enable`，未设置开机自动运行。模型服务使用 CPU 两线程、单并发、1536 token 上下文、64 MiB 提示缓存，并设置较低调度优先级及 1500 MiB 服务内存上限；这些参数不是实时延迟的保证。启动或重启任务服务后，需等待 ROS 节点完成初始化再提问。

任务栈的启动入口仍是 `tools/run_task_modes.sh`。它优先加载 `config/edge_dialogue.local.json`。如要前台调试，先停止已有 `rescue-task-modes`，保留相机与高速 YOLO，再运行：

```bash
cd /root/rescue/ros2_ws
systemctl stop rescue-task-modes
bash tools/run_task_modes.sh
```

这是持续运行的前台程序。另开终端提问，按 Ctrl+C 退出。切回纯规则模式可在前台启动时明确指定：

```bash
RESCUE_EDGE_DIALOGUE_CONFIG="$PWD/config/edge_dialogue.template.json" bash tools/run_task_modes.sh
```

不要和旧 `rescue.launch.py`、旧 `scene` 或另一个 `task_scene` 同时运行。

## 6. 语音、模型与实车之间还差什么

USB 麦克风和音箱尚未开启。当前先用文字命令确认模型和 ROS 链路；配置实际音频设备后，Vosk 的转写可以走同一入口，回答可经 espeak-ng 播报。音频节点会截断过时队列并避免播报回声进入新一轮识别。模型回答限制为短中文，ROS 播报文字最多 350 字。

本地模型接入没有启用底盘、云台或 MOS 实物输出，也没有替代双目标定、喷嘴落点标定、UART 回执和实车停车验证。要演示“AI 分析→人工明确选择火焰→独立开启喷水”，应按 [四种任务教程](四种任务运行节点_从启动到语音控制.md) 逐步执行。

## 7. 验收与学习顺序

先观察一条 YOLO 检测怎样进入 `/scene/report`，再比较规则回答与模型回答，最后理解 `task_scene.py` 中 JSON 检查和 `task_scene_node.py` 中异步等待。暂时不用先学习训练大语言模型。

最终 ROS 包已完成 colcon 编译，安装文件与源文件一致。**39 项针对性测试通过**（35 项纯测试＋4 项隔离 ROS 测试），覆盖慢模型、事实刷新、超时、去重、控制通道隔离和技术参考。记录见 [部署后测试](../logs/edge_ai_20261011/deployed_tests_final.log)。

**2026-10-11 已在 X5 上完成真实联合验收**：相机、三类 YOLO、任务节点和本地模型同时运行，硬件输出保持关闭。两个模型问题都返回 `edge_llm_generate`；模型运行期间，停止喷水指令和人数查询正常响应，场景报告最大间隔约 0.27 秒。

| 实测项目 | 本轮结果 |
|---|---:|
| “什么是机器视觉”——真实模型生成 | 31.11 秒 |
| “请分析当前场景”——真实模型参与建议选择，事实实时刷新 | 27.00 秒 |
| “解释双目测距原理”——本地技术参考 | 0.218 秒 |
| 模型忙时的人数查询 | 0.026 秒 |
| 模型忙时停止喷水指令回执 | 0.021 秒；不代表实体水泵响应时间 |
| 模型空闲，YOLO 处理帧率 | 平均约 15.7～15.8 FPS |
| 模型回答期间，YOLO 处理帧率 | 平均约 11.3～11.5 FPS |

这说明 **本地语言模型已经能用，但它仍是较慢的 CPU 问答助手**。适合停车观察时按需分析；持续检测、人数统计和控制通道不等待它。小模型的专业知识可能不准确，已知公式采用明确来源的本地参考。两次模型请求不能代表全部问句的成功率、最坏耗时或比赛现场性能。

联合测试全程保持 1280×1088 图像、640×640 YOLO 输入、JPEG 质量 90 和三类检测。实测结果见 [完整证据](../logs/edge_ai_20261011/live_acceptance.json)，命令输出见 [实机日志](../logs/edge_ai_20261011/live_acceptance.log)。`windows_model_probe.json` 仅用于检查同一权重的输出格式，不用作 X5 性能证据。当前服务为 `rescue-edge-llm`，模型空闲时不持续发起推理；任务最终停留在 `inspect` 观察模式。

增量代码包位于 `logs/edge_ai_20261011/deploy_code.zip`，包含本次代码、配置、测试与文档，不包含 GGUF 权重、运行器源码包或完整 ROS 工作区。电脑与板端都保留了本次验收记录。初次编译耗时较长，日常启动不需要重新编译。
