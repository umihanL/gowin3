# 任务：把「消防救援演示台」重做成工业 HMI 风格（只改视觉层，不改业务逻辑）

> 上一轮"工业风"改版（截图见 `reports/industrial_ui_20261012/overview.png`）方向不对。它本质上还是深色 SaaS 后台：蓝灰底、统一卡片、亮蓝主按钮、粉彩色、emoji 式图标（◌ ↗ ⌖ ✧ ▤ ◉），满屏灰色小字说明，看起来像网页表单，不像设备操作屏。
> 这一次请严格按下面的**具体规格**执行，不要自由发挥成"深色后台"，也不要做成"科技蓝发光大屏"。

---

## 0. 工程背景（先读）

- **界面本体**：`ros2_ws/web/presentation.html`、`presentation.css`、`presentation.js`，由 `ros2_ws/tools/presentation_server.py` 提供。离线演示时，这三个文件被打包进 `desktop/dist/runtime`；连接真车时，由 X5 板端的 8090 服务提供同一套文件。
- **Windows 外壳**：`desktop/src/App.cs`（WinForms + WebView2）。顶部有一条原生工具栏：地址框、连接 X5、离线演示、重连、说明。
- **观看端**：Windows 上的 WebView2（Chromium）。扣除原生工具栏后，常用视口为 **1366×680、1093×550、1920×1000**。
- **使用场景**：比赛答辩现场演示。评委要在几秒内看懂三件事：系统处于什么状态、看到了什么、哪些条件已经满足。

---

## 1. 硬性约束（违反任何一条都算失败）

### 1.1 改动范围

| 文件 | 允许做什么 |
|---|---|
| `presentation.css` | 可以整体重写 |
| `presentation.html` | 可以调整结构、静态文案和图标。**必须保留**：所有 id、所有 `data-*` 属性、表单控件的 `type` / `role` / `aria-*`、三个 `<dialog>` 的结构 |
| `presentation.js` | 只允许以下 4 类改动：① `drawFrame()` 的绘制样式；② 在 `render*()` 和 `showFrameState()` 里，**基于已经算好的变量**追加纯展示用的 `dataset.state`、class 或 CSS 变量（用 `el.style.setProperty`）；③ 删除 JS 字符串里纯装饰的符号（● ◌ ✓ · ↗ → 等），改由 CSS 绘制，文字部分保持不变；④ 修改 `mobileTabs` 的 class 和文案 |
| `desktop/src/App.cs` | 只改颜色、字体、工具栏高度等外观常量。连接、导航、同源校验、后端进程逻辑都不能动 |
| 其他文件 | 一律不改。尤其不能改 `presentation_server.py` 和任何 `/api/*` 接口 |

JS 中下列函数和逻辑**不能改变语义**：
- `request`、`sendCommand`、`dangerKind`、`confirmAction`、`selectionIdentity`
- `renderControls` 中的启用 / 禁用条件
- 令牌逻辑、`pollState`
- `pollFrame`、`unpack`、`showFrameState` 中的帧协议和过期判断
- 导览步骤

### 1.2 CSP 限制（见 `presentation_server.py:563`）

当前策略为 `default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob: data:`，因此：
- **禁止**以下写法：`style="..."` 属性（包括 JS 用 innerHTML 拼出来的字符串）、`<style>` 标签、内联 `<script>`、`onclick=` 这类内联事件。
- 动态数值只能用 `el.style.setProperty('--x', v)` 设置，CSSOM 不受 CSP 限制。
- **禁止**外链字体、CDN 和网络图片。服务器只放行 `/presentation.html`、`/presentation.css`、`/presentation.js` 三个静态文件（见 `presentation_server.py:520-523`），**不要新增任何静态文件**。
- 图标一律用 HTML 内联 `<svg>`，或 CSS 中的 `url("data:image/svg+xml,...")`。
- 必须完全离线可用。

### 1.3 JS 绑定（改 HTML/CSS 时必须遵守）

- **JS 切换的状态类**：`.active .hidden .on .selected .pass .warn .live .demo .offline .error .ok .listening .fresh .stale .missing .user`
- **JS 生成的结构类**：`.badge .frame-badge .connection-dot .target-pill .gate .scene-fact .system-fact .message .message-label .message-meta .timeline-entry .event-head .receipt .receipt-title .topic-status .empty-note .text-btn .toast .mobile-tabs .view`
- **由 JS 的 `text()` / `html()` 写入内容的元素，内部不能放装饰子元素**，否则会被 textContent 冲掉。装饰一律用 `::before` / `::after` 实现。
- **className 会被 JS 整体覆盖的元素**：`#connectionBadge`（由 `badge()` 覆盖）、`#railDot`、`#frameBadge`、`#toast`。给它们加额外 class 会丢失，请改用 id 选择器或 `data-*` 属性写样式。
- `#sharedVisionPanel` 会被 JS 在 `#cameraSlot-overview`、`#cameraSlot-road`、`#cameraSlot-fire` 之间移动，它的样式不能依赖所在的页面。

### 1.4 诚实性信息不能删

本系统非常强调"展示不等于实车结果"。以下内容必须**常驻可见**，不能只放在 hover 提示里：
- 模拟 / 离线横幅
- 视频 OSD 上的 SIM / LIVE 标识
- `DRY-RUN` 执行器标签
- "提交 ≠ 执行完成"
- "云台对准 ≠ 水流命中"

其余解释性长句可以压缩成单行注记，超长部分用 `text-overflow: ellipsis` 截断，并用 `title` 补全全文。
模拟数据**不能**被设计得看起来像实车数据。

### 1.5 布局硬指标（沿用上一版的验收标准）

- 在 1366×680、1093×550、1920×1000 三个视口下，5 个页面都不能出现整页滚动条，也不能裁切内容。长列表只在面板内部滚动。
- 所有文字不小于 12px；12–14px 文字的对比度不低于 4.5:1。
- 保留现有的断点思路：`max-height: 850px / 680px`、`max-width: 1180px / 1000px`，以及 `≥1700×900` 时放大。
- 保留 `prefers-reduced-motion` 支持和 `:focus-visible` 焦点框。

---

## 2. 设计方向

**一句话：车载消防设备的操作屏。**

参照西门子 WinCC Unified、倍福 TwinCAT HMI 以及 ISA-101 高性能 HMI 的克制风格：
- 石墨灰做底；
- 颜色只用来表示状态；
- 数字做成仪表读数；
- 标题做成设备铭牌；
- 危险操作做成实体急停按钮的样子。

### 六条原则

1. **灰阶为底，颜色即状态。** 画面 90% 是中性灰。绿、琥珀、红、青四种颜色只出现在指示灯、状态标签和异常读数上，不拿来装饰。正常态尽量不染色，只点亮指示灯（ISA-101 原则：越正常越安静）。
2. **三级材质。** 从暗到亮依次是：凹槽（视频、读数窗、日志、输入框）→ 面板（主体）→ 铭牌（面板标题条）。层级靠明度差和 1px 线区分，不靠阴影和大圆角。
3. **仪表化读数。** 关键数值用 DIN 字体、等宽数字、大字号；单位小一号，颜色更暗。状态必须同时用指示灯和文字表达，不能只靠颜色。
4. **直角与细线。** 圆角只用 0–2px，线宽 1px。面板不加投影，不用渐变（指示灯发光除外）。
5. **中英双语铭牌。** 中文是主标签，英文大写代号是副标签（例如"车载视觉 · VISION FEED"），像设备面板上的丝印。
6. **密而不乱。** 按 8px 栅格排布，面板边缘严格对齐；数字右对齐，使用等宽数字。

### 整体骨架（以总览页为例）

```
┌ 系统栏（原生 WinForms，44px，bg-0）：品牌 │ 地址 │ 连接X5 │ 离线演示 │ 重连 ......... 连接状态 │ 说明 ┐
├ 导航栏 ┬ 顶栏：标题 │ 副标题 │ 英文副标题               [状态标签][解锁][■ 急停：停车·停水] ┤
│ 01 ▌  ├ 模拟横幅：▚▚▚ SIM │ 模拟演示：画面、传感器……                                       ┤
│ 02    ├ 仪表条（一整条凹槽）：MODE │ INFERENCE │ TARGETS │ MOTION                         ┤
│ 03    ├ 取景器（视频 + 四角取景框 + OSD）               ┬ 任务控制（2×2 模式键、语义导航） ┤
│ 04    │                                                 │                                  │
│ 05    │                                                 │                                  │
│ 连接灯 ├─────────────────────────────────────────────────┴──────────────────────────────────┤
└───────┴ 状态栏（24px，等宽字）：SYS ● │ RDK X5 本地计算 │ ........................ │ 同步 22:34:26 ┘
```

---

## 3. 设计令牌（直接使用这些值）

```css
:root {
  color-scheme: dark;

  /* 中性灰阶：由暗到亮 */
  --well:   #070809;  /* 凹槽：视频、读数窗、日志、输入框 */
  --bg-0:   #0B0D10;  /* 应用底色、导航栏、状态栏 */
  --bg-1:   #121519;  /* 面板主体 */
  --bg-2:   #191D22;  /* 铭牌标题条、表头 */
  --bg-3:   #22272E;  /* 按键常态 */
  --bg-4:   #2B313A;  /* 按键悬停 */
  --line-1: #232930;  /* 面板内发丝分隔线 */
  --line-2: #313841;  /* 面板外框 */
  --line-3: #48515C;  /* 按键边框、强调框 */
  --tx-1:   #E6E9ED;  /* 主文字 */
  --tx-2:   #A9B2BC;  /* 次文字 */
  --tx-3:   #7F8994;  /* 辅助文字；12px 时只能放在 well / bg-0 / bg-1 / bg-2 上 */

  /* 状态色（IEC 60073 语义），每组依次为：文字/灯、底、线 */
  --ok:    #3DDC84; --ok-bg:    #0D2619; --ok-line:    #1E6B42;  /* 正常 / 满足 / 新鲜 / 运行 */
  --warn:  #FFB224; --warn-bg:  #2B2008; --warn-line:  #7A5A12;  /* 注意 / 待确认 / 过期 / 模拟 */
  --alarm: #FF4D4F; --alarm-bg: #330E10; --alarm-line: #8F2A2D;  /* 报警 / 故障 / 离线 / 拒绝 */
  --sel:   #3BC4E6; --sel-bg:   #0A2630; --sel-line:   #1F7690;  /* 选中 / 当前步骤 / 焦点 / 主操作 */
  --lamp-off: #3A414A;                                            /* 熄灭的指示灯 */

  /* 安全识别色：只用于急停按钮和警示斜纹 */
  --estop: #C8102E;
  --hazard-yellow: #FFC400;
  --hazard: repeating-linear-gradient(-45deg, var(--hazard-yellow) 0 6px, #0B0D10 6px 12px);

  /* 检测类别色：只用于画面检测框和图例，不得在 UI 状态上复用 */
  --det-fire: #FF6A2B; --det-person: #5B8CFF; --det-firefighter: #C6F135;

  /* 字体 */
  --font-ui:   "Microsoft YaHei UI", "Microsoft YaHei", "PingFang SC", "Noto Sans CJK SC", sans-serif;
  --font-num:  "Bahnschrift", "DIN Alternate", "Roboto Condensed", "Segoe UI", "Microsoft YaHei UI", sans-serif;
  --font-mono: "Cascadia Mono", Consolas, "Microsoft YaHei UI", monospace;

  --r: 2px;  /* 全局唯一圆角值 */
}
```

### 字体用法

| 字体变量 | 用在哪里 |
|---|---|
| `--font-ui` | 所有中文 |
| `--font-num` | 所有数字读数、英文代号、编号（01–05、S1–S4）、单位。同时加 `font-variant-numeric: tabular-nums`。Bahnschrift 是 Windows 10/11 自带的 DIN 字体，本机已确认安装 |
| `--font-mono` | ROS 话题名、ID、时间戳、分辨率与帧号、JSON |

英文代号的统一样式：`font-family: var(--font-num); font-size: 12px; letter-spacing: .08em; text-transform: uppercase; color: var(--tx-3)`。

### 字号

| 用途 | 字号 |
|---|---|
| 大读数 | 28–30px；视口高 ≤680 时 22–24px；≥1700×900 时 34px |
| 中读数 | 20–22px |
| 页面标题 | 18px，字重 700 |
| 面板标题 | 14px，字重 700 |
| 正文 | 13px |
| 辅助文字 | 12px |

不要使用 300 这类细字重。

### 间距

- 基础间距只用 4 / 8 / 12 / 16px。
- 面板间距 `--gap`：默认 10px；视口高 ≤850 时 8px；≥1700×900 时 12px。
- 面板内边距 12px。

---

## 4. 通用组件

### 4.1 面板 `.panel`
`bg-1` 底色，1px `line-2` 边框，圆角 `--r`，不加阴影。

### 4.2 铭牌标题条 `.panel-head`

- 高 36px（视口高 ≤680 时 32px），`bg-2` 底色，底边 1px `line-2`。
- 从左到右依次是：
  1. 编号块 `.section-no`：最小宽 22px，可随内容变宽；高 22px；`bg-0` 底色，1px `line-3` 边框；`--font-num` 12px，`tx-2` 色。
  2. 中文标题：14px，字重 700。
  3. 新增 `<span class="panel-code">英文代号</span>`：视口宽 ≤1180 时隐藏。
- 右侧放状态标签或开关。
- **编号规则**：视频面板的编号块固定写 `CAM`（因为它会在三个页面之间移动）；其余面板每页从 01 开始编号；证据页原本没有编号的面板也要补上。
- **各面板的英文代号**：

| 中文标题 | 英文代号 |
|---|---|
| 车载视觉 | `VISION FEED` |
| 任务控制 | `MISSION CONTROL` |
| 道路与信号灯 | `ROAD / SIGNAL` |
| 楼牌与场景 | `BUILDING ID` |
| 目标锁定与灭火 | `TARGET & SUPPRESSION` |
| 车载语音 | `ONBOARD VOICE` |
| 任务与场景对话 | `COMMS LOG` |
| 当前场景证据 | `SCENE FACTS` |
| 指令与反馈时间线 | `EVENT LOG` |
| 链路与运行状态 | `LINK STATUS` |
| 最近任务回执 | `RECEIPTS` |
| ROS 2 证据源 | `ROS 2 TOPICS` |

### 4.3 状态标签

**`.badge`（状态类标签）**
- 高 22px，左右内边距 8px，圆角 `--r`，字号 12px。
- 用 `::before` 画一个 6×6 的方形指示灯，颜色取 `currentColor`。
- 配色：默认为中性（`bg-0` 底、`line-2` 边、`tx-2` 字）；`.live` 用 ok 色组；`.warn` 用 warn 色组；`.offline` 和 `.error` 用 alarm 色组。

**`.tiny-tag`（说明类标签）**
- 透明底，1px `line-1` 边框，`tx-3` 色文字，不带指示灯。
- 例外：属于状态的 tiny-tag，按第 5 节映射表加上 `data-state` 后，样式与 `.badge` 保持一致。

### 4.4 指示灯 `.lamp`（新增通用类）

- 尺寸：小号 8×8，大号 12×12。
- 熄灭：颜色 `--lamp-off`，加 `box-shadow: inset 0 0 0 1px #0006`。
- 点亮：使用对应状态色，加 `box-shadow: 0 0 0 1px #0009, 0 0 6px <状态色，50% 透明度>`。这是全局**唯一允许的发光效果**。
- 闪烁只允许用在两处："正在收音"和"未连接时的连接灯"。频率 1Hz；`prefers-reduced-motion` 下不闪。

### 4.5 按键（做成实体键帽的质感，不要做成网页按钮）

**`.button`（次级按键）**
- `bg-3` 底，1px `line-3` 边，`tx-1` 字，圆角 `--r`。
- 高 32px；视口高 ≥900 时 36px。
- `box-shadow: inset 0 1px 0 #ffffff0d, 0 1px 0 #0009`。
- `:hover` 时底色改为 `bg-4`。
- `:active` 时 `transform: translateY(1px)`，并去掉外阴影。
- `:disabled` 时 `opacity: .38`，去掉阴影，`cursor: not-allowed`。

**`.button.primary`（主操作按键）**
- 用于：前往观察、发送、开始收音、使用此令牌。
- `--sel-bg` 底，1px `--sel` 边，`--sel` 色文字，字重 600。
- **不要再用亮蓝色实心按钮。**

**`.button.warm`（危险操作按键）**
- 用于：确认短时喷水、确认提交。
- `--warn-bg` 底，1px `--warn` 边，`--warn` 色文字，文字前加警告三角 SVG 图标。

**`.text-btn`（文字按钮）**
- `--sel` 色，12px，悬停时加下划线。

### 4.6 急停按钮（整个界面的视觉签名）

指顶栏里的 `.button.danger[data-stop]`：
- 高 36–40px，最小宽 128px。
- `--estop` 红底；外圈 2px `--hazard-yellow`（模仿急停按钮的黄色底座）；最外层再加 1px `#000`。
- 白色文字，14px，字重 700。
- 文字左侧放一个 16px、白色描边的八边形 STOP 图标，替换现在的 `■`。
- 悬停时底色 `#E0142F`；按下时呈内凹效果。
- 禁用时 `opacity: .5`，黄圈仍然要能看见。

火焰页的"停止喷水"按钮（`.button.ghost[data-stop]`）用**报警描边款**：
- `--alarm-bg` 底，1px `--alarm-line` 边，`--alarm` 色文字。
- **不加黄圈**，避免和总急停混淆。
- 选择器写法：`.button[data-stop="true"]:not(.danger)`。

### 4.7 输入控件

- `well` 底，1px `line-3` 边，圆角 `--r`，`tx-1` 色文字；placeholder 用 `tx-3`。
- 聚焦时：边框改为 `--sel`，外加 2px `--sel-bg` 外环，不要用发光。
- `select` 的下拉箭头用 CSS 中的 data: URI SVG 自绘。

### 4.8 拨动开关（画面传输 `.switch`）

- 槽：44×22，`well` 底，1px `line-3` 边。
- 滑块：18×18 方形，`bg-4` 底，顶部加 1px 高光。
- 开启时：槽内用 `::after` 显示 `--ok` 色的 "ON"。
- 关闭时：显示 `tx-3` 色的 "OFF"。
- 保留原有的 `input` 结构和 `role="switch"`。

### 4.9 读数窗 `.readout`（新增）

用于仪表条、火焰指标、场景事实和音频状态。
- `well` 底，1px `line-1` 边。
- 第一行：标签，由中文（12px，`tx-2`）加英文代号组成。
- 第二行：数值，`--font-num` 字重 600，`tx-1` 色；单位 13px，`tx-3` 色。
- 第三行：说明，12px，`tx-3` 色，只占一行，超长省略。

### 4.10 表格

- 表头：`bg-2` 底，`--font-num` 12px，字距 .06em，`tx-2` 色，sticky 固定。
- 行高 32px；斑马纹交替使用 `bg-1` 和 `#14181C`；悬停行改为 `bg-2`。
- 数值列右对齐，使用等宽数字。

### 4.11 对话框 `dialog`

- `bg-1` 底，1px `line-3` 边，圆角 `--r`，`box-shadow: 0 24px 64px #000b`。
- `.dialog-head` 做成全宽铭牌条：`bg-2` 底，用负 margin 抵消对话框的 padding。
- 遮罩颜色 `#030405c7`。

### 4.12 Toast

`bg-2` 底，1px `line-3` 边，左侧 3px `--sel` 色条（`.error` 时改为 `--alarm`），字号 13px。

### 4.13 滚动条

宽 6px，滑块 `#3A424C`（悬停 `#4A545F`），轨道透明，直角。

### 4.14 图标

- 所有图标统一换成内联 SVG 线性图标：viewBox 24，描边 1.5px，`stroke-linecap: square`，颜色 `currentColor`。
- 删除所有 emoji 和装饰字符图标：`◌ ↗ ⌖ ✧ ▤ ◉ ■`，以及按钮文字里的 `↗ →`。

---

## 5. 状态 → 颜色映射（P0，最能体现"颜色即状态"）

### 5.1 怎么接入

在对应的 `render*()` / `showFrameState()` 里，**紧挨着现有的 `text()` 调用**，用同一个条件设置 `el.dataset.state`。取值只有 5 种：`ok`、`warn`、`alarm`、`sel`、`idle`。

### 5.2 各取值的样式

| 取值 | 样式 |
|---|---|
| `ok` | 大读数：文字保持 `tx-1`，只点亮绿灯（正常态不抢眼）。小标签：可整体使用 ok 色组 |
| `warn` | 文字 `--warn`，琥珀灯 |
| `alarm` | 文字 `--alarm`，红灯；大读数格再加 3px `--alarm` 左边条 |
| `sel` | 文字 `--sel`，青灯 |
| `idle` | 文字 `tx-3`，灯熄灭 |

### 5.3 映射表

条件全部沿用 JS 现有的判断，不新增逻辑。

| 元素 | 条件 → state |
|---|---|
| `document.body`（设置 `data-link`） | 未连接 → `offline`；模拟 → `sim`；实车 → `live`。CSS 用它统一切换模拟 / 离线时的外观 |
| `#modeMetric` | mode 为 `fault` 或 `estop` → alarm；有新鲜 mode → ok；否则 → idle |
| `#motionMetric` | 允许行驶 → ok；已反馈停稳 → ok；保持停车 → warn；待反馈 → idle |
| `#controlBadge` | 可提交指令 → ok；模拟指令 → warn；等待任务链路或只读展示 → idle |
| `#roadFresh`、`#ocrFresh` | 新鲜证据或观测有效 → ok；观测待确认 → warn；待数据 → idle |
| `#lightLabel` | 红灯 → alarm；绿灯 → ok；未知 → idle |
| `#roadDecision` | 任务允许行驶 → ok；任务保持停车 → warn；等待 → idle |
| `#imageLocked` | 图像已锁定 → ok；正在对准 → warn；待有效目标 → idle |
| `#actuatorTag` | DRY-RUN → warn（并叠加低透明度斜纹）；执行器已启用 → ok；其余 → idle |
| `#voiceBadge` | 本地音频已启用 → ok；文字联调或音频未启用 → warn；语音事件已收到 → sel；等待 → idle |
| `#navigationState` | 有新鲜导航状态 → sel；否则 → idle |
| `#frameBadge` | 画面可见且为实车 → ok（已有 `.live` 类）；画面可见且为模拟 → warn；其余 → idle |

`#connectionBadge` 和 `#railDot` 已经有 `live` / `warn` / `demo` / `offline` 类，直接沿用即可。

---

## 6. 全局框架

### A. 原生系统栏（`App.cs`）

- 高度从 54 改为 44（上下 Padding 各 6），底色 `#0B0D10`。
- 在 header 下方加一条 1px 的 `#313841` 分隔线（用一个 Dock 为 Top、高 1px 的 Panel 即可，注意 Dock 顺序）。
- 品牌文字 "RESCUE / X5"：`Bahnschrift` 10.5pt 粗体，颜色 `#E6E9ED`。
- 地址框：底色 `#070809`，文字 `#E6E9ED`，字体 `Consolas`。
- 普通按钮：底 `#22272E`，边 `#48515C`，悬停 `#2B313A`，按下 `#070809`。
- "连接 X5" 按钮不再用亮蓝 `#3B82F6`，改为：底 `#0A2630`，边 `#3BC4E6`，文字 `#3BC4E6`，悬停底 `#0F3442`。
- `connection` 文字颜色 `#A9B2BC`。
- 窗体 `BackColor`、`view.DefaultBackgroundColor`、`cover` 都改为 `#0B0D10`；`coverTitle` 用 `#E6E9ED`，`coverBody` 用 `#A9B2BC`。
- DWM 标题栏颜色：属性 35 设为 `0x00100D0B`，属性 36 设为 `0x00EDE9E6`。这两个属性仅在 Win11 上生效，Win10 会忽略。保留属性 20（深色标题栏）的调用。

### B. 左侧导航 `.rail`

- `bg-0` 底，右边 1px `line-2`。
- **品牌区**：
  - 一个 40×40 方块：`--estop` 红底，白色 "X5"（`--font-num`，字重 700），右下角用 6px 黄色三角切角作点缀。
  - 方块右侧上行为 "消防救援 HMI"（14px，字重 700），下行为 "RDK X5 · FPGA"（`--font-mono` 12px，`tx-3` 色）。
  - 品牌区下方加 1px `line-1` 分隔线。
- **分组标签**："操作面板" 后接英文代号 `OPERATION`。
- **导航项**做成功能键：
  - 高 44px，左侧 3px 指示条，`--font-mono` 12px 编号，18px SVG 图标，13px 中文。
  - 常态：`tx-2` 色。悬停：`bg-1` 底。
  - `.active`：`bg-2` 底，指示条和编号改为 `--sel` 色，文字改为 `tx-1`。
- **"演示流程导览"** 改成全宽次级按键，带播放三角图标。
- **底部连接状态**：
  - `#railDot` 做成 8px 方形灯：`.live` 为绿灯，`.demo` 为琥珀灯；没有 class（未连接 / 离线）时为红灯，1Hz 闪烁。
  - 灯下方的说明文字 12px，`tx-3` 色。

### C. 顶栏 `.topbar`

- 高 48px，直接放在 `bg-0` 上，底边 1px `line-1`。
- **左侧**依次为：
  - "消防救援小车"（18px，字重 700）
  - 竖分隔线
  - "任务与视觉监控"（13px，`tx-2` 色）
  - 英文代号 `MISSION & VISION MONITOR`（视口宽 ≤1180 时隐藏）
- **右侧**依次为：`#connectionBadge`、`#unlockBtn`（次级键）、急停按钮。

### D. 模式横幅 `#modeBanner`

JS 会覆写它的 textContent，所以装饰全部用伪元素实现。

**默认（模拟模式）**
- 高 28–32px。
- `::before` 是一个 10px 宽的 `--hazard` 斜纹块，后面接一个 `content: "SIM"` 的小铭牌（黑底黄字，`--font-num`，字重 700）。
- 横幅本身：`--warn-bg` 底，1px `--warn-line` 边，`--warn` 色文字，12–13px。

**`.error`（连接中断）**
- 斜纹换成红黑相间，铭牌文字换成 `content: "OFFLINE"`。
- 横幅本身：`--alarm-bg` 底，`--alarm` 色文字。

### E. 样例切换条 `#demoTools`

- 左侧标签写 "SIM 样例"。
- 4 个按钮做成**相邻无间隙、共享边框**的分段键，像一排面板按键。

### F. 状态栏 `.page-footer`

- 高 24px，`bg-0` 底，顶边 1px `line-1`。
- 文字用 `--font-mono` 12px，`tx-3` 色。
- 用竖线分段；最左侧加一个 `SYS` 小灯。
- `#lastUpdated` 右对齐，使用等宽数字。

### 黄黑斜纹的使用范围

**只允许出现在以下 4 个地方**：
1. 模拟 / 离线横幅
2. 危险操作确认框顶部
3. 画面过期遮罩的边框
4. DRY-RUN 标签（低透明度）

---

## 7. 逐页规格

### 7.1 现场总览 `#view-overview`

#### 仪表条 `.status-strip`

- 不要做成 4 张卡片，而是**一整条凹槽**：`well` 底，1px `line-2` 边。
- 4 个格子之间用 1px `line-1` 竖线分隔，每格按"读数窗"的结构排版。
- 4 格的英文代号依次为 `MODE`、`INFERENCE`、`TARGETS`、`MOTION`。
- **MODE 格**：读数前放一个状态灯，颜色按 `#modeMetric` 的 data-state。alarm 时整格加 3px `--alarm` 左边条，读数变红。
- **INFERENCE 格**：
  - 读数下方加一根 4px 高的横条刻度表，量程 0–30 FPS，在 10 / 20 / 30 处画刻度。
  - 在 `renderStatus` 里，用**界面上显示的同一个 `fps` 值**设置 `--fps-ratio`，并截断到 0–1。
  - 没有数据或 YOLO 按任务关闭时，横条为空，刻度改为虚线。
- `#fpsMetric i` 和 `#targetMetric i` 是单位，按单位样式处理。

#### 取景器 `#sharedVisionPanel`

- **`.camera-stage`**：
  - `#050607` 底色。
  - 四角加 L 形取景框角标：长 20px，粗 2px，`tx-2` 色约 50% 不透明度。可以在静态 HTML 里加 4 个 `<i class="vf-corner">`，也可以用一个元素的多重背景实现。
  - **不要加中心十字准星**，以免被误读成云台瞄准点。
- **OSD 角标**（`.frame-badge`、`.frame-source`、`.legend`、`#frameGeometry`）：
  - 直角，底色 `#050607d1`，1px `line-2` 边，12px。
  - `#frameBadge` 按第 5 节映射表配灯。
  - `frameSource` 和 `frameGeometry` 用等宽字。
  - 图例色块使用新的检测类别色。
- **`.camera-empty`（无信号状态）**：
  - 中央放一块铭牌：第一行 `NO SIGNAL`（`--font-num` 12px，字距 .2em，`tx-3` 色），第二行 "等待车载画面"。
  - 配一个摄像头 SVG 图标，替换原来的 `⌖`。
  - 背景铺一层极淡的 45° 细斜线纹理。
- **`.camera-stale`（画面过期状态）**：
  - 叠一层 `#070809b8` 遮罩。
  - 四周加 4px 斜纹边框，60% 不透明度。
  - 中央显示 warn 色铭牌文字。
- **`.camera-footer`**：`bg-2` 底，等宽字，用竖线分段。

#### 任务控制 `.task-panel`

- **模式键 `.mode-button`（2×2 排列）**：
  - `bg-3` 键帽质感。
  - 左上角是英文代号，可以直接用 `::before { content: attr(data-mode) }` 加 uppercase 得到 STANDBY / PATROL / INSPECT / SCENE。
  - 下面依次是中文名（14px，字重 700）和副标题（12px，`tx-3` 色）。
  - 右上角放一个 10×10 指示灯（默认熄灭）。
  - `.active` 表示"车载回报的当前模式"，属于状态，所以用**绿色**：灯亮绿，边框改为 1px `--ok-line`，顶部加 2px `--ok` 色条。
  - 原来的 `◌ ↗ ⌖ ✧` 换成 SVG，或直接删掉。
- **语义导航**：
  - 小标题 "语义导航" 加英文代号 `SEMANTIC NAV`。
  - 下面是下拉框加主操作键；"到达火场后" 单独作为一行表单。
- **`#navigationState`**：放进 `well` 底的状态行，左侧加指示灯，文字用等宽字。
- **`#commandHint`**：做成面板底部的注记条：`bg-0` 底，顶边 1px `line-1`，12px，`tx-3` 色。

### 7.2 道路与楼牌 `#view-road`

#### 状态流程 `.road-flow` → 顺序功能图（SFC / GRAFCET 风格）

- 4 个步框横排，编号改为 `S1`–`S4`（`--font-num`）。
- 步框之间用 1px `line-3` 横线连接，线中间加一根短竖杠表示转移条件，替换原来的 `<i>→</i>`。
- 常态步框：`bg-0` 底，`line-2` 边。
- `.active` 步框：边框改为 `--sel`，编号块填 `--sel` 底、黑字。
- JS 只切换 `.active`，**不要**自己加"已完成"状态。

#### 信号灯 `.traffic-light` → 竖向信号灯头

- 黑色灯箱：`#050607` 底，1px `line-3` 边，3px 圆角，内边距 6px。
- 上红下绿两颗 18px 圆灯。
- 熄灭时是深色透镜（红 `#2A0E10`、绿 `#0B2416`），加内阴影。
- `.on` 时用亮色径向渐变做高光，外发光 `0 0 10px`。

#### 其他

- `#roadDecision` 做成输出状态铭牌，按映射表上色。
- **楼牌区**：
  - `#buildingNumber` 放进 `well` 读数框，用 `--font-num` 32–40px。
  - `.building-symbol` 里的 `▤` 换成楼宇 SVG 图标。
  - `.fact-line` 做成键值行。

### 7.3 火焰控制 `#view-fire`（另一个视觉重点）

面板内容分成三段，每段加一个英文代号小标题：
1. 目标登记 `TARGET REGISTRY`
2. 瞄准读数 `AIM DATA`
3. 喷水联锁 `PERMISSIVES` 与操作区

#### 目标卡 `.target-pill`

- 直角，`bg-3` 底，左侧 4px `--det-fire` 色条。
- 内容为 "火焰 #1"，后面跟等宽字的置信度。
- `.selected` 时：边框改为 1px `--sel`，并用 `::after` 显示 "已选定" 角标。
- 禁用时 `opacity: .55`，但仍然要能看清。

#### 瞄准读数 `.fire-metrics`

做成 3 个读数窗：选定目标、目标距离（m）、画面锁定。数值 22px。

#### 喷水联锁 `.gates` → 联锁光字牌（annunciator）

- 2 列 × 3 行的矩形灯窗，每个灯窗高 36–40px，文字居中，12–13px。
- **未满足**：`well` 底，`line-1` 边，`tx-3` 色文字，左上角放一颗熄灭的小灯。
- **`.pass`（已满足）**：`--ok-bg` 底，`--ok-line` 边，`--ok` 色文字，灯亮。
- 把 JS 里 `<span>✓</span>` / `<span>·</span>` 中的字形去掉（属于允许的改动 ③），span 留空，当作灯来画。
- 可选：在 `renderFire` 里用同一个 `gates` 数组统计已满足项数，在标题右侧显示 `n/6`。

#### 操作区 `.fire-actions`

一排三个键：

| 位置 | 按键 | 样式 |
|---|---|---|
| 左 | 自动找火灭火 | 次级键 |
| 中 | 确认短时喷水 | `.warm`，三者中最醒目；下方加 12px 小字"需二次确认" |
| 右 | 停止喷水 | 报警描边款 |

#### 状态注记

`#autoState` 和 `#pumpMeaning` 放进底部的 `well` 注记区，分别用 `::before` 加上等宽前缀 `AUTO` 和 `PUMP`。

### 7.4 语音与场景 `#view-voice`

#### 车载语音

- **`#micOrbit`**（现在是 `display: none`）：改为显示，做成 40px 圆形麦克风状态灯（SVG 麦克风加外圈）。`.listening` 时外圈变为 `--ok` 色，并做 1Hz 呼吸效果。
- **不要做假的音量电平条**，系统没有电平数据。
- **`.audio-facts`**：做成两个读数格，代号分别为 `ASR` 和 `TTS`。
- **`.transcript-box`**：`well` 底，左侧 3px `--sel` 条；转写文字 15–16px；下面一行显示等宽字的元信息。

#### 任务与场景对话 → 通信记录（COMMS LOG）

不要做成聊天气泡。
- **`.message`**：直角，`bg-2` 底，左侧 3px 色条——操作员消息（`.user`）用 `--sel`，车辆回答用 `tx-2`。
- **`.message-label`**：用 `::before` 加等宽前缀，`.user` 为 `OPR ▸`，其他为 `VEH ◂`。
- **`.message-meta`**：等宽字，`tx-3` 色。
- **`.prompt-chips`**：做成快捷指令小键帽，高 26px。
- **`.command-form`**：textarea 做成终端输入框（`well` 底），用 `::before` 在左侧显示 `>` 提示符。

#### 场景证据

`.scene-facts` 做成 3×2 的读数格。

#### 时间线 → 事件日志

- 每条日志用 grid 排成三列：`[时间，等宽，72px] [来源，12px，tx-2 色] [内容]`。
- 不改 JS 生成的 DOM，用 CSS 重排即可：可以给 `.event-head` 设 `display: contents`，再配合 grid-area 排列。
- 左侧画 1px 时间轴线，节点用方形。

### 7.5 系统证据 `#view-evidence`

#### 系统拓扑 `.architecture` → 单线图

- 每个节点框带一个 `bg-2` 标题条，写英文代号：`SENSOR`、`COMPUTE`、`CONTROL`、`ACTUATOR`。
- RDK X5 节点（`.accent`）顶部加 2px `--sel` 色条。
- 连接线用 1px `line-3` 加箭头。
- **线上的接口标注沿用现有文字（→、⇄ UART、＋），不要自己编造新的接口名称。**
- 做成静态图，不加流动动画，避免暗示有实时数据在流动。

#### 其他区块

- `.system-facts` 做成诊断键值行。
- `.receipt` 做成交易日志行：状态文字 `.ok` 为绿色，`.error` 为红色，默认为琥珀色；ID 用等宽字。
- ROS 2 话题表按 4.10 节的表格规范处理：`.topic-status.fresh`、`.stale`、`.missing` 前面分别加绿灯、琥珀灯、熄灭灯；"查看 JSON" 做成小号次级键。
- `#exportBtn` 用次级键，配一个下载 SVG 图标。
- `#evidenceJson` 用 `well` 底，等宽字 12px。

### 7.6 叠加层

- **导览条 `.walkthrough`**：停靠在底部，`bg-2` 底，顶边 2px `--sel`；左侧放 `GUIDE` 铭牌，步骤号用 `--font-num`（例如 `02 / 06`）。
- **`#actionDialog`（危险操作确认）**：
  - 顶部用 `::before` 加一条 6px 斜纹带。
  - `.confirmation-target`：`well` 底，等宽字，`--warn` 色文字，左侧 3px `--warn` 条。
  - 复选框自绘成 16×16 方形；选中后填充 `--warn` 色，中间显示黑色对勾（用 data: URI SVG）。
- **`#unlockDialog`**：令牌输入框用等宽字；"使用此令牌" 按钮用 primary 样式。

---

## 8. 画面检测框（`presentation.js` 中的 `drawFrame`）

### 保留不变

- 同帧配对
- 坐标裁剪
- 字号随 canvas 宽度缩放
- 置信度文字格式
- `detector_available === false` 时不画框

### 改样式

- **类别色**：`colors` 改为 fire `#FF6A2B`、person `#5B8CFF`、firefighter `#C6F135`；同时同步修改 CSS 里 `.fire`、`.person`、`.firefighter` 的图例色。
- **框**：
  1. 先画全框，线宽与现在相同，`globalAlpha = .55`。
  2. 再画四个 L 形角标：线宽为全框的 1.6 倍；长度取 `min(w, h) × 0.22`，并限制在 `[10, 28] × (canvas.width / 1280)` 范围内。
- **标签**：
  - 直角色块，深色文字 `#0B0D10`。
  - 字体 `600 ${font}px Bahnschrift, "Microsoft YaHei", sans-serif`。
  - 位置逻辑沿用现有代码。
- 不用 `shadowBlur`，也不加任何发光效果。

---

## 9. 禁止事项

- **不做"科技蓝数据大屏"风格**：霓虹发光描边、外发光、渐变光束、粒子、扫描线、六边形网格、装饰性角花、旋转圆环。
- **不做 SaaS 后台风格**：大圆角卡片、柔和投影、渐变按钮、胶囊形 pill、毛玻璃、彩色 emoji 图标。
- **不让颜色泛滥**：每屏除状态色外，只允许 `--sel` 一个强调色；不出现大面积彩色底。
- **不做伪数据动画**：假音量条、假波形、假的数据流动画、假进度条。
- **不删、不藏诚实性信息**（见 1.4 节）。
- **不引入任何框架、库或构建步骤**，保持纯 HTML / CSS / JS。

---

## 10. 执行顺序

按优先级分阶段完成，**每个阶段结束都要截图自查一次，再进入下一阶段**。

| 阶段 | 内容 |
|---|---|
| **P0** | 设计令牌、字体、通用组件（第 3、4 节）；全局框架（第 6 节 B–F）；急停按钮；第 5 节状态映射 |
| **P1** | 总览页的仪表条、取景器、任务控制；火焰页的光字牌；`drawFrame` |
| **P2** | 道路页的 SFC 与信号灯头；语音页的通信记录与事件日志；证据页的拓扑图与表格；对话框与导览条 |
| **P3** | `App.cs` 原生系统栏配色（第 6 节 A） |

---

## 11. 验收与交付

### 11.1 单元测试

运行以下命令，必须全部通过：

```
python -m unittest discover -s ros2_ws/tests -p "test_presentation_server.py" -v
```

### 11.2 启动本地演示服务

```
python ros2_ws/tools/presentation_server.py --demo --port 8090
```

然后用 Edge 或无头 Chromium 打开 `http://127.0.0.1:8090/#overview`。

### 11.3 截图

截图保存到 `reports/industrial_ui_v2_<日期>/`。

**基础矩阵**：1366×680、1093×550、1920×1000 三个视口，每个视口截全部 5 个页面。

**补充截图**（在 1366×680 下）：
- 点击样例"斑马线 / 信号灯"后的道路页
- 点击样例"火场 / 瞄准"后的火焰页（选中一个目标）
- 点击样例"语音 / 场景"后的语音页
- `#actionDialog`
- `#evidenceDialog`
- 导览条
- 停掉服务后的离线状态

同时输出 `responsive_checks.json`，逐页检查以下三项：
- document 没有溢出；
- 所有可见文字 ≥ 12px；
- 没有文字被裁切。

### 11.4 自查清单

对照截图逐条回答"是"或"否"：

- [ ] 第一眼看过去以灰阶为主，彩色只出现在状态位置
- [ ] 每屏最醒目的元素是急停按钮（火焰页还有"确认短时喷水"）
- [ ] 所有数字等宽对齐，单位比数字小且颜色更暗
- [ ] 所有面板都有铭牌标题条（编号 + 中文 + 英文代号）
- [ ] 界面上没有 emoji、胶囊形、渐变按钮、发光描边
- [ ] 模拟模式下，横幅、视频 OSD、执行器标签三处都能看到 SIM 或 DRY-RUN
- [ ] 断开服务后，红色 OFFLINE 横幅清晰可见，连接灯在闪烁
- [ ] 光字牌的"未满足"和"已满足"两种状态，隔两米远也能分辨

### 11.5 重新打包

1. 运行 `desktop/build-backend.ps1`，把新网页打包进 `dist/runtime`。
2. 如果改了 `App.cs`，再运行 `desktop/build-app.ps1 -Check`。
3. 启动 `desktop/dist/消防救援演示台.exe`，进入离线演示，最大化窗口后截一张图。

### 11.6 不要部署到 X5

**不要**运行 `board_setup/部署答辩上位机.ps1`。是否部署到 X5 由我决定。

### 11.7 交付说明

交付说明中写清以下内容：
- 改动了哪些文件；
- JS 的每一处改动（逐条列出，并说明为什么不影响业务逻辑）；
- 截图目录和测试结果；
- 没做完或偏离本规格的地方，以及原因。
