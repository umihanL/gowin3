# X5 Python 离线依赖

目标为 Ubuntu 22.04、Linux aarch64、CPython 3.10。电脑通过正常 HTTPS 证书校验从 PyPI 下载；未关闭 TLS 校验，未安装到电脑环境，也未由本步骤修改板端。

`wheels/` 包含 29 个 wheel；`srt` 没有对应官方 wheel，使用官方 3.5.3 源码在隔离构建环境生成了 `py3-none-any` 包。`onnxruntime 1.20.1` 和 `Pillow 11.3.0` 使用 manylinux 2.27/2.28 ARM64 包，适用于本次 Ubuntu 22.04 的 glibc 2.35。

核查文件：

- `wheels_SHA256SUMS.txt`：文件 SHA256。
- `wheels_manifest.json`：文件大小、哈希、目标说明。
- `wheels_audit.json`：29 个文件均通过 ZIP CRC、CPython 3.10 ARM64 ABI、Python 版本要求与 Linux 依赖闭包的静态核查。
- `download_arm64_dependencies.log`、`download_arm64_model_adapters.log`、`build_srt_wheel.log`：下载与构建日志。

静态核查不代表已完成板上导入测试或推理测试。板上已有的 NumPy 1.26.4 与 OpenCV 4.11.0 应优先复用；此目录没有第二份 OpenCV 包。RapidOCR 的 pip 元数据要求 `opencv-python`，而厂商的可导入 `cv2` 可能没有这个 pip 名称，因此 RapidOCR 需要单独 `--no-deps` 安装，并执行实际导入测试。

部署脚本应先在板端准备 `--system-site-packages` 虚拟环境，激活后按需求解析安装，避免对全部 wheel 文件无差别安装。例如在包含 `wheels/` 的目录下：

```bash
python -m pip install --no-index --find-links wheels \
  'onnxruntime==1.20.1' 'pyclipper==1.3.0.post6' \
  'Shapely>=1.8,<3' 'PyYAML>=6,<7' 'six>=1.16' 'Pillow>=9,<12' \
  'vosk==0.3.45' 'sounddevice>=0.4,<0.6'
python -m pip install --no-index --find-links wheels --no-deps 'rapidocr-onnxruntime==1.4.4'
```

主部署流程负责决定具体虚拟环境位置、系统音频库、ROS 2 环境与板端验证。这里的 Windows 下载器只负责下载。下载器在 Windows 上解析依赖时顺带包含了 `colorama`、`pyreadline3`；它们不需要在 Linux 上主动安装。
