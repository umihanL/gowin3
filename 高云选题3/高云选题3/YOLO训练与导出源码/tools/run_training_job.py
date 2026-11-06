"""Run GPU training and one held-out test as a logged, independent local job."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta, timezone
import html
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]
LOCAL_TZ = timezone(timedelta(hours=8))
TERMINAL_PHASES = {"failed", "completed", "stopped"}


def now():
    return datetime.now(LOCAL_TZ).isoformat(timespec="seconds")


def latest_metrics(path):
    if not path.is_file():
        return {}
    try:
        with path.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        complete = [r for r in rows if r.get("epoch") and all(v is not None for v in r.values())]
        return {k.strip(): v.strip() for k, v in complete[-1].items()} if complete else {}
    except (OSError, csv.Error, AttributeError):
        return {}


def save_status(folder, state):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    state["updated_at"] = now()
    temp = folder / "status.tmp.json"
    temp.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temp, folder / "status.json")
    labels = {"preparing": "检查数据与准备模型", "training": "GPU 训练中", "evaluating": "训练结束，评估独立测试集", "completed": "训练与测试完成", "failed": "任务失败，请查看日志", "stopped": "训练已按要求停止"}
    if state.get("paused") and state["phase"] == "stopped":
        labels["stopped"] = "训练已暂停，等待继续指令"
    metrics = state.get("latest_training_metrics", {})
    metrics_html = "".join(f"<tr><td>{html.escape(k)}</td><td>{html.escape(str(v))}</td></tr>" for k, v in metrics.items())
    refresh = '<meta http-equiv="refresh" content="15">' if state["phase"] not in TERMINAL_PHASES else ""
    counts = [state.get(k, "未记录") for k in ("training_images", "validation_images", "test_images")]
    kind = ("新增数据微调：从旧最佳权重初始化，创建新运行与新优化器；不恢复旧训练轮次。"
            if state.get("training_kind") == "finetune" else "从初始化权重开始新的训练运行。")
    dataset_text = str(state.get("dataset", "dataset/data.yaml"))
    model_text = str(state.get("initialization_model", state.get("model", "YOLO11n Detect")))
    config_text = str(state.get("config", "config/yolo11n_detect.yaml"))
    patience = state.get("patience", 25)
    artifacts = [("训练日志", folder / "train.log"), ("测试日志", folder / "test.log"),
                 ("详细状态", folder / "status.json"),
                 ("训练曲线数据", ROOT / "runs" / state["name"] / "results.csv"),
                 ("最佳权重", ROOT / "runs" / state["name"] / "weights/best.pt"),
                 ("最终测试指标", ROOT / "runs" / (state["name"] + "_test") / "metrics_summary.json")]
    if state.get("diagnostic_report"):
        artifacts.append(("本次暂停检查报告", Path(state["diagnostic_report"])))
    present = [(label, path) for label, path in artifacts if path.is_file()]
    def link_from(path, base):
        return quote(Path(os.path.relpath(path, base)).as_posix(), safe="/:")
    links_html = " · ".join(f'<a href="{html.escape(link_from(path, folder))}">{label}</a>' for label, path in present)
    links_md = " · ".join(f'[{label}]({link_from(path, ROOT)})' for label, path in present)
    active_epoch = state.get("active_epoch")
    epoch_note = f"当前正在运行第 {active_epoch} 轮；" if active_epoch and state["phase"] == "training" else ""
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">{refresh}
<title>YOLO11n 训练进度</title><style>body{{font:16px/1.7 "Microsoft YaHei",sans-serif;background:#edf2f5;color:#1c3746;margin:0}}main{{max-width:960px;margin:40px auto;padding:30px;background:white;border-radius:16px}}h1{{margin-top:0}}.state{{color:#076d7d;font-size:24px}}p{{color:#526978}}table{{border-collapse:collapse;width:100%}}td{{border-bottom:1px solid #e1e7eb;padding:8px}}a{{color:#08798c}}code{{overflow-wrap:anywhere}}@media(max-width:650px){{main{{margin:10px;padding:18px}}}}</style>
<main><h1>居民楼壁纸 · YOLO11n Detect</h1><div class="state">{labels[state['phase']]}</div>
<p>运行：{html.escape(state['name'])}<br>开始：{html.escape(state['started_at'])}<br>最后更新：{html.escape(state['updated_at'])}</p>
<p>设备：RTX 4060 Laptop / CUDA:0 · 输入 640 · batch {state['batch']}<br>{epoch_note}已记录完整轮数：{html.escape(str(metrics.get('epoch', '0')))} / 最多 {state['epochs']} 轮（patience {patience}，可能提前停止）</p>
<p>{kind}<br>数据：<code>{html.escape(dataset_text)}</code><br>初始化：<code>{html.escape(model_text)}</code><br>配置：<code>{html.escape(config_text)}</code></p>
<p>训练 {counts[0]} 张 · 验证 {counts[1]} 张 · 测试 {counts[2]} 张。下面的训练中指标来自验证集；仅在本次训练正常结束后执行一次测试集评估。合成指标不等于实拍精度。</p>
<p>{links_html}</p>
<p>{html.escape(state.get('message', '训练运行于独立后台进程；本页每 15 秒刷新。'))}</p>
<table>{metrics_html}</table></main></html>'''
    (folder / "progress.html").write_text(page, encoding="utf-8")
    note = f"""# 当前 GPU 训练状态（自动更新）

{labels[state['phase']]}。最后更新：{state['updated_at']}（北京时间）。

- 运行：`{state['name']}`
- GPU：RTX 4060 Laptop / CUDA:0
- 输入：640；batch：{state['batch']}；最多 {state['epochs']} 轮；patience {patience}。
- {epoch_note}已记录完整轮数：{metrics.get('epoch', '0')}。
- 数据：`{dataset_text}`。
- 初始化：`{model_text}`。{kind}
- 配置：`{config_text}`。
- 三类：fire、firefighter、person。
- 训练/验证/测试：{counts[0]}/{counts[1]}/{counts[2]} 张；不把验证或测试标签用于训练。

[打开进度页](runs/_jobs/{state['name']}/progress.html) · {links_md}

最佳权重保存位置：`runs/{state['name']}/weights/best.pt`。训练进行中时该文件可能尚不存在，或只是中间版本；以本页状态和详细记录为准。最终自动测试结果位于 `runs/{state['name']}_test/metrics_summary.json`。

{state.get('message', '后台进程持续运行，电脑需保持开机；进度页不需要一直打开。')}

初始交付的 NOT_TRAINED 和交付检查记录是训练开始前的历史状态。当前状态以本页、实际日志和权重为准。合成数据指标不代表 GS130W 实拍精度。
"""
    (ROOT / "TRAINING_STATUS.md").write_text(note, encoding="utf-8")


def run_phase(command, folder, state, stage):
    log_path = folder / f"{stage}.log"
    env = os.environ.copy()
    env.update(PYTHONUTF8="1", PYTHONUNBUFFERED="1", YOLO_AUTOINSTALL="false", YOLO_CONFIG_DIR=str(ROOT / ".ultralytics"))
    with log_path.open("w", encoding="utf-8", buffering=1) as log:
        log.write(f"Started {now()}\nArguments: {json.dumps(command, ensure_ascii=False)}\n")
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        state["worker_pid"] = process.pid
        state["command"] = command
        save_status(folder, state)
        while process.poll() is None:
            # A controller can record an authorized stop through save_status().
            # Do not overwrite that terminal state or start final testing.
            try:
                recorded = json.loads((folder / "status.json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                recorded = {}
            if recorded.get("phase") == "stopped":
                state.update(recorded)
                process.terminate()
                process.wait(timeout=30)
                state[f"{stage}_exit_code"] = process.returncode
                save_status(folder, state)
                return process.returncode or 130
            metrics = latest_metrics(ROOT / "runs" / state["name"] / "results.csv")
            if metrics:
                state["latest_training_metrics"] = metrics
                if stage == "train":
                    state["phase"] = "training"
            if stage == "train":
                update_training_progress(log_path, state)
            save_status(folder, state)
            time.sleep(10)
        metrics = latest_metrics(ROOT / "runs" / state["name"] / "results.csv")
        if metrics:
            state["latest_training_metrics"] = metrics
        try:
            recorded = json.loads((folder / "status.json").read_text(encoding="utf-8"))
            if recorded.get("phase") == "stopped":
                state.update(recorded)
        except (OSError, ValueError):
            pass
        state[f"{stage}_exit_code"] = process.returncode
        return process.returncode


def update_training_progress(log_path, state):
    """Observe the first batch before epoch1 creates results.csv."""
    if state.get("phase") in TERMINAL_PHASES:
        return
    try:
        with Path(log_path).open("rb") as stream:
            stream.seek(max(0, Path(log_path).stat().st_size - 131072))
            tail = stream.read().decode("utf-8", errors="replace")
    except OSError:
        return
    tail = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", tail)
    batches = re.findall(r"(?:^|[\r\n])\s*(\d+)\s*/\s*(\d+)\s+\S+", tail)
    current = [int(epoch) for epoch, total in batches if int(total) == state["epochs"]]
    if current:
        state["phase"] = "training"
        state["active_epoch"] = current[-1]
        state.setdefault("training_started_at", now())
    elif "Starting training for" in tail and "epochs" in tail:
        state["phase"] = "training"
        state.setdefault("training_started_at", now())


def inspect_inputs(data, model):
    # Light path/list inspection; train.py still runs the full independent audit.
    sys.path.insert(0, str(ROOT))
    from train import resolve_dataset
    data = Path(data).resolve()
    _, splits = resolve_dataset(data)
    initialization = model or "yolo11n.pt"
    if model:
        checkpoint = Path(model).resolve()
        if not checkpoint.is_file() or checkpoint.suffix.lower() != ".pt":
            raise FileNotFoundError("--model must be an existing local .pt checkpoint for a new fine-tune")
        initialization = str(checkpoint)
    return data, initialization, {key: len(images) for key, images in splits.items()}


def commands(args, data, initialization):
    base = [sys.executable, "-u", str(ROOT / "train.py")]
    shared = ["--data", str(data), "--config", str(args.config), "--device", "0", "--batch", str(args.batch), "--workers", "0", "--name", args.name]
    training = base + ["--mode", "train", "--model", initialization, "--epochs", str(args.epochs), "--imgsz", "640", "--patience", str(args.patience)] + shared
    best = ROOT / "runs" / args.name / "weights/best.pt"
    testing = base + ["--mode", "test", "--weights", str(best)] + shared
    return training, testing, best


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--data", type=Path, default=ROOT / "dataset/data.yaml")
    parser.add_argument("--model", help="Existing .pt to initialize a new fine-tune, not resume")
    parser.add_argument("--config", type=Path, default=ROOT / "config/yolo11n_detect.yaml")
    parser.add_argument("--patience", type=int, default=25)
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.name):
        raise ValueError("Use a simple run name")
    if args.epochs < 1 or args.batch < 1 or args.patience < 0:
        raise ValueError("epochs and batch must be positive; patience must be nonnegative")
    args.config = args.config.resolve()
    if not args.config.is_file():
        raise FileNotFoundError(args.config)
    data, initialization, counts = inspect_inputs(args.data, args.model)
    folder = ROOT / "runs/_jobs" / args.name
    if folder.exists() or (ROOT / "runs" / args.name).exists() or (ROOT / "runs" / (args.name + "_test")).exists():
        raise FileExistsError("Refusing to overwrite an existing job/run")
    folder.mkdir(parents=True)
    state = dict(name=args.name, phase="preparing", started_at=now(), supervisor_pid=os.getpid(), epochs=args.epochs, batch=args.batch,
                 dataset=str(data), training_images=counts["train"], validation_images=counts["val"], test_images=counts["test"],
                 initialization_model=initialization, training_kind="finetune" if args.model else "pretrained",
                 config=str(args.config), patience=args.patience,
                 model="YOLO11n Detect", classes=["fire", "firefighter", "person"])
    save_status(folder, state)
    try:
        training, testing, best = commands(args, data, initialization)
        train_code = run_phase(training, folder, state, "train")
        if state["phase"] == "stopped":
            save_status(folder, state)
            return
        if train_code != 0:
            raise RuntimeError("训练进程报错；详情见 train.log。")
        if not best.is_file():
            raise FileNotFoundError("Training exited without the expected best.pt")
        state.update(phase="evaluating", best_checkpoint=str(best))
        save_status(folder, state)
        test_code = run_phase(testing, folder, state, "test")
        if state["phase"] == "stopped":
            save_status(folder, state)
            return
        if test_code != 0:
            raise RuntimeError("权重已生成，但独立测试评估报错；详情见 test.log。")
        state.update(phase="completed", completed_at=now(), message="训练和独立测试已完成；权重与曲线保存在本工程 runs 目录。")
    except Exception as exc:
        if state["phase"] != "stopped":
            state.update(phase="failed", completed_at=now(), message=f"{type(exc).__name__}: {exc}")
        save_status(folder, state)
        raise
    save_status(folder, state)


if __name__ == "__main__":
    main()
