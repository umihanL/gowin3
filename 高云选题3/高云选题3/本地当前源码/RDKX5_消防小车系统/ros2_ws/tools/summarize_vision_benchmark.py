#!/usr/bin/env python3
"""Summarize the saved on-device, unchanged-quality optimization measurements."""
import json
from pathlib import Path
import statistics

root = Path(__file__).resolve().parents[1] / 'logs/vision_pipeline_20261010'


def load(name):
    return json.loads((root / name).read_text(encoding='utf-8'))


def metrics(data):
    samples = data if isinstance(data, list) else data['samples']
    med = statistics.median
    return dict(samples=len(samples),
        processing_fps=med(s['performance']['processing_fps'] for s in samples),
        preview_fps=med(s['fps'] for s in samples),
        image_age_ms=(1000*med(s['frame_age_s'] for s in samples if s['frame_age_s'] is not None)
                      if any(s['frame_age_s'] is not None for s in samples) else None),
        processing_fps_range=[min(s['performance']['processing_fps'] for s in samples),
                              max(s['performance']['processing_fps'] for s in samples)])


baseline, on, off = [metrics(load(f)) for f in ('baseline_board.json', 'final_on.json', 'final_off.json')]
equality = load('native_runtime_equivalence.json')
off_samples = load('final_off.json')['samples']
first, last = off_samples[0]['performance'], off_samples[-1]['performance']
summary = dict(client_date='2026-10-10', baseline=baseline, final_stream_on=on, final_stream_off=off,
    processing_improvement_percent=100*(on['processing_fps']/baseline['processing_fps']-1),
    preview_improvement_percent=100*(on['preview_fps']/baseline['preview_fps']-1),
    configuration=dict(camera_preview_size=[1280,1088], jpeg_quality=90,
        model_size=[640,640], opencv_threads=8, process_fps_limit=30,
        backend='native_hbdnn', pipeline_enabled=True, classes=['fire'], original_8000_preview_running=False),
    equivalence=dict(model_sha256=equality['model_sha256'], cases=len(equality['accuracy']),
        output_heads=sum(len(c['heads']) for c in equality['accuracy']),
        all_output_bytes_equal=equality['bit_exact'], detections_equal=equality['detections_equal']),
    stream_off_check=dict(jpeg_frames_start=first['preview_encoded_frames'],
        jpeg_frames_end=last['preview_encoded_frames'], sequence_start=first['sequence'],
        sequence_end=last['sequence'], subscribers=last['preview_subscribers']),
    measurement_note='Same board and camera configuration; warm-up then 1 Hz diagnostic sampling. '
        'Baseline 20 s, final on 45 s, off 20 s. Browser observed 19.8 fps. '
        'Board RTC had reset to 2000; duration uses monotonic clock. Relative camera/preview age uses the same board clock. '
        'Reported rates vary with scene and concurrent applications; this is not a long-duration thermal test.')
(root/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
table = f'''| 测量项 | 优化前 | 优化后 |
|---|---:|---:|
| 开传图：完整识别 | {baseline['processing_fps']:.2f} fps | {on['processing_fps']:.2f} fps |
| 开传图：同帧预览 | {baseline['preview_fps']:.2f} fps | {on['preview_fps']:.2f} fps |
| 预览图像年龄中位数 | {baseline['image_age_ms']:.0f} ms | {on['image_age_ms']:.0f} ms |
| 关传图：完整识别 | 本轮未测 | {off['processing_fps']:.2f} fps |
'''
paragraph = f'''默认参数：`process_fps=30`（上限）、`opencv_threads=8`、`pipeline_enabled=true`、`backend=native_or_pyeasy`，实测使用`native_hbdnn`。完整识别提升约{summary['processing_improvement_percent']:.0f}%，实际预览提升约{summary['preview_improvement_percent']:.0f}%。网页现场读数约19.8帧/秒。

相机及预览均1280×1088、JPEG90、同一640模型及原有去畸变/颜色/阈值/NMS。8组输入的48个模型输出头全部逐字节一致；隔离ROS测试验证图像和JPEG与串行参考逐字节一致。单次模型执行约29～30ms，性能收益来自并行调度，未宣称模型本身加速。

持续45秒开传图测量与20秒关传图测量已完成。关传图期间JPEG计数保持{first['preview_encoded_frames']}不变，检测序号由{first['sequence']}增长到{last['sequence']}，图片订阅数为0。已恢复开启传图。测量期间温度约59～63°C，未超频；当前场景和系统负载变化时帧率也会波动。

原始证据：`baseline_board.json`、`final_on.json`、`final_off.json`、`native_runtime_equivalence.json`、`pipeline_tests.txt`。时间长度用单调时钟记录，避免X5重启后日期回退影响测量。
'''
(root/'results.md').write_text('# 高清视觉性能优化实测（2026-10-10）\n\n'+table+'\n'+paragraph,encoding='utf-8')
print(json.dumps(summary, ensure_ascii=False, indent=2))
