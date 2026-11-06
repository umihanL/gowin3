#!/usr/bin/env python3
"""Audit and summarize this round without overwriting first-round evidence."""
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / 'logs/vision_pipeline_v2_20261010'


def read(name):
    return json.loads((REPORT / name).read_text(encoding='utf-8'))


def metrics(name, stream_on):
    data = read(name)
    samples = data['samples']
    for s in samples:
        p = s['performance']
        assert p and not p['stage_errors'], (name, 'stale/error diagnostics')
        assert s['stream']['enabled'] == stream_on
        assert s['stream']['active'] == stream_on
        assert s['state'] == ('live' if stream_on else 'paused'), (name, s['state'])
        assert p['capture_resolution'] == p['preview_resolution'] == [1280, 1088]
        assert p['model_resolution'] == [640, 640] and p['jpeg_quality'] == 90
        assert p['inference_backend'] == 'native_hbdnn' and p['enabled_classes'] == ['fire']
        assert p['processing_fps'] > 0 and p['processing_last_age_s'] < 2
    sequence = [s['performance']['sequence'] for s in samples]
    assert all(b > a for a, b in zip(sequence, sequence[1:])), (name, 'non-advancing inference')
    fps = [s['performance']['processing_fps'] for s in samples]
    ages = sorted(1000*s['frame_age_s'] for s in samples if s['frame_age_s'] is not None)
    elapsed = samples[-1]['benchmark_monotonic'] - samples[0]['benchmark_monotonic']
    result = dict(duration_s=data['summary']['elapsed_s'], samples=len(samples),
        processing_fps_median=statistics.median(fps), processing_fps_range=[min(fps), max(fps)],
        stage_ms_median=data['summary']['stage_ms_median'], audit_all_samples_passed=True)
    if stream_on:
        result.update(preview_fps_median=statistics.median(s['fps'] for s in samples),
            preview_pair_rate=(samples[-1]['paired_frames']-samples[0]['paired_frames'])/elapsed,
            image_age_ms_median=statistics.median(ages), image_age_ms_p90=ages[int(.9*(len(ages)-1))])
    else:
        counts = [s['performance']['preview_encoded_frames'] for s in samples]
        assert len(set(counts)) == 1, 'JPEG must stop when transmission is disabled'
        assert all(s['performance']['preview_subscribers'] == 0 for s in samples)
        result['switch_check'] = dict(jpeg_count_start=counts[0], jpeg_count_end=counts[-1],
            sequence_start=sequence[0], sequence_end=sequence[-1], preview_subscribers=0)
    return result


baseline = metrics('baseline_on.json', True)
intermediate = metrics('raw_fused_on.json', True)
native_trial = metrics('native_remap_on.json', True)
final = metrics('final_on.json', True)
off = metrics('final_off.json', False)
for name in ('final_on.json', 'final_off.json'):
    for s in read(name)['samples']:
        p = s['performance']
        assert p['raw_subscription'] and p['preprocess_path'] == 'native_fused_hd'
        assert p['lens_metadata']['remap_backend'] == 'native_fixed_bilinear'
improvement = 100*(final['processing_fps_median']/baseline['processing_fps_median']-1)
summary = dict(client_date='2026-10-10', baseline=baseline, raw_fused_and_candidates=intermediate,
    native_trial=native_trial, final_stream_on=final, final_stream_off=off,
    improvement_percent=improvement,
    configuration=dict(camera_preview=[1280,1088], model_input=[640,640], jpeg_quality=90,
        remap_threads=8, preprocess_threads=2, opencv_threads=8, rate_cap=30,
        raw_subscription=True, remap_backend='native_fixed_bilinear',
        preprocess_path='native_fused_hd', inference_backend='native_hbdnn'),
    note='Detection FPS is the median of 1 Hz samples of a 60-frame rolling rate; '
         'preview_fps is the server paired-frame rolling rate, not browser rendering FPS. '
         'All samples audited for live/paused state, fresh advancing diagnostics, errors, '
         'resolution and backend. Relative time uses the board clock; run duration is monotonic. '
         'Camera stalled between baseline and deployment; it was restarted and the outage is excluded. '
         'This is a short live-scene comparison, not a long thermal or recognition-accuracy test.')
(REPORT/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
checks = off['switch_check']
text = f'''# 第二轮高清视觉提速实测（2026-10-10）

| 测量项 | 本轮优化前 | 本轮优化后 |
|---|---:|---:|
| 开传图：检测速率中位数 | {baseline['processing_fps_median']:.2f} FPS | {final['processing_fps_median']:.2f} FPS |
| 开传图：服务端配对速率中位数 | {baseline['preview_fps_median']:.2f} FPS | {final['preview_fps_median']:.2f} FPS |
| 预览图像年龄中位数 | {baseline['image_age_ms_median']:.0f} ms | {final['image_age_ms_median']:.0f} ms |
| 关传图：检测速率中位数 | 本轮未测 | {off['processing_fps_median']:.2f} FPS |

本轮检测提升约 **{improvement:.1f}%**。最终开传图测量 {final['duration_s']:.0f} 秒，检测滚动速率范围 {final['processing_fps_range'][0]:.2f}～{final['processing_fps_range'][1]:.2f} FPS；预览整段服务端配对速率 {final['preview_pair_rate']:.2f} FPS，图像年龄 P90 为 {final['image_age_ms_p90']:.0f} ms。服务端配对速率不等于浏览器绘制速率，瞬间峰值不代表持续性能。

中间版本（减少ROS像素复制、融合预处理、候选框预筛）为 {intermediate['processing_fps_median']:.2f} FPS；再接入相同像素输出的专用去畸变实现。仍为1280×1088/JPEG90、同一640×640模型、同一颜色/插值/阈值/NMS。默认原生去畸变8线程、融合预处理2线程、OpenCV8线程，OpenMP等待策略PASSIVE。

关闭传图后测量 {off['duration_s']:.0f} 秒，JPEG计数始终 {checks['jpeg_count_start']}，检测序号 {checks['sequence_start']}→{checks['sequence_end']}，订阅数为0，证明检测继续而网页编码停止。检测序号可能跳过未处理原图，因此不用序号差估算检测FPS。

验证证据：生产去畸变6项测试、原有镜头8项测试、强制原生后端的3项ROS流水线测试全部通过。流水线测试比较完整像素、JPEG字节、检测坐标和时间戳配对；融合预处理4项、raw CDR6项、候选框8项测试也通过。此前原型34组像素对比、融合输入32组逐字节对比是补充证据。模型BIN与BPU运行内核未改变，前一轮同输入48个模型输出头一致的证据仍保留在兄弟目录。

所有最终样本均已检查实时状态、配置、错误和检测持续前进。基线之后曾发生参考眼停流，重启相机进程后恢复；故障区间不计入结果，原因尚未确定。测试使用单调时钟记录时长，板端RTC回退未在采集中强行修改。该结果属于当前画面与负载的短时测试，不能证明任何场景都达到相同FPS，也不能替代打印壁纸的实拍准确率验收。

原始数据与回退包都在本目录：`baseline_on.json`、`raw_fused_on.json`、`native_remap_on.json`、`final_on.json`、`final_off.json`、`final_health.json`、`installed_hashes.json`、`before_code.tgz`。
'''
(REPORT/'results.md').write_text(text, encoding='utf-8')
print(json.dumps(summary, ensure_ascii=False, indent=2))
