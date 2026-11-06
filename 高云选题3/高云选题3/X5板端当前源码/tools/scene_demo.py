"""Actual OCR + float ONNX inference on existing synthetic map views.

Creates an offline scene report for teaching. No camera, ROS, UART or hardware
is accessed. Three brightness variants exercise temporal logic, not real video.
"""
from pathlib import Path
import argparse
import base64
import hashlib
import html
import importlib.util
import json
import sys
import time

import cv2
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
REVIEWED_VIEW_SHA256 = '5772a72c94059f762cd6eaeb3547173b505a6604e3df19534430c7c2b05b6556'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', type=Path, default=ROOT.parents[1] / 'map_assets/rescue_maze_v6')
    parser.add_argument('--float-package', type=Path, default=ROOT.parents[1] / 'YOLO11n_model/deployment/rdkx5_firefighter_v4')
    parser.add_argument('--desktop-vendor', action='store_true',
                        help='Use Windows desktop OCR verification dependencies; never use on X5')
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
    if args.desktop_vendor:
        sys.path.insert(0, str(ROOT / 'vendor/ocr_python'))
    from rescue_vision.building_ocr import OCRReader, BuildingTracker
    from rescue_vision.semantics import SceneMemory
    spec = importlib.util.spec_from_file_location('demo_float_detector', args.float_package / 'runtime/detector.py')
    detector = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(detector)
    onnx_path = args.float_package / 'models/wallpaper_yolo11n_raw.onnx'
    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = 2
    session_options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(onnx_path), session_options, providers=['CPUExecutionProvider'])
    ocr = OCRReader(ROOT / 'models/ocr', threads=2)
    config = ROOT / 'config/buildings.json'
    tracker = BuildingTracker(config)
    memory = SceneMemory(config)
    path = args.map_dir / 'renders/sign_H_08.png'
    if sha(path) != REVIEWED_VIEW_SHA256:
        raise ValueError('Map view changed: manually review the single-facade ROI again before accepting this source')
    image = detector.read_image(path)
    if image.shape[:2] != (960, 1280):
        raise ValueError('Demonstration ROI was manually reviewed only for the original 1280x960 H view')
    # Manually reviewed only for this exact static synthetic view. It is NOT a
    # camera calibration and must not be transferred to a moving robot.
    facade_roi = [240, 95, 975, 700]
    frames = []
    for index, gain in enumerate((.97, 1.0, 1.02)):
        frame = np.clip(image.astype(np.float32) * gain, 0, 255).astype(np.uint8)
        stamp = index * .25
        frame_id = f'simulated_brightness_frame_{index}'
        start = time.perf_counter()
        readings = ocr.predict(frame)
        padded, meta = detector.letterbox(frame)
        raw = session.run(list(detector.OUTPUT_NAMES), {session.get_inputs()[0].name: detector.rgb_tensor(padded)})
        detected = detector.detections(raw, meta, conf=.4)
        boxes = [dict(class_name=d['class_name'], confidence=d['confidence'], bbox=d['bbox_xyxy']) for d in detected]
        building = tracker.update(readings, stamp, frame_id)
        report = memory.observe(building, boxes, stamp, frame_id, facade_roi, True)
        frames.append(dict(frame_id=frame_id, simulated_capture_stamp_s=stamp, brightness_gain=gain,
                           input_pixel_sha256=hashlib.sha256(frame.tobytes()).hexdigest(),
                           inference_seconds_windows_host=time.perf_counter()-start,
                           ocr=readings, building=building, detections=boxes, report=report))
    last = frames[-1]
    unverified = SceneMemory(config).observe(last['building'], last['detections'], .5, last['frame_id'])
    expired = memory.current(3.0)
    conflict_path = args.map_dir / 'renders/sign_B_02.png'
    conflict_readings = ocr.predict(detector.read_image(conflict_path))
    conflict_building = BuildingTracker(config).update(conflict_readings, 0, 'conflicting_buildings')
    conflict = SceneMemory(config).observe(conflict_building, [], 0, 'conflicting_buildings')
    result = dict(
        purpose='Offline scene-understanding algorithm exercise; not recognition accuracy or hardware validation',
        input_kind='existing synthetic map render, three simulated brightness variants',
        real_camera=False, board_inference=False, bpu_inference=False,
        fake_or_handwritten_detections=False,
        temporal_test_scope='Three synthetic variants of one static frame; not independent real-camera temporal evidence',
        source=dict(file=str(path), sha256=sha(path), image_size_wh=[1280, 960]),
        object_model=dict(file=str(onnx_path), sha256=sha(onnx_path), runtime='ONNX Runtime CPU FP32',
                          confidence_threshold=.4, quantized_bin_accuracy_not_inferred=True),
        ocr_model_manifest=json.loads((ROOT / 'models/ocr/manifest.json').read_text(encoding='utf-8')),
        facade_roi=dict(xyxy=facade_roi, verification='manually reviewed fixed synthetic H facade only',
                        valid_only_for_source_sha256=sha(path), real_robot_verified=False),
        frames=frames,
        states=dict(confirmed=last['report'], unverified=unverified, expired=expired, conflict=conflict),
        conflicting_view=dict(file=str(conflict_path), sha256=sha(conflict_path), ocr=conflict_readings,
                              building=conflict_building),
        semantic_request=dict(text='去3号楼观察', result=memory.resolve_request('去3号楼观察', .5)),
    )
    reports = ROOT / 'reports'
    reports.mkdir(exist_ok=True)
    (reports / 'scene_demo.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    build_html(result, image, detector.read_image(conflict_path), reports / 'scene_demo.html')
    print(json.dumps(dict(report=last['report'], semantic_goal=result['semantic_request']['result'],
                          html=str(reports / 'scene_demo.html')), ensure_ascii=False, indent=2))


def build_html(result, image, conflict_image, target):
    def jpeg_url(bgr):
        okay, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not okay:
            raise RuntimeError('JPEG encoding failed')
        return 'data:image/jpeg;base64,' + base64.b64encode(encoded).decode('ascii')

    payload = json.dumps(result, ensure_ascii=False).replace('<', '\\u003c')
    template = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>街区场景观察 · RDK X5 离线演示</title>
<style>
:root{font-family:Inter,"Microsoft YaHei",system-ui,sans-serif;color:#17263e;background:#f1f4f7;font-synthesis:none}
*{box-sizing:border-box}body{margin:0}header{background:#142c46;color:white;padding:34px max(24px,calc((100vw - 1340px)/2));border-bottom:5px solid #28b8b0}
header small{letter-spacing:2px;color:#8cdbd8}h1{margin:12px 0;font-size:30px;font-weight:650}header p{margin:0;line-height:1.8;color:#d0dce6;font-size:14px;max-width:1060px}
main{max-width:1388px;padding:24px;margin:auto}.notice{padding:16px 20px;border:1px solid #c6dce9;background:#e6f0f7;border-radius:12px;font-size:14px;line-height:1.8;margin-bottom:22px}
.toolbar{display:flex;gap:9px;flex-wrap:wrap;margin-bottom:18px}button{font:inherit;font-size:14px;border:1px solid #d1dbe4;color:#35475b;background:white;border-radius:9px;padding:11px 17px;cursor:pointer}button[aria-pressed=true]{background:#176a75;border-color:#176a75;color:white}button:focus-visible{outline:3px solid #40b9ca;outline-offset:3px}
.layout{display:grid;grid-template-columns:minmax(0,1.55fr) minmax(350px,1fr);gap:22px}.card{background:white;border:1px solid #dce4eb;border-radius:16px;overflow:hidden;box-shadow:0 4px 18px #1d344709}
.cardhead{padding:18px 22px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid #e4eaf0}.cardhead h2{font-size:17px;margin:0}.tag{padding:5px 9px;border-radius:6px;background:#e8f5f3;color:#176a65;font-size:12px;white-space:nowrap}.view{background:#dfe5e7;position:relative}.view svg{display:block;width:100%;height:auto}.legend{display:flex;gap:18px;flex-wrap:wrap;padding:14px 20px;font-size:12px;color:#5b6978}.dot{display:inline-block;width:9px;height:9px;border-radius:3px;margin-right:5px}
.summary{padding:24px}.building{font-size:28px;margin:3px 0 10px;letter-spacing:-.5px}.explain{font-size:14px;color:#607185;line-height:1.75;margin:0 0 18px}.metrics{display:grid;grid-template-columns:1fr 1fr;gap:12px}.metric{padding:17px;border-radius:11px;background:#f4f7fa;min-height:101px}.metric span{display:block;color:#61758a;font-size:13px;margin-bottom:10px}.metric strong{font-size:25px;font-weight:600}.metric.fire{background:#fff0e9}.metric.fire strong{color:#c84c25}.metric small{font-size:12px;color:#748192;margin-left:5px}.unknown{background:#fff6de;color:#7c6022;padding:13px 15px;margin-top:16px;border-radius:9px;font-size:13px;line-height:1.7}.action{padding:15px 0 0;line-height:1.75;font-size:13px;color:#4d6378}
.row{display:grid;grid-template-columns:1fr 1fr;gap:22px;margin-top:22px}.body{padding:20px 22px;font-size:14px;line-height:1.85}.route{display:flex;gap:9px;flex-wrap:wrap;align-items:center;margin:14px 0}.route b{background:#edf4f6;padding:8px 12px;border-radius:8px;font-size:13px}.muted{color:#607185;font-size:13px}.timeline{display:grid;grid-template-columns:repeat(3,1fr);gap:9px}.frame{padding:13px;background:#f4f7fa;border-radius:9px}.frame strong{display:block;color:#1c6d77;font-size:20px;margin-bottom:7px}.frame span{font-size:12px;line-height:1.6}.data{margin-top:22px}details summary{padding:18px 22px;cursor:pointer;font-weight:600}pre{margin:0;background:#132537;color:#cfdee8;padding:20px;font-size:12px;line-height:1.65;overflow:auto;max-height:530px;white-space:pre-wrap;overflow-wrap:anywhere}footer{font-size:12px;color:#6e7d8c;line-height:1.85;padding:22px 3px}.path{overflow-wrap:anywhere}a{color:#166a77}
@media(max-width:950px){.layout,.row{grid-template-columns:1fr}header{padding:25px}h1{font-size:25px}main{padding:16px}.summary{padding:20px}}@media(max-width:450px){.timeline{grid-template-columns:1fr}.cardhead{gap:8px;padding:15px}.toolbar button{flex:1}.metric{padding:13px}}
</style></head><body>
<header><small>RESCUE ROBOT · SCENE OBSERVATION</small><h1>小车怎样“看懂”眼前这栋楼</h1><p>实际 OCR + 已训练 YOLO 浮点模型推理 → 同楼面证据关联 → 场景报告 → 符号导航目标。所有画面来自现有三维地图，未连接真实相机、RDK X5、云台或 FPGA。</p></header>
<main><div class="notice"><b>这是离线功能演示，不是准确率或实机验收。</b> 目标框来自真正的模型推理；楼面范围由人工核对这张固定模拟视图。三帧仅为同一模拟画面的轻微亮度变化，用来演示确认逻辑，不能当作独立实拍验证。</div>
<div class="toolbar" role="group" aria-label="场景报告分支"><button data-state="confirmed" aria-pressed="true">已确认楼面</button><button data-state="unverified" aria-pressed="false">楼面尚未验证</button><button data-state="expired" aria-pressed="false">观察已过期</button><button data-state="conflict" aria-pressed="false">多个楼牌冲突</button></div>
<div class="layout"><section class="card"><div class="cardhead"><h2 id="view-title">08 幸福楼 · 模拟相机视图</h2><span class="tag">原图 1280 × 960</span></div><div class="view"><svg viewBox="0 0 1280 960" aria-label="模拟相机画面和识别结果"><image id="scene-image" href="__MAIN_IMAGE__" width="1280" height="960"></image><g id="overlays"></g></svg></div><div class="legend"><span><i class="dot" style="background:#ff7049"></i>火焰</span><span><i class="dot" style="background:#4bb4fc"></i>普通人物</span><span><i class="dot" style="background:#2ed4a0"></i>消防员</span><span><i class="dot" style="background:#cc9aff"></i>OCR</span><span>虚线：人工核对的主楼面范围</span></div></section>
<section class="card"><div class="cardhead"><h2>当前观察报告</h2><span class="tag" id="valid-tag">证据有效</span></div><div class="summary"><h3 class="building" id="building"></h3><p class="explain" id="explain"></p><div class="metrics"><div class="metric fire"><span>火焰状态</span><strong id="fire"></strong></div><div class="metric"><span>当前可见普通人物</span><strong id="people"></strong><small>个</small></div><div class="metric"><span>当前可见消防员</span><strong id="firefighter"></strong><small>个</small></div><div class="metric"><span>受困 / 伤亡人数</span><strong>未知</strong></div></div><div class="unknown">person 类只说明发现普通人物图案，不能据此判定受困或伤亡。人数限于本次可见楼面；不代表整栋楼，也不跨帧累计。</div><div class="action" id="action"></div></div></section></div>
<div class="row"><section class="card"><div class="cardhead"><h2>三帧模拟观察</h2><span class="tag">算法流程演练</span></div><div class="body"><div class="timeline" id="timeline"></div><p class="muted">每一帧都重新执行 OCR 与 YOLO。固定视角下识别同一楼牌，第三帧满足确认条件。火焰和人物计数取第三帧，而非三帧相加。</p></div></section>
<section class="card"><div class="cardhead"><h2>把语言目标交给导航</h2><span class="tag">仅输出目标</span></div><div class="body"><b>输入：“去 3 号楼观察”</b><div class="route"><b>03 春晖楼</b><span>→</span><b>building_id: C</b><span>→</span><b>observe_C</b></div><p class="muted">已解析为地图中的观察目标。can_execute=false：仍需实物地图对齐、定位、路径规划、避障和阿克曼行驶控制，当前不会发送行车命令。</p></div></section></div>
<section class="card data"><details><summary>查看当前分支的结构化 JSON 与证据</summary><pre id="json"></pre></details></section>
<footer>识别来源：RapidOCR 1.4.4 / PP-OCRv4，YOLO11n 原始浮点 ONNX，均在这台 Windows 电脑的 CPU 上执行。本页结果不能代表 X5 量化 BIN 的质量或性能。<br>地图源图：<span class="path" id="source"></span><br>全部输入 SHA256、真实推理结果与分支：<a href="scene_demo.json">scene_demo.json</a>。楼面 ROI 只对这张固定模拟视图有效，不可直接作为移动云台的真实楼面标定。</footer></main>
<script id="demo-data" type="application/json">__PAYLOAD__</script><script>
const data=JSON.parse(document.getElementById('demo-data').textContent);
const mainImage='__MAIN_IMAGE__',conflictImage='__CONFLICT_IMAGE__';
const colors={fire:'#ff7049',person:'#4bb4fc',firefighter:'#2ed4a0'};
const esc=s=>String(s).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;');
const rect=(box,color,label,dashed=false)=>`<rect x="${box[0]}" y="${box[1]}" width="${box[2]-box[0]}" height="${box[3]-box[1]}" fill="none" stroke="${color}" stroke-width="3" ${dashed?'stroke-dasharray="12 8"':''}/>${label?`<text x="${box[0]+3}" y="${Math.max(23,box[1]-7)}" fill="${color}" stroke="#142638" stroke-width="4" paint-order="stroke" font-size="19" font-family="Microsoft YaHei,sans-serif">${esc(label)}</text>`:''}`;
function render(state){
 const r=data.states[state],f=data.frames.at(-1),conflict=state==='conflict';
 document.querySelectorAll('[data-state]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.state===state)));
 document.getElementById('scene-image').setAttribute('href',conflict?conflictImage:mainImage);
 document.getElementById('view-title').textContent=conflict?'多个楼牌 · 停止自动归属':'08 幸福楼 · 模拟相机视图';
 let overlay='';
 if(!conflict){overlay+=rect(data.facade_roi.xyxy,'#ffffff','人工核对的固定主楼面',true);for(const d of f.detections)overlay+=rect(d.bbox,colors[d.class_name],d.class_name+' '+d.confidence.toFixed(2));}
 const readings=conflict?data.conflicting_view.ocr:f.ocr;
 for(const o of readings){const xs=o.quad.map(p=>p[0]),ys=o.quad.map(p=>p[1]);overlay+=rect([Math.min(...xs),Math.min(...ys),Math.max(...xs),Math.max(...ys)],'#cc9aff',o.text);}
 document.getElementById('overlays').innerHTML=overlay;
 document.getElementById('building').textContent=r.building_number?`${r.building_number} ${r.building_name}`:'楼栋尚未确认';
 const descriptions={confirmed:'已把楼牌与本次主楼面检测框关联；背景楼的目标被排除。',unverified:'楼牌可以读出，但还没有验证这些目标属于同一楼面，暂不分配楼内状态。',expired:'这是历史楼牌证据，观察已超过有效期；当前火焰与人数全部撤销为未知。',conflict:'OCR 同时读到 02 向阳楼和 01 安居楼，程序不擅自选择其中一栋。'};
 document.getElementById('explain').textContent=descriptions[state];
 document.getElementById('valid-tag').textContent=r.valid?'同帧楼面证据有效':'需要重新观察';
 document.getElementById('fire').textContent={observed:'检出火焰',not_observed:'本次未检出',unknown:'未知'}[r.fire_status];
 document.getElementById('people').textContent=r.visible_person_count??'—';document.getElementById('firefighter').textContent=r.visible_firefighter_count??'—';
 document.getElementById('action').textContent='建议：'+(r.valid?'停车并进一步观察火焰；喷水须满足独立标定与执行条件。':'保持未知，重新核对楼牌、楼面范围及新鲜图像。');
 document.getElementById('json').textContent=JSON.stringify(r,null,2);
}
document.getElementById('timeline').innerHTML=data.frames.map((f,i)=>`<div class="frame"><strong>${i+1} / 3</strong><span>亮度系数 ${f.brightness_gain}<br>楼牌：${esc(f.building.number||'待确认')}<br>${f.building.confirmed?'已确认':'积累证据'}</span></div>`).join('');
document.getElementById('source').textContent=data.source.file;
document.querySelectorAll('[data-state]').forEach(b=>b.addEventListener('click',()=>render(b.dataset.state)));
render('confirmed');
</script></body></html>'''
    text = template.replace('__PAYLOAD__', payload).replace('__MAIN_IMAGE__', jpeg_url(image)).replace('__CONFLICT_IMAGE__', jpeg_url(conflict_image))
    target.write_text(text, encoding='utf-8')


if __name__ == '__main__':
    main()
