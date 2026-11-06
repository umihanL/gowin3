"""Build an offline image/automatic-label browser for the merged V5/V6 dataset and optional train-only V7 captures.

Only dataset/review.html and review_index.json are written. No network, trained
model or prediction is used; labels remain the packaged annotation truth.
"""
from __future__ import annotations

import argparse
from collections import Counter
import html
import json
from pathlib import Path

DEFAULT_ROOT = Path(__file__).resolve().parents[1] / "dataset"
CLASSES = ["fire", "firefighter", "person"]


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


TEMPLATE = r'''<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>壁纸数据 · 相机采集审阅</title>
<style>
:root{--bg:#edf2f4;--card:#fff;--line:#d7e1e7;--ink:#173447;--muted:#586e7c;--blue:#126f8a;--fire:#ff7047;--fighter:#25c5cb;--person:#ae7dff}*{box-sizing:border-box}body{margin:0;color:var(--ink);background:var(--bg);font:14px/1.65 "Microsoft YaHei",system-ui,sans-serif}header,main{max-width:1530px;margin:auto;padding:24px 30px}header{padding-top:30px}small{color:var(--blue);letter-spacing:1.5px}h1{margin:8px 0;font-size:30px;line-height:1.4}p{margin:7px 0;color:var(--muted)}.notice{border-left:3px solid #c29347;padding:9px 14px;background:#fff9ed;border-radius:0 7px 7px 0}.stats{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:13px;margin:20px 0}.stat,.panel,.frame{border:1px solid var(--line);border-radius:12px;background:var(--card)}.stat{padding:13px 19px}.stat b{display:block;font-size:25px}.stat span{font-size:12px;color:var(--muted)}main{padding-top:0}.filters{display:flex;align-items:end;gap:10px;flex-wrap:wrap;padding:17px}.filters label{display:flex;flex-direction:column;gap:3px;font-size:12px;color:var(--muted)}select,button,input[type=search]{border:1px solid #cbd9e2;border-radius:7px;min-height:39px;padding:8px 10px;background:#fff;color:var(--ink);font:inherit}select{max-width:175px}input[type=search]{width:190px}button{cursor:pointer}button.primary{background:var(--blue);color:white;border-color:var(--blue)}button:disabled{opacity:.4;cursor:default}.actions{display:flex;gap:9px;align-items:center;flex-wrap:wrap;margin:14px 0}.spacer{flex:1}.tag{padding:3px 9px;border-radius:5px;background:#e7f0f5;font-size:12px}.workspace{display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:18px}.frame{overflow:hidden}.frame-title{display:flex;justify-content:space-between;gap:10px;padding:12px 15px;border-bottom:1px solid var(--line);font-size:13px}.canvas-wrap{background:#dfe7eb;min-height:240px;display:flex;align-items:center;justify-content:center}canvas{display:block;width:100%;height:auto;max-height:72vh;object-fit:contain}aside{display:flex;flex-direction:column;gap:15px}.panel{padding:17px}.panel h2{font-size:15px;margin:0 0 10px}dl{display:grid;grid-template-columns:68px 1fr;gap:6px;margin:0;font-size:12px}dt{color:var(--muted)}dd{margin:0;overflow-wrap:anywhere}.legend{display:flex;gap:15px;flex-wrap:wrap;font-size:12px;padding:11px 15px}.legend i{display:inline-block;width:10px;height:10px;margin-right:5px;border-radius:2px}.toggle{display:flex;align-items:center;gap:6px;font-size:12px}.object-table{max-height:245px;overflow:auto}table{border-collapse:collapse;width:100%;font-size:12px}th,td{padding:5px;border-bottom:1px solid var(--line);text-align:left}th{position:sticky;top:0;background:white}a{color:var(--blue)}footer{font-size:12px;color:var(--muted);padding:20px 0}#status{font-size:12px;color:var(--muted);min-height:25px;overflow-wrap:anywhere}.quality{padding:10px 0;font-size:12px}.empty{padding:70px 20px;text-align:center;color:var(--muted)}@media(max-width:1000px){.workspace{grid-template-columns:1fr}aside{display:grid;grid-template-columns:1fr 1fr}canvas{max-height:none}}@media(max-width:650px){header,main{padding:18px 13px}.stats{grid-template-columns:1fr 1fr}h1{font-size:25px}aside{display:flex}.filters{padding:12px}select{max-width:150px}.frame-title{flex-direction:column}.spacer{display:none}}
</style></head><body>
<header><small>RESCUE LAB / OFFLINE DATA REVIEW</small><h1>居民楼壁纸，多角度的相机数据。</h1><p>按采集来源、划分、光照和左右目检查火焰、消防员、普通人的自动标注。</p><div class="stats" id="stats"></div><p class="notice">画面来自仿真渲染，框来自可见实例掩膜，<strong>不是模型预测结果</strong>。这里只训练 fire / firefighter / person 三类；门牌作为场景背景。查看本页不需要启动服务器或联网。</p></header>
<main><section class="panel filters"><label>采集来源<select id="source"><option value="all">全部来源</option></select></label><label>划分<select id="split"><option value="all">全部</option><option value="train">训练 train</option><option value="val">验证 val</option><option value="test">测试 test</option></select></label><label>类别<select id="class"><option value="all">全部类别</option><option value="0">火焰 fire</option><option value="1">消防员 firefighter</option><option value="2">普通人 person</option><option value="negative">无目标画面</option></select></label><label>光照<select id="lighting"><option value="all">全部光照</option></select></label><label>左右目<select id="eye"><option value="all">双目全部</option><option value="L">左目 L</option><option value="R">右目 R</option></select></label><label>增强<select id="variant"><option value="all">全部版本</option><option value="original">原始渲染</option><option value="augmented">训练增强</option></select></label><label>样本 ID<input id="search" type="search" placeholder="输入部分 ID"></label><button id="reset">重置</button></section>
<div class="actions"><button id="prev">← 上一张</button><button id="next" class="primary">下一张 →</button><button id="random">随机</button><button id="other-eye">切换同对另一目</button><span id="counter" aria-live="polite"></span><span class="spacer"></span><label class="toggle"><input id="boxes" type="checkbox" checked>自动标注框</label><label class="toggle"><input id="tiny" type="checkbox" checked>极小目标</label></div>
<div class="workspace"><section class="frame"><div class="frame-title"><strong id="sample-title">加载数据</strong><span id="version-tag" class="tag"></span></div><div class="canvas-wrap"><canvas id="image" width="640" height="480" aria-label="相机图像和自动标注框"></canvas></div><div class="legend"><span><i style="background:var(--fire)"></i>火焰</span><span><i style="background:var(--fighter)"></i>消防员</span><span><i style="background:var(--person)"></i>普通人</span><a id="raw-link" href="#" target="_blank">原始图片</a><a id="label-link" href="#" target="_blank">YOLO 标签</a></div></section>
<aside><section class="panel"><h2>采集与追溯</h2><dl id="metadata"></dl></section><section class="panel"><h2>当前图可见目标</h2><div id="objects" class="object-table"></div><div id="quality" class="quality"></div></section></aside></div><div id="status"></div>
<footer>同一双目对、原始位姿组和空间采集组跨来源固定在原划分中；增强仅进入训练集。V7 人物近中距离采集与 V8 楼前观察采集均使用原训练区域中的新位姿，仅加入训练集；V9 是经用户指定转入训练的新场景，不能再作独立验证。原验证和测试图像及标签保持不变。极小目标原标注保留，隐藏框只影响本页显示。<br>相机参数是约定的合成近似，尚非实机标定；重复壁纸图案也不是新外观样本。合成测试指标必须与实拍指标分开。<br><a href="data.yaml">数据配置</a> · <a href="manifest.json">融合来源清单</a> · <a href="independent_validation.json">独立审计</a> · <a href="review_index.json">审阅页索引摘要</a></footer></main>
<script id="review-data" type="application/json">__DATA__</script><script>
'use strict';
const DATA=JSON.parse(document.getElementById('review-data').textContent),$=id=>document.getElementById(id),names=DATA.classes,colors=['#ff7047','#25c5cb','#ae7dff'];let selected=[],index=0,token=0,current=null;
const byKey=new Map(DATA.samples.map(s=>[s.source+'|'+s.pair+'|'+s.variant+'|'+s.eye,s]));
$('stats').innerHTML=[[DATA.samples.length,'可审阅图片'],[DATA.labels,'自动标注框'],[DATA.negative_images,'无目标图片'],[DATA.sources.length,'采集来源']].map(([n,l])=>`<div class="stat"><b>${n.toLocaleString()}</b><span>${l}</span></div>`).join('');
for(const [id,values] of [['source',DATA.sources],['lighting',[...new Set(DATA.samples.map(s=>s.light))].sort()]]){for(const value of values){const option=document.createElement('option');option.value=value;option.textContent=id==='source'?({v5:'V5 原版',v6:'V6 一楼门头版',v7:'V7 人物近中距离版',v8:'V8 楼前观察·暗光·斜视·遮挡版',v9:'V9 消防员新场景训练版'}[value]??value):value;$(id).append(option)}}
function filter(preferred){const needle=$('search').value.trim().toLowerCase();selected=DATA.samples.filter(s=>($('source').value==='all'||s.source===$('source').value)&&($('split').value==='all'||s.split===$('split').value)&&($('lighting').value==='all'||s.light===$('lighting').value)&&($('eye').value==='all'||s.eye===$('eye').value)&&($('variant').value==='all'||($('variant').value==='original'?s.variant==='original':s.variant!=='original'))&&(!needle||s.id.toLowerCase().includes(needle))&&($('class').value==='all'||($('class').value==='negative'?s.objects.length===0:s.objects.some(o=>o.c===Number($('class').value)))));index=Math.max(0,selected.findIndex(s=>s.id===preferred));show()}
function show(){const request=++token;current=selected.length?selected[(index+selected.length)%selected.length]:null;const canvas=$('image'),ctx=canvas.getContext('2d');$('status').textContent='';for(const id of ['prev','next','random'])$(id).disabled=!current;if(!current){$('counter').textContent='没有符合筛选条件的图片';$('sample-title').textContent='调整筛选条件';$('version-tag').textContent='';ctx.clearRect(0,0,canvas.width,canvas.height);$('metadata').replaceChildren();$('objects').replaceChildren();$('quality').textContent='';$('other-eye').disabled=true;return}index=(index+selected.length)%selected.length;const s=current;$('counter').textContent=`${index+1} / ${selected.length}`;$('sample-title').textContent=s.id;$('version-tag').textContent=s.source+' · '+s.split+' · '+s.eye;$('raw-link').href=s.image;$('label-link').href=s.label;const other=byKey.get(s.source+'|'+s.pair+'|'+s.variant+'|'+(s.eye==='L'?'R':'L'));$('other-eye').disabled=!other;
const rows={'来源':s.source,'划分':s.split,'双目对':s.pair,'原始对':s.parentPair,'位姿组':s.pose,'空间组':s.group,'光照':s.light,'目标距离':s.distance==null?'—':s.distance.toFixed(2)+' m','墙面角度':s.angle==null?'—':s.angle.toFixed(1)+'°','局部遮挡':s.occluded?'三维薄板（训练增强）':'无新增遮挡','图像版本':s.variant,'分辨率':s.width+' × '+s.height,'源样本':s.sourceId,'内容SHA':s.sha.slice(0,16)+'…'};$('metadata').replaceChildren();for(const [key,value]of Object.entries(rows)){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=key;dd.textContent=String(value??'—');$('metadata').append(dt,dd)}
const table=document.createElement('table');table.innerHTML='<thead><tr><th>类别</th><th>框尺寸</th><th>可见像素</th></tr></thead>';const body=document.createElement('tbody');for(const o of s.objects){const tr=document.createElement('tr');for(const value of [names[o.c],(o.b[2]-o.b[0])+' × '+(o.b[3]-o.b[1])+(o.t?' · 小':''),o.a??'—']){const td=document.createElement('td');td.textContent=value;tr.append(td)}body.append(tr)}table.append(body);$('objects').replaceChildren(table);if(!s.objects.length)$('objects').textContent='空标签：该图没有可见的三类目标。';const tiny=s.objects.filter(o=>o.t).length;$('quality').textContent=`${s.objects.length} 个可见目标，${tiny} 个极小目标。框是自动标注。`;
const image=new Image();image.onload=()=>{if(request!==token)return;canvas.width=image.naturalWidth;canvas.height=image.naturalHeight;ctx.drawImage(image,0,0);if($('boxes').checked){for(const o of s.objects){if(o.t&&!$('tiny').checked)continue;const [x0,y0,x1,y1]=o.b;ctx.strokeStyle=colors[o.c];ctx.lineWidth=o.t?1:2;ctx.strokeRect(x0+.5,y0+.5,x1-x0,y1-y0);if(x1-x0>=16&&y1-y0>=18){ctx.font='11px sans-serif';const text=names[o.c],w=ctx.measureText(text).width+6,x=Math.min(x0,canvas.width-w),y=Math.max(13,y0);ctx.fillStyle='rgba(15,29,41,.85)';ctx.fillRect(x,y-13,w,14);ctx.fillStyle=colors[o.c];ctx.fillText(text,x+3,y-2)}}}$('status').textContent='读取成功 · '+s.image};image.onerror=()=>{if(request===token)$('status').textContent='图像读取失败：'+s.image};image.src=s.image;}
for(const id of ['source','split','class','lighting','eye','variant'])$(id).addEventListener('change',()=>filter());$('search').addEventListener('input',()=>filter());for(const id of ['boxes','tiny'])$(id).addEventListener('change',show);$('prev').onclick=()=>{index--;show()};$('next').onclick=()=>{index++;show()};$('random').onclick=()=>{index=Math.floor(Math.random()*selected.length);show()};$('reset').onclick=()=>{for(const id of ['source','split','class','lighting','eye','variant'])$(id).value='all';$('search').value='';filter()};$('other-eye').onclick=()=>{if(!current)return;const other=byKey.get(current.source+'|'+current.pair+'|'+current.variant+'|'+(current.eye==='L'?'R':'L'));if(other){$('eye').value='all';$('search').value='';$('class').value='all';filter(other.id)}};document.addEventListener('keydown',event=>{if(['INPUT','SELECT','TEXTAREA'].includes(event.target.tagName))return;if(event.key==='ArrowRight'){index++;show()}if(event.key==='ArrowLeft'){index--;show()}});filter();
</script></body></html>'''


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", "--input", dest="input", type=Path, default=DEFAULT_ROOT,
                   help="Merged dataset directory (--input is a compatible alias)")
    args = p.parse_args()
    root = args.input.resolve()
    manifest = read_json(root / "manifest.json")
    if manifest["classes"] != CLASSES:
        raise ValueError("This browser expects exactly fire/firefighter/person detection classes")
    samples = []
    for line in (root / "metadata.jsonl").read_text(encoding="utf-8-sig").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        annotation = read_json(root / row["annotation"])
        capture = row.get("capture_plan", {})
        provenance = row.get("provenance", {})
        objects = [{"c": a["class_id"], "b": a["bbox_xyxy"], "a": a.get("area_px"),
                    "t": bool(a.get("tiny", False)), "id": a.get("instance_id")}
                   for a in annotation["instances"]]
        samples.append({"id": row["sample_id"], "source": row.get("source_version", provenance.get("source_version", "unknown")),
                        "sourceId": row.get("source_sample_id", provenance.get("source_sample_id", row["sample_id"])),
                        "pair": row["pair_id"], "parentPair": row.get("lineage_pair_id", row.get("parent_pair_id", capture.get("parent_pair_id", row["pair_id"]))),
                        "eye": row["eye"], "split": row["split"], "group": row["group_id"], "pose": row["pose_group"],
                        "variant": row["variant"], "light": capture.get("lighting_profile", "unknown"),
                        "distance": capture.get("sampling_distance_m"), "angle": capture.get("obliquity_deg"), "occluded": bool(capture.get("occluder")),
                        "image": row["image"], "label": row["label"], "width": row["width"], "height": row["height"],
                        "sha": row["image_sha256"], "objects": objects})
    data = {"classes": CLASSES, "sources": sorted({s["source"] for s in samples}), "samples": samples,
            "labels": sum(len(s["objects"]) for s in samples), "negative_images": sum(not s["objects"] for s in samples)}
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", r"<\/")
    (root / "review.html").write_text(TEMPLATE.replace("__DATA__", payload), encoding="utf-8")
    index = {"images": len(samples), "labels": data["labels"], "negative_images": data["negative_images"],
             "sources": dict(Counter(s["source"] for s in samples)), "splits": dict(Counter(s["split"] for s in samples)),
             "source_split": dict(Counter(s["source"]+"/"+s["split"] for s in samples)),
             "offline": True, "label_origin": "Automatic visible-instance annotations, not model predictions"}
    (root / "review_index.json").write_text(json.dumps(index, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"viewer": str(root/"review.html"), **index}, ensure_ascii=False))


if __name__ == "__main__":
    main()
