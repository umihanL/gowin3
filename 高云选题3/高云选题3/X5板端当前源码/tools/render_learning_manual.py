"""Render the single canonical Markdown manual as a standalone offline reader.

Development-only dependency: pip install Markdown. No network, robot or ROS use.
"""
from pathlib import Path
import hashlib
import html
import json
import re

import markdown


WS = Path(__file__).resolve().parents[1]
SOURCE = WS / 'docs' / '完整学习与验收手册.md'
OUTPUT = WS.parent / '打开学习手册.html'


def diagram(cards, note):
    body = ''.join(f'<li><span>{i:02d}</span><div>{html.escape(text)}</div></li>'
                   for i, text in enumerate(cards, 1))
    return f'<div class="diagram"><ol>{body}</ol><p>{html.escape(note)}</p></div>'


def render():
    source = SOURCE.read_text(encoding='utf-8')
    digest = hashlib.sha256(source.encode('utf-8')).hexdigest()
    diagrams = [
        diagram(['待机 · 等待明确任务', '文字 / 语音 · 去 3 号楼',
                 'FPGA · 规划、定位、驾驶', '识别红绿灯指令 · STOP → 等 2 秒 → 找灯',
                 '红灯 / 未知 · 停；稳定绿灯且满足条件 · GO', 'FPGA · 过街完成；继续导航并到达停稳',
                 'X5 · YOLO + 楼牌 OCR', '手动 · 等指令；自动 · 本轮选一处目标短喷复查',
                 '任务结束 · 关泵、取消导航、回待机'],
                '每一步都受任务、时效和实物验收条件约束。待机看到绿灯不会自行行驶。'),
        diagram(['MIPI 相机 → 校正图像 → YOLO 与 OCR', '红绿灯观察 → mission 任务仲裁',
                 '麦克风 / 文字 → task_interaction 命令解析', '语义目的地 → fpga_navigation → UART → FPGA',
                 '火焰目标 → fire_aim → mission → 正式执行器（未启用）', '新鲜事实即时回答；提问时快照 → Qwen3-1.7B分析建议'],
                '实际电机为PD42＋ZDT_XS；ZDT已部署空载测试并取得真实通信反馈，未接入正式双轴后端。云台与水泵保持dry-run，普通PWM仅作兼容备选。暂无音箱，播报关闭。Qwen不直接执行底盘或水泵命令。'),
        diagram(['上位机开始收音 → X5 无线麦克风接收器', '默认15秒；可提前结束并提交',
                 'Silero 检测人声 → SenseVoice 转文字 → 展示转写', '提问 → 即时事实 / 历史快照与Qwen分析',
                 '明确动作 → Vosk独立复核、任务状态检查 → FPGA / 执行器', '结果 → 上位机；有音箱并启用后可播报'],
                '收声测试只检查麦克风。正式收音才进入问答或任务；模型文本不能直接驱动车辆。')]
    parts = re.findall(r'```mermaid\n(.*?)```', source, flags=re.S)
    if len(parts) != len(diagrams):
        raise ValueError('Update the offline diagram layout after changing manual diagrams')
    prepared = source
    for text, visual in zip(parts, diagrams):
        details = '<details class="graph-source"><summary>查看完整流程图源码</summary><pre>' + html.escape(text) + '</pre></details>'
        prepared = prepared.replace('```mermaid\n' + text + '```', visual + details, 1)
    # Navigation is generated from the canonical chapter headings.
    prepared = re.sub(r'## 目录\n.*?(?=<a id="c1")', '', prepared, count=1, flags=re.S)
    body = markdown.markdown(prepared, extensions=['tables', 'fenced_code', 'sane_lists'])
    body = re.sub(r'(<table>.*?</table>)', r'<div class="table-wrap">\1</div>', body, flags=re.S)
    headings = re.findall(r'^## (\d+)\. (.+)$', source, re.M)
    nav = ''.join(f'<a href="#c{n}"><span>{int(n):02d}</span>{html.escape(title)}</a>' for n, title in headings)
    page = '''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RDK X5 消防小车 · 完整学习与验收手册</title>
<style>
:root{--bg:#f3f5f3;--paper:#fff;--ink:#24332e;--muted:#68796f;--green:#14745e;--line:#dce5df;--nav:286px}*{box-sizing:border-box}html{scroll-behavior:smooth;scroll-padding-top:26px}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.9 "Microsoft YaHei","Segoe UI",sans-serif}aside{position:fixed;inset:0 auto 0 0;width:var(--nav);padding:28px 20px;background:#173a30;color:#e8f3ed;overflow:auto}.brand{font-size:20px;font-weight:700;line-height:1.5;letter-spacing:.4px}.edition{font-size:12px;color:#bdd5c7;margin:8px 0 18px}nav a{color:#d7e8de;text-decoration:none;display:flex;align-items:baseline;gap:10px;font-size:12px;line-height:1.7;padding:7px 8px;border-radius:6px}nav a span{font-family:Consolas,monospace;color:#8bb6a2;min-width:20px}nav a:hover,nav a.active{background:#285343;color:#fff}aside input{width:100%;margin-bottom:16px;padding:10px;border:1px solid #527164;border-radius:7px;background:#21483b;color:#fff;font:inherit;font-size:12px}aside input::placeholder{color:#bbd3c7}.nav-note{font-size:11px;color:#a9c8b7;margin:18px 8px}.content{margin-left:var(--nav);padding:36px 46px 70px;max-width:1510px}.top{display:flex;justify-content:space-between;align-items:center;gap:20px;color:var(--muted);font-size:12px;margin-bottom:22px}.top a{color:var(--green)}button{font-family:inherit;cursor:pointer}button.print{border:1px solid var(--line);border-radius:7px;background:#fff;padding:8px 14px;color:var(--green)}.quick{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin:0 0 26px}.quick a{background:#e5eee8;border:1px solid #d4e2d8;border-radius:10px;padding:16px;color:#224d3d;text-decoration:none}.quick small{display:block;color:#638170;font-size:11px}.quick strong{font-size:16px}article{background:var(--paper);border:1px solid var(--line);border-radius:14px;padding:34px 38px;box-shadow:0 7px 28px #22392a06}h1{font-size:28px;line-height:1.45;letter-spacing:-.3px;margin:4px 0 20px}h2{font-size:23px;line-height:1.5;color:#124f3c;padding-top:28px;border-top:1px solid var(--line);margin-top:44px}h3{font-size:18px;line-height:1.6;margin-top:29px;color:#355146}p{margin:13px 0}a{color:#117258;text-underline-offset:3px}strong{font-weight:700}code{font:13px/1.6 Consolas,"Microsoft YaHei",monospace;background:#eef2ef;padding:2px 5px;border-radius:4px;overflow-wrap:anywhere}pre{position:relative;background:#192d25;color:#ecf6ef;padding:19px 20px;border-radius:9px;overflow:auto;line-height:1.7;margin:19px 0}pre code{display:block;background:none;padding:15px 0 0;color:inherit;white-space:pre;font-size:12px;overflow-wrap:normal}.copy{position:absolute;right:9px;top:7px;padding:3px 9px;background:#375446;color:#e4f2e8;border:0;border-radius:5px;font-size:11px}.table-wrap{overflow-x:auto;margin:18px 0}table{border-collapse:collapse;width:100%;font-size:13px;line-height:1.8}th{background:#eaf1ec;color:#24503d;text-align:left}td,th{padding:11px 13px;border:1px solid var(--line);vertical-align:top}td{min-width:115px}tr:nth-child(even) td{background:#fafcf9}td code{font-size:12px}li{padding-left:2px;margin:7px 0}ol,ul{padding-left:25px}blockquote{margin:18px 0;padding:5px 18px;border-left:3px solid var(--green);background:#f0f5f0}.diagram{margin:23px 0;border:1px solid #cddfd2;border-radius:12px;background:#f7fbf6;padding:20px}.diagram ol{list-style:none;padding:0;margin:0;display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.diagram li{background:#fff;border:1px solid #d9e5db;border-radius:8px;margin:0;padding:12px;line-height:1.6;font-size:13px}.diagram li span{display:block;color:#198364;font:700 12px Consolas,monospace;margin-bottom:7px}.diagram p{font-size:12px;color:var(--muted);margin-bottom:0}.graph-source{font-size:12px;color:var(--muted)}.graph-source summary{cursor:pointer}.graph-source pre{white-space:pre-wrap;font:12px/1.7 Consolas,monospace}footer{color:var(--muted);font-size:11px;line-height:1.8;margin:25px 4px;overflow-wrap:anywhere}.mobile-nav{display:none}#progress{position:fixed;top:0;left:var(--nav);height:3px;background:#2d9874;width:0;z-index:3}
@media(max-width:1050px){:root{--nav:235px}.content{padding:25px 22px}article{padding:24px}.diagram ol{grid-template-columns:repeat(2,1fr)}}@media(max-width:750px){aside{position:relative;width:auto;max-height:280px;padding:20px}.brand{font-size:18px}nav{display:grid;grid-template-columns:1fr 1fr;gap:0 8px}nav a{font-size:11px}.nav-note{display:none}.content{margin:0;padding:18px 12px}article{padding:18px;border-radius:10px}h1{font-size:23px}h2{font-size:20px}.top{font-size:11px;margin:0 4px 17px}.quick{gap:6px}.quick a{padding:11px 9px}.quick strong{font-size:13px}.quick small{font-size:10px}#progress{left:0}.diagram{padding:12px}.diagram ol{grid-template-columns:1fr}.table-wrap{margin-right:-3px}table{min-width:520px}}
@media print{aside,.top,.quick,#progress,.copy,.graph-source{display:none}.content{margin:0;padding:0;max-width:none}body{background:white;font:11pt/1.6 sans-serif}article{padding:0;border:0;box-shadow:none}h2{break-before:page}pre{white-space:pre-wrap;background:#f1f4f2;color:#172b21}pre code{white-space:pre-wrap}a{color:inherit}table{font-size:9pt}tr,pre,.diagram li{break-inside:avoid}.table-wrap{overflow:visible}}
</style></head><body><div id="progress"></div>
<aside><div class="brand">消防小车<br>学习与验收手册</div><div class="edition">RDK X5 × FPGA · __EDITION__</div><input id="search" aria-label="筛选章节" placeholder="筛选章节；查正文用 Ctrl+F"><nav>__NAV__</nav><p class="nav-note">学习顺序：先看图与文字控制，再接语音和真实执行器。只有实测证据才能标记硬件通过。</p></aside>
<main class="content"><div class="top"><span>当前唯一学习入口 · 离线可读</span><span><a href="ros2_ws/docs/完整学习与验收手册.md">Markdown 原文</a>　<button class="print" onclick="window.print()">打印 / 保存 PDF</button></span></div>
<div class="quick"><a href="#c3"><small>第一次使用</small><strong>连接与启动 →</strong></a><a href="#c14"><small>自己验证</small><strong>自检与验收 →</strong></a><a href="#c5"><small>马上查找</small><strong>指令速查 →</strong></a></div>
<article>__BODY__</article><footer>本页由同一份最终 Markdown 自动生成。文字命令复制后需在指定终端执行；网页本身不连接控制接口。<br>正文 SHA256：__HASH__</footer></main>
<script>
'use strict';
const nav=[...document.querySelectorAll('nav a')];
document.querySelector('#search').addEventListener('input',e=>{const q=e.target.value.trim().toLowerCase();nav.forEach(a=>a.style.display=a.textContent.toLowerCase().includes(q)?'':'none');});
for(const pre of document.querySelectorAll('pre')){const code=pre.querySelector('code');if(!code)continue;const b=document.createElement('button');b.className='copy';b.textContent='复制';b.addEventListener('click',async()=>{try{await navigator.clipboard.writeText(code.textContent);b.textContent='已复制';}catch(e){const s=window.getSelection(),r=document.createRange();r.selectNodeContents(code);s.removeAllRanges();s.addRange(r);b.textContent='已选中，请 Ctrl+C';}setTimeout(()=>b.textContent='复制',2000);});pre.appendChild(b);}
const anchors=nav.map(a=>document.getElementById(a.hash.slice(1))).filter(Boolean);let scheduled=false;
function update(){scheduled=false;let active=anchors[0];for(const a of anchors){if(a.getBoundingClientRect().top<130)active=a;}nav.forEach(a=>a.classList.toggle('active',a.hash==='#'+active?.id));const max=document.documentElement.scrollHeight-innerHeight;document.querySelector('#progress').style.width=(max>0?Math.min(100,scrollY/max*100):0)+'%';}
addEventListener('scroll',()=>{if(!scheduled){scheduled=true;requestAnimationFrame(update);}},{passive:true});update();
</script></body></html>'''
    edition = re.search(r'最终学习入口\s*·\s*(\d{4}-\d{2}-\d{2})', source)
    if not edition:
        raise ValueError('Canonical manual needs an explicit edition date')
    page = (page.replace('__NAV__', nav).replace('__BODY__', body).replace('__HASH__', digest)
            .replace('__EDITION__', edition.group(1).replace('-', '.')))
    OUTPUT.write_text(page, encoding='utf-8')
    print(json.dumps({'source': str(SOURCE), 'output': str(OUTPUT), 'chapters': len(headings),
                      'source_sha256': digest}, ensure_ascii=False))


if __name__ == '__main__':
    render()
