"""Publish the V6 map preview while preserving historical V5 pages."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    html = (ROOT / 'scene-preview-v5.html').read_text(encoding='utf-8')
    html = html.replace('rescue_maze_v5', 'rescue_maze_v6').replace('community_fire_rescue_v5', 'community_fire_rescue_v6')
    html = html.replace('消防街区 V5', '消防街区 V6 · 一楼贴墙楼牌').replace("version:5", "version:6").replace('V5 · 静态场景', 'V6 · 一楼贴墙楼牌')
    html = html.replace('<button class="fold"', '<button id="signs" title="切换到离地13厘米的楼牌观察点">楼牌</button><button class="fold"')
    html = html.replace('<summary>历史版本</summary>', '<summary>历史版本</summary><a href="scene-preview-v5.html">V5 · 原始小号牌</a>')
    html = html.replace('</div><details><summary>历史版本', '</div><details><summary>13 栋编号与打印素材</summary><a href="rescue_maze_v6/assets/building_signs/contact_sheet.png" target="_blank">01–13 楼号与名称总览</a><a href="rescue_maze_v6/assets/building_signs/">原尺寸 PNG / SVG</a><p>30 × 3.8 cm，一楼门头贴纸，按实际尺寸打印。楼牌视角使用 13 cm 镜头高度。</p></details><details><summary>历史版本')
    html = html.replace("$('reset').onclick=()=>updateSensor(true);", "$('reset').onclick=()=>updateSensor(true);$('signs').onclick=()=>{if(!cfg)return;const v=cfg.camera_views.find(v=>v.id.startsWith('sign_'));if(v){$('camera').value=v.id;updateSensor(true);setMode('sensor')}};")
    html = html.replace("$('loading').hidden=true}catch(e)", "$('loading').hidden=true;const requested=new URLSearchParams(location.search).get('view');if(requested&&cfg.camera_views.some(v=>v.id===requested)){$('camera').value=requested;updateSensor(true);setMode('sensor')}else if(requested==='signs'){$('signs').click()}}catch(e)")
    (ROOT / 'scene-preview-v6.html').write_text(html, encoding='utf-8')
    if not (ROOT / 'rescue_maze_v6/community_fire_rescue_v6.glb').is_file():
        raise SystemExit('V6 page prepared; build V6 GLB before publishing main entry')
    (ROOT / 'scene-preview.html').write_text(html, encoding='utf-8')
    (ROOT / '打开消防街区.cmd').write_text('@echo off\nchcp 65001 >nul\nstart "" "D:\\仿真工具\\Blender\\blender.exe" "%~dp0rescue_maze_v6\\community_fire_rescue_v6.blend"\n', encoding='utf-8')
    print('Updated scene-preview.html, scene-preview-v6.html and Blender launcher')


if __name__ == '__main__':
    main()
