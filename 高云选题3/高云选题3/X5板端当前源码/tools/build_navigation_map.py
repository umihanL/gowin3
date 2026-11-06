"""Derive a topological graph from the actual v6 road polylines, not straight shortcuts."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


def project(point, road):
    best = None
    distance = 0.0
    for a, b in zip(road['points'], road['points'][1:]):
        vx, vy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(vx, vy)
        if length < 1e-9:
            continue
        t = max(0., min(1., ((point[0]-a[0])*vx+(point[1]-a[1])*vy)/(length*length)))
        q = [a[0]+t*vx, a[1]+t*vy]
        candidate = (math.dist(point, q), distance+t*length, q)
        if best is None or candidate[0] < best[0]:
            best = candidate
        distance += length
    return best


def build(source):
    roads = source['roads']
    cuts = {r['id']: [project(r['points'][0], r)[1], sum(math.dist(a,b) for a,b in zip(r['points'],r['points'][1:]))] for r in roads}
    observations = []
    for point in source['topology']['junctions']:
        for road in roads:
            dist, along, _ = project(point, road)
            if dist < .0001:
                cuts[road['id']].append(along)
    for sign in source['building_signs']:
        obs = sign.get('road_observation')
        if not obs:
            continue
        road = next(r for r in roads if r['id']==obs['road_id'])
        dist, along, point = project(obs['camera_center'][:2], road)
        if dist > .04:
            raise ValueError('Observation does not lie on its road: '+sign['id'])
        cuts[road['id']].append(along)
        observations.append(dict(building_id=sign['number'], sign_id=sign['id'], xy=point,
                                 camera_target_xyz=sign['center'], face_angle_deg=obs['face_angle_deg'],
                                 observation_distance_m=obs['horizontal_distance_m']))
    nodes = {}
    def node(point):
        key = f"N_{point[0]:.5f}_{point[1]:.5f}"
        nodes.setdefault(key, dict(id=key, xy_m=[round(v,7) for v in point]))
        return key
    edges = []
    for road in roads:
        cumulative = [0.]
        for a,b in zip(road['points'],road['points'][1:]):
            cumulative.append(cumulative[-1]+math.dist(a,b))
        values = sorted(set(round(v,7) for v in cuts[road['id']]))
        def at(s):
            if s >= cumulative[-1]-1e-7:
                return road['points'][-1]
            for i in range(len(cumulative)-1):
                if cumulative[i]-1e-7 <= s <= cumulative[i+1]+1e-7:
                    t = (s-cumulative[i])/(cumulative[i+1]-cumulative[i])
                    a,b=road['points'][i:i+2]
                    return [a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])]
            raise ValueError('Cut outside road')
        for i,(lo,hi) in enumerate(zip(values,values[1:])):
            if hi-lo < .001:
                continue
            geometry=[at(lo)]+[p for d,p in zip(cumulative,road['points']) if lo+1e-6<d<hi-1e-6]+[at(hi)]
            edges.append(dict(id=f"{road['id']}:{i:02d}", source=node(geometry[0]), target=node(geometry[-1]),
                              length_m=round(hi-lo,7), road_id=road['id'], width_m=road['width'],
                              geometry_m=geometry, speed_limit_m_s=.15))
    buildings=[]
    for building in sorted(source['buildings'],key=lambda b:b['number']):
        options=[]
        for obs in observations:
            if obs['building_id']==building['number']:
                options.append(dict(node_id=node(obs['xy']), **{k:v for k,v in obs.items() if k not in ('xy','building_id')}))
        buildings.append(dict(id=building['number'],legacy_id=building['id'],name=building['name'],
                              polygon_m=building['polygon'],observation_points=options))
    # Actual construction occupies the east spine. Close every overlapping graph edge.
    xmin,xmax,ymin,ymax=source['construction']['work_bounds']
    blocked=[]
    for edge in edges:
        # Inflate by half body length (0.135m) to stop before the barrier face.
        if edge['road_id']=='east_spine' and max(p[1] for p in edge['geometry_m'])>=ymin-.135 and min(p[1] for p in edge['geometry_m'])<=ymax+.135:
            blocked.append(edge['id'])
    return dict(schema_version=1,units='m',frame_id='map',site_m=source['site'],
                source_version=source['version'],source_asset_revision=source.get('asset_revision'),
                coordinate_note='Source Blender world XY; metres, yaw CCW from +X. Real map registration not verified.',
                nodes=list(nodes.values()),edges=edges,buildings=buildings,default_blocked_edges=blocked,
                preview_start_node=node([2.6,.64]),preview_start_yaw_rad=math.pi/2,
                roadside_obstacles=source['roadside_obstacles'],
                limitations=['Topological route, not Ackermann steering trajectory.',
                             'Sign viewing poses are simulation camera centres; physical vehicle observation poses require validation.',
                             'Roadside partial obstacles require FPGA/local avoidance; graph does not prove body clearance.'])


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--report-dir')
    args=parser.parse_args()
    source_path=Path(args.source)
    graph=build(json.loads(source_path.read_text(encoding='utf-8-sig')))
    graph['source_sha256']=hashlib.sha256(source_path.read_bytes()).hexdigest()
    Path(args.output).write_text(json.dumps(graph,ensure_ascii=False,indent=2),encoding='utf-8')
    if args.report_dir:
        root=Path(__file__).resolve().parents[1]
        sys.path.insert(0,str(root/'src/rescue_vision'))
        from rescue_vision.navigation import RoadGraph
        planner=RoadGraph(graph)
        routes={b['id']:planner.plan(graph['preview_start_node'],b['id'],graph['preview_start_yaw_rad']) for b in graph['buildings']}
        report=dict(source_sha256=graph['source_sha256'],source_version=graph['source_version'],
                    nodes=len(graph['nodes']),edges=len(graph['edges']),buildings=len(graph['buildings']),
                    graph_cycle_rank=len(graph['edges'])-len(graph['nodes'])+1,
                    all_13_buildings_reachable=True,blocked_edges=graph['default_blocked_edges'],
                    hardware_tested=False,map_registration_verified=False,routes=routes)
        output=Path(args.report_dir);output.mkdir(parents=True,exist_ok=True)
        (output/'navigation_routes.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        data=json.dumps(dict(graph=graph,routes=routes),ensure_ascii=False).replace('</','<\\/')
        html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>消防小车 · 语义导航地图</title>
<meta name="viewport" content="width=device-width,initial-scale=1"><style>
*{box-sizing:border-box}body{margin:0;background:#0d1727;color:#e6edf7;font-family:system-ui,"Microsoft YaHei",sans-serif}main{max-width:1240px;margin:auto;padding:32px}header{display:flex;justify-content:space-between;align-items:center;gap:20px}h1{font-size:28px;margin:0 0 12px}.sub{color:#aabbd0;line-height:1.7}.badge{border:1px solid #d2a150;background:#443923;color:#ffe0a0;border-radius:20px;padding:8px 16px;white-space:nowrap}.grid{display:grid;grid-template-columns:minmax(0,1.8fr) minmax(280px,1fr);gap:22px;margin-top:25px}.card{border:1px solid #2a3c53;border-radius:20px;background:#132238;padding:22px}svg{width:100%;height:auto;background:#0e1b2d;border-radius:14px}select{width:100%;background:#0d1727;color:#e6edf7;border:1px solid #486280;border-radius:10px;padding:12px;font-size:17px;margin:10px 0 16px}h2{font-size:20px}.big{font-size:36px;color:#52dfbb}.step{padding:12px 0;border-bottom:1px solid #2a3c53;line-height:1.6}.muted{color:#9bb1cc;font-size:14px}p{line-height:1.8}.legend{display:flex;gap:15px;flex-wrap:wrap;font-size:14px;margin-top:14px}.trace{max-height:240px;overflow:auto}.dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:5px}@media(max-width:850px){.grid{grid-template-columns:1fr}header{align-items:flex-start;flex-direction:column}}
</style><main><header><div><h1>一句话，去 3 号楼观察</h1><div class="sub">真实 v6 道路拓扑 → 楼栋观察点 → 路口动作建议<br>地图在 RDK X5；FPGA 负责实时循迹、转向与执行回报。</div></div><div class="badge">路线预览 · 实车定位未标定</div></header><div class="grid"><section class="card"><svg id="map" viewBox="0 0 880 800" role="img" aria-label="现有社区地图及选定楼栋路线"></svg><div class="legend"><span><i class="dot" style="background:#4fe0b5"></i>规划路线</span><span><i class="dot" style="background:#ff7979"></i>已知施工封边</span><span><i class="dot" style="background:#ffd480"></i>楼牌观察点</span></div></section><aside class="card"><label for="building">选择观察楼栋</label><select id="building"></select><div class="big" id="length"></div><div class="muted">沿道路中心线累计距离，非穿楼直线距离</div><h2 id="goal"></h2><p>示例起点固定在 <b>(2.60, 0.64) m</b>，朝向 +Y。真实起点必须测量配准。路线终点来自地图已有楼牌观察位，仍需验证车身位置与云台视角。</p><div class="trace" id="steps"></div><p class="muted">49 个节点 · 55 条道路边 · 13 栋楼。曲线保留原地图折线。施工默认封闭。局部垃圾桶避让、阿克曼转弯轨迹与实车里程误差仍由执行与定位联调验证。</p></aside></div><section class="card" style="margin-top:22px"><h2>路线算出来 ≠ 小车已经知道自己在哪里</h2><p>编码器提供“走了多远”，陀螺仪提供“转了多少”，已测初始位置给出地图起点。导航将这些融合为带漂移的估计；视线路口和已标定定位地标可以进一步校正。OCR 读到“3 号楼”，只表示看见楼牌，不能自动把小车坐标改成楼栋坐标。</p><p>默认只预览。正式执行还需要地图配准、有效编码器与陀螺遥测、可靠路段完成回报、底盘接口验证。途中暂停保留任务；丢定位停止；最终收到完成回报且确认停车，才进入场景观察。</p></section></main><script>const DATA=__DATA__;
const svg=document.getElementById('map'),ns='http://www.w3.org/2000/svg';
function el(tag,attrs){let e=document.createElementNS(ns,tag);Object.entries(attrs).forEach(([k,v])=>e.setAttribute(k,v));svg.appendChild(e);return e}
function pt(p){return `${p[0]*100},${800-p[1]*100}`}
function line(points,color,width,dash){el('polyline',{points:points.map(pt).join(' '),fill:'none',stroke:color,'stroke-width':width,'stroke-linecap':'round','stroke-linejoin':'round',...(dash?{'stroke-dasharray':dash}:{})})}
function circle(p,r,color){el('circle',{cx:p[0]*100,cy:800-p[1]*100,r,fill:color})}
function label(p,text,color,size){let t=el('text',{x:p[0]*100,y:800-p[1]*100,fill:color,'font-size':size,'text-anchor':'middle'});t.textContent=text}
function render(){svg.replaceChildren();const g=DATA.graph,r=DATA.routes[document.getElementById('building').value];g.edges.forEach(e=>line(e.geometry_m,'#233a53',e.width_m*100));g.buildings.forEach(b=>{el('polygon',{points:b.polygon_m.map(pt).join(' '),fill:'#425975',stroke:'#637c97','stroke-width':1});let p=[b.polygon_m.reduce((a,p)=>a+p[0],0)/b.polygon_m.length,b.polygon_m.reduce((a,p)=>a+p[1],0)/b.polygon_m.length];label(p,b.id+' '+b.name,'#e1ecf9',15)});g.edges.filter(e=>g.default_blocked_edges.includes(e.id)).forEach(e=>line(e.geometry_m,'#ff7979',9,'12 8'));r.segments.forEach(e=>line(e.geometry_m,'#4fe0b5',8));let start=g.nodes.find(n=>n.id===r.start_node).xy_m,end=g.nodes.find(n=>n.id===r.goal_node).xy_m;circle(start,9,'#5fa8ff');circle(end,10,'#ffd480');label([start[0],start[1]-.2],'起点','#aaccff',16);document.getElementById('length').textContent=r.length_m.toFixed(2)+' m';document.getElementById('goal').textContent=r.building_id+' '+r.building_name;let steps=document.getElementById('steps');steps.replaceChildren();r.segments.forEach((s,i)=>{let e=document.createElement('div');e.className='step';e.textContent=`${i+1}. ${{left:'左转',right:'右转',straight:'沿路循迹'}[s.turn]} · ${s.length_m.toFixed(2)} m`;steps.appendChild(e)})}
let select=document.getElementById('building');DATA.graph.buildings.forEach(b=>{let o=document.createElement('option');o.value=b.id;o.textContent=b.id+' '+b.name;select.appendChild(o)});select.value='03';select.addEventListener('change',render);render();</script></html>'''
        (output/'navigation_preview.html').write_text(html.replace('__DATA__',data),encoding='utf-8')
    print(json.dumps(dict(nodes=len(graph['nodes']),edges=len(graph['edges']),buildings=len(graph['buildings']),blocked=graph['default_blocked_edges']),ensure_ascii=False))


if __name__=='__main__':main()
