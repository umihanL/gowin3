"""Build a directed, tangent-connected line/circle road graph from the supplied map.

Geometry is in source-image pixels, scaled to the existing 352 x 320 cm ROM.
Turns join ports ON road centrelines, so overlapping intersection fillets do not
shorten the adjoining straight roads. No iterative smoothing runs on the FPGA.
"""
from pathlib import Path
import math, json, re
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'src/navigation'
cfg={k:int(v) for k,v in re.findall(r'^`define (VEHICLE_\w+) (-?\d+)$',
                                 (ROOT/'src/vehicle/vehicle_config.vh').read_text(),re.M)}
# Road measurements supplied on 2026-10-24. The outer-edge radius converts to
# the centreline radius by subtracting half the measured outer-road width.
road_dimensions=json.loads((OUT/'road_dimensions.json').read_text())
ROAD_OUTER_WIDTH_MM=road_dimensions['measured_2026_10_24']['outer_width_mm']
ROAD_INNER_WIDTH_MM=road_dimensions['measured_2026_10_24']['inner_width_mm']
ROAD_OUTER_EDGE_RADIUS_MM=road_dimensions['measured_2026_10_24']['outer_edge_radius_mm']
OUTER_CENTER_RADIUS_MM=ROAD_OUTER_EDGE_RADIUS_MM-ROAD_OUTER_WIDTH_MM/2
# Preserve the accepted outer-loop geometry of the supplied map. Measured
# physical radius is recorded separately; it is not a uniform image scale.
outer_pixels=80.0
# Late-entry, two-circle fillet. Its tight initial nose arc is followed by
# a larger radius while rear heading catches up. No sharp tangent joins.
entry_r1_mm,entry_r2_mm,entry_angle=65.0,315.0,math.radians(50)
entry_mm=entry_r1_mm*math.sin(entry_angle)+entry_r2_mm*(1-math.sin(entry_angle))
exit_mm=entry_r1_mm*(1-math.cos(entry_angle))+entry_r2_mm*math.cos(entry_angle)
entry_px,exit_px=entry_mm*895/3520,exit_mm*895/3520
def junction_peak_steer():
    yaw=0.;tangent=0.;worst=0.
    for radius,angle in [(entry_r1_mm,entry_angle),(entry_r2_mm,math.pi/2-entry_angle)]:
        for i in range(1200):
            step=angle/1200;mid=tangent+step/2
            ds=radius*step
            yaw_mid=yaw+ds/2*math.sin(mid-yaw)/cfg['VEHICLE_NOSE_FROM_REAR_MM']
            yaw+=ds*math.sin(mid-yaw_mid)/cfg['VEHICLE_NOSE_FROM_REAR_MM'];tangent+=step
            worst=max(worst,abs(math.degrees(math.atan(cfg['VEHICLE_WHEELBASE_MM']*
                math.tan(tangent-yaw)/cfg['VEHICLE_NOSE_FROM_REAR_MM']))))
    return worst
peak_junction_steer=junction_peak_steer()
if peak_junction_steer>cfg['VEHICLE_MAX_STEER_DEG']-1:
    raise ValueError(
        f"Existing junction geometry requires {peak_junction_steer:.2f} deg wheel steering; "
        f"the measured limit is {cfg['VEHICLE_MAX_STEER_DEG']} deg "
        "with a 1 deg planning margin. No road assets were overwritten. "
        "Recheck the metric road registration and feasible turn geometry; "
        "do not increase the measured servo limit to bypass this check."
    )

vertices = {
 'tl':(65+outer_pixels,68),'a':(265,68),'b':(630,68),'tr':(830-outer_pixels,68),
 'rt':(830,68+outer_pixels),'c':(830,492),'rb':(830,752-outer_pixels),'br':(830-outer_pixels,752),
 'd':(630,752),'e':(265,752),'bl':(65+outer_pixels,752),'lb':(65,752-outer_pixels),
 'f':(65,565),'g':(65,310),'lt':(65,68+outer_pixels),
 'h':(265,310),'i':(265,440),'j':(265,565),
 'k':(630,315),'l':(630,492),
 's0':(350,440),'s2':(545,315),
}
lines = [('tl','a'),('a','b'),('b','tr'),('rt','c'),('c','rb'),
 ('br','d'),('d','e'),('e','bl'),('lb','f'),('f','g'),('g','lt'),
 ('a','h'),('h','i'),('i','j'),('j','e'),('b','k'),('k','l'),('l','d'),
 ('g','h'),('f','j'),('l','c'),('i','s0'),('s2','k')]
arcs = [('tr','rt',(830-outer_pixels,68+outer_pixels),-90,0),
 ('rb','br',(830-outer_pixels,752-outer_pixels),0,90),
 ('bl','lb',(65+outer_pixels,752-outer_pixels),90,180),
 ('lt','tl',(65+outer_pixels,68+outer_pixels),180,270)]
cuts = {e:{0.,1.} for e in lines}
incidence = {v:[] for v in vertices}
for e in lines:
 a,b=e; pa,pb=np.array(vertices[a],float),np.array(vertices[b],float)
 length=float(np.linalg.norm(pb-pa))
 incidence[a].append((e,(pb-pa)/length,length,False))
 incidence[b].append((e,(pa-pb)/length,length,True))
junctions=[v for v,branches in incidence.items() if len(branches)>=3]
junction_text=['// Generated junction touch regions; function arguments x,y are cm Q8.']
for v in junctions:
 x,y=vertices[v];cx=round(x*352/895);cy=round(y*320/817)
 junction_text.append(f"if(x[16:8]+9'd4>=9'd{cx} && x[16:8]<=9'd{cx+4} && "
                      f"y[16:8]+9'd4>=9'd{cy} && y[16:8]<=9'd{cy+4})junction_near=1;")
(OUT/'nav_centerline_junctions.vh').write_text('\n'.join(junction_text)+'\n',encoding='ascii')
primitives=[]
for v,branches in incidence.items():
 if len(branches)<3:continue
 p=np.array(vertices[v],float)
 for n,(ea,ua,la,ra) in enumerate(branches):
  for eb,ub,lb,rb in branches[n+1:]:
   if abs(float(ua@ub))>0.01:continue # straight motion already exists
   assert min(la,lb)>exit_px+5,(v,la,lb)
   # Each direction gets its own late entry. Reversing an asymmetric fillet
   # would put the long exit on entry and reproduce the premature turn.
   for incoming,outgoing,elin,elout,lin,lout,rin,rout in (
       (ua,ub,ea,eb,la,lb,ra,rb),(ub,ua,eb,ea,lb,la,rb,ra)):
    cuts[elin].add(1-entry_px/lin if rin else entry_px/lin)
    cuts[elout].add(1-exit_px/lout if rout else exit_px/lout)
    heading=-incoming;sign=float(np.linalg.det(np.stack([heading,outgoing],axis=1)))
    r1,r2=entry_r1_mm*895/3520,entry_r2_mm*895/3520
    start=p+incoming*entry_px;c1=start+outgoing*r1
    mid=start+heading*r1*math.sin(entry_angle)+outgoing*r1*(1-math.cos(entry_angle))
    c2=mid+r2*(-heading*math.sin(entry_angle)+outgoing*math.cos(entry_angle))
    a0=math.atan2(*(-outgoing)[::-1])
    primitives.append(dict(kind='arc',centre=c1.tolist(),radius=r1,a0=a0,
                           delta=sign*entry_angle,directed=True,role='junction_entry'))
    primitives.append(dict(kind='arc',centre=c2.tolist(),radius=r2,
                           a0=a0+sign*entry_angle,delta=sign*(math.pi/2-entry_angle),
                           directed=True,role='junction_exit'))
for (a,b),values in cuts.items():
 pa,pb=np.array(vertices[a],float),np.array(vertices[b],float)
 values=sorted(values)
 for lo,hi in zip(values,values[1:]):
  if hi-lo>1e-6:primitives.append(dict(kind='line',a=(pa+(pb-pa)*lo).tolist(),b=(pa+(pb-pa)*hi).tolist()))
for a,b,c,a0,a1 in arcs:
 primitives.append(dict(kind='arc',centre=list(c),radius=math.dist(vertices[a],c),
                        a0=math.radians(a0),delta=math.radians(a1-a0)))

# Preserve the provisional 80px radius, but fit the actual S orientation:
# two directly tangent reverse arcs, not a long diagonal across the road.
# Their common tangent is steep, matching the near-vertical middle of the S.
r=float(road_dimensions['provisional']['s_bend_source_radius_px']);rise=vertices['s0'][1]-vertices['s2'][1]
theta=math.acos(1-rise/(2*r));mid_x=452.5
vertices['s0']=(mid_x-r*math.sin(theta),440)
vertices['s2']=(mid_x+r*math.sin(theta),315)
# Update the horizontal S leads already generated from the old endpoints.
for p in primitives:
 if p['kind']=='line':
  if p['b']==[350.0,440.0]:p['b']=list(vertices['s0'])
  if p['a']==[545.0,315.0]:p['a']=list(vertices['s2'])
a=np.array(vertices['s0'],float);b=np.array(vertices['s2'],float)
c0=a+np.array([0.,-r]);c1=b+np.array([0.,r])
primitives.extend([
 dict(kind='arc',centre=c0.tolist(),radius=r,a0=math.pi/2,delta=-theta,role='s_bend'),
 dict(kind='arc',centre=c1.tolist(),radius=r,a0=-math.pi/2-theta,delta=theta,role='s_bend')])

def samples(p):
 length=math.dist(p['a'],p['b']) if p['kind']=='line' else abs(p['radius']*p['delta'])
 spacing_cm=1.0 if p.get('role')=='s_bend' else 4.5
 count=max(1,math.ceil(length*352/895/spacing_cm))
 if p['kind']=='line':
  a,b=np.array(p['a']),np.array(p['b']); u=(b-a)/np.linalg.norm(b-a)
  return [(a+(b-a)*t/count,u) for t in range(count+1)]
 result=[]
 for t in range(count+1):
  a=p['a0']+p['delta']*t/count;sign=1 if p['delta']>0 else -1
  result.append((np.array(p['centre'])+p['radius']*np.array([math.cos(a),math.sin(a)]),
                 sign*np.array([-math.sin(a),math.cos(a)])))
 return result

nodes=[]; ends=[]; starts={}; chains=[]
for pi,p in enumerate(primitives):
 seq=samples(p)
 for reverse in ((False,) if p.get('directed') else (False,True)):
  ordered=[(xy,-u) for xy,u in seq[::-1]] if reverse else seq
  ids=[]
  for xy,u in ordered:
   x=round(float(xy[0])*352/895*256);y=round(float(xy[1])*320/817*256)
   ids.append(len(nodes));nodes.append(dict(x=x,y=y,next=[],primitive=pi))
  for a,b in zip(ids,ids[1:]):nodes[a]['next'].append(b)
  first,last=ids[0],ids[-1]
  starts.setdefault((nodes[first]['x'],nodes[first]['y']),[]).append((first,ordered[0][1]))
  ends.append((last,ordered[-1][1]));chains.append(ids)
for last,u in ends:
 p=nodes[last]
 for first,v in starts.get((p['x'],p['y']),[]):
  if float(u@v)>.999:nodes[last]['next'].append(first)
 assert len(p['next'])<=3,(last,p)
# Keep the straight stubs at T-junction centres. They are legitimate start /
# finish positions even when they have no tangent continuation. Pruning these
# made a goal on the left crossing reachable only after a complete outer loop.
# Intermediate waypoints still require a reachable suffix, so a dead end does
# not introduce an instantaneous U-turn or a non-tangent graph connection.
assert len(nodes)<4095,len(nodes)
bits=max(1,(len(nodes)-1).bit_length());capacity=1<<bits
words=[]
for p in nodes:
 word=p['x'] | p['y']<<17
 for slot in range(3):
  if slot<len(p['next']):
   nxt=p['next'][slot];q=nodes[nxt]
   cost=max(1,round(math.hypot(q['x']-p['x'],q['y']-p['y'])/256))
  else:nxt=capacity-1;cost=0
  assert cost<=15
  word|=(nxt | cost<<bits)<<(34+slot*(bits+4))
 words.append(word)
width=34+3*(bits+4);digits=(width+3)//4
text='// Generated by tools/generate_centerline.py; fixed line/circle graph.\n'
text+='\n'.join(f"graph[{i}]={width}'h{w:0{digits}x};" for i,w in enumerate(words))+'\n'
(OUT/'nav_centerline_init.vh').write_text(text,encoding='ascii')
(OUT/'nav_centerline_config.vh').write_text(f'`define ROAD_NODES {len(nodes)}\n`define ROAD_BITS {bits}\n`define ROAD_WORD_BITS {width}\n',encoding='ascii')
grid=Image.new('1',(176,160));draw=ImageDraw.Draw(grid)
preview=Image.open(ROOT/'地图.jpg').convert('RGB');pd=ImageDraw.Draw(preview)
for p in primitives:
 pts=[tuple(xy) for xy,_ in samples(p)]
 pd.line(pts,fill=(20,205,255),width=3)
 draw.line([(round(x*352/895/2),round(y*320/817/2)) for x,y in pts],fill=1,width=1)
raw=np.array([int(t,16) for t in (OUT/'map_raw_1cm.mem').read_text().split()]).reshape(320,352)
for p in nodes:
 assert raw[min(319,p['y']//256),min(351,p['x']//256)]==0,('centreline off road',p)
 for j in p['next']:
  q=nodes[j];steps=max(1,math.ceil(math.hypot(q['x']-p['x'],q['y']-p['y'])/64))
  for t in range(steps+1):
   x=round(p['x']+(q['x']-p['x'])*t/steps)//256
   y=round(p['y']+(q['y']-p['y'])*t/steps)//256
   assert 0<=x<352 and 0<=y<320 and raw[y,x]==0,('edge off white road',x,y)
values=list(grid.getdata());packed=[]
for start in range(0,len(values),16):packed.append(sum(int(v)<<b for b,v in enumerate(values[start:start+16])))
(OUT/'nav_centerline_bitmap.vh').write_text('\n'.join(f"centre_map[{i}]=16'h{v:04x};" for i,v in enumerate(packed))+'\n',encoding='ascii')
preview.save(ROOT/'docs/road_centerlines.png')
(OUT/'road_centerlines.json').write_text(json.dumps(dict(nodes=len(nodes),index_bits=bits,word_bits=width,
 spacing_cm=4.5,s_bend_spacing_cm=1.0,junction_turn_nose_radius_mm=None,
 junction_turn_entry_mm=entry_mm,junction_turn_exit_mm=exit_mm,
 junction_turn_radii_mm=[entry_r1_mm,entry_r2_mm],junction_turn_split_deg=50,
 body_swept_clearance_verified=False,
 road_geometry=dict(outer_road_width_mm=ROAD_OUTER_WIDTH_MM,inner_road_width_mm=ROAD_INNER_WIDTH_MM,
                    outer_edge_radius_mm=ROAD_OUTER_EDGE_RADIUS_MM,outer_centerline_radius_mm=OUTER_CENTER_RADIUS_MM,
                    outer_centerline_radius_source_px=outer_pixels,
                    physical_vs_map_scale_calibrated=False,s_bend_source_radius_px=r,
                    s_bend_radius_mm=r*3520/895),
 estimated_peak_junction_steer_deg=junction_peak_steer(),
 junctions=junctions,primitives=primitives,vertices=vertices,graph=nodes),indent=2)+'\n',encoding='ascii')
print(f'Late junction entry {entry_mm:.1f} mm; exit {exit_mm:.1f} mm; paired tangent circular arcs. All centre edges checked inside white road; body clearance requires separate audit.')
print(f'Generated {len(nodes)} directed nodes, {len(primitives)} lines/arcs, {width}-bit ROM; all nodes on road.')
from write_reverse_road import generate as generate_reverse
generate_reverse()
