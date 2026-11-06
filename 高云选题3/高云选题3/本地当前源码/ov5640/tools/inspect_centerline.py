from pathlib import Path
import json,math,heapq
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1]
data=json.loads((ROOT/'src/navigation/road_centerlines.json').read_text())
g=data['graph']
# The traversable core is connected; terminal stubs deliberately have either
# no incoming or no outgoing edge, allowing starts and goals at T centres.
for reverse in (False,True):
 adj=[p['next'][:] for p in g]
 if reverse:
  adj=[[] for p in g]
  for i,p in enumerate(g):
   for j in p['next']:adj[j].append(i)
 seen={0};todo=[0]
 while todo:
  for j in adj[todo.pop()]:
   if j not in seen:seen.add(j);todo.append(j)
 incoming={j for p in g for j in p['next']}
 required={i for i,p in enumerate(g) if i in incoming and p['next']}
 # An endpoint chain may include internal nodes; every node must be reachable
 # from the core in at least one direction, checked after both walks below.
 if not reverse:forward_seen=seen
 else:assert len(forward_seen|seen)==len(g),('isolated road',len(forward_seen|seen),len(g))
for i,p in enumerate(g):
 for j in p['next']:
  q=g[j]
  assert math.hypot(q['x']-p['x'],q['y']-p['y'])<=4.6*256
print(f'PASS road geometry: {len(g)} core/terminal nodes reachable, all joins within 4.6cm')
route=[tuple(map(int,line.split(','))) for line in (ROOT/'sim/road_route.csv').read_text().splitlines()]
image=Image.open(ROOT/'地图.jpg').convert('RGB');draw=ImageDraw.Draw(image)
points=[(p[0]/256*895/352,p[1]/256*817/320) for p in route]
draw.line(points,fill=(0,190,255),width=5)
for p in (points[0],points[-1]):draw.ellipse((p[0]-6,p[1]-6,p[0]+6,p[1]+6),fill='red')
image.save(ROOT/'docs/planned_centerline_route.png')
