"""Compare RTL with an independent node x waypoint-layer shortest-path search."""
from pathlib import Path
import json, math, heapq, subprocess, collections,concurrent.futures,threading,argparse,re
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--validate-only',action='store_true',help='recheck existing RTL outputs without rerunning simulation')
args=parser.parse_args()
network=json.loads((ROOT/'src/navigation/road_centerlines.json').read_text());nodes=network['graph']
junctions=[(round(network['vertices'][n][0]*352/895),round(network['vertices'][n][1]*320/817))
           for n in network['junctions']]
costs=[[(j,max(1,round(math.hypot(nodes[j]['x']-p['x'],nodes[j]['y']-p['y'])/256)))
        for j in p['next']] for p in nodes]
cases=[
 [(25,199),(25,159)],
 [(25,199),(29,279),(25,159)],
 [(25,199),(29,279),(149,293),(235,293),(225,197),(103,171),(73,199),(25,159)],
 [(25,199),(104,120),(104,220),(104,120),(104,220),(104,120),(104,220),(104,120),(104,220),(25,159)],
 [(104,120),(248,190),(248,50),(104,120)],
 [(209,295),(249,295),(249,230)],  # touch on straight beside the desired fillet
 [(209,295),(235,295),(249,230)],  # off-junction 6cm region must also reach the fillet
 [(104,215),(248,80)],  # forced S road, with approach/departure straights
 [(291,293),(199,293),(105,237),(69,199),(25,199)], # exact user photo, Y up
 [(327,155),(327,227),(189,293),(105,255),(73,199),(25,177)], # current UI pixels
 [(327,241),(243,293),(105,255),(77,199),(25,177)], # 2026-10-23 exact screenshot, Y up
 [(248,80),(104,215)], # reverse S traversal
]
def grid(p):return (int(p[0]//2)*2+1,int(p[1]//2)*2+1)

# The photo lists physical Y-up coordinates; the ROM uses image Y-down.
cases[8]=[(x,319-y) for x,y in cases[8]]
cases[10]=[(x,319-y) for x,y in cases[10]]
def nearest(p):
 x,y=grid(p)
 i=min(range(len(nodes)),key=lambda i:abs(nodes[i]['x']-x*256)+abs(nodes[i]['y']-y*256))
 return nodes[i]['x'],nodes[i]['y']
def radius(p):
 x,y=p[0]//256,p[1]//256
 return (20 if any(abs(x-a)<=4 and abs(y-b)<=4 for a,b in junctions) else 6)*256
def matches(p,target,index,last):
 return abs(p[0]-target[0])+abs(p[1]-target[1])<=(0 if index==0 else 6*256 if index==last else radius(target))
def oracle(points):
 targets=list(map(nearest,points));dist={};queue=[]
 for i,n in enumerate(nodes):
  if (n['x'],n['y'])==targets[0]:dist[i,1]=0;heapq.heappush(queue,(0,i,1))
 while queue:
  d,i,s=heapq.heappop(queue)
  if dist[i,s]!=d:continue
  if matches((nodes[i]['x'],nodes[i]['y']),targets[s],s,len(targets)-1):
   if s==len(targets)-1:return d,targets
   if d<dist.get((i,s+1),10**9):dist[i,s+1]=d;heapq.heappush(queue,(d,i,s+1))
  for j,c in costs[i]:
   if d+c<dist.get((j,s),10**9):dist[j,s]=d+c;heapq.heappush(queue,(d+c,j,s))
 raise AssertionError('disconnected oracle')
expected=[];lines=[]
for points in cases:
 answer,targets=oracle(points);expected.append((answer,targets))
 coords=[(int(x//2),int(y//2)) for x,y in points];waypoints=coords[1:-1]
 wx=sum(x<<(8*i) for i,(x,y) in enumerate(waypoints));wy=sum(y<<(8*i) for i,(x,y) in enumerate(waypoints))
 lines.append(f'{coords[0][0]} {coords[0][1]} {coords[-1][0]} {coords[-1][1]} {len(waypoints)} {wx:x} {wy:x} {answer} 0 0 0 0 0')
case_text='\n'.join(lines)+'\n'
if args.validate_only:
 assert (ROOT/'sim/road_global_cases.txt').read_text()==case_text,'Cases changed: rerun RTL'
 recorded=(ROOT/'sim/road_global_test_results.txt').read_text()
 costs_found={int(i):int(c) for i,c in re.findall(r'PASS global road case (\d+): .*?optimal ROM cost=(\d+)cm',recorded)}
 assert costs_found=={i:x[0] for i,x in enumerate(expected)},'Incomplete or stale RTL case results'
 assert (ROOT/'sim/road_global_routes.csv').stat().st_mtime>=max(
  p.stat().st_mtime for p in [ROOT/'src/navigation/nav_road_planner.v',
  *list((ROOT/'src/navigation').glob('nav_centerline_*.vh'))]),'RTL outputs predate the current design'
else:(ROOT/'sim/road_global_cases.txt').write_text(case_text)
with (ROOT/'sim/road_global_test_results.txt').open('a' if args.validate_only else 'w') as log:
 if not args.validate_only:
  subprocess.run(['iverilog','-g2012','-I','src/navigation','-s','road_global_tb','-o','sim/road_global_tb.vvp',
                  'sim/road_global_tb.sv','src/navigation/nav_road_planner.v'],cwd=ROOT,check=True)
  output_lock=threading.Lock()
  def run_group(bounds):
   lo,hi=bounds;filename=f'sim/road_global_routes_{lo}_{hi}.csv'
   cmd=['vvp','sim/road_global_tb.vvp',f'+CASE_FROM={lo}',f'+CASE_TO={hi}',f'+ROUTES={filename}']
   with subprocess.Popen(cmd,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True) as proc:
    for line in proc.stdout:
     with output_lock:print(line,end='',flush=True);log.write(line);log.flush()
    assert proc.wait()==0,bounds
   return (ROOT/filename).read_text()
  with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
   outputs=list(pool.map(run_group,[(0,1),(2,2),(3,3),(4,len(cases)-1)]))
  all_rows=[line for output in outputs for line in output.splitlines() if line.strip()]
  all_rows.sort(key=lambda s:int(s.split(',')[0]))
  (ROOT/'sim/road_global_routes.csv').write_text('\n'.join(all_rows)+'\n')
 routes=collections.defaultdict(list)
 for line in (ROOT/'sim/road_global_routes.csv').read_text().splitlines():
  case,x,y,segment=map(int,line.split(','));routes[case].append((x,y))
 for case,(cost,targets) in enumerate(expected):
  path=routes[case];cursor=0
  for index,target in enumerate(targets):
   while cursor<len(path) and not matches(path[cursor],target,index,len(targets)-1):cursor+=1
   assert cursor<len(path),(case,'waypoint missing or out of order',target)
  assert path[0]==targets[0] and matches(path[-1],targets[-1],len(targets)-1,len(targets)-1)
 assert len(set(routes[3]))<len(routes[3]),'repeat-road regression did not actually revisit a road'
 assert expected[5][0]<120 and expected[6][0]<120,'junction/fillet touch still forces an avoidable full loop'
 assert expected[8][0]<400,'photo route still makes the 729cm outer loop'
 assert max(y for x,y in routes[8])<145*256,'photo route enters lower half of map'
 # Shorter fillets preserve more straight road, so the valid route now has
 # 122 samples, rather than 115. Bound actual length and lower-block visits;
 # a sample-count cap tied to the old smoothing radius is not a loop test.
 assert expected[9][0]<550 and max(y for x,y in routes[9])<305*256,'current UI route makes an avoidable loop'
 assert abs(routes[9][0][0]-326*256)<20*256 and abs(routes[9][-1][0]-25*256)<20*256,'current UI route misses the selected endpoints'
 assert expected[10][0]<450,'October 23 exact screenshot still takes the 779 cm loop'
 assert max(y for x,y in routes[10])<165*256,'October 23 route enters lower blocks'
 # Check every rasterized sub-centimetre edge, not just ROM nodes.
 raw=[int(s,16) for s in (ROOT/'src/navigation/map_raw_1cm.mem').read_text().split()]
 for case,path in routes.items():
  for a,b in zip(path,path[1:]):
   steps=max(1,math.ceil(math.dist(a,b)/64))
   for t in range(steps+1):
    x=round(a[0]+(b[0]-a[0])*t/steps)//256;y=round(a[1]+(b[1]-a[1])*t/steps)//256
    assert 0<=x<352 and 0<=y<320 and raw[y*352+x]==0,('route outside white road',case,x,y)
 message='PASS global road oracle: shortest ordered photo/UI routes including exact October 23 screenshot, terminal arrival region, aliases, repeated roads, tangent joins and every edge inside white road\n'
 print(message,end='');log.write(message)
