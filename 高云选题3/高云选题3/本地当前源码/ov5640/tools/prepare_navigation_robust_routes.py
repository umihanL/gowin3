"""Export independent graph routes for the latest photograph and S bends."""
from pathlib import Path
import heapq,json,math
ROOT=Path(__file__).resolve().parents[1]
network=json.loads((ROOT/'src/navigation/road_centerlines.json').read_text())
nodes=network['graph']
junctions=[(round(network['vertices'][n][0]*352/895),round(network['vertices'][n][1]*320/817)) for n in network['junctions']]
def nearest(point):
 x=(point[0]//2*2+1)*256;y=(point[1]//2*2+1)*256
 n=min(nodes,key=lambda n:abs(n['x']-x)+abs(n['y']-y))
 return n['x'],n['y']
def route(points):
 targets=list(map(nearest,points));dist={};parents={};queue=[]
 for i,n in enumerate(nodes):
  if(n['x'],n['y'])==targets[0]:dist[i,1]=0;heapq.heappush(queue,(0,i,1))
 while queue:
  d,i,s=heapq.heappop(queue)
  if dist[i,s]!=d:continue
  n=nodes[i];t=targets[s];radius=6 if s==len(points)-1 else 20 if any(abs(t[0]//256-a)<=4 and abs(t[1]//256-b)<=4 for a,b in junctions) else 6
  if abs(n['x']-t[0])+abs(n['y']-t[1])<=radius*256:
   if s==len(points)-1:goal=i,s;break
   if d<dist.get((i,s+1),1e9):dist[i,s+1]=d;parents[i,s+1]=i,s;heapq.heappush(queue,(d,i,s+1))
  for j in n['next']:
   cost=max(1,round(math.hypot(nodes[j]['x']-n['x'],nodes[j]['y']-n['y'])/256))
   if d+cost<dist.get((j,s),1e9):dist[j,s]=d+cost;parents[j,s]=i,s;heapq.heappush(queue,(d+cost,j,s))
 else:raise AssertionError('No route')
 path=[];g=goal
 while True:
  if not path or path[-1]!=g[0]:path.append(g[0])
  if g not in parents:break
  g=parents[g]
 path.reverse();return path,dist[goal]
cases={
 'photo':[(327,319-175),(193,319-293),(181,319-181),(247,319-237)],
 's_forward':[(104,215),(248,80)],
 's_reverse':[(248,80),(104,215)],
 'outer':[(327,100),(180,27)],
 'junction':[(190,27),(105,150)],
 's_forward_exit':[(104,215),(248,27)],
 's_reverse_exit':[(248,80),(104,260)],
}
if __name__=='__main__':
 manifest={}
 for name,points in cases.items():
  path,cost=route(points);filename=f'sim/navigation_robust_{name}_route.csv'
  (ROOT/filename).write_text(''.join(f"{nodes[i]['x']},{nodes[i]['y']},{int(k in (0,len(path)-1))}\n" for k,i in enumerate(path)))
  manifest[name]=dict(file=filename,count=len(path),cost_cm=cost,selected_points=points)
  print(name,'samples',len(path),'cost',cost,'cm')
 (ROOT/'sim/navigation_robust_routes.json').write_text(json.dumps(manifest,indent=2)+'\n')
