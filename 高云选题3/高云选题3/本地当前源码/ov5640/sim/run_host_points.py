"""Numbered destination geometry and real serial/UI/planner/profile/motion checks."""
from pathlib import Path
import concurrent.futures,hashlib,json,math,re,subprocess,time
R=Path(__file__).resolve().parents[1]
sha=lambda n:hashlib.sha256((R/n).read_bytes()).hexdigest()
sources=[p.relative_to(R).as_posix() for folder in ['src/navigation','src/vehicle'] for p in (R/folder).rglob('*.v')]+['src/imu/imu_math.v']
inputs={n:sha(n) for n in ['sim/host_points_tb.sv',*sources,*[p.relative_to(R).as_posix() for p in (R/'src').rglob('*.vh')]]}
subprocess.run(['iverilog','-g2012','-s','host_points_tb','-I','src/navigation','-I','src/vga','-o','sim/host_points.vvp','sim/host_points_tb.sv',*sources],cwd=R,check=True)
d=json.loads((R/'src/navigation/road_centerlines.json').read_text())
points=[(17,17),(12,80),(17,142),(88,147),(158,142),(163,80),(158,17),(88,13),(88,73)]
ideal=[]
for j in [194,None,193,None,192,None,191,None,None]:
 if j is None:ideal.append(None);continue
 p=d['primitives'][j];a=p['a0']+p['delta']/2
 ideal.append(((p['centre'][0]+p['radius']*math.cos(a))*352/895,(p['centre'][1]+p['radius']*math.sin(a))*320/817))
ideal[1]=(65*352/895,410*320/817);ideal[3]=(447.5*352/895,752*320/817)
ideal[5]=(830*352/895,410*320/817);ideal[7]=(447.5*352/895,68*320/817)
p=d['primitives'][195];a=p['a0']+p['delta'];ideal[8]=((p['centre'][0]+80*math.cos(a))*352/895,(p['centre'][1]+80*math.sin(a))*320/817)
geometry=[]
for number,((x,y),(ix,iy)) in enumerate(zip(points,ideal),1):
 error=math.hypot(2*x+1-ix,2*y+1-iy)*10
 assert error<20,(number,error)
 geometry.append(dict(point=number,cell=[x,y],display_cm=[2*x+1,319-2*y],ideal_raw_cm=[ix,iy],quantization_error_mm=round(error,2)))
def run(number):
 start=time.monotonic();log=f'sim/host_points_{number}.log'
 with (R/log).open('w',encoding='utf-8') as f:
  q=subprocess.run(['vvp','-i','sim/host_points.vvp',f'+point={number}',*(['+snapshot'] if number==9 else [])],cwd=R,stdout=f,stderr=subprocess.STDOUT,timeout=1800)
 text=(R/log).read_text();assert q.returncode==0 and 'PASS numbered point' in text and 'FATAL:' not in text,log
 rows=(R/f'sim/host_point_{number}_route.csv').read_text().splitlines();last=list(map(int,rows[-1].split(',')))
 goal=points[number-1];distance=(abs(last[1]/256-(2*goal[0]+1))+abs(last[2]/256-(2*goal[1]+1)))*10
 assert distance<85,(number,distance)
 result=dict(point=number,passed=True,seconds=round(time.monotonic()-start,3),route_samples=len(rows)-1,goal_error_manhattan_mm=round(distance,2),log=log,log_sha256=sha(log),route=f'sim/host_point_{number}_route.csv')
 print('PASS point',number,'samples',len(rows)-1,'seconds',result['seconds'],flush=True)
 return result
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as ex:results=list(ex.map(run,range(1,10)))
assert all(sha(n)==h for n,h in inputs.items())
(R/'sim/host_points_20261031_runs.json').write_text(json.dumps(dict(geometry=geometry,cases=results,inputs=inputs),indent=2)+'\n')
from PIL import Image
Image.open(R/'sim/host_points_map.ppm').save(R/'docs/host_points_map.png')
print('PASS all nine destination routes, numbering and map snapshot')
