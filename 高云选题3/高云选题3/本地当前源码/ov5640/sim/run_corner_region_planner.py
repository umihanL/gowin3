"""Independent graph shortest-path oracle versus actual planning RTL; no I/O."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import hashlib, heapq, json, math, re, subprocess, time

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'diagnostics/corner_region_20261103'
OUT.mkdir(parents=True, exist_ok=True)
words=[int(x,16) for x in re.findall(r"=79'h([0-9a-fA-F]+)", (ROOT/'src/navigation/nav_centerline_init.vh').read_text())]
headings=[int(x,16) for x in re.findall(r"=9'h([0-9a-fA-F]+)", (ROOT/'src/navigation/nav_centerline_heading.vh').read_text())]
headings=[x-512 if x>=256 else x for x in headings]
xy=[(v&131071,(v>>17)&131071) for v in words]
edges=[[(e&2047,e>>11) for slot in range(3)
        if (e:=(v>>(34+slot*15))&32767)&2047!=2047] for v in words]

def l1(a,b):return sum(abs(x-y) for x,y in zip(a,b))
def point(cell):return tuple(v*512+256 for v in cell)
def nearest(cell):return min(range(len(xy)),key=lambda i:l1(xy[i],point(cell)))
def oracle(case, fallback=False):
    pose=point(case['start']);near=nearest(case['start']);heading=case['heading']
    goals=[xy[nearest(p)] for p in case.get('waypoints', [])+[case['goal']]]
    def forward(p):
        x,y=p;px,py=pose
        return (x+512>=px if -450<=heading<=450 else y<=py+512 if 450<heading<1350
                else x<=px+512 if heading>=1350 or heading<=-1350 else y+512>=py)
    seeds=[i for i,p in enumerate(xy) if headings[i]!=255 and abs((headings[i]*10-heading+1800)%3600-1800)<=350
        and ((l1(p,pose)<=1536 and forward(p)) if fallback else p==xy[near])]
    distances={(i,0):math.ceil(l1(xy[i],pose)/256) if fallback else 0 for i in seeds}
    queue=[(d,i,0) for (i,_),d in distances.items()];heapq.heapify(queue)
    previous={};box=case.get('box')
    while queue:
        distance,i,stage=heapq.heappop(queue)
        if distances.get((i,stage))!=distance:continue
        if l1(xy[i],goals[stage])<=1536:
            if stage==len(goals)-1:
                return dict(cost=distance,fallback=int(fallback))
            nxt=(i,stage+1)
            if distance<distances.get(nxt,65535):
                distances[nxt]=distance;heapq.heappush(queue,(distance,*nxt))
        outside=bool(box and not (box[0]<=xy[i][0]//512<=box[2] and box[1]<=xy[i][1]//512<=box[3]))
        for j,cost in edges[i]:
            candidate=distance+cost*(33 if outside else 1);nxt=(j,stage)
            if candidate<distances.get(nxt,65535):
                distances[nxt]=candidate;heapq.heappush(queue,(candidate,*nxt))
    return None if fallback else oracle(case, True)

CASES=[
 dict(name='corner_wrong_nearest_alias',start=(56,13),goal=(88,147),heading=-1430),
 dict(name='curve_replan_actual_pose',start=(160,93),goal=(88,147),heading=-1520),
 dict(name='normal_length_unchanged',start=(12,80),goal=(17,17),heading=-900,length=378),
 dict(name='region_outside_penalty',start=(12,80),goal=(17,17),heading=-900,box=(0,80,88,159),length=514),
 dict(name='reverse_suffix_region_cost',start=(12,80),goal=(17,17),waypoints=[(88,147)],heading=-900,box=(0,80,88,159)),
 dict(name='perpendicular_still_rejected',start=(88,13),goal=(88,147),heading=900,error=2,fallback=1),
 dict(name='zero_length_not_faked_arrival',start=(88,13),goal=(88,13),heading=1800,error=1,fallback=0),
]

def invoke(args, logfile):
    with logfile.open('wb') as out:
        result=subprocess.run(['wsl.exe','--exec',*args],cwd=ROOT,stdout=out,stderr=subprocess.STDOUT,timeout=900)
    return result.returncode, logfile.read_bytes().decode('utf-8',errors='replace')

def run(case):
    began=time.monotonic();expected=oracle(case) if not case.get('error') else None
    if not case.get('error') and expected is None:raise RuntimeError('oracle unreachable '+case['name'])
    args={**dict(zip(('sx','sy'),case['start'])),**dict(zip(('gx','gy'),case['goal'])),
          'heading':case['heading'],'error':case.get('error',0),'fallback':case.get('fallback',expected['fallback'] if expected else 0)}
    if expected:args['cost']=expected['cost']
    if 'length' in case:args['length']=case['length']
    if case.get('box'):args.update(region=1,**dict(zip(('rx0','ry0','rx1','ry1'),case['box'])))
    if case.get('waypoints'):args.update(dict(zip(('wx','wy'),case['waypoints'][0])))
    code,text=invoke(['vvp','-i','sim/corner_region_planner.vvp',*[f'+{k}={v}' for k,v in args.items()]],OUT/(case['name']+'.log'))
    passed=code==0 and 'PASS ' in text and 'FATAL' not in text
    result=dict(case=case['name'],passed=passed,oracle=expected,seconds=round(time.monotonic()-began,2),log=case['name']+'.log')
    print(('PASS ' if passed else 'FAIL ')+case['name'],flush=True)
    return result

if __name__=='__main__':
    code,text=invoke(['iverilog','-g2012','-s','corner_region_planner_tb','-I','src/navigation',
        '-o','sim/corner_region_planner.vvp','sim/corner_region_planner_tb.sv','src/navigation/nav_road_planner.v'],OUT/'compile.log')
    if code:raise RuntimeError(text)
    with ThreadPoolExecutor(max_workers=4) as pool:results=list(pool.map(run,CASES))
    report=dict(passed=all(r['passed'] for r in results),cases=results,hardware_tested=False,flashed=False,
        source_sha256=hashlib.sha256((ROOT/'src/navigation/nav_road_planner.v').read_bytes()).hexdigest())
    (OUT/'planner_results.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    raise SystemExit(0 if report['passed'] else 1)
