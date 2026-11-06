"""Vary camera cadence/phase and initial yaw on real navigation graph paths."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import hashlib,json,re,subprocess,time

ROOT=Path(__file__).resolve().parents[1]
sources=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v']
cases=[dict(route=route,camera_ticks=6,phase=phase,yaw=0,offset=0)
       for route in ('s_forward','s_reverse') for phase in (1,4)]
cases += [dict(route='junction',camera_ticks=3,phase=phase,yaw=0,offset=0) for phase in (1,2)]
cases += [dict(route='outer',camera_ticks=3,phase=1,yaw=yaw,offset=0) for yaw in (-50,50)]

def run(case):
    name='{route}_cadence{camera_ticks}_phase{phase}_yaw{yaw}'.format(**case)
    output=f'sim/navigation_followup_phase_{name}.vvp'
    log=ROOT/f'sim/navigation_followup_phase_{name}.log'
    params=[f'-Pnavigation_route_camera_tb.{key}={case[value]}' for key,value in
            [('CAMERA_TICKS','camera_ticks'),('CAMERA_PHASE','phase'),
             ('START_YAW_TENTHS','yaw'),('START_OFFSET_MM','offset')]]
    params += ['-Pnavigation_route_camera_tb.EDGE_MODE=1']
    start=time.monotonic()
    with log.open('w',encoding='utf-8') as stream:
        for command in [
            ['iverilog','-g2012','-s','navigation_route_camera_tb',*params,'-o',output,
             'sim/navigation_route_camera_tb.sv',*sources],
            ['vvp',output,f'+ROUTE=sim/navigation_robust_{case["route"]}_route.csv',
             f'+CSV=sim/navigation_followup_phase_{name}_trace.csv']]:
            result=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=600)
            if result.returncode:raise RuntimeError(log)
    match=re.search(r'PASS actual graph navigation.*count=(\d+) maxError=([\d.]+) fallbackTicks=(\d+) handbacks=(\d+)',log.read_text())
    assert match,name
    result=dict(**case,name=name,passed=True,max_nose_error_mm=float(match[2]),
                seconds=round(time.monotonic()-start,3),log=log.relative_to(ROOT).as_posix(),
                sha256=hashlib.sha256(log.read_bytes()).hexdigest())
    print('PASS',name,'noseError',match[2],flush=True)
    return result

if __name__=='__main__':
    with ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,cases))
    (ROOT/'sim/navigation_followup_phase_runs.json').write_text(json.dumps(results,indent=2)+'\n')
