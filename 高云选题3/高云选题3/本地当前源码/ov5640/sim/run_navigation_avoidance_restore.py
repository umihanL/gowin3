"""Focused local motion RTL regressions; no synthesis, timing or hardware I/O."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import hashlib,json,subprocess,time

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'diagnostics/restore_avoidance_20261102'
OUT.mkdir(parents=True,exist_ok=True)
SOURCES=['src/vehicle/'+name+'.v' for name in ('vehicle_motion','vehicle_bypass','vehicle_odometry',
    'vehicle_tracker','vehicle_sincos')]+['src/imu/imu_math.v']
CASES=[('avoidance_on','navigation_avoidance_tb',[]),
       ('inertial_off','navigation_uninterrupted_tb',[]),
       ('drift_continue','navigation_drift_continue_tb',[]),
       ('physical_bypass','vehicle_motion_tb',['+navigation_bypass_only'])]
sha=lambda name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
dependencies=SOURCES+['src/vehicle/vehicle_config.vh']+[f'sim/{top}.sv' for _,top,_ in CASES]
frozen={name:sha(name) for name in dependencies}

def run(case):
    name,top,args=case;started=time.monotonic()
    binary=f'diagnostics/restore_avoidance_20261102/{name}.vvp'
    log=OUT/(name+'.log')
    commands=[['iverilog','-g2012','-s',top,'-o',binary,f'sim/{top}.sv',*SOURCES],
              ['vvp','-i',binary,*args]]
    code=0
    with log.open('wb') as stream:
        for command in commands:
            result=subprocess.run(['wsl.exe','--exec',*command],cwd=ROOT,stdout=stream,
                                  stderr=subprocess.STDOUT,timeout=600)
            code=result.returncode
            if code:break
    text=log.read_text(encoding='utf-8',errors='replace')
    passed=code==0 and 'PASS ' in text and 'FATAL:' not in text
    print(('PASS ' if passed else 'FAIL ')+name,flush=True)
    return dict(name=name,passed=passed,seconds=round(time.monotonic()-started,3),
                commands=commands,log=log.relative_to(ROOT).as_posix(),log_sha256=sha(log.relative_to(ROOT).as_posix()))

if __name__=='__main__':
    with ThreadPoolExecutor(max_workers=4) as pool:cases=list(pool.map(run,CASES))
    unchanged=all(sha(name)==value for name,value in frozen.items())
    result=dict(passed=unchanged and all(case['passed'] for case in cases),source_unchanged=unchanged,
                cases=cases,hardware_inputs={n:v for n,v in frozen.items() if n.startswith('src/')},
                simulation_inputs={n:v for n,v in frozen.items() if n.startswith('sim/')},
                hardware_tested=False,flashed=False,full_project_compiled=False,
                scope='Entry-latched map avoidance ON/OFF; inertial fault protections; drift continuation; independent bicycle/range-beam/body collision and rejoin/arrival model.')
    (OUT/'motion.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    raise SystemExit(0 if result['passed'] else 1)
