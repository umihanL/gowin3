"""Frozen-source, pin-level UART/UI/real-planner/odometry heading regression."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'sim/heading_validation'/datetime.now().strftime('%Y%m%d_%H%M%S')
OUT.mkdir(parents=True)
WORK = OUT/'project'
shutil.copytree(ROOT/'src', WORK/'src')
(WORK/'sim').mkdir()
shutil.copy2(ROOT/'sim/heading_navigation_tb.sv', WORK/'sim/heading_navigation_tb.sv')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
inputs = {p.relative_to(WORK).as_posix():sha(p) for p in (WORK/'src').rglob('*') if p.is_file()}
sources = [p.relative_to(WORK).as_posix() for folder in ('navigation','vehicle')
           for p in (WORK/'src'/folder).rglob('*.v')]+['src/imu/imu_math.v']

def invoke(args, timeout=1800):
    result = subprocess.run((['wsl.exe','--exec'] if os.name=='nt' else [])+args,
                            cwd=WORK,capture_output=True,timeout=timeout)
    return result.returncode,(result.stdout+result.stderr).decode('utf-8',errors='replace').replace('\0','')

code, text = invoke(['iverilog','-g2012','-s','heading_navigation_tb','-I','src/navigation','-I','src/vga',
                     '-o','heading.vvp','sim/heading_navigation_tb.sv',*sources])
(OUT/'compile.log').write_text(text,encoding='utf-8')
if code:
    raise RuntimeError(text)

cases = [('left',['+heading=1800']),('right',['+heading=0']),
         ('perpendicular',['+heading=900','+unsupported']),
         ('rotated_after_plan',['+heading=1800','+rotate_after_plan'])]
def run(case):
    name,args=case;started=time.monotonic()
    code,text=invoke(['vvp','-i','heading.vvp',*args])
    (OUT/(name+'.log')).write_text(text,encoding='utf-8')
    result={'name':name,'passed':code==0 and 'PASS ' in text and 'FATAL:' not in text,
            'seconds':round(time.monotonic()-started,3),'log':name+'.log'}
    print(('PASS' if result['passed'] else 'FAIL'),name,result['seconds'],flush=True)
    return result

print('Output:',OUT,flush=True)
with ThreadPoolExecutor(max_workers=4) as pool:
    results=list(pool.map(run,cases))
report={'passed':all(x['passed'] for x in results),'cases':results,'source_inputs':inputs,
        'test_sha256':sha(WORK/'sim/heading_navigation_tb.sv'),
        'source_unchanged':all((ROOT/p).is_file() and sha(ROOT/p)==h for p,h in inputs.items()),
        'scope':'Pin-level UART, start-heading reference+IMU delta, directed graph, real odometry startup; no physical movement or arrival accuracy claim.',
        'output':str(OUT)}
for target in (OUT/'validation.json',ROOT/'sim/heading_validation/latest.json'):
    target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
if not report['passed'] or not report['source_unchanged']:
    raise SystemExit(1)
