"""Record the two regressions against the accepted pre-change RTL."""
from pathlib import Path
import json
import subprocess
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'impl/baseline_navigation_20261025'
sources=[p.relative_to(ROOT).as_posix() for p in (BASE/'src/vehicle').glob('*.v')]
cases=[('navigation_response_tb', [str(BASE/'src/vehicle/vehicle_vision_fusion.v')], 'skipped-single-edge bend failed'),
       ('vision_motion_tb', sources+['src/imu/imu_math.v'], 'startup drove before visual steering was prepared')]
results=[]
for top,dependencies,expected in cases:
    binary=f'sim/navigation_old_{top}.vvp'
    compile=subprocess.run(['iverilog','-g2012','-s',top,'-o',binary,f'sim/{top}.sv',*dependencies],
                           cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    assert compile.returncode==0,compile.stdout
    run=subprocess.run(['vvp',binary],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=120)
    assert run.returncode!=0 and expected in run.stdout,run.stdout
    path=ROOT/f'sim/navigation_old_{top}.log'
    path.write_text(run.stdout,encoding='utf-8')
    results.append(dict(test=top,expected_failure=expected,exit_code=run.returncode,log=path.relative_to(ROOT).as_posix()))
(ROOT/'sim/navigation_20261025_reproductions.json').write_text(json.dumps(results,indent=2)+'\n')
print('PASS expected old-code failures reproduced:',len(results))
