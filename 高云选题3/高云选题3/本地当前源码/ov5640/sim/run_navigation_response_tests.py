"""Navigation-only response regression with independent vehicle dynamics."""
from pathlib import Path
import json
import subprocess
import time
ROOT = Path(__file__).resolve().parents[1]
vehicle = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]
plant = vehicle + ['src/imu/imu_math.v']
fusion = ['src/vehicle/vehicle_vision_fusion.v']
cases = [('navigation_response_tb', 'response', [], fusion),
         ('vision_fusion_tb', 'pd', [], fusion),
         ('vision_blind_turn_tb', 'edge_sequence', [], fusion)]
cases += [('vision_turn_motion_tb', f'side{s}_skip{k}',
           [f'-Pvision_turn_motion_tb.SIDE={s}', f'-Pvision_turn_motion_tb.SKIP_SINGLE={k}'], plant)
          for s in (-1, 1) for k in (0, 1)]
cases += [('vision_motion_tb', f'yaw{y}', [f'-Pvision_motion_tb.INITIAL_YAW_TENTHS={y}'], plant)
          for y in (0, 80, -80)]
cases += [('vision_goal_tb', f'dense{d}_vertical{v}',
           [f'-Pvision_goal_tb.DENSE={d}', f'-Pvision_goal_tb.VERTICAL={v}'], plant)
          for d in (0, 1) for v in (0, 1)]
results = []
with (ROOT/'sim/navigation_response_20261025_results.txt').open('w', encoding='utf-8') as log:
    for top, variant, params, sources in cases:
        output = f'sim/navigation_20261025_{top}_{variant}.vvp'
        started = time.monotonic()
        for command in (['iverilog','-g2012','-I','src/navigation','-I','src/vga','-s',top,*params,
                         '-o',output,f'sim/{top}.sv',*sources], ['vvp',output]):
            result = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=600)
            print(result.stdout, end='', flush=True); log.write(result.stdout); log.flush()
            if result.returncode: raise SystemExit(result.returncode)
        assert '$finish called' in result.stdout and 'PASS ' in result.stdout
        results.append(dict(top=top, variant=variant, seconds=round(time.monotonic()-started, 3)))
(ROOT/'sim/navigation_response_20261025_runs.json').write_text(json.dumps(results, indent=2)+'\n')
print('PASS navigation response suite:', len(results), 'runs', flush=True)
