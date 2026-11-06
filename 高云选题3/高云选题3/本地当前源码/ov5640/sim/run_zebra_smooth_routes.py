"""Run actual planner/IMU-encoder replay for the S-only resampling change."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import argparse
import csv
import hashlib
import json
import re
import subprocess
import time

R = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(); p.add_argument('--stage', choices=['plan', 'motion'], required=True)
p.add_argument('--cases', nargs='+'); a = p.parse_args()
plans = dict(s_forward=['+SX=52', '+SY=107', '+GX=124', '+GY=40'],
             s_reverse=['+SX=124', '+SY=40', '+GX=52', '+GY=107'],
             outer_left=['+SX=163', '+SY=50', '+GX=90', '+GY=13'],
             outer_right=['+SX=90', '+SY=13', '+GX=163', '+GY=50'], photo=['+PHOTO'])
vehicle = ['src/vehicle/'+n+'.v' for n in ('vehicle_motion', 'vehicle_tracker', 'vehicle_odometry',
    'vehicle_sincos', 'vehicle_route_profile', 'vehicle_bypass')]+['src/imu/imu_math.v']


def run(name):
    top = 'inertial_planner_audit_tb' if a.stage == 'plan' else 'inertial_zebra_route_tb'
    base = 's_forward' if name == 's_forward_zebra' else name
    route = f'sim/zebra_smooth_{base}_route.csv'; trace = f'sim/zebra_smooth_{name}_trace.csv'
    log = f'sim/zebra_smooth_{a.stage}_{name}.log'; out = f'sim/zebra_smooth_{a.stage}_{name}.vvp'
    sources = ['src/navigation/nav_road_planner.v'] if a.stage == 'plan' else vehicle
    params = ['-Pinertial_zebra_route_tb.ZEBRA=1'] if name.endswith('_zebra') else []
    runtime = plans[name] if a.stage == 'plan' else [f'+CSV={trace}']
    start = time.monotonic()
    with (R/log).open('w') as f:
        for cmd in [['iverilog', '-g2012', '-I', 'src/navigation', '-s', top, *params, '-o', out, f'sim/{top}.sv', *sources],
                    ['vvp', '-i', out, f'+ROUTE={route}', *runtime]]:
            code = subprocess.run(cmd, cwd=R, stdout=f, stderr=subprocess.STDOUT, timeout=600).returncode
            if code: break
    text = (R/log).read_text(); assert code == 0 and 'PASS ' in text, (name, text)
    result = dict(name=name, passed=True, seconds=round(time.monotonic()-start, 2), log=log,
                  log_sha256=hashlib.sha256((R/log).read_bytes()).hexdigest())
    if a.stage == 'motion':
        m = re.search(r'status=(\d+) index=(\d+)/(\d+) maxCross=([\d.]+) maxPose=([\d.]+)', text)
        result['metrics'] = dict(zip(['status', 'index', 'count', 'max_cross_mm', 'max_pose_mm'], map(float, m.groups())))
        rows = list(csv.DictReader((R/trace).open()))
        s = [r for r in rows if 1640 < float(r['nose_x']) < 1940 and 1500 < float(r['nose_y']) < 1950]
        if s:
            commands = [int(r['command']) for r in s]
            result['s_transition'] = dict(max_cross_mm=max(float(r['cross_error']) for r in s),
                maximum_command_step_deg=max(abs(x-y) for x, y in zip(commands, commands[1:])),
                near_zero_command_ms=20*sum(abs(x) <= 1 for x in commands))
        if name in ('outer_left', 'outer_right'):
            assert (R/trace).read_bytes() == (R/f'sim/inertial_zebra_{name}_trace.csv').read_bytes(), 'Non-S route changed'
            result['identical_to_previous_release'] = True
        result['trace'] = trace
    else:
        result['route'] = route
    print('PASS', a.stage, name, text.strip(), flush=True)
    return result


names = a.cases or list(plans)
if a.stage == 'motion' and not a.cases: names += ['s_forward_zebra']
with ThreadPoolExecutor(max_workers=3) as pool:
    results = list(pool.map(run, names))
record = R/f'sim/zebra_smooth_{a.stage}_runs.json'
if a.cases and record.exists():
    replaced = {r['name'] for r in results}
    results = [r for r in json.loads(record.read_text()) if r['name'] not in replaced]+results
record.write_text(json.dumps(results, indent=2)+'\n')
