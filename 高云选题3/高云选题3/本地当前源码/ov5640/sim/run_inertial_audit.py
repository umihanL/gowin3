"""Read-only RTL audit: actual planner output and inertial route execution."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import argparse, csv, hashlib, json, re, subprocess, time

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--stage', choices=['plan', 'motion', 'control'], required=True)
parser.add_argument('--cases', nargs='+')
args = parser.parse_args()
sha = lambda n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
vehicle = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]
plan = {'outer_left': ['+SX=163', '+SY=50', '+GX=90', '+GY=13'],
        'outer_right': ['+SX=90', '+SY=13', '+GX=163', '+GY=50'],
        's_forward': ['+SX=52', '+SY=107', '+GX=124', '+GY=40'],
        'photo': ['+PHOTO']}

def run(name):
    top = {'plan': 'inertial_planner_audit_tb', 'motion': 'inertial_route_audit_tb',
           'control': 'inertial_control_audit_tb'}[args.stage]
    log = f'sim/inertial_audit_{args.stage}_{name}.log'
    out = f'sim/inertial_audit_{args.stage}_{name}.vvp'
    route = f'sim/inertial_audit_{name}_route.csv'
    trace = f'sim/inertial_audit_{name}_trace.csv'
    sources = ['src/navigation/nav_road_planner.v'] if args.stage == 'plan' else vehicle+['src/imu/imu_math.v']
    params = []
    runtime = plan[name] if args.stage == 'plan' else [f'+CSV={trace}'] if args.stage == 'motion' else []
    if name.endswith('_hostile'):
        route = f'sim/inertial_audit_{name[:-8]}_route.csv'
        params = ['-Pinertial_route_audit_tb.HOSTILE_VISION=1']
    deps = [f'sim/{top}.sv', *sources, *[p.relative_to(ROOT).as_posix() for p in (ROOT/'src').rglob('*.vh')]]
    if args.stage == 'motion': deps.append(route)
    frozen = {n: sha(n) for n in sorted(set(deps))}
    started = time.monotonic()
    with (ROOT/log).open('w', encoding='utf-8') as stream:
        code = 0
        for cmd in (['iverilog', '-g2012', '-s', top, '-I', 'src/navigation', '-o', out,
                     *params, f'sim/{top}.sv', *sources],
                    ['vvp', '-i', out, f'+ROUTE={route}', *runtime]):
            p = subprocess.run(cmd, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, timeout=900)
            code = p.returncode
            if code: break
    content = (ROOT/log).read_text(encoding='utf-8')
    assert all(sha(n) == digest for n, digest in frozen.items()), 'RTL changed during audit'
    result = dict(name=name, passed=code == 0 and 'PASS ' in content,
                  seconds=round(time.monotonic()-started, 2), log=log, log_sha256=sha(log), inputs=frozen)
    if args.stage == 'plan' and result['passed']:
        points = [tuple(map(int, r)) for r in csv.reader((ROOT/route).open())]
        network = json.loads((ROOT/'src/navigation/road_centerlines.json').read_text())['graph']
        indices = {}
        for i, n in enumerate(network): indices.setdefault((n['x'], n['y']), []).append(i)
        # Direction states can share the same geometric point. Check that at
        # least one continuous directed graph walk explains every RAM sample.
        possible = set(indices[points[0][:2]])
        for point in points[1:]:
            candidates = set(indices[point[:2]])
            # RTL intentionally merges duplicate geometry at primitive joins.
            closure = set(possible)
            pending = list(possible)
            while pending:
                i = pending.pop()
                for j in network[i]['next']:
                    if j not in closure and (network[j]['x'], network[j]['y']) == (network[i]['x'], network[i]['y']):
                        closure.add(j)
                        pending.append(j)
            possible = {j for i in closure for j in network[i]['next'] if j in candidates}
            assert possible, (name, point, 'not a directed graph edge')
        result.update(route=route, route_sha256=sha(route), count=len(points), directed_graph_valid=True)
    if args.stage == 'motion' and (ROOT/trace).exists():
        result.update(trace=trace, trace_sha256=sha(trace))
        found = re.search(r'RESULT inertial: status=(\d+) index=(\d+)/(\d+) maxCross=([\d.]+) maxPose=([\d.]+) oppositeCurveTicks=(\d+) ticks=(\d+) targetChecks=(\d+)', content)
        if found:
            result['metrics'] = dict(zip(('status', 'index', 'count', 'max_cross_mm', 'max_pose_mm',
                                          'opposite_curve_ticks', 'ticks', 'target_checks'),
                                         map(float, found.groups())))
    print(('PASS' if result['passed'] else 'FAIL'), args.stage, name, result['seconds'], content[-600:], flush=True)
    return result

if __name__ == '__main__':
    names = args.cases or (['overrides'] if args.stage == 'control' else list(plan))
    with ThreadPoolExecutor(max_workers=3) as pool:
        results = list(pool.map(run, names))
    (ROOT/f'sim/inertial_audit_{args.stage}_runs.json').write_text(json.dumps(results, indent=2)+'\n')
    raise SystemExit(not all(r['passed'] for r in results))
