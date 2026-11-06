"""Export real FPGA four-checkpoint paths in all four regions; no hardware I/O."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'diagnostics/continuous_route_20261103'


def invoke(args, log):
    with log.open('wb') as stream:
        result = subprocess.run(['wsl.exe', '--exec', *args], cwd=ROOT,
                                stdout=stream, stderr=subprocess.STDOUT, timeout=2400)
    return result.returncode, log.read_bytes()


def run(case):
    began = time.monotonic()
    log = OUT / (case.lower() + '_planner.log')
    code, text = invoke(['vvp', '-i', 'diagnostics/continuous_route_20261103/regions.vvp',
                         '+region=' + str('ABCD'.index(case))], log)
    passed = code == 0 and b'PASS continuous task' in text and b'FATAL' not in text
    csv = OUT / (case.lower() + '_route.csv')
    record = dict(region=case, passed=passed, seconds=round(time.monotonic() - began, 2),
                  log=log.name, route_csv=csv.name)
    if passed:
        record.update(samples=len(csv.read_text().splitlines()) - 1,
                      route_sha256=hashlib.sha256(csv.read_bytes()).hexdigest())
    print(('PASS ' if passed else 'FAIL ') + case, flush=True)
    return record


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    selected = sys.argv[1:] or list('ABCD')
    if any(case not in 'ABCD' or len(case) != 1 for case in selected):
        raise SystemExit('Expected A B C D')
    code, output = invoke(['iverilog', '-g2012', '-s', 'continuous_waypoint_planner_tb',
        '-I', 'src/navigation', '-o', 'diagnostics/continuous_route_20261103/regions.vvp',
        'sim/continuous_waypoint_planner_tb.sv', 'src/navigation/nav_road_planner.v'], OUT / 'regions_compile.log')
    if code:
        raise RuntimeError(output.decode('utf-8', errors='replace'))
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(run, selected))
    report = dict(passed=all(row['passed'] for row in results), cases=results,
        hardware_tested=False, flashed=False,
        source_sha256=hashlib.sha256((ROOT / 'src/navigation/nav_road_planner.v').read_bytes()).hexdigest())
    (OUT / 'regions_results.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    raise SystemExit(0 if report['passed'] else 1)
