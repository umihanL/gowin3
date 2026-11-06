"""Preserve separate logs and reproduce new faults against the frozen RTL."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'navigation_followup_20261026'
BASE = 'impl/baseline_navigation_followup_20261026'

def run(name, top, sources, params=(), expect=0):
    output = f'sim/{LABEL}_{name}.vvp'
    log = ROOT / f'sim/{LABEL}_{name}.log'
    start = time.monotonic()
    commands = [
        ['iverilog', '-g2012', '-s', top, *params, '-o', output,
         f'sim/{top}.sv', *sources],
        ['vvp', output],
    ]
    with log.open('w', encoding='utf-8') as stream:
        for cmd in commands:
            result = subprocess.run(cmd, cwd=ROOT, stdout=stream,
                                    stderr=subprocess.STDOUT, timeout=600)
            if result.returncode:
                break
    assert (result.returncode == 0) == (expect == 0), (name, log.read_text())
    content = log.read_text()
    assert ('PASS ' in content and '$finish called' in content) if expect == 0 else 'FATAL:' in content
    print(name, 'PASS' if expect == 0 else 'REPRODUCED', flush=True)
    return dict(name=name, passed=expect == 0, expected_failure=expect != 0,
                exit_code=result.returncode, log=log.relative_to(ROOT).as_posix(),
                sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
                seconds=round(time.monotonic()-start, 3))

if __name__ == '__main__':
    results = []
    for case in range(3):
        for baseline in (True, False):
            source = f'{BASE}/src/vehicle/vehicle_vision_fusion.v' if baseline else 'src/vehicle/vehicle_vision_fusion.v'
            params = [f'-Pnavigation_followup_tb.CASE={case}']
            if baseline:
                params += ['-DBASELINE']
            results.append(run(f'{"before" if baseline else "after"}_{case}',
                               'navigation_followup_tb', [source], params,
                               expect=1 if baseline else 0))
    (ROOT / f'sim/{LABEL}_fault_runs.json').write_text(json.dumps(results, indent=2)+'\n')
