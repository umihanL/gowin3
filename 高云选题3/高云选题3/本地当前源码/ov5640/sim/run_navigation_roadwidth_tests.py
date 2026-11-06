"""Exercise the measured 340/373 mm lanes and retain source-bound results."""
from pathlib import Path
import datetime, hashlib, json, subprocess, time

ROOT = Path(__file__).resolve().parents[1]
TOP = 'vision_wide_road_tb'
RTL = ['sim/vision_wide_road_tb.sv', 'src/portrait/vision_video.v',
       'src/portrait/vision_zebra.v', 'src/vehicle/vehicle_vision_fusion.v']
cases = [('measured_lanes', []), ('lane_transitions_and_curves', ['-Pvision_wide_road_tb.EXTENDED=1'])]
sha = lambda p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
results = []
for name, parameters in cases:
    executable = f'sim/vision_wide_road_{name}.vvp'
    log = ROOT/f'sim/vision_wide_road_{name}.log'
    started = time.monotonic()
    with log.open('w', encoding='utf-8') as output:
        for command in (
            ['iverilog', '-g2012', '-s', TOP, *parameters, '-o', executable, *RTL],
            ['vvp', executable],
        ):
            result = subprocess.run(command, cwd=ROOT, stdout=output,
                                    stderr=subprocess.STDOUT, timeout=900)
            if result.returncode:
                raise SystemExit(f'{name} failed; see {log.relative_to(ROOT)}')
    content = log.read_text(encoding='utf-8')
    assert 'PASS ' in content and '$finish called' in content, name
    results.append({
        'name': name,
        'passed': True,
        'seconds': round(time.monotonic()-started, 3),
        'log': log.relative_to(ROOT).as_posix(),
        'sha256': sha(log.relative_to(ROOT)),
        'inputs': {path: sha(path) for path in RTL},
    })
    print('PASS', name, results[-1]['seconds'], flush=True)

summary = {
    'status': 'passed',
    'recorded_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'road_widths_mm': {'inner': 340, 's_bend': 340, 'outer': 373},
    'cases': results,
}
(ROOT/'sim/navigation_roadwidth_runs.json').write_text(
    json.dumps(summary, indent=2)+'\n', encoding='utf-8')
