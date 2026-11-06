"""Source-bound regressions for selectable inertial/visual navigation."""
from pathlib import Path
import argparse, hashlib, json, subprocess, time

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--only', nargs='+')
parser.add_argument('--snapshot', action='store_true')
args = parser.parse_args()
vehicle = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]
navigation = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/navigation').glob('*.v')]
vision = ['src/portrait/vision_video.v', 'src/portrait/vision_zebra.v']
cases = [
    ('gaussian', 'vision_gaussian_tb', vision, [], []),
    ('modes', 'navigation_modes_filter_tb', vehicle+['src/imu/imu_math.v'], [], []),
    ('settings', 'vehicle_settings_tb', navigation+['src/vehicle/motor/ps2_test_control.v',
      'src/imu/imu_math.v'], [], ['+algorithm_snapshot'] if args.snapshot else []),
    ('noise_2', 'vision_wide_road_tb', vision+['src/vehicle/vehicle_vision_fusion.v'],
      ['-Pvision_wide_road_tb.NOISE_PERCENT=2', '-Pvision_wide_road_tb.NOISE_SMOKE=1'], []),
    ('noise_5', 'vision_wide_road_tb', vision+['src/vehicle/vehicle_vision_fusion.v'],
      ['-Pvision_wide_road_tb.NOISE_PERCENT=5', '-Pvision_wide_road_tb.NOISE_SMOKE=1'], []),
]
sha = lambda n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
results = []
for name, top, sources, parameters, runtime in cases:
    if args.only and name not in args.only:
        continue
    log = f'sim/navigation_modes_filter_{name}.log'
    output = f'sim/navigation_modes_filter_{name}.vvp'
    dependencies = set([f'sim/{top}.sv', *sources, *[
        p.relative_to(ROOT).as_posix() for p in (ROOT/'src').rglob('*.vh')]])
    frozen = {n: sha(n) for n in sorted(dependencies)}
    start = time.monotonic()
    with (ROOT/log).open('w', encoding='utf-8') as stream:
        for command in (
            ['iverilog', '-g2012', '-s', top, '-I', 'src/navigation', '-I', 'src/vga',
             '-o', output, *parameters, f'sim/{top}.sv', *sources],
            ['vvp', output, *runtime],
        ):
            p = subprocess.run(command, cwd=ROOT, stdout=stream,
                               stderr=subprocess.STDOUT, timeout=600)
            assert p.returncode == 0, log
    content = (ROOT/log).read_text(encoding='utf-8')
    assert 'PASS ' in content and '$finish called' in content, log
    assert frozen == {n: sha(n) for n in sorted(dependencies)}, 'Inputs changed during test'
    results.append(dict(name=name, top=top, passed=True, seconds=round(time.monotonic()-start, 3),
                        log=log, log_sha256=sha(log), inputs=frozen))
    print('PASS', name, results[-1]['seconds'], 'seconds', flush=True)
record = ROOT/'sim/navigation_modes_filter_runs.json'
if args.only and record.exists():
    replaced = {row['name'] for row in results}
    results = [row for row in json.loads(record.read_text()) if row['name'] not in replaced]+results
record.write_text(json.dumps(results, indent=2)+'\n')
