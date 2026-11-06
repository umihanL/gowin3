"""Exercise real image extraction and the calibrated servo output pin."""
from pathlib import Path
import argparse, hashlib, json, subprocess, time
ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--case', choices=('image', 'actuator', 'both'), default='both')
parser.add_argument('--before', action='store_true')
args = parser.parse_args()
cases = []
if args.case in ('image', 'both'):
    cases.append(('image', 'navigation_image_start_tb',
                  ['src/portrait/vision_video.v', 'src/portrait/vision_zebra.v',
                   'src/vehicle/vehicle_vision_fusion.v'],
                  ['src/portrait/vision_camera_config.vh', 'src/portrait/vision_ipm_lut.vh']))
if args.case in ('actuator', 'both'):
    cases.append(('actuator', 'navigation_actuator_direction_tb',
                  [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]
                  + ['src/imu/imu_math.v'], ['src/vehicle/servo_offset.mem']))
for name, top, sources, auxiliary in cases:
    if args.before:
        sources = ['impl/baseline_navigation_direction_20261027/'+n for n in sources]
    label = 'before_' if args.before else ''
    log = ROOT/f'sim/navigation_direction_{label}{name}.log'
    out = f'sim/navigation_direction_{label}{name}.vvp'
    inputs = [f'sim/{top}.sv', *sources, *auxiliary, 'src/vehicle/vehicle_config.vh']
    frozen = {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in inputs}
    started = time.monotonic()
    with log.open('w') as stream:
        for command in (['iverilog', '-g2012', '-s', top, '-o', out, f'sim/{top}.sv', *sources],
                        ['vvp', out]):
            result = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, timeout=600)
            if result.returncode:
                break
    if args.before:
        assert name == 'image' and result.returncode and 'PIXEL_START_PRELOAD' in log.read_text(), log
    else:
        assert not result.returncode, log.read_text()
    assert all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest() == h for n, h in frozen.items())
    record = dict(name=name, passed=True, expected_failure=args.before, exit_code=result.returncode,
                  seconds=round(time.monotonic()-started, 3), inputs=frozen,
                  log=log.relative_to(ROOT).as_posix(), sha256=hashlib.sha256(log.read_bytes()).hexdigest())
    (ROOT/f'sim/navigation_direction_{label}{name}_runs.json').write_text(json.dumps([record], indent=2)+'\n')
    print(('REPRODUCED ' if args.before else 'PASS ')+name, record['seconds'], flush=True)
