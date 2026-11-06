"""Functional controls regression using an independent Ackermann plant."""
from pathlib import Path
import subprocess
import argparse

ROOT = Path(__file__).resolve().parents[1]
sources = [p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]
sources += ['src/imu/imu_math.v']
parser=argparse.ArgumentParser()
parser.add_argument('--only',choices=('math','motion','profile','io'))
args=parser.parse_args()
with (ROOT/'sim/vehicle_test_results.txt').open('a' if args.only else 'w', encoding='utf-8') as log:
    for name in ([args.only] if args.only else ('math', 'motion', 'profile', 'io')):
        top = f'vehicle_{name}_tb'
        output = f'sim/{top}.vvp'
        for command in (
            ['iverilog', '-g2012', '-s', top, '-o', output, f'sim/{top}.sv', *sources,
             'sim/ps2_receiver_model.sv','sim/vl53l0x_sensor_model.v','sim/vl53l1x_sensor_model.v'],
            ['vvp', output],
        ):
            result = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, timeout=600)
            print(result.stdout, end='', flush=True)
            log.write(result.stdout)
            log.flush()
            if result.returncode:
                raise SystemExit(result.returncode)
