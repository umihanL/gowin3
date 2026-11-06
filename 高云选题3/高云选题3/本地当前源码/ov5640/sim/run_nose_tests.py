"""Nose-reference geometry, touch alignment and electrical braking regression."""
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
vehicle=[p.relative_to(root).as_posix() for p in (root/'src/vehicle').rglob('*.v')]
cases={
    'vehicle_nose_tb':vehicle+['src/imu/imu_math.v'],
    'motor_brake_tb':[p.relative_to(root).as_posix() for p in (root/'src/vehicle/motor').glob('*.v')],
}
with (root/'sim/nose_test_results.txt').open('w',encoding='utf-8') as log:
    for top,sources in cases.items():
        for command in (
            ['iverilog','-g2012','-s',top,'-o',f'sim/{top}.vvp',f'sim/{top}.sv',*sources],
            ['vvp',f'sim/{top}.vvp'],
        ):
            with subprocess.Popen(command,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True) as process:
                for line in process.stdout:
                    print(line,end='',flush=True);log.write(line);log.flush()
                if process.wait():raise SystemExit(process.returncode)
