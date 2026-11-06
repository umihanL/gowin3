"""Independent inertia/noise/transport regression for the October 20 fixes."""
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
sources=[p.relative_to(root).as_posix() for p in (root/'src/vehicle').rglob('*.v')]
sources+=['src/imu/imu_math.v','sim/ps2_receiver_model.sv','sim/vl53l0x_sensor_model.v']
cases={'vehicle_followup_tb':sources,'vehicle_close_bypass_tb':sources}
with (root/'sim/followup_test_results.txt').open('w',encoding='utf-8') as log:
    for top,files in cases.items():
        for command in (
            ['iverilog','-g2012','-s',top,'-o',f'sim/{top}.vvp',f'sim/{top}.sv',*files],
            ['vvp',f'sim/{top}.vvp'],
        ):
            with subprocess.Popen(command,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True) as p:
                for line in p.stdout:
                    print(line,end='',flush=True);log.write(line);log.flush()
                if p.wait():raise SystemExit(p.returncode)
