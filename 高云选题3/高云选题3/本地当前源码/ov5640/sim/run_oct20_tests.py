"""Focused regressions for editable barriers, adaptive braking and close bypass."""
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
vehicle=[p.relative_to(root).as_posix() for p in (root/'src/vehicle').rglob('*.v')]
navigation=[p.relative_to(root).as_posix() for p in (root/'src/navigation').glob('*.v')]
cases={
    'nav_barrier_tb':['src/navigation/nav_barrier_overlay.v','src/navigation/astar_grid.v'],
    'nav_barrier_ui_tb':navigation+['src/imu/imu_math.v'],
    'brake_envelope_tb':['src/vehicle/vehicle_motion.v'],
    'vehicle_close_bypass_tb':vehicle+['src/imu/imu_math.v','sim/ps2_receiver_model.sv','sim/vl53l0x_sensor_model.v'],
    'vl53l1x_array_tb':['src/vehicle/motor/vl53l1x_array_controller.v','src/vehicle/motor/vl53l1x_i2c_master.v','sim/vl53l1x_sensor_model.v'],
}
with (root/'sim/oct20_test_results.txt').open('w',encoding='utf-8') as log:
    for top,sources in cases.items():
        for command in (
            ['iverilog','-g2012','-s',top,'-I','src/navigation','-o',f'sim/{top}.vvp',f'sim/{top}.sv',*sources],
            ['vvp',f'sim/{top}.vvp'],
        ):
            with subprocess.Popen(command,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True) as process:
                for line in process.stdout:
                    print(line,end='',flush=True);log.write(line);log.flush()
                if process.wait()!=0:raise SystemExit(process.returncode)
