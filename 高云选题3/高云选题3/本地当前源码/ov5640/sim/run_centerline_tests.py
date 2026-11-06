"""Current road network/menu, vehicle/body geometry and vision regressions."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
nav=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/navigation').glob('*.v')]
vehicle=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]
cases={
 'road_planner_tb':['src/navigation/nav_road_planner.v'],
 'road_ui_tb':nav+['src/imu/imu_math.v'],
 'road_motion_tb':vehicle+['src/imu/imu_math.v'],
 'nav_barrier_ui_tb':nav+['src/imu/imu_math.v'],
 'vehicle_close_bypass_tb':vehicle+['src/imu/imu_math.v'],
 'bypass_edge_trend_tb':vehicle+['src/imu/imu_math.v'],
 'vehicle_nose_tb':vehicle+['src/imu/imu_math.v'],
 'vision_fusion_tb':['src/vehicle/vehicle_vision_fusion.v'],
 'vision_motion_tb':vehicle+['src/imu/imu_math.v'],
}
with (ROOT/'sim/centerline_test_results.txt').open('w',encoding='utf-8') as log:
 for top,sources in cases.items():
  ext='sv'
  case_output=[]
  for cmd in (['iverilog','-g2012','-I','src/navigation','-I','src/vga','-s',top,
               '-o',f'sim/{top}.vvp',f'sim/{top}.{ext}',*sources],['vvp',f'sim/{top}.vvp']):
   with subprocess.Popen(cmd,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True) as p:
    for line in p.stdout:
     print(line,end='',flush=True);log.write(line);log.flush();case_output.append(line)
    if p.wait():raise SystemExit(p.returncode)
  filename={'road_motion_tb':'road_motion.log','vehicle_close_bypass_tb':'centerline_bypass_extended.log'}.get(top)
  if filename:(ROOT/'sim'/filename).write_text(''.join(case_output),encoding='utf-8')
