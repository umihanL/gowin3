"""Rotated preview, black-road guidance, and real motor electrical braking."""
from pathlib import Path
import subprocess
import argparse
root=Path(__file__).resolve().parents[1]
motor=[p.relative_to(root).as_posix() for p in (root/'src/vehicle/motor').glob('*.v')]
navigation=[p.relative_to(root).as_posix() for p in (root/'src/navigation').glob('*.v')]
vehicle=[p.relative_to(root).as_posix() for p in (root/'src/vehicle').glob('*.v')]
cases={
 'font_rows_tb':[],
 'vision_display_timing_tb':['src/vga/disp_driver.v','src/imu/attitude_overlay.v','src/navigation/nav_renderer.v'],
 'vision_video_tb':['src/portrait/vision_video.v','src/portrait/vision_zebra.v','src/vehicle/vehicle_vision_fusion.v'],
 'vision_zebra_tb':['src/portrait/vision_zebra.v'],
 'vision_fusion_tb':['src/vehicle/vehicle_vision_fusion.v'],
 'vision_blind_turn_tb':['src/vehicle/vehicle_vision_fusion.v'],
 'vision_turn_motion_tb':[*vehicle,'src/imu/imu_math.v'],
 'vision_motion_tb':[*vehicle,'src/imu/imu_math.v'],
 'vision_goal_tb':[*vehicle,'src/imu/imu_math.v'],
 'vision_ui_tb':['src/portrait/vision_video.v','src/portrait/vision_zebra.v','src/imu/imu_math.v','src/imu/attitude_overlay.v',*navigation],
 'motor_brake_tb':motor,
 'motor_brake_plant_tb':motor,
}
parser=argparse.ArgumentParser()
parser.add_argument('--only',nargs='+')
args=parser.parse_args()
if args.only:cases={k:v for k,v in cases.items() if k in args.only}
with (root/'sim/vision_test_results.txt').open('w',encoding='utf-8') as log:
 for top,sources in cases.items():
  variants=([(f'dense{d}_vertical{v}',[f'-P{top}.DENSE={d}',f'-P{top}.VERTICAL={v}']) for d in (0,1) for v in (0,1)]
            if top=='vision_goal_tb' else [('right',[f'-P{top}.SIDE=-1']),('left',[f'-P{top}.SIDE=1'])]
            if top=='vision_turn_motion_tb' else
            [('straight',[]),('yaw_left',[f'-P{top}.INITIAL_YAW_TENTHS=80']),
             ('yaw_right',[f'-P{top}.INITIAL_YAW_TENTHS=-80'])]
            if top=='vision_motion_tb' else [('default',[])])
  for variant,params in variants:
   output=f'sim/{top}_{variant}.vvp'
   for cmd in (['iverilog','-g2012','-I','src/navigation','-I','src/vga','-s',top,*params,'-o',output,f'sim/{top}.sv',*sources],
               ['vvp',output]):
    result=subprocess.run(cmd,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=600)
    print(result.stdout,end='',flush=True);log.write(result.stdout);log.flush()
    if result.returncode:raise SystemExit(result.returncode)
