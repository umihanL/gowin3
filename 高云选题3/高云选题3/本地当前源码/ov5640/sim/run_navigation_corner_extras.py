from pathlib import Path
import subprocess,time,json
r=Path(__file__).resolve().parents[1]
sources=[p.relative_to(r).as_posix() for p in (r/'src/vehicle').rglob('*.v')]+['src/imu/imu_math.v']
cases=[('video',['iverilog','-g2012','-I','src/navigation','-I','src/vga','-s','vision_video_tb','-o','sim/navigation_corner_video.vvp','sim/vision_video_tb.sv','src/portrait/vision_video.v','src/portrait/vision_zebra.v','src/vehicle/vehicle_vision_fusion.v'],'sim/navigation_corner_video.vvp'),('vehicle',['iverilog','-g2012','-s','vehicle_motion_tb','-o','sim/navigation_corner_vehicle.vvp','sim/vehicle_motion_tb.sv',*sources,'sim/ps2_receiver_model.sv','sim/vl53l0x_sensor_model.v','sim/vl53l1x_sensor_model.v'],'sim/navigation_corner_vehicle.vvp'),('profile',['iverilog','-g2012','-s','vehicle_profile_tb','-o','sim/navigation_corner_profile.vvp','sim/vehicle_profile_tb.sv',*sources],'sim/navigation_corner_profile.vvp')]
results=[]
for name,compile_cmd,exe in cases:
 start=time.monotonic()
 with (r/f'sim/navigation_corner_{name}_20261025.log').open('w',encoding='utf-8') as log:
  for cmd in (compile_cmd,['vvp',exe]):
   p=subprocess.run(cmd,cwd=r,stdout=log,stderr=subprocess.STDOUT,timeout=600)
   if p.returncode:raise SystemExit(f'{name} failed: {p.returncode}')
 results.append(dict(name=name,seconds=round(time.monotonic()-start,2)))
 print(name,'PASS',results[-1]['seconds'],flush=True)
(r/'sim/navigation_corner_extras_20261025_runs.json').write_text(json.dumps(results,indent=2)+chr(10))
