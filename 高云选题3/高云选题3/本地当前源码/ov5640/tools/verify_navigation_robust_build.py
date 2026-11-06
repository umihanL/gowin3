"""Audit current navigation-only build, actual map tests and immutable inputs."""
from pathlib import Path
import datetime,hashlib,json,re
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'impl/baseline_navigation_robust_20261025'
def read(path):
 data=path.read_bytes()
 return data.decode('utf-16' if data.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig').replace('\r\n','\n')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def dump(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
baseline=json.loads(read(BASE/'manifest.json'))
assert all(sha(BASE/n)==h for n,h in baseline.items()),'Backup changed'
changed=[n for n,h in baseline.items() if n.startswith('src/') and sha(ROOT/n)!=h]
expected={'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_vision_fusion.v','src/vehicle/vehicle_route_profile.v'}
assert set(changed)==expected,changed
for n,h in baseline.items():
 if n.startswith('src/') and n not in expected:assert sha(ROOT/n)==h,n
snapshot_path=ROOT/'impl/navigation_robust_20261025_inputs.json';snapshot=json.loads(read(snapshot_path))
inputs=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in ('.v','.vo','.vh','.mem','.cst','.sdc','.ipc','.mod')]
inputs += [ROOT/'ov5640.gprj',ROOT/'build.tcl']
actual={p.relative_to(ROOT).as_posix():sha(p) for p in inputs}
assert actual==snapshot['files'],'Hardware inputs changed during build'
fs=ROOT/'impl/pnr/ov5640.fs'
assert fs.stat().st_mtime>=max([snapshot_path.stat().st_mtime]+[p.stat().st_mtime for p in inputs]),'Stale bitstream'
report_path=ROOT/'impl/pnr/ov5640.rpt.txt';timing_path=ROOT/'impl/pnr/ov5640.tr'
assert min(report_path.stat().st_mtime,timing_path.stat().st_mtime)>=fs.stat().st_mtime
report,timing=read(report_path),read(timing_path)
build_path=ROOT/'impl/navigation_robust_20261025_build.log';build=read(build_path)
metrics=json.loads(read(ROOT/'impl/navigation_robust_20261025_build_metrics.json'))
assert metrics['exit_code']==0 and metrics['mode']=='standard_syn_and_pnr'
assert 'Bitstream generation completed' in build and not re.search(r'^\s*ERROR\b',build,re.M)
assert 'Prior incremental matching was slower; use standard implementation' in build
assert '-inc_pnr ' not in read(ROOT/'impl/pnr/cmd.do') or '-inc_pnr 0' in read(ROOT/'impl/pnr/cmd.do')
resources={}
for name in ('Logic','Register','CLS','BSRAM','DSP','I/O Port'):
 m=re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|',report,re.M);assert m,name
 used,total=map(float,m.groups());assert used<=total
 resources[name]=dict(used=used,total=total,percent=round(100*used/total,2))
violations={};slack={}
for kind in ('Setup','Hold'):
 m=re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)',timing);assert m
 violations[kind]=int(m[1])
for kind in ('Setup','Hold','Recovery','Removal'):
 m=re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)',timing,re.M|re.S);assert m,kind
 values=[float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s',m[1],re.M)];assert values,kind
 slack[kind]=min(values)
assert not any(violations.values()) and min(slack.values())>=0,(violations,slack)
pins=dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',read(ROOT/'src/ov5640_ddr3_tft.cst'),re.M))
placed=dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',report,re.M))
assert pins and len(pins)==len(set(pins.values())) and all(placed.get(n)==p for n,p in pins.items())
assert str(ROOT/'impl/gwsynthesis/ov5640.vg') in report
for n in ('src/ov5640_ddr3_tft.cst','src/ov5640_ddr3_tft.sdc'):assert str(ROOT/n) in report
dependencies=[ROOT/n for n in changed]+[ROOT/'src/vehicle/vehicle_config.vh']
regressions={}
def check(name,filename,count,markers,extra=()):
 path=ROOT/filename;content=read(path)
 assert content.count('$finish called')==count,(name,content.count('$finish called'))
 assert not re.search(r'^(FATAL|FAIL|ERROR|Traceback)\b',content,re.M),name
 assert all(m in content for m in markers),(name,markers)
 scoped=dependencies
 if name in ('handoff','synthesized_handoff'):
  scoped=[ROOT/'src/vehicle/vehicle_vision_fusion.v',ROOT/'src/vehicle/vehicle_config.vh']
 elif name in ('graph_tangents','synthesized_profiles'):
  scoped=[ROOT/'src/vehicle/vehicle_route_profile.v',ROOT/'src/vehicle/vehicle_config.vh',ROOT/'src/imu/imu_math.v']
 elif name=='photo_planner':
  scoped=[ROOT/'src/navigation/nav_road_planner.v']+[p for p in (ROOT/'src/navigation').glob('nav_centerline*.vh')]
 assert path.stat().st_mtime>=max(p.stat().st_mtime for p in scoped+[ROOT/n for n in extra]),('Stale test',name)
 regressions[name]=dict(passed=True,completed_runs=count,log=filename,sha256=sha(path))
check('legacy','sim/navigation_robust_legacy_results.txt',22,['PASS committed quarter turn side=-1:','PASS committed quarter turn side=1:','PASS terminal arrival:'],['sim/run_navigation_corner_tests.py'])
check('handoff','sim/navigation_robust_handoff.log',1,['PASS persistent fragments mode=0:','PASS persistent fragments mode=1:','PASS persistent fragments mode=2:','PASS visual handback:','PASS S reversal:','PASS bounded recovery:'],['sim/navigation_handoff_tb.sv'])
check('synthesized_handoff','sim/navigation_robust_netlist.log',1,['PASS persistent fragments mode=2:','PASS S reversal:','PASS bounded recovery:'],['sim/navigation_handoff_tb.sv','sim/run_navigation_robust_netlist.py'])
check('graph_tangents','sim/navigation_robust_profile_graph.log',1,['PASS graph tangent metadata:'],['sim/navigation_profile_graph_tb.sv'])
check('synthesized_profiles','sim/navigation_robust_profile_netlist.log',2,['PASS actual profile shape=0:','PASS actual profile shape=1:','PASS actual profile shape=2:','PASS graph tangent metadata:'],['sim/navigation_profile_graph_tb.sv','sim/navigation_profile_corner_tb.sv','sim/run_navigation_robust_profile_netlist.py'])
check('photo_planner','sim/navigation_robust_photo_planner.log',1,['PASS photograph RTL planner:'],['sim/navigation_photo_planner_tb.sv','src/navigation/nav_road_planner.v'])
check('photo_closed_loop','sim/navigation_robust_photo_rtl_camera.log',1,['PASS actual graph navigation'],['sim/navigation_route_camera_tb.sv','sim/navigation_robust_photo_rtl_route.csv'])
for name,markers in [('video',['PASS video-to-fusion sequence:','PASS dense contours:']),('vehicle',['PASS smooth low-speed stop:','PASS navigation dynamic obstacle:','PASS committed bypass:']),('profile',['PASS route profile:'])]:
 check(name,f'sim/navigation_corner_{name}_20261025.log',1,markers)
route_runs=json.loads(read(ROOT/'sim/navigation_robust_20261025_runs.json'));assert len(route_runs)==21
for case in route_runs:
 check(case['name'],case['log'],1,['PASS actual graph navigation'],['sim/navigation_route_camera_tb.sv','sim/run_navigation_robust_tests.py',f'sim/navigation_robust_{case["route"]}_route.csv'])
 assert case['passed'] and case['max_nose_error_mm']<=50
 assert sha(ROOT/case['log'])==case['sha256']
 if case['handback']:assert case['handbacks']>0
assert sum(v['completed_runs'] for v in regressions.values())==53
def geometry(filename):
 points=[]
 for row in read(ROOT/filename).splitlines():
  p=tuple(map(int,row.split(',')))[:2]
  if not points or points[-1]!=p:points.append(p)
 return points
assert geometry('sim/navigation_robust_photo_route.csv')==geometry('sim/navigation_robust_photo_rtl_route.csv')
repro=json.loads(read(ROOT/'sim/navigation_robust_reproduction.json'))
assert repro['exit_code']!=0 and repro['expected_failure'] in read(ROOT/repro['log'])
assert sha(ROOT/repro['source'])==repro['source_sha256']
netlist=read(ROOT/'impl/gwsynthesis/ov5640.vg')
for module in ('vehicle_motion','vehicle_vision_fusion','vehicle_bypass','vision_video','lsm6dsv16x_spi','vl53l1x_array_controller'):
 assert re.search(r'^module '+module+r'\b',netlist,re.M),module
roms=dict(map=audit_map(),font=audit_font(),road=audit_road())
result=dict(status='digital_verification_passed',recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
 bitstream='impl/pnr/ov5640.fs',sha256=sha(fs),bytes=fs.stat().st_size,generated_utc=datetime.datetime.fromtimestamp(fs.stat().st_mtime,datetime.timezone.utc).isoformat(),
 tool='Gowin V1.9.12.03',device='GW5AT-LV60PG484AC1/I0 B',hardware_tested=False,flashed=False,
 synthesis_input_manifest=snapshot_path.relative_to(ROOT).as_posix(),input_count=len(actual),input_sha256=actual,source_inputs_unchanged=True,
 changed_baseline_rtl=changed,unchanged_hardware_source_count=sum(n.startswith('src/') for n in baseline)-len(changed),
 bypass_source_unchanged=True,non_navigation_hardware_unchanged=True,resources=resources,violated_endpoints=violations,min_slack_ns=slack,timing_passed=True,pins_verified=pins,roms=roms,
 regression_completed_runs=53,regressions=regressions,actual_route_runs=route_runs,old_source_failure_reproduction=repro,
 photograph_route=dict(cost_cm=634,rtl_samples=160,oracle_unique_geometry_matches_rtl=True),
 build=dict(**metrics,log=build_path.relative_to(ROOT).as_posix(),log_sha256=sha(build_path),reason='Previous measured incremental matching was slower; standard implementation used.'),
 rollback=dict(bitstream='impl/baseline_navigation_robust_20261025/impl/pnr/ov5640.fs',sha256=baseline['impl/pnr/ov5640.fs']),
 navigation=dict(visual_centred_lane_priority=True,confirmed_curve_uses_pose_pursuit_with_visual_trim=True,
 map_guidance_predicate_registered=True,
 invalid_centre_handoff_complete_frames=2,recovery_uses_map_imu_encoders=True,recovery_speed_cap_mm_s=100,
 recovery_max_seconds=60,recovery_max_travel_mm=2400,recovery_camera_timeout_ms=200,recovery_map_invalid_ms=200,
 recovery_exit_heading_tolerance_degrees=10,s_curve_uses_actual_exit_not_forced_90=True,tiny_joins_ignore_direction_below_mm=1,
 pd_gains_unchanged=True,camera_geometry_unchanged=True,servo_and_bypass_unchanged=True),
 limitations=['Not flashed or tested on the physical car.',
 'Camera tests use geometric observations and synthetic images, not replay of calibrated user camera frames.',
 'Measured simulation errors refer to the nose centre, not guaranteed full-body wall clearance.',
 'Map recovery depends on the existing encoder/IMU pose estimate; long-term localization drift is not eliminated.',
 'The frozen hardware modules include existing unrelated tool warnings; no zero-warning claim is made.'])
for path in ('docs/navigation_robust_20261025_validation.json','impl/pnr/ov5640_navigation_robust_audit.json','impl/pnr/ov5640_current_audit.json'):dump(ROOT/path,result)
dump(ROOT/'impl/pnr/ov5640_download.json',{k:result[k] for k in ('bitstream','sha256','bytes','generated_utc','tool','device','timing_passed','hardware_tested','flashed')})
print(json.dumps({k:result[k] for k in ('status','sha256','generated_utc','regression_completed_runs','resources','violated_endpoints','min_slack_ns','build')},indent=2))
