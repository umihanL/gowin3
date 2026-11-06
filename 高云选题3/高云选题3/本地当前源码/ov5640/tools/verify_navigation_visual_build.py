"""Audit the navigation/visual registration release against its frozen inputs."""
from pathlib import Path
import datetime, hashlib, json, re, shutil
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road

ROOT=Path(__file__).resolve().parents[1]
LABEL='navigation_visual_20261027'
BASE='impl/baseline_navigation_visual_20261027'
ALLOWED={'ov5640.gprj','src/ov5640_ddr3_tft.v','src/portrait/vision_video.v',
 'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_subsystem.v',
 'src/vehicle/vehicle_vision_fusion.v','src/vehicle/vehicle_visual_localizer.v'}
def read(n):
 b=(ROOT/n).read_bytes()
 return b.decode('utf-16' if b.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig').replace('\r\n','\n')
def sha(n):return hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
def dump(n,v):(ROOT/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def newer(n,deps):assert (ROOT/n).stat().st_mtime>=max((ROOT/d).stat().st_mtime for d in deps),('stale',n)

frozen=json.loads(read(f'impl/{LABEL}_inputs.json'))['files']
files=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in ('.v','.vo','.vh','.mem','.cst','.sdc','.ipc','.mod')]
files += [ROOT/'ov5640.gprj',ROOT/'build.tcl']
actual={p.relative_to(ROOT).as_posix():sha(p.relative_to(ROOT)) for p in files}
assert actual==frozen,'Build inputs changed after freeze'
baseline=json.loads(read(BASE+'/manifest.json'))
assert all(sha(BASE+'/'+n)==h for n,h in baseline.items()),'Snapshot corrupted'
changed=sorted(n for n,h in actual.items() if baseline.get(n)!=h)
assert set(changed)==ALLOWED,changed
metrics=json.loads(read(f'impl/{LABEL}_build_metrics.json'))
build=read(f'impl/{LABEL}_build.log')
assert metrics['exit_code']==0 and 'Bitstream generation completed' in build
assert 'Placement and routing completed' in build and not re.search(r'^\s*ERROR\b',build,re.M)
fs='impl/pnr/ov5640.fs'
assert sha(fs)!=baseline[fs]
for n in (fs,'impl/gwsynthesis/ov5640.vg','impl/pnr/ov5640.rpt.txt','impl/pnr/ov5640.tr'):
 newer(n,[*actual,f'impl/{LABEL}_inputs.json'])
timing=read('impl/pnr/ov5640.tr');report=read('impl/pnr/ov5640.rpt.txt')
violations={kind:int(re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)',timing)[1]) for kind in ('Setup','Hold')}
slack={}
for kind in ('Setup','Hold','Recovery','Removal'):
 table=re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)',timing,re.M|re.S)
 values=[float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s',table[1],re.M)]
 assert values,kind
 slack[kind]=min(values)
assert not any(violations.values()) and min(slack.values())>=0,(violations,slack)
prior=json.loads(read('docs/host_navigation_20261027_validation.json'))
resources={}
for name in ('Logic','Register','CLS','BSRAM','DSP','I/O Port'):
 m=re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|',report,re.M)
 used,total=map(float,m.groups());assert used<=total,(name,used,total)
 resources[name]=dict(used=used,total=total,percent=round(100*used/total,2),delta_from_previous=used-prior['resources'][name]['used'])
pinpat=r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;'
pins=dict(re.findall(pinpat,read('src/ov5640_ddr3_tft.cst'),re.M))
assert pins==dict(re.findall(pinpat,read(BASE+'/src/ov5640_ddr3_tft.cst'),re.M))
assert pins['host_tx']=='E14' and pins['host_rx']=='E13'
placed=dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',report,re.M))
assert all(placed.get(n)==p for n,p in pins.items()),'PNR pin mismatch'
netlist=read('impl/gwsynthesis/ov5640.vg')
for module in ('vehicle_motion','vehicle_route_profile','vehicle_vision_fusion','vehicle_visual_localizer',
 'vehicle_bypass','vision_video','vision_zebra','lsm6dsv16x_spi','vl53l1x_array_controller','vehicle_host_link','host_uart_rx'):
 assert re.search(r'^module '+module+r'\b',netlist,re.M),module
roms=dict(map=audit_map(),font=audit_font(),road=audit_road())

plant=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/vehicle/vehicle_config.vh','src/imu/imu_math.v']
fusion=['src/vehicle/vehicle_vision_fusion.v','src/vehicle/vehicle_config.vh']
regressions={}
def check(name,log,count,deps,digest=None):
 content=read(log)
 assert content.count('$finish called')==count,(name,content.count('$finish called'))
 assert 'PASS ' in content and not re.search(r'^\s*(FATAL|FAIL|ERROR|Traceback)\b',content,re.M),name
 newer(log,deps)
 if digest:assert sha(log)==digest,name
 regressions[name]=dict(passed=True,completed_runs=count,log=log,sha256=sha(log))

corners=json.loads(read('sim/navigation_visual_corner_runs.json'));assert len(corners)==22
check('navigation_corners','sim/navigation_visual_corner_results.log',22,plant)
for c in corners:
 deps=fusion if c['top'] in ('navigation_corner_tb','navigation_response_tb','vision_fusion_tb','vision_blind_turn_tb') else plant
 newer(f'sim/navigation_visual_corner_{c["top"]}_{c["variant"]}.vvp',deps+[f'sim/{c["top"]}.sv'])
routes=json.loads(read('sim/navigation_visual_routes_runs.json'))
starts=json.loads(read('sim/navigation_visual_starts_runs.json'))
legacy=json.loads(read('sim/navigation_visual_route_runs.json'))
assert (len(routes),len(starts),len(legacy))==(9,3,25)
for c in routes+starts+legacy:
 assert c['passed'] and c['max_nose_error_mm']<=50,c['name']
 if c['route']=='reported':assert c['reported_right_turn_radians']<-.9
 for n,h in c.get('inputs',{}).items():assert sha(n)==h,(c['name'],n)
 check(c['name'],c['log'],1,plant+['sim/navigation_route_camera_tb.sv'],c['sha256'])
targeted=json.loads(read('sim/navigation_visual_targeted_runs.json'));assert len(targeted)==6
for c in targeted:
 deps=fusion if c['name']!='video' else fusion+['src/portrait/vision_video.v','src/portrait/vision_zebra.v','src/portrait/vision_camera_config.vh','src/portrait/vision_ipm_lut.vh','sim/vision_video_tb.sv']
 check('targeted_'+c['name'],c['log'],1,deps,c['sha256'])
host=json.loads(read('sim/navigation_visual_host_runs.json'));assert len(host)==4
for c in host:
 assert c['passed'] and sha(c['log'])==c['sha256']
 assert all(sha(n)==h for n,h in c['inputs'].items())
 if c['expected_failure']:assert c['exit_code']!=0 and 'NEXT_ENTRY_EARLY' in read(c['log'])
 else:check('host_'+c['name'],c['log'],1,list(c['inputs']),c['sha256'])
gates=json.loads(read('sim/navigation_visual_netlist_runs.json'));assert len(gates)==4
for c in gates:
 assert c['passed'] and all(sha(n)==h for n,h in c['inputs'].items())
 check('synthesized_'+c['name'],c['log'],1,list(c['inputs']),c['sha256'])
check('visual_registration_unit','sim/visual_localizer_tb.log',1,['src/vehicle/vehicle_visual_localizer.v','sim/visual_localizer_tb.sv'])
before_dir='impl/navigation_visual_20261027_attempt1'
startup_before=json.loads(read(before_dir+'/startup_before.json'))
before_inputs=json.loads(read(before_dir+'/navigation_visual_20261027_inputs.json'))['files']
assert sha(before_dir+'/vehicle_motion.v')==before_inputs['src/vehicle/vehicle_motion.v']
for c in startup_before:
 assert c['expected_failure'] and c['max_nose_error_mm']>150 and 'ACTUAL_ROUTE_CAMERA status=3' in read(c['log'])
 assert c['motion_sha256']==sha(before_dir+'/vehicle_motion.v')
interrupted=json.loads(read(before_dir+'/navigation_visual_20261027_build_metrics.json'))
assert interrupted['exit_code']!=0
preserved_names=['src/vehicle/vehicle_bypass.v','src/vehicle/vehicle_route_profile.v','src/vehicle/vehicle_config.vh',
 'src/vehicle/vehicle_odometry.v','src/vehicle/motor/servo_pwm.v','src/vehicle/motor/motor_pi.v',
 'src/vehicle/motor/vl53l1x_array_controller.v','src/imu/lsm6dsv16x_spi.v','src/portrait/vision_zebra.v',
 'src/vehicle/vehicle_host_link.v','src/vehicle/host_uart_rx.v']
assert all(actual[n]==baseline[n] for n in preserved_names)
result=dict(status='digital_verification_passed',recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
 bitstream=fs,sha256=sha(fs),bytes=(ROOT/fs).stat().st_size,
 generated_utc=datetime.datetime.fromtimestamp((ROOT/fs).stat().st_mtime,datetime.timezone.utc).isoformat(),
 tool='Gowin V1.9.12.03',device='GW5AT-LV60PG484AC1/I0 B',hardware_tested=False,flashed=False,
 synthesis_input_manifest=f'impl/{LABEL}_inputs.json',source_inputs_unchanged=True,input_count=len(actual),input_sha256=actual,
 changed_baseline_hardware_inputs=changed,unchanged_hardware_source_count=sum(n.startswith('src/') and baseline.get(n)==h for n,h in actual.items()),
 resources=resources,violated_endpoints=violations,min_slack_ns=slack,timing_passed=True,pins_verified=pins,roms=roms,
 regression_completed_runs=sum(c['completed_runs'] for c in regressions.values()),regressions=regressions,
 actual_visual_route_runs=routes,startup_pose_runs=starts,legacy_route_runs=legacy,
 build=dict(**metrics,log=f'impl/{LABEL}_build.log',log_sha256=sha(f'impl/{LABEL}_build.log'),preserved_other_module_placement=False,
  interrupted_attempt=dict(**interrupted,artifact_directory=before_dir,interruption_reason='Skewed startup tests exposed a suppressed visual heading alignment.'),
  total_attempt_seconds=round(metrics['seconds']+interrupted['seconds'],3)),
 startup_skew_before_after=dict(before=startup_before,after=[c for c in starts if c.get('yaw')]),
 rollback=dict(bitstream=BASE+'/'+fs,sha256=baseline[fs],clean_complete_snapshot=True),preserved={n:actual[n] for n in preserved_names},
 navigation=dict(controller_hz=100,entry_lead_mm=0,visual_longitudinal_update_mm_per_frame=10,
  visual_lateral_update_mm_per_frame=3,visual_fit_max_distance_mm=450,encoder_count_reset_on_observation=False,
  widened_junction_centroid_rejected=True,initial_90deg_branch_direction_guard=True,per_corner_recovery_watchdog=True,
  near_curve_startup_centered_guard_mm=16,
  unobservable_straight_longitudinal_position=True,servo_limits_unchanged=[40,150],wheel_limits_unchanged=[-28,28]),
 limitations=['Not flashed; no physical-car test.',
  'A featureless straight road cannot observe longitudinal startup displacement; a visible bend or another landmark is required.',
  'The old geometric camera model did not reproduce a complete wrong-left at the reported right junction; real scene branch ambiguity remains unproven.',
  'Route errors measure nose centre. Full body/wheel swept clearance is not an acceptance result; the source map is not physically scale-calibrated.',
  'Camera intrinsics remain provisional; synthetic images cannot cover every lighting and road scene.',
  'Unrelated source code is preserved; physical placement and routing are not frozen.',
  'Existing synthesis warnings remain.'])
release=f'impl/pnr/ov5640_{LABEL}.fs'
if not (ROOT/release).exists() or sha(release)!=result['sha256']:
 shutil.copyfile(ROOT/fs,ROOT/release)
assert sha(release)==result['sha256']
result['release_bitstream']=release
for n in (f'docs/{LABEL}_validation.json',f'impl/pnr/ov5640_{LABEL}_audit.json','impl/pnr/ov5640_current_audit.json'):dump(n,result)
dump('impl/pnr/ov5640_download.json',{k:result[k] for k in ('bitstream','release_bitstream','sha256','bytes','generated_utc','tool','device','timing_passed','hardware_tested','flashed')})
print(json.dumps({k:result[k] for k in ('status','sha256','regression_completed_runs','resources','violated_endpoints','min_slack_ns','build')},indent=2))
