"""Reject stale hardware, audit PNR/timing/pins/ROMs and record this release."""
from pathlib import Path
import datetime,hashlib,json,re
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road

ROOT=Path(__file__).resolve().parents[1]
LABEL='host_navigation_20261027'
BASE='impl/baseline_host_navigation_20261027'
ALLOWED={
 'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_vision_fusion.v','src/vehicle/vehicle_subsystem.v',
 'src/vehicle/vehicle_host_link.v','src/vehicle/host_uart_rx.v',
 'src/navigation/navigation_ui.v','src/navigation/nav_renderer.v','src/navigation/vehicle_settings.v',
 'src/navigation/nav_font.vh','src/navigation/nav_font_rows.mem',
 'src/ov5640_ddr3_tft.v','src/ov5640_ddr3_tft.cst','src/ov5640_ddr3_tft.sdc','ov5640.gprj'}

def read(name):
 data=(ROOT/name).read_bytes()
 return data.decode('utf-16' if data.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig').replace('\r\n','\n')
def sha(name):return hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
def dump(name,data):
 (ROOT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def newer(name,inputs):
 assert (ROOT/name).stat().st_mtime>=max((ROOT/n).stat().st_mtime for n in inputs),('stale',name)

snapshot=f'impl/{LABEL}_inputs.json'
frozen=json.loads(read(snapshot))['files']
hardware=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in ('.v','.vo','.vh','.mem','.cst','.sdc','.ipc','.mod')]
hardware += [ROOT/'ov5640.gprj',ROOT/'build.tcl']
actual={p.relative_to(ROOT).as_posix():sha(p) for p in hardware}
assert frozen==actual,'Hardware inputs changed after build freeze'
manifest=json.loads(read(BASE+'/manifest.json'))
assert all(sha(BASE+'/'+n)==h for n,h in manifest.items()),'Baseline snapshot corrupted'
changed=sorted(n for n,h in actual.items() if manifest.get(n)!=h)
assert set(changed)==ALLOWED,changed
metrics=json.loads(read(f'impl/{LABEL}_build_metrics.json'))
assert metrics['exit_code']==0,metrics
build_name=f'impl/{LABEL}_build.log';build=read(build_name)
assert 'Bitstream generation completed' in build and 'Placement and routing completed' in build
assert not re.search(r'^\s*ERROR\b',build,re.M)
fs='impl/pnr/ov5640.fs'
for output in (fs,'impl/gwsynthesis/ov5640.vg','impl/pnr/ov5640.rpt.txt','impl/pnr/ov5640.tr'):
 newer(output,list(actual)+[snapshot])
assert sha(fs)!=manifest[fs],'Delivered bitstream equals previous release'

timing=read('impl/pnr/ov5640.tr');report=read('impl/pnr/ov5640.rpt.txt')
violations={};slack={}
for kind in ('Setup','Hold'):
 match=re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)',timing)
 assert match,kind
 violations[kind]=int(match[1])
for kind in ('Setup','Hold','Recovery','Removal'):
 table=re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)',timing,re.M|re.S)
 assert table,kind
 values=[float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s',table[1],re.M)]
 assert values,kind
 slack[kind]=min(values)
assert not any(violations.values()) and min(slack.values())>=0,(violations,slack)
prior=json.loads(read('docs/navigation_followup_20261026_validation.json'))
resources={}
for name in ('Logic','Register','CLS','BSRAM','DSP','I/O Port'):
 match=re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|',report,re.M)
 assert match,name
 used,total=map(float,match.groups());assert used<=total
 resources[name]=dict(used=used,total=total,percent=round(100*used/total,2),delta_from_previous=used-prior['resources'][name]['used'])
pin_pattern=r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;'
pins=dict(re.findall(pin_pattern,read('src/ov5640_ddr3_tft.cst'),re.M))
old_pins=dict(re.findall(pin_pattern,read(BASE+'/src/ov5640_ddr3_tft.cst'),re.M))
assert pins['host_tx']=='E14' and pins['host_rx']=='E13'
assert {n:p for n,p in pins.items() if n not in ('host_rx','host_tx')}==old_pins
assert len(pins)==len(set(pins.values()))
placed=dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',report,re.M))
assert all(placed.get(n)==p for n,p in pins.items()),'Actual package pins differ from CST'
netlist=read('impl/gwsynthesis/ov5640.vg')
for module in ('vehicle_motion','vehicle_route_profile','vehicle_vision_fusion','vehicle_bypass',
               'vision_video','lsm6dsv16x_spi','vl53l1x_array_controller','vehicle_host_link','host_uart_rx'):
 assert re.search(r'^module '+module+r'\b',netlist,re.M),module
roms=dict(map=audit_map(),font=audit_font(),road=audit_road())

plant=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').rglob('*.v')]+['src/vehicle/vehicle_config.vh','src/imu/imu_math.v']
fusion=['src/vehicle/vehicle_vision_fusion.v','src/vehicle/vehicle_config.vh']
regressions={}
def check(name,log,count,deps,digest=None):
 content=read(log)
 assert content.count('$finish called')==count,(name,content.count('$finish called'))
 assert 'PASS ' in content and not re.search(r'^\s*(FATAL|FAIL|ERROR|Traceback)\b',content,re.M),name
 newer(log,deps)
 if digest:assert sha(log)==digest,(name,'log hash mismatch')
 regressions[name]=dict(passed=True,completed_runs=count,log=log,sha256=sha(log))

corners=json.loads(read('sim/host_navigation_corner_runs.json'));assert len(corners)==22
check('corners','sim/host_navigation_corner_results.log',22,plant)
for c in corners:
 deps=fusion if c['top'] in ('navigation_corner_tb','navigation_response_tb','vision_fusion_tb','vision_blind_turn_tb') else plant
 newer(f'sim/host_navigation_corner_{c["top"]}_{c["variant"]}.vvp',deps+[f'sim/{c["top"]}.sv'])
routes=json.loads(read('sim/host_navigation_route_runs.json'))
phases=json.loads(read('sim/host_navigation_phase_runs.json'))
boxes=json.loads(read('sim/host_navigation_box_runs.json'))
assert (len(routes),len(phases),len(boxes))==(25,8,12)
for c in routes+phases+boxes:
 assert c['passed'] and c['max_nose_error_mm']<=50,c['name']
 if c.get('handback'):assert c['handbacks']>0
 check(c['name'],c['log'],1,plant+['sim/navigation_route_camera_tb.sv'],c['sha256'])
targeted=json.loads(read('sim/host_navigation_targeted_runs.json'))
extras=json.loads(read('sim/host_navigation_extras_runs.json'))
assert (len(targeted),len(extras))==(6,3)
for c in targeted+extras:
 name=c['name'];assert c['passed']
 deps=plant if name=='vehicle' else (['src/vehicle/vehicle_route_profile.v','src/imu/imu_math.v'] if name=='profile' else
      ['src/portrait/vision_zebra.v'] if name=='zebra' else fusion+['src/portrait/vision_video.v','src/portrait/vision_zebra.v','src/portrait/vision_camera_config.vh','src/portrait/vision_ipm_lut.vh'] if name=='video' else fusion)
 check('targeted_'+name,c['log'],1,deps,c['sha256'])
host=json.loads(read('sim/host_link_runs.json'));assert len(host)==4
for c in host:
 assert c['passed'] and sha(c['log'])==c['sha256']
 assert all(sha(n)==h for n,h in c['inputs'].items()),('test source changed',c['name'])
 if c['expected_failure']:
  assert c['exit_code']!=0 and 'NEXT_ENTRY_EARLY' in read(c['log'])
 else:
  assert c['exit_code']==0
  check('host_'+c['name'],c['log'],1,list(c['inputs']),c['sha256'])
gates=json.loads(read('sim/host_navigation_netlist_runs.json'));assert len(gates)==3
for c in gates:
 assert c['passed'] and all(sha(n)==h for n,h in c['inputs'].items())
 check('synthesized_'+c['name'],c['log'],1,list(c['inputs']),c['sha256'])
physical=json.loads(read('sim/host_physical_entry_runs.json'));assert len(physical)==2
for c in physical:
 assert c['passed'] and c['early_guard_mm']==2
 check('physical_entry_'+str(c['side']),c['log'],1,plant+['sim/host_physical_entry_tb.sv'],c['sha256'])
ui=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/navigation').glob('*.v')]
check('settings_and_motor_response','sim/settings_test_results.txt',2,ui+['sim/vehicle_settings_tb.sv','src/vehicle/motor/motor_pi.v'])
client_log='sim/host_serial_client_test.log'
assert 'PASS host Python client:' in read(client_log) and 'Traceback' not in read(client_log)
newer(client_log,['tools/host_serial_client.py','sim/host_serial_client_test.py'])
regressions['python_client']=dict(passed=True,completed_runs=1,log=client_log,sha256=sha(client_log))

generated=datetime.datetime.fromtimestamp((ROOT/fs).stat().st_mtime,datetime.timezone.utc).isoformat()
preserved_names=['src/vehicle/vehicle_bypass.v','src/vehicle/vehicle_route_profile.v','src/vehicle/vehicle_config.vh',
 'src/vehicle/motor/servo_pwm.v','src/vehicle/motor/motor_pi.v','src/vehicle/motor/motor_closed_loop.v',
 'src/vehicle/motor/vl53l1x_array_controller.v','src/imu/lsm6dsv16x_spi.v','src/portrait/vision_video.v','src/portrait/vision_zebra.v']
assert all(actual[n]==manifest[n] for n in preserved_names)
result=dict(status='digital_verification_passed',recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
 bitstream=fs,sha256=sha(fs),bytes=(ROOT/fs).stat().st_size,generated_utc=generated,
 tool='Gowin V1.9.12.03',device='GW5AT-LV60PG484AC1/I0 B',hardware_tested=False,flashed=False,
 synthesis_input_manifest=snapshot,input_count=len(actual),input_sha256=actual,source_inputs_unchanged=True,
 changed_baseline_hardware_inputs=changed,unchanged_hardware_source_count=sum(n.startswith('src/') and manifest.get(n)==h for n,h in actual.items()),
 resources=resources,violated_endpoints=violations,min_slack_ns=slack,timing_passed=True,pins_verified=pins,roms=roms,
 regression_completed_runs=sum(c['completed_runs'] for c in regressions.values()),regressions=regressions,
 actual_route_runs=routes,camera_phase_runs=phases,boxed_junction_runs=boxes,reported_fault_before_after=host,
 build=dict(**metrics,log=build_name,log_sha256=sha(build_name),preserved_other_module_placement=False),
 rollback=dict(bitstream=BASE+'/'+fs,sha256=manifest[fs],clean_complete_snapshot=True),
 navigation=dict(controller_hz=100,entry_lead_mm=0,physical_entry_position_latched=True,
                 next_entry_during_recovery_guard=True,stationary_curve_preload_guard=True,
                 near_curve_startup_pose_rebase_suppressed_mm=240,servo_limits_unchanged=[40,150],wheel_limits_unchanged=[-28,28]),
 communication=dict(tx_pin='E14',rx_pin='E13',baud=115200,format='8N1',watchdog_ms=2000,
                    commands=['heartbeat','speed','start','slowdown','stop','status','resume'],
                    navigation_pause_preserves_progress=True,navigation_completion_event='F0',generic_completion_hook_connected=False),
 preserved={n:actual[n] for n in preserved_names},
 limitations=['Not flashed; no physical-car test.',
              'Synthetic camera routes cannot cover every real scene; camera intrinsics remain provisional.',
              'Simulated geometric errors refer to nose centre, not full body swept clearance.',
              'Sources of unrelated functions preserved; physical placement and routes were not frozen.',
              'Protocol framework has one queued event and no authentication, retry or deduplication.',
              'Generic future business completion hook is not connected to a task source.',
              'Existing synthesis warnings remain.'])
assert result['regression_completed_runs']==87,result['regression_completed_runs']
for name in (f'docs/{LABEL}_validation.json','impl/pnr/ov5640_host_navigation_audit.json','impl/pnr/ov5640_current_audit.json'):
 dump(name,result)
dump('impl/pnr/ov5640_download.json',{k:result[k] for k in ('bitstream','sha256','bytes','generated_utc','tool','device','timing_passed','hardware_tested','flashed')})
print(json.dumps({k:result[k] for k in ('status','sha256','regression_completed_runs','resources','violated_endpoints','min_slack_ns','build')},indent=2))
