"""Final bitstream, scope, timing, ROM and regression audit for this fix."""
from pathlib import Path
import datetime,hashlib,json,re,shutil
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road
ROOT=Path(__file__).resolve().parents[1];LABEL='navigation_startfix_20261027'
BASE='impl/baseline_navigation_startfix_20261027'
def read(n):
 b=(ROOT/n).read_bytes()
 return b.decode('utf-16' if b.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig').replace('\r\n','\n')
def sha(n):return hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
def dump(n,v):(ROOT/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
frozen=json.loads(read(f'impl/{LABEL}_inputs.json'))['files']
assert all(sha(n)==h for n,h in frozen.items()),'Sources differ from build inputs'
baseline=json.loads(read(BASE+'/manifest.json'))
assert all(sha(BASE+'/'+n)==h for n,h in baseline.items()),'Baseline snapshot changed'
changed=sorted(n for n,h in frozen.items() if baseline.get(n)!=h)
assert changed==['src/vehicle/vehicle_motion.v','src/vehicle/vehicle_vision_fusion.v'],changed
metrics=json.loads(read(f'impl/{LABEL}_build_metrics.json'));assert metrics['exit_code']==0
log=read(f'impl/{LABEL}_build.log')
assert 'Bitstream generation completed' in log and 'Placement and routing completed' in log
assert not re.search(r'^\s*ERROR\b',log,re.M)
fs='impl/pnr/ov5640.fs';assert sha(fs)!=baseline[fs]
for n in (fs,'impl/gwsynthesis/ov5640.vg','impl/pnr/ov5640.rpt.txt','impl/pnr/ov5640.tr'):
 assert (ROOT/n).stat().st_mtime>=max((ROOT/d).stat().st_mtime for d in [*frozen,f'impl/{LABEL}_inputs.json']),('stale',n)
timing=read('impl/pnr/ov5640.tr');report=read('impl/pnr/ov5640.rpt.txt')
violations={kind:int(re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)',timing)[1]) for kind in ('Setup','Hold')}
slack={}
for kind in ('Setup','Hold','Recovery','Removal'):
 table=re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)',timing,re.M|re.S)
 values=[float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s',table[1],re.M)]
 assert values,kind
 slack[kind]=min(values)
assert not any(violations.values()) and min(slack.values())>=0,(violations,slack)
resources={}
for name in ('Logic','Register','CLS','BSRAM','DSP','I/O Port'):
 m=re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|',report,re.M)
 used,total=map(float,m.groups());assert used<=total,(name,used,total)
 resources[name]=dict(used=used,total=total,percent=round(100*used/total,2))
pinpat=r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;'
pins=dict(re.findall(pinpat,read('src/ov5640_ddr3_tft.cst'),re.M))
assert pins==dict(re.findall(pinpat,read(BASE+'/src/ov5640_ddr3_tft.cst'),re.M))
placed=dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',report,re.M))
assert all(placed.get(n)==p for n,p in pins.items()),'PNR pin mismatch'
roms=dict(map=audit_map(),font=audit_font(),road=audit_road())
regressions={}
for suite,count in [('focus',8),('netlist',4),('motion_netlist',1),('routes',8)]:
 runs=json.loads(read(f'sim/navigation_startfix_{suite}_runs.json'));assert len(runs)==count
 for c in runs:
  assert c['passed'],c
  assert all(sha(n)==h for n,h in c['inputs'].items()),('stale test inputs',c.get('name',c.get('top')))
  if 'sha256' in c:assert sha(c['log'])==c['sha256']
  assert 'PASS ' in read(c['log']),c['log']
 regressions[suite]=runs
before=json.loads(read('sim/navigation_startfix_before_runs.json'));assert len(before)==2
for c in before:
 assert c['expected_failure'] and c['message'] in read(c['log'])
 assert all(sha(n)==h for n,h in c['inputs'].items()) and sha(c['log'])==c['sha256']
previous_route=json.loads(read('sim/navigation_startfix_reported_previous.json'))
assert previous_route['exit_code']!=0 and 'ACTUAL_ROUTE_CAMERA status=3' in read(previous_route['log'])
assert all(sha(n)==h for n,h in previous_route['inputs'].items())
assert sha(previous_route['log'])==previous_route['sha256']
host=json.loads(read('sim/navigation_visual_host_runs.json'));assert len(host)==4
for c in host:
 assert c['passed'] and all(sha(n)==h for n,h in c['inputs'].items())
 assert sha(c['log'])==c['sha256']
 if c['expected_failure']:assert c['exit_code']!=0 and 'NEXT_ENTRY_EARLY' in read(c['log'])
 else:assert c['exit_code']==0 and 'PASS ' in read(c['log'])
attempt='impl/navigation_startfix_20261027_attempt1'
interrupted=json.loads(read(attempt+'/impl/navigation_startfix_20261027_build_metrics.json'))
assert interrupted['exit_code']!=0
assert 'START_PRELOAD_LOST' in read(attempt+'/preload_failure.log')
result=dict(status='digital_verification_passed',recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
 bitstream=fs,sha256=sha(fs),bytes=(ROOT/fs).stat().st_size,
 generated_utc=datetime.datetime.fromtimestamp((ROOT/fs).stat().st_mtime,datetime.timezone.utc).isoformat(),
 tool='Gowin V1.9.12.03',device='GW5AT-LV60PG484AC1/I0 B',hardware_tested=False,flashed=False,
 timing_passed=True,resources=resources,violated_endpoints=violations,min_slack_ns=slack,
 source_inputs_unchanged=True,input_sha256=frozen,changed_hardware_files=changed,
 build=dict(**metrics,interrupted_attempt=interrupted,total_attempt_seconds=metrics['seconds']+interrupted['seconds']),pins_verified=pins,roms=roms,regressions=regressions,
 previous_release_faults=before,previous_release_full_route=previous_route,host_regressions=host,
 rollback_bitstream=BASE+'/'+fs,rollback_sha256=baseline[fs],
 limitations=['Not flashed; physical car validation remains required.',
 'Synthetic camera geometry cannot reproduce every real junction scene.',
 'The full reported route still exceeds the original 50mm nose-centre guard in an S bend, although it improves on the previous release; full-body road clearance is not accepted.',
 'Placement/routing of unrelated modules is not frozen; their source is unchanged.'])
release=f'impl/pnr/ov5640_{LABEL}.fs';shutil.copyfile(ROOT/fs,ROOT/release)
assert sha(release)==result['sha256'];result['release_bitstream']=release
for n in (f'docs/{LABEL}_validation.json',f'impl/pnr/ov5640_{LABEL}_audit.json','impl/pnr/ov5640_current_audit.json'):dump(n,result)
dump('impl/pnr/ov5640_download.json',{k:result[k] for k in ('bitstream','release_bitstream','sha256','bytes','generated_utc','tool','device','timing_passed','hardware_tested','flashed')})
print(json.dumps({k:result[k] for k in ('status','sha256','resources','violated_endpoints','min_slack_ns')},indent=2))
