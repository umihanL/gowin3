"""Audit the navigation-only change and its actual incremental build."""
from pathlib import Path
import datetime
import hashlib
import json
import re
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'impl/baseline_navigation_20261025'
def read(path):
    data=path.read_bytes()
    return data.decode('utf-16' if data.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig').replace('\r\n','\n')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def dump(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
baseline=json.loads(read(BASE/'manifest.json'))
changed=[n for n,h in baseline.items() if n.startswith('src/') and sha(ROOT/n)!=h]
assert set(changed)=={'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_vision_fusion.v'},changed
snapshot_path=ROOT/'impl/navigation_response_20261025_inputs.json'
snapshot=json.loads(read(snapshot_path))
inputs=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in
        ('.v','.vo','.vh','.mem','.cst','.sdc','.ipc','.mod')]
inputs += [ROOT/'ov5640.gprj',ROOT/'build.tcl']
actual={p.relative_to(ROOT).as_posix():sha(p) for p in inputs}
assert actual==snapshot['files'],'Frozen inputs changed'
fs=ROOT/'impl/pnr/ov5640.fs'
assert fs.stat().st_mtime>=max([snapshot_path.stat().st_mtime]+[p.stat().st_mtime for p in inputs])
report_path,timing_path=ROOT/'impl/pnr/ov5640.rpt.txt',ROOT/'impl/pnr/ov5640.tr'
assert min(report_path.stat().st_mtime,timing_path.stat().st_mtime)>=fs.stat().st_mtime
report,timing=read(report_path),read(timing_path)
build_path=ROOT/'impl/navigation_response_20261025_build.log'
build=read(build_path)
assert 'Bitstream generation completed' in build and not re.search(r'^\s*ERROR\b',build,re.M)
metrics=json.loads(read(ROOT/'impl/navigation_response_20261025_build_metrics.json'))
assert metrics['exit_code']==0
assert metrics['mode']=='standard_pnr_reusing_current_synthesis'
assert 'NAVIGATION_FALLBACK_REUSE_SYNTHESIS=1' in build
assert '-inc_pnr ' not in read(ROOT/'impl/pnr/cmd.do') or '-inc_pnr 0' in read(ROOT/'impl/pnr/cmd.do')
assert sha(ROOT/'impl/gwsynthesis/ov5640.vg')==metrics['synthesis_netlist_sha256']
assert sha(ROOT/'tools/route_navigation_20261025.tcl')==metrics['fallback_script_sha256']
attempt=metrics['incremental_attempt']
incremental_path=ROOT/'impl/navigation_response_20261025_incremental.log'
assert attempt['exit_code']!=0 and 'NAVIGATION_INCREMENTAL_CHECKPOINT=' in read(incremental_path)
checkpoint=ROOT/attempt['incremental_checkpoint']
assert sha(checkpoint)==attempt['checkpoint_sha256']==baseline['impl/pnr/ov5640.pr']
resources={}
for name in ('Logic','Register','CLS','BSRAM','DSP','I/O Port'):
    m=re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|',report,re.M)
    assert m,name
    used,total=map(float,m.groups());assert used<=total
    resources[name]=dict(used=used,total=total,percent=round(100*used/total,2))
violations,slack={},{}
for kind in ('Setup','Hold'):
    m=re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)',timing);assert m
    violations[kind]=int(m[1])
for kind in ('Setup','Hold','Recovery','Removal'):
    m=re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)',timing,re.M|re.S);assert m
    values=[float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s',m[1],re.M)];assert values
    slack[kind]=min(values)
assert not any(violations.values()) and min(slack.values())>=0,(violations,slack)
pins=dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',read(ROOT/'src/ov5640_ddr3_tft.cst'),re.M))
placed=dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',report,re.M))
assert pins and len(pins)==len(set(pins.values())) and all(placed.get(n)==p for n,p in pins.items())
assert str(ROOT/'impl/gwsynthesis/ov5640.vg') in report
for name in ('src/ov5640_ddr3_tft.cst','src/ov5640_ddr3_tft.sdc'):assert str(ROOT/name) in report
dependencies=[ROOT/n for n in changed]+[ROOT/'src/vehicle/vehicle_config.vh']
specs=[('navigation',14,'sim/navigation_response_20261025_results.txt',
        ['PASS skipped edge phase:','PASS visual bend memory:','PASS bend expiry:',
         'PASS stationary steering preload:','PASS terminal arrival:','PASS integrated limited-FOV turn side=-1:',
         'PASS integrated limited-FOV turn side=1:'],
        [ROOT/f'sim/{n}.sv' for n in ('navigation_response_tb','vision_fusion_tb','vision_blind_turn_tb',
         'vision_turn_motion_tb','vision_motion_tb','vision_goal_tb')]+[ROOT/'sim/run_navigation_response_tests.py']),
       ('video',1,'sim/navigation_video_20261025.log',['PASS video-to-fusion sequence:','PASS dense contours:'],
        [ROOT/'sim/vision_video_tb.sv',ROOT/'src/portrait/vision_video.v']),
       ('vehicle',1,'sim/navigation_vehicle_20261025.log',['PASS smooth low-speed stop:',
        'PASS navigation dynamic obstacle:','PASS committed bypass:'],[ROOT/'sim/vehicle_motion_tb.sv']),
       ('navigation_netlist',1,'sim/navigation_response_netlist_20261025.log',
        ['PASS skipped edge phase:','PASS visual bend memory:','PASS bend expiry:'],
        [ROOT/'sim/navigation_response_tb.sv',ROOT/'sim/run_navigation_response_netlist.py'])]
regressions={}
for name,count,filename,markers,extra in specs:
    path=ROOT/filename;content=read(path)
    assert content.count('$finish called')==count,(name,content.count('$finish called'))
    assert not re.search(r'^(FATAL|FAIL|ERROR|Traceback)\b',content,re.M),name
    assert all(s in content for s in markers),(name,[s for s in markers if s not in content])
    assert path.stat().st_mtime>=max(p.stat().st_mtime for p in dependencies+extra),('Stale regression',name)
    regressions[name]=dict(passed=True,completed_runs=count,log=filename,sha256=sha(path))
runs=json.loads(read(ROOT/'sim/navigation_response_20261025_runs.json'));assert len(runs)==14
reproductions=json.loads(read(ROOT/'sim/navigation_20261025_reproductions.json'));assert len(reproductions)==2
for case in reproductions:
    assert case['exit_code']!=0 and case['expected_failure'] in read(ROOT/case['log'])
netlist=read(ROOT/'impl/gwsynthesis/ov5640.vg')
for module in ('vehicle_motion','vehicle_vision_fusion','vehicle_bypass','vision_video',
               'lsm6dsv16x_spi','vl53l1x_array_controller'):
    assert re.search(r'^module '+module+r'\b',netlist,re.M),module
roms=dict(map=audit_map(),font=audit_font(),road=audit_road())
old_download=json.loads(read(BASE/'impl/pnr/ov5640_download.json'))
assert sha(BASE/'impl/pnr/ov5640.fs')==old_download['sha256']
baseline_seconds=None
old_metrics=ROOT/'impl/navigation_response_20261025_build_comparison.json'
if old_metrics.exists():baseline_seconds=json.loads(read(old_metrics)).get('previous_full_seconds')
result=dict(status='digital_verification_passed',recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    bitstream='impl/pnr/ov5640.fs',sha256=sha(fs),bytes=fs.stat().st_size,
    generated_utc=datetime.datetime.fromtimestamp(fs.stat().st_mtime,datetime.timezone.utc).isoformat(),
    tool='Gowin V1.9.12.03',device='GW5AT-LV60PG484AC1/I0 B',hardware_tested=False,flashed=False,
    synthesis_input_manifest=snapshot_path.relative_to(ROOT).as_posix(),input_count=len(actual),
    source_inputs_unchanged=True,input_sha256=actual,changed_baseline_rtl=changed,
    unchanged_hardware_source_count=sum(n.startswith('src/') for n in baseline)-len(changed),
    bypass_source_unchanged=True,non_navigation_hardware_unchanged=True,
    resources=resources,violated_endpoints=violations,min_slack_ns=slack,timing_passed=True,
    pins_verified=pins,roms=roms,regressions=regressions,regression_runs=runs,
    old_source_failure_reproductions=reproductions,
    build=dict(**metrics,log=build_path.relative_to(ROOT).as_posix(),log_sha256=sha(build_path),
               incremental_attempted_but_slow=True,incremental_used_for_final_bitstream=False,
               incremental_log=incremental_path.relative_to(ROOT).as_posix(),
               incremental_log_sha256=sha(incremental_path),previous_full_seconds=baseline_seconds),
    rollback=dict(bitstream='impl/baseline_navigation_20261025/impl/pnr/ov5640.fs',sha256=old_download['sha256']),
    navigation=dict(stationary_visual_steering=True,stable_command_wait_ms=120,
       direct_complete_loss_can_commit_with_recent_direction=True,bend_memory_ms=1200,
       complete_empty_frames_required=2,terminal_suppression_preserved=True,
       pd_gains_unchanged=True,camera_geometry_unchanged=True),
    limitations=['Not flashed or tested on the real car.',
      'Tests use generated images and independent vehicle dynamics, not calibrated user photographs.',
      'No inverse-perspective change or unmeasured 35-degree installation was applied.',
      'Existing map turn-feasibility and full-chassis boundary limitations are unchanged.'])
dump(ROOT/'docs/navigation_response_20261025_validation.json',result)
dump(ROOT/'impl/pnr/ov5640_navigation_response_audit.json',result)
dump(ROOT/'impl/pnr/ov5640_current_audit.json',result)
dump(ROOT/'impl/pnr/ov5640_download.json',{k:result[k] for k in ('bitstream','sha256','bytes',
     'generated_utc','tool','device','timing_passed','hardware_tested','flashed')})
print(json.dumps({k:result[k] for k in ('status','sha256','generated_utc','resources','violated_endpoints',
                                      'min_slack_ns','build')},indent=2))
