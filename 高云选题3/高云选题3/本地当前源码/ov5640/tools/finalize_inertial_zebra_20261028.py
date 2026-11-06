"""Publish and cross-check the fully routed inertial/zebra-only release."""
from pathlib import Path
import datetime, hashlib, json, subprocess, sys

R=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(R))
from tools.verify_font_netlist import audit as audit_font
from tools.verify_map_netlist import audit as audit_map
from tools.verify_road_netlist import audit as audit_road
subprocess.run([sys.executable,str(R/'tools/build_inertial_zebra_20261028.py'),'--audit-existing'],cwd=R,check=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads((R/p).read_text(encoding='utf-8'))
controls=read('sim/inertial_zebra_runs.json')
routes=read('sim/inertial_zebra_motion_runs.json')
vehicle=read('sim/inertial_zebra_vehicle_run.json')
roms={'font':audit_font(),'map':audit_map(),'road':audit_road()}
required={'modes','avoidance','zebra','camera','settings','uart','host'}
assert {r['name'] for r in controls}==required and all(r['passed'] for r in controls)
assert len(routes)==5 and all(r['passed'] for r in routes)
assert {r['name'] for r in routes}=={'outer_left','outer_right','s_forward','photo','outer_left_zebra'}
assert all(r['identical_to_previous_release'] for r in routes if not r['name'].endswith('_zebra'))
assert vehicle['passed'] and 'PASS navigation dynamic obstacle:' in (R/vehicle['log']).read_text(encoding='utf-8')

audit=read('impl/pnr/ov5640_current_audit.json')
audit.update(
    status='place_route_timing_and_digital_regressions_passed',
    camera_intrinsics_calibrated=False,
    zebra_speed_control={'scope':'autonomous route mode only','rule':'half normal target, capped at 60 mm/s',
                         'deceleration_mm_s_per_10ms':4,'clear':'restore the prior route speed target on the next complete clear frame',
                         'incomplete_frame':'hold the last valid detection'},
    visual_navigation={'steering':False,'pose_correction':False,'ipm':False,'lane_extraction':False},
    verified_behavior={'pure_inertial_navigation':True,'camera_has_no_route_or_steering_authority':True,
                       'zebra_detection_slows_navigation':True,'clear_frame_restores_prior_target':True,
                       'pure_route_traces_identical_to_previous_release':True,
                       'route_traces':[{'name':r['name'],'metrics':r.get('metrics'),
                                        'identical_to_previous_release':r.get('identical_to_previous_release',False),
                                        'trace_sha256':r['trace_sha256']} for r in routes]},
    digital_regressions=[{'name':r['name'],'passed':r['passed'],'seconds':r['seconds'],
                          'log':r['log'],'log_sha256':r['log_sha256']} for r in controls],
    rom_audits=roms,
    vehicle_regression=vehicle,
    limitations=['Not flashed or physically tested.',
                 'A clear crosswalk release requires a complete valid camera frame.',
                 'Real-world recognition distance and braking distance require vehicle testing.'])
for name in [f'impl/pnr/ov5640_{audit["release"]}_audit.json','impl/pnr/ov5640_current_audit.json',
             f'docs/{audit["release"]}_validation.json']:
    (R/name).write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

res=audit['resources'];slack=audit['min_slack_ns'];viol=audit['timing_violations']
lines=['# 当前布局布线资源','',f'版本：`{audit["release"]}`。工具：{audit["tool"]}；器件：{audit["device"]}。',
       '',f'位流生成时间：{audit["generated_utc"]}。下载：`{audit["release_bitstream"]}` 或默认 `impl/pnr/ov5640.fs`。','',
       '| 资源 | 已用 / 总量 | 使用率 | 剩余 |','| --- | ---: | ---: | ---: |']
for name,label in [('Logic','Logic'),('Register','Register'),('CLS','CLS'),('BSRAM','BSRAM'),('DSP','DSP'),('I/O Port','I/O Port')]:
    r=res[name];lines.append(f'| {label} | {int(r["used"])} / {int(r["total"])} | {r["percent"]:.2f}% | {int(r["remaining"])} |')
lines += ['',f'Setup/Hold 违规端点：{viol}；最小时序裕量（ns）：{slack}。',
          '',f'SHA-256：`{audit["sha256"]}`。具名位流和默认位流内容一致；未烧录。',
          '',f'[本轮修改与验证](inertial_zebra_20261028.md)。']
(R/'docs/resource_usage.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')

readme=(R/'README.md').read_text(encoding='utf-8')
assert 'inertial_zebra_20261028' in readme
print(json.dumps({'release':audit['release'],'sha256':audit['sha256'],'resources':res,
                  'timing_violations':viol,'min_slack_ns':slack,
                  'route_traces_identical':True,'generated':datetime.datetime.now(datetime.timezone.utc).isoformat()},
                 ensure_ascii=False,indent=2),flush=True)
