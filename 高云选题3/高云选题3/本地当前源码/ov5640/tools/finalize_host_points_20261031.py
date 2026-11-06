"""Bind numbered-point protocol, UI and pure inertial replay evidence to this bitstream."""
from pathlib import Path
import hashlib,json
R=Path(__file__).resolve().parents[1]
LABEL='host_points_20261031'
sha=lambda n:hashlib.sha256((R/n).read_bytes()).hexdigest()
read=lambda n:json.loads((R/n).read_text(encoding='utf-8'))
def read_log(n):
 b=(R/n).read_bytes()
 return b.decode('utf-16' if b.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig')
v=read(f'docs/{LABEL}_validation.json')
assert v['timing_passed'] and not v['flashed'] and v['sha256']==sha(v['bitstream'])==sha(v['release_bitstream'])
assert all(sha(n)==h for n,h in v['hardware_inputs'].items())
previous=read(f'impl/{LABEL}_previous_inputs.json')
for n in ['src/vehicle/vehicle_motion.v','src/vehicle/vehicle_tracker.v','src/vehicle/vehicle_odometry.v',
 'src/vehicle/vehicle_config.vh','src/vehicle/servo_offset.mem','src/vehicle/rear_ratio.mem',
 'src/navigation/nav_road_planner.v','src/navigation/nav_centerline_init.vh','src/navigation/nav_centerline_reverse.vh',
 'src/portrait/vision_zebra.v','src/portrait/vision_video.v','src/navigation/nav_font.vh','src/navigation/nav_font_rows.mem']:
 assert sha(n)==previous[n],n
points=read(f'sim/{LABEL}_runs.json');motion=read('sim/host_points_motion_runs.json')
assert len(points['cases'])==len(motion['cases'])==9
for evidence in (points,motion):
 assert all(sha(n)==h for n,h in evidence['inputs'].items())
 for c in evidence['cases']:assert c['passed'] and sha(c['log'])==c['log_sha256']
for c in motion['cases']:assert c['metrics']['status']==3
logs=[]
for n in ['sim/host_points_fault.log','sim/host_points_ui_cancel.log','sim/diguapi_navigation_test.log',
 'sim/host_points_python_client.log','sim/host_points_settings.log',
 *[f'sim/inertial_zebra_{name}.log' for name in ('modes','avoidance','zebra','camera','uart','host')]]:
 text=read_log(n);assert 'PASS ' in text and 'FATAL:' not in text,n
 logs.append(dict(path=n,sha256=sha(n),results=[l for l in text.splitlines() if 'PASS ' in l]))
images=[dict(path=n,sha256=sha(n)) for n in ['docs/host_points_map.png','docs/host_points_map_preview.png']]
v['simulation_evidence']=dict(numbered_points=points,motion=motion,logs=logs,images=images,graph_checks=read('sim/host_points_graph_checks.json'),destination_table=read('docs/host_destinations_20261031.json'))
v['software_inputs']={n:sha(n) for n in ['tools/host_serial_client.py','tools/diguapi_navigation.py','sim/diguapi_navigation_test.py','sim/host_serial_client_test.py']}
v['known_limitations']=['Not flashed or tested on the physical vehicle.',
 'DiGuaPi AI application source was not present; serial API and callable adapter are provided but not deployed into that application.',
 'Point coordinates use existing map geometry and 2 cm user grid; graph sampling, arrival tolerance and inertial drift still apply.',
 'Encoder/IMU movement replays use an ideal bicycle model; they do not establish real vehicle centimetre accuracy.',
 'Finished navigation sessions require leaving/re-entering the run page before a new task, as in the existing controller.',
 'A new task after movement still requires the user to set the actual start pose; fixed goals do not provide absolute localization.']
for n in [f'docs/{LABEL}_validation.json',f'impl/pnr/ov5640_{LABEL}_audit.json','impl/pnr/ov5640_current_audit.json']:
 (R/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
resources=v['resources'];prior=read('diagnostics/host_points_20261031/baseline/impl/pnr/ov5640_current_audit.json')['resources']
report=['# FPGA \u8d44\u6e90\u62a5\u544a - \u7f16\u53f7\u70b9\u4f4d\u5bfc\u822a','','Release: '+LABEL,
 '\u5668\u4ef6: GW5AT-LV60PG484AC1/I0 B\u3002\u5de5\u5177: Gowin V1.9.12.03\u3002\u5c1a\u672a\u70e7\u5f55\u6216\u5b9e\u8f66\u6d4b\u8bd5\u3002','',
 '| \u8d44\u6e90 | \u5df2\u7528 / \u603b\u91cf | \u4f7f\u7528\u7387 | \u5269\u4f59 | \u6bd4\u4e0a\u7248\u589e\u52a0 |',
 '| --- | ---: | ---: | ---: | ---: |']
fmt=lambda x:f'{x:,.1f}'.rstrip('0').rstrip('.')
for n,d in resources.items():
 report.append(f"| {n} | {fmt(d['used'])} / {fmt(d['total'])} | {d['percent']:.2f}% | {fmt(d['remaining'])} | {d['used']-prior[n]['used']:+g} |")
report+=['',f"Setup/Hold violations: {v['timing_violations']}. Minimum slack (ns): {v['min_slack_ns']}.",
 f"Build elapsed: {v['build_metrics']['seconds']:.1f} s.",f"SHA256: {v['sha256']}",
 '', 'Bitstream: impl/pnr/ov5640_host_points_20261031.fs',
 'Nine real UART/UI/planner/profile/start cases and nine IMU/encoder route movement replays passed.',
 'Map/font/road/servo ROM content and all pin locations were audited. Existing motion/tracker/odometry/zebra/servo mechanics remain byte-identical.',
 '', 'See host_points_20261031.md and host_points_20261031_validation.json for operation and evidence.']
(R/'docs/resource_usage.md').write_text('\n'.join(report)+'\n',encoding='utf-8')
# Preserve the historical README after a concise current-release banner.
p=R/'README.md';text=p.read_text(encoding='utf-8')
marker='## Current release: host_points_20261031'
if marker not in text:
 p.write_text(marker+'\n\nNumbered destinations 1..9, UART-select/auto-plan, explicit START and automatic map-to-navigation page switch.\n'+
 'Pure IMU + encoder navigation, zebra slowdown and adjustable servo centre are preserved.\n'+
 'See [operation guide](docs/host_points_20261031.md), [map](docs/host_points_map.png), '+
 '[resource report](docs/resource_usage.md) and [build/test evidence](docs/host_points_20261031_validation.json).\n'+
 'Bitstream: impl/pnr/ov5640_host_points_20261031.fs. Not flashed or hardware tested.\n\n'+text,encoding='utf-8')
print(json.dumps(dict(release=LABEL,resources=resources,minimum_slack_ns=v['min_slack_ns'],numbered_point_tests=9,movement_replays=9),indent=2))
