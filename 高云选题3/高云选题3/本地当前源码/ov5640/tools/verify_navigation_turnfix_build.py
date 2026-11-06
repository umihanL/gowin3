"""Audit fresh bitstream, unchanged peripherals, actual pins, ROMs and tests."""
from pathlib import Path
import datetime,hashlib,json,re,shutil
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road
ROOT=Path(__file__).resolve().parents[1];LABEL='navigation_turnfix_20261027'
BASE='impl/baseline_navigation_turnfix_20261027'
def sha(n):return hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
def read(n):
 b=(ROOT/n).read_bytes()
 return b.decode('utf-16' if b.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig').replace('\r\n','\n')
def dump(n,v):(ROOT/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
frozen=json.loads(read(f'impl/{LABEL}_inputs.json'))['files']
assert all(sha(n)==h for n,h in frozen.items()),'Sources changed during build'
baseline=json.loads(read(BASE+'/manifest.json'))
assert all(sha(BASE+'/'+n)==h for n,h in baseline.items()),'Rollback snapshot changed'
changed=sorted(n for n,h in frozen.items() if baseline.get(n)!=h)
assert set(changed)=={'src/portrait/vision_video.v','src/vehicle/vehicle_motion.v',
 'src/vehicle/vehicle_vision_fusion.v','src/vehicle/vehicle_visual_localizer.v'},changed
metrics=json.loads(read(f'impl/{LABEL}_build_metrics.json'));assert metrics['exit_code']==0
log=read(f'impl/{LABEL}_build.log')
assert 'Bitstream generation completed' in log and 'Placement and routing completed' in log
assert not re.search(r'^\s*ERROR\b',log,re.M)
fs='impl/pnr/ov5640.fs';assert sha(fs)!=baseline[fs]
frozen_time=(ROOT/f'impl/{LABEL}_inputs.json').stat().st_mtime
for n in (fs,'impl/gwsynthesis/ov5640.vg','impl/pnr/ov5640.rpt.txt','impl/pnr/ov5640.tr'):
 assert (ROOT/n).stat().st_mtime>=frozen_time,('stale build output',n)
timing=read('impl/pnr/ov5640.tr');report=read('impl/pnr/ov5640.rpt.txt')
violations={k:int(re.search(fr'<Numbers of {k} Violated Endpoints>:(\d+)',timing)[1]) for k in ('Setup','Hold')}
slack={}
for kind in ('Setup','Hold','Recovery','Removal'):
 table=re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)',timing,re.M|re.S)
 assert table,kind
 values=[float(n) for n in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s',table[1],re.M)]
 assert values,kind
 slack[kind]=min(values)
assert not any(violations.values()) and min(slack.values())>=0,(violations,slack)
resources={}
for name in ('Logic','Register','CLS','BSRAM','DSP','I/O Port'):
 m=re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|',report,re.M)
 assert m,name
 used,total=map(float,m.groups());assert used<=total
 resources[name]=dict(used=used,total=total,percent=round(100*used/total,2))
pinpat=r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;'
pins=dict(re.findall(pinpat,read('src/ov5640_ddr3_tft.cst'),re.M))
assert pins==dict(re.findall(pinpat,read(BASE+'/src/ov5640_ddr3_tft.cst'),re.M))
placed=dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',report,re.M))
assert all(placed.get(n)==p for n,p in pins.items()),'Actual pin placement mismatch'
assert pins['host_tx']=='E14' and pins['host_rx']=='E13'
roms=dict(map=audit_map(),font=audit_font(),road=audit_road())
regressions={}
for suite,count in [('targeted',12),('routes',8),('netlist',5)]:
 runs=json.loads(read(f'sim/navigation_turnfix_{suite}_runs.json'));assert len(runs)==count
 for c in runs:
  assert c['passed'],c
  assert c.get('inputs') and all(sha(n)==h for n,h in c['inputs'].items()),('stale inputs',c['name'])
  assert sha(c['log'])==c['sha256']
  assert 'PASS ' in read(c['log']) and 'FATAL:' not in read(c['log'])
 regressions[suite]=runs
for c in regressions['routes']:
 assert c['max_nose_error_mm']<=50,c['name']
 if c['name'].startswith('reported'):
  assert set(c['junctions'])=={'upper_right_left','circled_right'}
  assert all(j['no_midturn_reversal'] for j in c['junctions'].values())
compat=[('corners','corner',22),('targeted','targeted',6),('extras','extras',3),('host','host',4)]
for label,suffix,count in compat:
 name=f'sim/navigation_turnfix_compat_{label}_{suffix}_runs.json'
 runs=json.loads(read(name));assert len(runs)==count,(name,len(runs))
 for c in runs:
  if 'log' in c:
   assert c.get('passed',True)
   if 'sha256' in c:assert sha(c['log'])==c['sha256']
   if c.get('expected_failure'):assert c['exit_code']!=0 and 'NEXT_ENTRY_EARLY' in read(c['log'])
   else:assert 'PASS ' in read(c['log']) and 'FATAL:' not in read(c['log'])
 regressions['compat_'+label]=runs
 if label=='corners':
  combined=read('sim/navigation_turnfix_compat_corners_corner_results.log')
  assert combined.count('$finish called')==count and 'FATAL:' not in combined
before_log='sim/navigation_turnfix_ownership_before.log'
assert 'delayedYaw=150 steer=28' in read(before_log) and 'FATAL:' in read(before_log)
result=dict(status='digital_verification_passed',recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
 bitstream=fs,sha256=sha(fs),bytes=(ROOT/fs).stat().st_size,
 generated_utc=datetime.datetime.fromtimestamp((ROOT/fs).stat().st_mtime,datetime.timezone.utc).isoformat(),
 tool='Gowin V1.9.12.03',device='GW5AT-LV60PG484AC1/I0 B',hardware_tested=False,flashed=False,
 timing_passed=True,resources=resources,violated_endpoints=violations,min_slack_ns=slack,
 input_sha256=frozen,changed_hardware_files=changed,build=metrics,pins_verified=pins,roms=roms,regressions=regressions,
 previous_release_reproduction=dict(log=before_log,sha256=sha(before_log),
  source=BASE+'/src/vehicle/vehicle_vision_fusion.v',source_sha256=sha(BASE+'/src/vehicle/vehicle_vision_fusion.v')),
 rollback_bitstream=BASE+'/'+fs,rollback_sha256=baseline[fs],
 limitations=['Not flashed or tested on the physical car.',
  'Camera-ground simulation does not reproduce every real image, junction or calibration error.',
  'The <=50mm check bounds nose-centre error, not swept whole-body road clearance; the narrow inner lane leaves only 27mm per side when aligned.',
  'No physical routing freeze of other modules; measured faster standard implementation used.'])
release=f'impl/pnr/ov5640_{LABEL}.fs';shutil.copyfile(ROOT/fs,ROOT/release)
assert sha(release)==result['sha256'];result['release_bitstream']=release
for n in (f'docs/{LABEL}_validation.json',f'impl/pnr/ov5640_{LABEL}_audit.json','impl/pnr/ov5640_current_audit.json'):dump(n,result)
dump('impl/pnr/ov5640_download.json',{k:result[k] for k in ('bitstream','release_bitstream','sha256','bytes','generated_utc','tool','device','timing_passed','hardware_tested','flashed')})
max_error=max(c['max_nose_error_mm'] for c in regressions['routes'])
notes=f'''# 自主导航两处反向转弯与起步修正

新位流：`{release}`，默认 `impl/pnr/ov5640.fs` 内容相同。

SHA-256：`{result['sha256']}`。生成时间：{result['generated_utc']}。

- 右上外圈左转、红圈路口右转分别验证。转弯出口相对固定来向判断，车辆已开始偏转后接管仍识别为完整的 90°弯。当前弯尚未完成时，视觉分支提示保持本弯方向。
- 新弯的物理入口到达后，立即交接控制，清掉相反的旧弯指令。轻微内侧误差或零转角不会再被扩大为满舵。
- 起步使用一次完整有效道路观测开始预置舵角，正常帧间波动不重启等待。图像持续无有效道路时，原两秒倒计时后最多再等待三秒，显示采集故障并保持停止。
- S 弯短连接段的来向回正上限由 8°调整为行驶时 16°，静止仍为 8°；这段控制对准来向，不提前执行下一弯。
- 保留原连接边线扫描的行有效性，避免同一扫描行的其他宽色块使道路中心全部失效。视觉地图观测只在可观测曲线中修正纵向进度，停止横向平移地图，横向居中继续由视觉控制。

仅四个导航相关源文件变化：`vehicle_motion.v`、`vehicle_vision_fusion.v`、`vehicle_visual_localizer.v`、`vision_video.v`。绕障算法、测距及陀螺仪驱动、电机控制、舵机标定、斑马线和串口协议源码哈希保持一致。保留的完整旧版在 `{BASE}/`。

实际规划路线共 281 点、约 953cm；正向及起点前后各偏移 60mm均完成，两个路口各自检查方向及转弯中途反向指令。另检查起步左右偏角、噪声帧和正反 S 弯。八组路线最大车头中心误差 {max_error:.2f}mm，全部通过原有 50mm门限；这不等于车身整体始终在道路内，25.4cm内圈道路与20cm车宽在平行时每边余量仅27mm。

旧位流对应源码在接管晚15°的路口测试中复现应右转却左打28°；本次源码和综合后网表通过。还通过缺失起步道路的有界故障、视觉交接、终点停车、整车绕障、测距、斑马线和UART回归。具体记录见同名 validation.json。

构建耗时 {metrics['seconds']:.1f}秒。使用本工程既有记录中更快的普通实现流程，其他模块物理布线没有冻结。Setup/Hold违规端点 {violations['Setup']}/{violations['Hold']}，最小裕量 {slack['Setup']}/{slack['Hold']}ns。

尚未烧录和实车验证。图像模型不能覆盖真实光照、镜头标定误差和所有路口画面，不能据仿真声称实车已通过。
'''
(ROOT/f'docs/{LABEL}.md').write_text(notes,encoding='utf-8')
print(json.dumps({k:result[k] for k in ('status','sha256','resources','violated_endpoints','min_slack_ns')},indent=2))
