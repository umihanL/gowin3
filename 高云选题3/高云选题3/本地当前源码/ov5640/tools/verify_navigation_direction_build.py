"""Audit fresh bitstream, unchanged peripherals, actual pins, ROMs and tests."""
from pathlib import Path
import datetime,hashlib,json,re,shutil
from verify_map_netlist import audit as audit_map
from verify_font_netlist import audit as audit_font
from verify_road_netlist import audit as audit_road
ROOT=Path(__file__).resolve().parents[1];LABEL='navigation_direction_20261027'
BASE='impl/baseline_navigation_direction_20261027'
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
assert set(changed)=={'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_vision_fusion.v'},changed
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
for suite,count in [('targeted',15),('routes',9),('netlist',6),('image',1),('actuator',1)]:
 runs=json.loads(read(f'sim/navigation_direction_{suite}_runs.json'));assert len(runs)==count
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
 name=f'sim/navigation_direction_compat_{label}_{suffix}_runs.json'
 runs=json.loads(read(name));assert len(runs)==count,(name,len(runs))
 for c in runs:
  if 'log' in c:
   assert c.get('passed',True)
   if 'sha256' in c:assert sha(c['log'])==c['sha256']
   if c.get('expected_failure'):assert c['exit_code']!=0 and 'NEXT_ENTRY_EARLY' in read(c['log'])
   else:assert 'PASS ' in read(c['log']) and 'FATAL:' not in read(c['log'])
 regressions['compat_'+label]=runs
 if label=='corners':
  combined=read('sim/navigation_direction_compat_corners_corner_results.log')
  assert combined.count('$finish called')==count and 'FATAL:' not in combined
before_log='sim/navigation_direction_before_contract0.log'
assert 'QUARTER_OUTPUT_REVERSED' in read(before_log) and 'FATAL:' in read(before_log)
for i in (1,2):assert 'UNTRUSTED_START_PRELOAD' in read(f'sim/navigation_direction_before_contract{i}.log')
assert 'PIXEL_START_PRELOAD' in read('sim/navigation_direction_before_image.log')
result=dict(status='digital_verification_passed',recorded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
 bitstream=fs,sha256=sha(fs),bytes=(ROOT/fs).stat().st_size,
 generated_utc=datetime.datetime.fromtimestamp((ROOT/fs).stat().st_mtime,datetime.timezone.utc).isoformat(),
 tool='Gowin V1.9.12.03',device='GW5AT-LV60PG484AC1/I0 B',hardware_tested=False,flashed=False,
 timing_passed=True,resources=resources,violated_endpoints=violations,min_slack_ns=slack,
 input_sha256=frozen,changed_hardware_files=changed,build=metrics,pins_verified=pins,roms=roms,regressions=regressions,
 previous_release_reproduction=dict(log=before_log,sha256=sha(before_log),additional_logs=['sim/navigation_direction_before_contract1.log','sim/navigation_direction_before_contract2.log','sim/navigation_direction_before_image.log'],
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
notes=f'''# 导航起步与转弯方向修正

新位流：`{release}`；默认 `impl/pnr/ov5640.fs` 与其完全相同。
SHA-256：`{result['sha256']}`。生成时间：{result['generated_utc']}。

## 本次改动

仅修改 `src/vehicle/vehicle_motion.v` 与 `src/vehicle/vehicle_vision_fusion.v`。

- 将“收到完整有效图像”与“获得可信的起始姿态”分开。远处仍可拟合道路但近处两条边线都已消失时，不用它给静止车辆预置舵角、改写初始航向或横向坐标。路口宽块同样不能充当初始车身姿态。
- 起步姿态只使用连续、稳定、具备两侧近边界的普通道路观测。缺少姿态证据时仍可中立起步，不增加反复重启的 WAIT 条件。真正没有任何有效起始道路时保留原有的有界采集故障处理。
- 远处或路口的无效转向拟合不会残留在低通历史中；近处有效道路恢复后，第一帧直接装入真实误差。
- 90° 弯接管后，转向输出端也执行本弯方向约束，清除仍在平滑退出的反向旧舵角，直到到达出口航向。保持 S 弯必要的反向回调，出弯后交回视觉居中。
- 导航准备阶段建立入弯元数据，但不提前置位物理入口到达事件；新任务清理旧入口与旧曲线提示。

绕障、电机、IMU/测距驱动、图像显示、斑马线、舵机标定、串口及引脚源文件保持原哈希。
完整旧版快照：`{BASE}/`。

## 验证与范围

旧代码在新增测试中复现：已经接管的 90° 弯仍输出相反满舵；远处/路口观测给静止车辆约 22° 预打角，并误改初始航向。完整 640×480 图像经真实扫描器输入旧控制器，也复现无近边界时预打角。
新代码通过 15 组定向 RTL、6 组 Gowin 综合网表测试、9 组实际地图路径与独立车辆模型、完整图像链和真实舵机 PWM 输出链测试；另保留弯道、终点、视觉、整车绕障、制动、斑马线及通信回归。
实际路线分别检查右上外圈左转与内部路口右转，并包含起点前后偏移、初始偏角、噪声和双向 S 弯。

9 组路线的最大车头中心误差为 {max_error:.2f}mm。50mm 检查只表示模型中的车头中心误差限制；20cm 车宽在25.4cm直道居中时每侧余量27mm，不能据此断言整个车身始终在道路内。

Gowin 构建耗时 {metrics['seconds']:.1f} 秒。使用本工程实测较快的普通实现流程；其他模块物理布线未冻结。
Setup/Hold 违规端点：{violations['Setup']}/{violations['Hold']}；最小裕量：{slack['Setup']}/{slack['Hold']}ns。

尚未烧录或进行本轮实车验证。图像模拟不覆盖所有真实光照、相机误差和路口画面；不能用数字测试通过代替实车通过。
作品名称与简介见 `docs/competition_project.md`。
'''
(ROOT/f'docs/{LABEL}.md').write_text(notes,encoding='utf-8')
rows=['# 当前构建资源与验证记录','',f'当前位流：`{release}`。默认 `impl/pnr/ov5640.fs` 与其相同。','',
      '| 资源 | 使用 / 总量 | 使用率 |','| --- | ---: | ---: |']
for name,v in resources.items():
 rows.append(f"| {name} | {v['used']:g} / {v['total']:g} | {v['percent']:.2f}% |")
rows += ['',f"Setup/Hold 违规端点：{violations['Setup']}/{violations['Hold']}；最小裕量 {slack['Setup']}/{slack['Hold']}ns。",'',
         f"构建耗时 {metrics['seconds']:.1f} 秒。普通实现流程；未冻结其他模块物理布线。",'',
         f"SHA-256：`{result['sha256']}`。",'',
         '仅修改两个导航源文件；外围模块源哈希保持一致。完整验证见 [本次记录](navigation_direction_20261027.md) 与同名 validation.json。','',
         '未烧录、未实车验证。']
(ROOT/'docs/resource_usage.md').write_text('\n'.join(rows)+'\n',encoding='utf-8')
print(json.dumps({k:result[k] for k in ('status','release_bitstream','sha256','violated_endpoints','min_slack_ns','resources')},indent=2))

