'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),test=require('node:test'),assert=require('node:assert/strict');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const html=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
const code=source.slice(source.indexOf('  function taskWorkflowGuide('),source.indexOf('  function renderTaskWorkflow('));
const context={reason:value=>value};vm.runInNewContext(code,context);
const base={online:true,extensionFresh:true,missionFresh:true,navigationFresh:true,actuatorFresh:true,selectionFresh:true,proposalFresh:true,
  extension:{task:'dialogue',real_uart:true,protocol_ready:true,status_fresh:true},mission:{mode:'inspect'}};
const guide=changes=>context.taskWorkflowGuide({...base,...changes});

test('B13 pending or lost readback is unknown, never presented as water off or flowing',()=>{
 const p={reason:x=>x};vm.runInNewContext(source.slice(source.indexOf('  function pumpOutputText('),source.indexOf('  function renderFire(')),p);
 const a={pump_backend:'fpga_uart',dry_run:false,pump_on:null,pump_requested_on:false};
 assert.match(p.pumpOutputText(a,true),/B13.*状态未知/);
 a.pump_requested_on=true;assert.match(p.pumpOutputText(a,true),/状态未知/);
 a.pump_on=true;assert.match(p.pumpOutputText(a,true),/开启；尚不代表实际出水/);
 assert.match(p.pumpOutputText(a,false),/状态未知/);
 a.pump_on=false;assert.match(p.pumpOutputText(a,true),/B13.*关闭/);
 a.dry_run=true;assert.match(p.pumpOutputText(a,true),/dry-run/);
});
test('selecting either extension waits for the appropriate user input',()=>{
 assert.match(guide({extension:{task:'vision'}}).title,/现场识别与问答/);
 assert.match(guide({}).title,/等待你的导航指令/);
 assert.match(guide({extension:{task:'idle'}}).next,/开启任务1/);
 assert.match(guide({extension:{task:'vision'}}).next,/分析当前火场/);
});
test('queued or asserted arrival cannot become an actual arrived guide',()=>{
 const navigation={command_id:'nav1',target_point:3,phase:'running',valid:true,arrived:false,vehicle_stopped:false};
 assert.match(guide({navigation}).title,/正在前往/);
 navigation.phase='arrived';navigation.arrived=true;
 assert.doesNotMatch(guide({navigation}).title,/已到达/);
 navigation.vehicle_stopped=true;assert.match(guide({navigation}).title,/已到达 3/);
 assert.doesNotMatch(guide({navigation,navigationFresh:false}).title,/已到达/);
 assert.doesNotMatch(guide({navigation,extension:{...base.extension,status_fresh:false}}).title,/已到达/);
});
test('fresh selection, image lock and pump output are separate observed states',()=>{
 const selection={valid:true,target_id:2};
 assert.match(guide({selection}).title,/等待闭环瞄准/);
 assert.match(guide({selection,proposal:{valid:true,image_locked:true}}).title,/图像已锁定/);
 assert.doesNotMatch(guide({selection,selectionFresh:false}).title,/已选火焰/);
 assert.doesNotMatch(guide({actuators:{dry_run:true,pump_on:true}}).title,/水泵控制输出开启/);
 assert.match(guide({actuators:{dry_run:false,pump_on:true}}).title,/水泵控制输出开启/);
 assert.doesNotMatch(guide({actuators:{dry_run:false,pump_on:true},actuatorFresh:false}).title,/水泵控制输出开启/);
});
test('task transitions, lost link and faults override positive guidance',()=>{
 assert.match(guide({extension:{...base.extension,mode_transition:{phase:'waiting_stop'}}}).title,/切换任务/);
 assert.match(guide({online:false,selection:{valid:true,target_id:1}}).title,/等待任务连接/);
 assert.match(guide({mission:{mode:'estop'}}).title,/任务已停止/);
});
test('active region mission describes automatic search and cannot claim progress without fresh hardware',()=>{
 const extension={...base.extension,region_search:{active:true,region_id:'B',phase:'NAVIGATING'}};
 assert.match(guide({extension}).title,/B 区/);assert.match(guide({extension}).next,/确认关水后继续剩余航点/);
 extension.status_fresh=false;assert.match(guide({extension}).title,/反馈待恢复/);
});

test('continuous patrol guidance uses the new one-second batch instead of the legacy checkpoint flow',()=>{
 const region={active:true,continuous_route:true,region_id:'C',phase:'NAVIGATING',execute_fire:false,
  checkpoint_alignment:{enabled:true,active:false,phase:'DONE',current_target_id:7}};
 const extension={...base.extension,region_search:region};
 const result=guide({extension});
 assert.match(result.title,/C 区.*连续巡检，途经点不停/);
 assert.match(result.next,/前后微调让本批每团火焰居中停稳 1 秒，再返回并继续原路线/);
 assert.doesNotMatch(result.title+result.next,/检测点|#7|等待你选择/);
 region.phase='CENTER_DWELL';region.centering={current_target_id:9};
 assert.match(guide({extension}).title,/已居中，停留 1 秒 #9/);
 region.execute_fire=true;region.phase='WAIT_PUMP_OFF';
 assert.match(guide({extension}).title,/等待实际关水/);
 assert.match(guide({extension}).next,/确认关水再继续原路线/);
 region.phase='EXITING';assert.match(guide({extension}).title,/前往出口停车 · 继续检测火焰/);
 extension.status_fresh=false;assert.match(guide({extension}).title,/反馈待恢复/);
});

test('checkpoint alignment guide reports the real target and keeps search-only water off',()=>{
 const extension={...base.extension,region_search:{active:true,region_id:'C',phase:'ALIGNING',execute_fire:false,
  checkpoint_alignment:{enabled:true,active:true,phase:'ALIGNING',current_target_id:7}}};
 assert.match(guide({extension}).title,/C 区.*前进对齐火焰 #7/);
 assert.match(guide({extension}).next,/继续下一目标和检测点；不喷水/);
 extension.region_search.execute_fire=true;
 assert.match(guide({extension}).next,/三维定位和喷水条件/);
 assert.match(guide({extension}).next,/中线对齐不代表水流命中/);
 extension.status_fresh=false;assert.doesNotMatch(guide({extension}).title,/#7/);
});

test('all checkpoints visited is not completion until an actual outside stop at the exit',()=>{
 const extension={...base.extension,task:'semantic',region_search:{command_id:'r1',active:true,region_id:'C',phase:'EXITING',exit:{confirmed_stopped:false}}};
 assert.match(guide({extension}).title,/前往出口停车/);
 extension.region_search.active=false;extension.region_search.phase='COMPLETE';
 assert.doesNotMatch(guide({extension}).title,/已在区外停车/);
 extension.region_search.exit.confirmed_stopped=true;assert.match(guide({extension}).title,/已在区外停车/);
 assert.match(guide({extension,navigation:{command_id:'new-point',target_point:4,phase:'navigating'}}).title,/正在前往 4/);
 extension.status_fresh=false;assert.doesNotMatch(guide({extension}).title,/已在区外停车/);
});

test('search-only region guide never implies automatic extinguishing and preserves found-fire wait',()=>{
 const extension={...base.extension,region_search:{active:true,region_id:'C',execute_fire:false,phase:'NAVIGATING'}};
 assert.match(guide({extension}).title,/C 区 · 搜索火焰/);
 assert.match(guide({extension}).next,/确认发现火焰后停车/);
 assert.doesNotMatch(guide({extension}).next,/自动靠近|短喷/);
 extension.region_search={...extension.region_search,active:false,phase:'COMPLETE',reason:'fire_found_waiting_operator'};
 assert.match(guide({extension}).title,/已停车待指令/);
 extension.status_fresh=false;assert.doesNotMatch(guide({extension}).title,/已停车/);
});

test('cancelled regional navigation shows its cause and next action instead of silently waiting',()=>{
 const extension={...base.extension,region_reference:{valid:false},region_search:{command_id:'region1',active:false,region_id:'C',phase:'CANCELLED',reason:'pump_output_pending_or_not_confirmed_off'}};
 const result=guide({extension});assert.match(result.title,/C 区任务未完成/);
 assert.match(result.next,/pump_output_pending_or_not_confirmed_off/);assert.match(result.next,/重新确认地图起点与方向/);
 assert.match(guide({extension,navigation:{command_id:'new-point',target_point:4,phase:'navigating'}}).title,/正在前往 4/);
 extension.region_reference.valid=true;assert.match(guide({extension}).next,/不会自动重发/);
 extension.status_fresh=false;assert.doesNotMatch(guide({extension}).title,/任务未完成/);
});
test('fixed body camera uses nozzle geometry feedback rather than image centering',()=>{
 const h={};vm.runInNewContext(source.slice(source.indexOf('  function fireLockDisplay('),source.indexOf('  function renderFire(')),h);
 const proposal={aim_mode:'fixed_camera_3d',camera_mount:'fixed_to_vehicle',valid:true,geometry_locked:false,image_locked:true};
 assert.equal(h.fireLockDisplay(proposal).locked,false);
 assert.equal(h.fireLockDisplay(proposal).label,'喷嘴方向');
 proposal.geometry_locked=true;proposal.image_locked=false;
 assert.equal(h.fireLockDisplay(proposal).value,'角度已到位');
 assert.match(guide({selection:{valid:true,target_id:2},proposal}).title,/喷嘴角度到位/);
 assert.match(guide({selection:{valid:true,target_id:2},proposal}).next,/落点标定尚未完成/);
 proposal.valid=false;proposal.calibration_ready=false;
 assert.equal(h.fireLockDisplay(proposal).locked,false);
 assert.match(guide({selection:{valid:true,target_id:2},proposal}).title,/等待安装标定/);
 assert.equal(h.fireLockDisplay({valid:true,image_locked:true}).value,'图像已锁定');
});
test('main controls and examples follow operator-selected fire and manual spray workflow',()=>{
 assert.match(html,/id="taskFlowTitle"/);assert.match(html,/id="taskFlowNext"/);
 assert.match(html,/data-command="分析当前火场"/);
 assert.match(html,/data-example="我现在在8号点，车头朝左，请前往4号点后停下"/);
 assert.doesNotMatch(html,/data-(?:command|example)="(?:自动找火灭火|到达后自动找火灭火)"/);
 assert.match(html,/data-example="开始喷水"/);assert.match(html,/data-example="停止喷水"/);
});
test('aim readiness displays concrete serial faults and rejects stale measured feedback',()=>{
 const topics={mission:{vehicle_stopped:true},actuators:{dry_run:false,enabled:true,feedback_required:true,status_fresh:true,feedback_fresh:true}};
 const h={data:key=>topics[key],fresh:()=>true,reason:x=>x};
 vm.runInNewContext(source.slice(source.indexOf('  function aimingBlockReason('),source.indexOf('  const aimingReady')),h);
 assert.equal(h.aimingBlockReason(),'');
 topics.actuators.error='retained_target_mismatch';assert.match(h.aimingBlockReason(),/retained_target_mismatch/);
 topics.actuators.error='';topics.actuators.feedback_fresh=false;assert.match(h.aimingBlockReason(),/编码器反馈/);
 topics.mission.vehicle_stopped=false;assert.match(h.aimingBlockReason(),/停稳反馈/);
});
