'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict'),test=require('node:test');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const block=(a,b)=>source.slice(source.indexOf(a),source.indexOf(b,source.indexOf(a)));
function harness(){
  const values={},topics={},flags={connected:true,fresh:true};
  const context={data:key=>topics[key]||{},fresh:()=>flags.fresh,connected:()=>flags.connected,
    finite:Number.isFinite,num:(x,n)=>Number(x).toFixed(n),text:(id,value)=>{values[id]=value;},reason:x=>x};
  vm.runInNewContext(block('  function extensionCommand(', '  function submitExtension(')+
    block('  function extensionMotionReadiness(', '  function renderControls()')+
    block('  function normalizedCommand(', '  function selectionIdentity('),context);
  return {context,topics,flags,values};
}
test('distance and junction controls emit exact bounded task sentences',()=>{
  const {context:h}=harness();
  assert.equal(h.extensionCommand('distance','1000','forward'),'前进1000厘米');
  assert.equal(h.extensionCommand('distance','20','reverse'),'后退20厘米');
  assert.equal(h.extensionCommand('turn','3','left'),'一直前进在第3个路口左转');
  for(const n of [0,4,1001,1.5,NaN,'abc'])assert.throws(()=>h.extensionCommand('distance',n,'forward'));
  for(const n of [0,9,10,1.5,'abc'])assert.throws(()=>h.extensionCommand('turn',n,'left'));
});
test('reference confirmation preserves decimal map coordinates and rejects out of map values',()=>{
  const {context:h}=harness();
  assert.equal(h.extensionCommand('reference',['104.22','40','90']),'确认起点X104.22厘米Y40厘米朝向90度');
  for(const value of [[352,40,90],[100,320,90],[-1,40,90],[100,40,361],[100,40,NaN]])assert.throws(()=>h.extensionCommand('reference',value));
});
test('automatic seek and suppression has the same explicit confirmation as other automatic fire tasks',()=>{
  const {context:h}=harness();
  assert.equal(h.extensionCommand('fire','observe'),'前进直到火焰居中');
  const automatic=h.extensionCommand('fire','auto');
  assert.equal(automatic,'前进直到火焰居中然后自动灭火');
  assert.equal(h.dangerKind(automatic),'auto');
  assert.equal(h.dangerKind('不要'+automatic),null);
  assert.equal(h.dangerKind('能否'+automatic),null);
});
test('movement requires fresh physical protocol evidence and per-action capabilities',()=>{
  const {context:h,topics,flags}=harness();
  topics.extension={real_uart:true,protocol_ready:true,status_fresh:true,can_move:true,capabilities:{distance:true,velocity:true,route_turn:false}};
  assert.equal(h.extensionMotionReadiness('distance').ready,true);
  assert.equal(h.extensionMotionReadiness('route_turn').ready,false);
  for(const key of ['real_uart','protocol_ready','status_fresh','can_move']){
    topics.extension[key]=false;assert.equal(h.extensionMotionReadiness('distance').ready,false,key);topics.extension[key]=true;
  }
  flags.fresh=false;assert.equal(h.extensionMotionReadiness('distance').ready,false);
  flags.fresh=true;flags.connected=false;assert.equal(h.extensionMotionReadiness('distance').ready,false);
});
test('traffic drive requires separate confirmed traffic mode and fresh stopped capability',()=>{
  const {context:h,topics,flags}=harness();
  topics.extension={task:'vision',real_uart:true,protocol_ready:true,status_fresh:true,can_move:true,fpga_task_mode:1,capabilities:{traffic:true}};
  assert.equal(h.trafficDriveReadiness().ready,false);
  topics.extension.task='traffic';assert.equal(h.trafficDriveReadiness().ready,false);
  topics.extension.fpga_task_mode=3;assert.equal(h.trafficDriveReadiness().ready,false);
  topics.extension.zebra_detection_enabled=false;assert.equal(h.trafficDriveReadiness().ready,false);
  topics.extension.zebra_detection_enabled=true;assert.equal(h.trafficDriveReadiness().ready,true);
  topics.extension.zebra_transition={enabled:false,phase:'waiting_ack'};assert.equal(h.trafficDriveReadiness().ready,false);
  topics.extension.zebra_transition=null;
  topics.extension.mode_transition={phase:'waiting_stop'};assert.equal(h.trafficDriveReadiness().ready,false);
  topics.extension.mode_transition=null;flags.fresh=false;assert.equal(h.trafficDriveReadiness().ready,false);
});

test('zebra indication uses only real readback and never pending target or accepted receipt',()=>{
  const {context:h,topics,flags,values}=harness();
  topics.extension={real_uart:false};assert.equal(h.zebraDisplayState(),'未连接');
  Object.assign(topics.extension,{real_uart:true,status_fresh:true,protocol_ready:true,capabilities:{zebra_switch:true}});
  assert.equal(h.zebraDisplayState(),'未知');
  topics.extension.zebra_detection_enabled=false;assert.equal(h.zebraDisplayState(),'关');
  topics.extension.zebra_transition={enabled:true,phase:'waiting_ack'};h.renderExtension();
  assert.equal(values.extensionZebraState,'关');assert.match(values.extensionZebraDetail,/等待 FPGA 回执/);
  topics.extension.zebra_transition.phase='waiting_readback';topics.extension.last_result={accepted:true};h.renderExtension();
  assert.equal(values.extensionZebraState,'关');assert.match(values.extensionZebraDetail,/等待实际开关回读/);
  topics.extension.zebra_transition=null;topics.extension.zebra_detection_enabled=true;h.renderExtension();
  assert.equal(values.extensionZebraState,'开');
  topics.extension.status_fresh=false;assert.equal(h.zebraDisplayState(),'未知');
  flags.connected=false;assert.equal(h.zebraDisplayState(),'未连接');
});

test('zebra buttons allow a confirmed stopped red-light wait without requiring can_move',()=>{
  const {context:h,topics}=harness();
  topics.extension={real_uart:true,status_fresh:true,protocol_ready:true,capabilities:{zebra_switch:true},
    zebra_can_change:true,can_move:false,traffic_waiting:true};
  assert.equal(h.zebraSwitchReadiness().ready,true);
  topics.extension.zebra_can_change=false;topics.extension.zebra_change_reason='measured_stop_required_for_zebra_switch';
  assert.equal(h.zebraSwitchReadiness().ready,false);
  topics.extension.zebra_can_change=true;topics.extension.zebra_transition={phase:'waiting_ack'};
  assert.equal(h.zebraSwitchReadiness().ready,false);
});

test('extension task selection distinguishes vision from dialogue with the same inspect camera pose',()=>{
  const {context:h,topics,flags}=harness();
  topics.extension={task:'vision'};assert.equal(h.modeButtonActive('extension_one',{mode:'inspect'}),true);
  assert.equal(h.modeButtonActive('extension_two',{mode:'inspect'}),false);
  for(const task of ['dialogue','semantic']){
    topics.extension.task=task;assert.equal(h.modeButtonActive('extension_one',{mode:'inspect'}),false);
    assert.equal(h.modeButtonActive('extension_two',{mode:'inspect'}),true);
  }
  flags.fresh=false;assert.equal(h.modeButtonActive('extension_two',{mode:'inspect'}),false);
});

test('extension two activates via real command then opens numbered tasks only after successful queueing',async()=>{
  const page=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
  assert.match(page,/data-command="开启拓展任务一" data-mode="extension_one"/);
  assert.match(page,/data-command="开启拓展任务二" data-after-view="overview" data-mode="extension_two"/);
  assert.equal((page.match(/id="commandForm"/g)||[]).length,1);
  const calls=[],views=[],h={sendCommand:async(...args)=>{calls.push(args);return {action:'extension_dialogue',status:'queued'};},selectView:x=>views.push(x)};
  vm.runInNewContext(block('  async function handleCommandButton(', "  document.querySelectorAll('[data-command]').forEach"),h);
  await h.handleCommandButton({dataset:{command:'开启拓展任务二',afterView:'extension'}});
  assert.equal(calls[0][0],'开启拓展任务二');assert.deepEqual(views,['extension']);
  h.sendCommand=async()=>undefined;await h.handleCommandButton({dataset:{command:'开启拓展任务二',afterView:'extension'}});
  assert.deepEqual(views,['extension']);
});
test('traffic start command is a separate explicit button from starting observation',()=>{
  const page=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
  assert.match(page,/data-command="开启交通任务">开启识别<\/button>/);
  assert.match(page,/data-command="开始交通行驶" data-traffic-drive(?:="")?>开始行驶<\/button>/);
  const {context:h,topics,values}=harness();
  topics.extension={task:'traffic',mode_transition:{phase:'waiting_stop'}};h.renderExtension();
  assert.equal(values.extensionPhase,'切换任务前等待停稳');
  topics.extension={task:'traffic',active:{action:'traffic_drive',phase:'waiting_traffic_speed'}};h.renderExtension();
  assert.equal(values.extensionPhase,'等待 FPGA 确认交通限速');
});
test('accepted receipt is never presented as completed and stale data is hidden',()=>{
  const {context:h,topics,flags,values}=harness();
  topics.extension={task:'semantic',active:{command_id:'real-1',action:'seek_fire',phase:'seeking'},last_result:{accepted:true,command_id:'real-1',completed:false}};
  h.renderExtension();assert.match(values.extensionAck,/不代表动作完成/);assert.equal(values.extensionPhase,'前进并寻找火焰');
  topics.extension.active={};topics.extension.last_result.completed=true;h.renderExtension();assert.equal(values.extensionAck,'完成已确认');
  flags.fresh=false;h.renderExtension();assert.equal(values.extensionAck,'—');assert.equal(values.extensionPhase,'等待车载任务状态');
});
test('task transition keeps dedicated traffic ownership visible and semantic activity separate',()=>{
  const {context:h,topics,values}=harness();
  topics.extension={task:'traffic',active:null,real_uart:false,protocol_ready:false};
  h.renderExtension();assert.equal(values.extensionTrafficState,'识别已开 · 未自动行驶');
  assert.equal(values.extensionPhase,'独立交通任务 · 等待灯色与 FPGA 反馈');
  topics.extension={task:'semantic',active:{phase:'waiting_mode_stop',action:'route_turn'},last_result:{accepted:true}};
  h.renderExtension();assert.equal(values.extensionTrafficState,'交通任务关闭');
  assert.equal(values.extensionPhase,'任务模式已切换，等待 FPGA 停稳');
});
test('a later rejected request cannot be presented as the active movement result',()=>{
  const {context:h,topics,values}=harness();
  topics.extension={task:'semantic',active:{command_id:'moving-1',phase:'moving',action:'manual_move'},
    last_result:{command_id:'new-request',accepted:false,reason:'extension_movement_in_progress'}};
  h.renderExtension();assert.equal(values.extensionCommandId,'moving-1');
  assert.notEqual(values.extensionAck,'请求未接受');
  topics.extension.active.phase='completed';h.renderExtension();
  assert.equal(values.extensionCommandId,'new-request');assert.equal(values.extensionAck,'请求未接受');
});
test('new capability, route and UART fault reasons have readable Chinese output',()=>{
  const context={};
  vm.runInNewContext(block('  const names =','  const state =')+'\nglobalThis.explain=reason;',context);
  for(const code of ['extension_status_unavailable','fpga_capability_not_implemented_distance',
    'fpga_capability_not_implemented_route_turn','confirm_physical_start_before_junction_route',
    'start_not_on_unique_straight_road','requested_turn_not_present_at_junction',
    'straight_road_ends_before_requested_junction','fpga_mode_stop_not_confirmed',
    'verify_fixed_side_camera_before_seek','selected_fire_track_lost','fpga_command_rejected_3']){
    assert.match(context.explain(code),/[\u4e00-\u9fff]/,code);
    assert.notEqual(context.explain(code),code);
  }
});
test('leaving the traffic task suppresses old green lamps instead of looking like a release',()=>{
  const nodes=new Map(),topics={extension:{task:'traffic'},mission:{traffic_active:true,motion_allowed:true},
    traffic:{state:'RELEASED'},traffic_observation:{valid:true,status:'green',fixture_bbox:[1,2,3,4]}};
  const node=id=>{if(!nodes.has(id)){const classes=new Set();nodes.set(id,{textContent:'',dataset:{},classList:{toggle:(key,value)=>value?classes.add(key):classes.delete(key),remove:key=>classes.delete(key),contains:key=>classes.has(key)}});}return nodes.get(id);};
  const context={state:{view:'extension',stream:true},roadView:{mode:'detection'},roadFrameFresh:()=>false,roadFrameResult:()=>({}),
    fresh:key=>Boolean(topics[key]),data:key=>topics[key]||{},$:node,text:(id,value)=>{node(id).textContent=value;},reason:x=>x,phaseNames:{},
    document:{querySelectorAll:()=>[]},roadResultReason:x=>x};
  vm.runInNewContext(block('  function renderRoad()', '  function renderBuilding()'),context);
  context.renderRoad();assert.equal(node('greenLamp').classList.contains('on'),true);
  topics.extension.task='vision';context.renderRoad();
  assert.equal(node('greenLamp').classList.contains('on'),false);
  assert.equal(node('roadFresh').textContent,'交通任务关闭');
  assert.equal(node('lightLabel').textContent,'灯色检测未启用');
});
