'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const test=require('node:test'),assert=require('node:assert/strict');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const start=source.indexOf('  function dialogueInputReady(');
const end=source.indexOf("  document.querySelectorAll('[data-point-target]').forEach(button=>button.addEventListener",start);
const code=source.slice(start,end);
function harness(){
  const nodes=new Map(),requests=[],toasts=[],flags={connected:true,fresh:true};
  const node=id=>{if(!nodes.has(id))nodes.set(id,{value:'',checked:false,textContent:'',disabled:false,title:'',dataset:{},setAttribute(){}});return nodes.get(id);};
  const topics={extension:{real_uart:true,protocol_ready:true,status_fresh:true,vehicle_stopped:true,
    task:'dialogue',can_confirm_point_start:true,can_navigate_point:true,point_reference:{confirmed:true,point_id:2,source:'operator'}}};
  const context={state:{pointTarget:6,pending:false,localMessages:[]},data:key=>flags.fresh?topics[key]||{}:{},
    connected:()=>flags.connected,fresh:()=>flags.fresh,canControl:()=>flags.connected&&!context.state.pending,
    $:node,document:{querySelectorAll:()=>[]},text:(id,value)=>node(id).textContent=value,reason:x=>x,
    toast:(...args)=>toasts.push(args),uuid:()=> 'test-request',renderControls(){},renderDialogue(){},
    request:async(url,payload)=>{requests.push({url,payload});return {command_id:'queued-1',status:'queued'};}};
  vm.runInNewContext(code,context);return {h:context,nodes,node,topics,flags,requests,toasts};
}
test('visual dialogue needs neither FPGA nor a navigation reference, but traffic blocks input',()=>{
  const {h,topics}=harness();topics.extension={task:'vision',dialogue_enabled:true,real_uart:false};assert.equal(h.dialogueInputReady(),true);
  topics.extension.task='traffic';assert.equal(h.dialogueInputReady(),false);
  topics.extension.task='dialogue';assert.equal(h.dialogueInputReady(),true);
  topics.extension={task:'idle',dialogue_enabled:false};assert.equal(h.dialogueInputReady(),true);
});
test('point actions need live physical UART feedback, independently of old movement capabilities',()=>{
  const {h,topics,flags}=harness();assert.equal(h.pointNavigationReadiness().ready,true);
  for(const field of ['real_uart','protocol_ready','status_fresh','can_navigate_point']){
    topics.extension[field]=false;assert.equal(h.pointNavigationReadiness().ready,false,field);topics.extension[field]=true;
  }
  flags.fresh=false;assert.equal(h.pointNavigationReadiness(true).ready,false);
  flags.fresh=true;flags.connected=false;assert.equal(h.pointNavigationReadiness().ready,false);
});
test('observing and chatting does not enable vehicle navigation; traffic also cannot navigate',()=>{
  const {h,topics}=harness();for(const task of ['vision','traffic','idle']){
    topics.extension.task=task;assert.equal(h.pointNavigationReadiness().ready,false);
  }
  topics.extension.task='semantic';assert.equal(h.pointNavigationReadiness().ready,true);
});
test('physical start confirmation requires placement checkbox and bounded integer point',()=>{
  const {h,node}=harness();node('pointStartSelect').value='2';assert.throws(()=>h.pointRequestPayload('confirm_point_start'));
  node('pointPlacementCheck').checked=true;node('pointStartHeading').value='90';
  assert.equal(JSON.stringify(h.pointRequestPayload('confirm_point_start')),JSON.stringify({action:'confirm_point_start',point_id:2,confirmed:true,start_heading_deg:90}));
  for(const value of ['','0','10','2.5','abc']){node('pointStartSelect').value=value;assert.throws(()=>h.pointRequestPayload('confirm_point_start'));}
});
test('selection never supplies a fake start; navigation uses the received confirmed point',()=>{
  const {h,topics,node}=harness();node('pointStartSelect').value='9';
  assert.equal(h.pointRequestPayload('navigate_point').start_point,2);
  topics.extension.point_reference.confirmed=false;assert.throws(()=>h.pointRequestPayload('navigate_point'));
  assert.equal(h.pointNavigationReadiness().ready,false);
});
test('arrival-confirmed reference can start the next leg without a second invented location',()=>{
  const {h,topics}=harness();topics.extension.point_reference={confirmed:true,point_id:4,source:'confirmed_arrival'};
  assert.equal(h.pointRequestPayload('navigate_point').start_point,4);
  h.state.pointTarget=10;assert.throws(()=>h.pointRequestPayload('navigate_point'));
});

test('confirmed arbitrary map origin navigates to a numbered destination without inventing a start point',async()=>{
  const {h,topics,node,requests}=harness();
  topics.extension.point_reference={confirmed:false};
  topics.extension.point_start_reference={valid:true,kind:'map',cell:[17,17],heading_deg:37,source:'operator_hmi'};
  assert.equal(h.pointNavigationReadiness().ready,true);
  const payload=h.pointRequestPayload('navigate_point');
  assert.equal(payload.target_point,6);assert.equal('start_point' in payload,false);
  await h.sendPointAction('navigate_point');
  assert.equal(requests.length,1);assert.equal('start_point' in requests[0].payload,false);
  assert.match(h.state.localMessages[0].text,/从已确认当前位置前往 6 号点/);
  h.renderPointNavigation();assert.equal(node('pointReferenceBadge').textContent,'地图起点');
  assert.match(node('pointReferenceState').textContent,/X 35 \/ Y 35 cm · 朝向 37°/);
  topics.navigation={motion_kind:'numbered_point',command_id:'map-to-6',start_point:null,start_cell:[17,17],target_point:6,phase:'running'};
  h.renderPointNavigation();assert.equal(node('pointRouteSummary').textContent,'地图起点 → 6 号点');
});

test('invalid or stale general map reference cannot fall back to a conflicting old numbered point',()=>{
  const {h,topics,flags,node}=harness();
  for(const ref of [{valid:false,kind:'map'}, {valid:true,kind:'map',cell:[176,1],heading_deg:90},
    {valid:true,kind:'map',cell:[1,1],heading_deg:NaN}, {valid:true,kind:'unknown'}]){
    topics.extension.point_start_reference=ref;
    assert.equal(h.pointNavigationReadiness().ready,false);assert.throws(()=>h.pointRequestPayload('navigate_point'));
  }
  topics.extension.point_start_reference={valid:true,kind:'map',cell:[17,17],heading_deg:37,source:'fpga_touchscreen'};
  flags.fresh=false;assert.equal(h.pointNavigationReadiness().ready,false);assert.throws(()=>h.pointRequestPayload('navigate_point'));
  h.renderPointNavigation();assert.equal(node('pointReferenceBadge').textContent,'未确认');
  flags.fresh=true;topics.extension.can_navigate_point=false;
  assert.equal(h.pointNavigationReadiness().ready,false);
});
test('confirmation submits only the structured point endpoint and waits for real readback',async()=>{
  const {h,node,topics,requests}=harness();topics.extension.point_reference={confirmed:false};
  node('pointStartSelect').value='2';node('pointPlacementCheck').checked=true;node('pointStartHeading').value='90';
  await h.sendPointAction('confirm_point_start');
  assert.equal(requests.length,1);assert.equal(requests[0].url,'/api/point-navigation');
  assert.equal(requests[0].payload.point_id,2);assert.equal(requests[0].payload.confirmed,true);
  assert.equal(topics.extension.point_reference.confirmed,false);assert.equal(node('pointPlacementCheck').checked,false);
  assert.equal(h.state.localMessages[0].action,'confirm_point_start');
});
test('navigation sends target and actual confirmed start, with no spray policy or automatic retries',async()=>{
  const {h,requests}=harness();await h.sendPointAction('navigate_point');
  assert.equal(requests.length,1);assert.equal(requests[0].payload.action,'navigate_point');
  assert.equal(requests[0].payload.target_point,6);assert.equal(requests[0].payload.start_point,2);
  assert.equal('arrival_policy' in requests[0].payload,false);assert.equal('pump' in requests[0].payload,false);
  h.request=async()=>{throw Error('offline');};await h.sendPointAction('navigate_point');assert.equal(h.state.pending,false);
});
test('HTTP acceptance or asserted completion without stopped evidence never becomes arrived',()=>{
  const {h,topics,node}=harness();topics.navigation={motion_kind:'numbered_point',command_id:'job-1',start_point:2,target_point:6,phase:'arrived',valid:true,arrived:true,vehicle_stopped:false};
  h.renderPointNavigation();assert.equal(node('pointPhase').textContent,'等待到达与停稳证据');
  topics.navigation.vehicle_stopped=true;h.renderPointNavigation();assert.equal(node('pointPhase').textContent,'已反馈到达');
  topics.navigation.phase='running';topics.navigation.arrived=false;h.renderPointNavigation();assert.equal(node('pointPhase').textContent,'FPGA 正在导航');
});
test('expired feedback removes origin and successful navigation display',()=>{
  const {h,topics,node,flags}=harness();topics.navigation={motion_kind:'numbered_point',command_id:'job-1',target_point:6,phase:'arrived',valid:true,arrived:true,vehicle_stopped:true};
  h.renderPointNavigation();assert.equal(node('pointReferenceBadge').textContent,'2 号点');
  flags.fresh=false;h.renderPointNavigation();assert.equal(node('pointReferenceBadge').textContent,'未确认');assert.equal(node('pointPhase').textContent,'尚未下发');
});
test('a fresh ROS wrapper with stale FPGA feedback cannot report current arrival',()=>{
  const {h,topics,node}=harness();topics.extension.status_fresh=false;
  topics.navigation={motion_kind:'numbered_point',command_id:'job-1',target_point:6,phase:'arrived',valid:true,arrived:true,vehicle_stopped:true};
  node('pointPlacementCheck').checked=true;node('pointStartHeading').value='90';h.renderPointNavigation();h.renderPointControls();
  assert.equal(node('pointPhase').textContent,'反馈过期 · 等待车载状态');
  assert.equal(node('pointReferenceBadge').textContent,'未确认');assert.equal(node('pointPlacementCheck').checked,false);
});

test('region progress displays actual waypoints and rounds without claiming extinguishment',()=>{
  const {h,topics,node}=harness();
  topics.extension.region_search={command_id:'region-1',region_id:'A',active:true,phase:'EXTINGUISHING',
    waypoints_total:3,visited_waypoint_ids:['A1'],current_waypoint_id:'A2',rounds_completed:1};
  h.renderPointNavigation();
  assert.equal(node('regionPhase').textContent,'停稳后瞄准短喷');
  assert.match(node('regionProgress').textContent,/A 区 · 途经点 1\/3 · 下一途经点 A2/);
  assert.match(node('regionProgress').textContent,/完成 1 轮喷水流程/);
  assert.doesNotMatch(node('regionProgress').textContent,/火焰已扑灭|灭火成功/);
});

test('region display removes progress on stale physical feedback',()=>{
  const {h,topics,node}=harness();
  topics.extension.region_search={command_id:'region-1',region_id:'A',phase:'COMPLETE',rounds_completed:2};
  topics.extension.status_fresh=false;h.renderPointNavigation();
  assert.match(node('regionPhase').textContent,/反馈过期/);
  assert.doesNotMatch(node('regionProgress').textContent,/完成 2 轮/);
});
test('region action requires explicit readiness but not a numbered origin',()=>{
  const {h,topics,node}=harness();topics.extension.point_reference={};
  h.state.regionTarget='A';topics.extension.region_observe_ready=true;h.renderPointControls();
  assert.equal(node('regionSearchStart').disabled,false);
  topics.extension.region_observe_ready=false;h.renderPointControls();assert.equal(node('regionSearchStart').disabled,true);
});

test('region search defaults to observation and does not inherit the extinguishing gimbal gate',()=>{
  const {h,topics,node,flags}=harness();h.state.regionTarget='C';
  Object.assign(topics.extension,{region_observe_ready:true,region_ready:false,region_reason:'region_real_gimbal_reference_required'});
  h.renderPointControls();assert.equal(node('regionSearchStart').disabled,false);
  assert.equal(node('regionSearchStart').textContent,'开始搜索');assert.equal(h.regionSearchCommand(),'在C区域搜索火焰');
  h.state.regionMode='extinguish';h.renderPointControls();assert.equal(node('regionSearchStart').disabled,true);
  assert.equal(node('regionSearchStart').title,'region_real_gimbal_reference_required');
  assert.equal(h.regionSearchCommand(),'在C区域搜寻火源并执行灭火');
  topics.extension.region_ready=true;h.renderPointControls();assert.equal(node('regionSearchStart').disabled,false);
  h.state.regionMode='observe';flags.fresh=false;h.renderPointControls();assert.equal(node('regionSearchStart').disabled,true);
});

test('search-only fire discovery reports stopped and awaiting input without spray completion',()=>{
  const {h,topics,node}=harness();
  topics.extension.region_search={command_id:'observe-1',region_id:'C',execute_fire:false,
    phase:'COMPLETE',reason:'fire_found_waiting_operator',rounds_completed:0,waypoints_total:3,visited_waypoint_ids:[]};
  h.renderRegionSearch();assert.equal(node('regionPhase').textContent,'发现火焰 · 已停车待指令');
  assert.match(node('regionProgress').textContent,/仅搜索/);assert.doesNotMatch(node('regionProgress').textContent,/喷水流程/);
  topics.extension.status_fresh=false;h.renderRegionSearch();assert.match(node('regionPhase').textContent,/反馈过期/);
});
test('page exposes nine targets and distinguishes WiFi requests from USB microphone audio',()=>{
  const html=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
  assert.deepEqual([...html.matchAll(/data-point-target="(\d+)"/g)].map(x=>Number(x[1])),[1,2,3,4,5,6,7,8,9]);
  assert.match(html,/声音由 X5 的 USB 接收器采集，不使用电脑麦克风/);assert.match(html,/Wi-Fi 发送收音请求/);
  assert.match(html,/到达后开启 YOLO，再由你选火焰和下令喷水/);
  assert.match(html,/data-example="在A区域搜寻火源并执行灭火"/);
  assert.match(html,/id="regionSearchMode"><option value="observe">仅搜索<\/option><option value="extinguish">搜索并灭火/);
  assert.match(html,/data-fill="在A区域搜索火焰"/);
  assert.deepEqual([...html.matchAll(/data-region="([A-D])"/g)].map(x=>x[1]),['A','B','C','D']);
  assert.doesNotMatch(html,/沿途自动灭火/);
  assert.doesNotMatch(source,/select_extension2_for_voice_and_dialogue/);
});
