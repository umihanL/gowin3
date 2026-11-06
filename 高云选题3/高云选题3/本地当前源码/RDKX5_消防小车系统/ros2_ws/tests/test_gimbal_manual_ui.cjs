'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict'),test=require('node:test');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const begin=source.indexOf('  function manualGimbalView('),end=source.indexOf('  function taskWorkflowGuide(',begin);
function harness(){
 const nodes={},requests=[],flags={online:true,fresh:true,staleKeys:new Set()},envelopes={};
 const topics={actuators:{dry_run:false,enabled:true,reference_confirmed_this_start:true,status_fresh:true,feedback_fresh:true,axis_limits_deg:[[-90,90],[-30,40]],actual_angles_deg:[12,7]},
  gimbal_manual:{can_start:true,phase:'idle',motion_axes:['pan'],reason:'ready'}};
 const h={state:{data:{ros:{available:true,gimbal_manual_ready:true}}},data:key=>topics[key]||{},topic:key=>envelopes[key],fresh:key=>flags.fresh&&!flags.staleKeys.has(key),connected:()=>flags.online,finite:Number.isFinite,num:(x,n)=>x.toFixed(n),reason:x=>x,
  $:id=>nodes[id]||=( {value:'',disabled:false,dataset:{},listeners:{},removeAttribute(k){delete this[k];},addEventListener(event,handler){this.listeners[event]=handler;},click(){if(!this.disabled)return this.listeners.click?.();}} ),text:(id,value)=>{h.$(id).textContent=value;},uuid:()=> 'manual-1',toast(){},
  request:async(url,value)=>{requests.push({url,value});return {command_id:'published-1'};}};
 vm.runInNewContext(source.slice(begin,end),h);return {h,nodes,topics,flags,requests,envelopes};
}
test('manual gimbal accepts drafts but submits only actual limits and permitted axes',()=>{
 const {h,nodes}=harness();h.renderManualGimbal();
 assert.equal(nodes.gimbalManualPan.value,'');assert.equal(nodes.gimbalManualPan.min,'-90');assert.equal(nodes.gimbalManualPan.max,'90');
 assert.equal(nodes.gimbalManualTilt.disabled,false);assert.match(nodes.gimbalManualTiltRange.textContent,/尚未验收/);
 assert.equal(nodes.gimbalManualPanActual.textContent,'12.0°');assert.equal(nodes.gimbalManualStop.disabled,false);
 assert.throws(()=>h.manualGimbalPayload());
 nodes.gimbalManualPan.value='45';nodes.gimbalManualTilt.value='25';
 assert.throws(()=>h.manualGimbalPayload(),/俯仰轴尚未开放/);h.renderManualGimbal();assert.equal(nodes.gimbalManualStart.disabled,true);
 nodes.gimbalManualTilt.value='';h.renderManualGimbal();assert.equal(nodes.gimbalManualStart.disabled,false);
 assert.equal(JSON.stringify(h.manualGimbalPayload()),JSON.stringify({action:'start',request_id:'manual-1',pan_deg:45}));
 nodes.gimbalManualPan.value='91';assert.throws(()=>h.manualGimbalPayload());
});

test('missing current-boot reference keeps editable drafts without trusting template limits',async()=>{
 const {h,topics,nodes,requests}=harness();
 topics.actuators={dry_run:true,enabled:false,reference_confirmed_this_start:false,status_fresh:false,feedback_fresh:false,axis_limits_deg:[[30,150],[40,140]]};
 topics.gimbal_manual.can_start=false;topics.gimbal_manual.ready_reason='real_actuator_not_enabled';
 h.$('gimbalManualPan').value='-45';h.$('gimbalManualTilt').value='15';h.renderManualGimbal();
 assert.equal(nodes.gimbalManualPan.disabled,false);assert.equal(nodes.gimbalManualTilt.disabled,false);
 assert.equal(nodes.gimbalManualPan.value,'-45');assert.equal(nodes.gimbalManualTilt.value,'15');
 assert.equal(nodes.gimbalManualPan.min,undefined);assert.equal(nodes.gimbalManualPan.max,undefined);
 assert.equal(nodes.gimbalManualStart.disabled,true);assert.match(nodes.gimbalManualReason.textContent,/本次开机零位尚未记录/);
 await h.submitManualGimbal('start');assert.equal(requests.length,0);
});

test('reconnection preserves drafts and requires a fresh explicit click',async()=>{
 const {h,topics,flags,nodes,requests}=harness();flags.online=false;flags.fresh=false;
 h.$('gimbalManualPan').value='20';h.$('gimbalManualTilt').value='10';h.renderManualGimbal();
 assert.equal(nodes.gimbalManualPan.disabled,false);assert.equal(nodes.gimbalManualTilt.disabled,false);
 assert.equal(nodes.gimbalManualStart.disabled,true);assert.equal(nodes.gimbalManualPan.min,undefined);
 flags.online=true;flags.fresh=true;topics.gimbal_manual.motion_axes=['pan','tilt'];h.renderManualGimbal();
 assert.equal(nodes.gimbalManualPan.value,'20');assert.equal(nodes.gimbalManualTilt.value,'10');
 assert.equal(nodes.gimbalManualStart.disabled,false);assert.equal(requests.length,0);
 await h.submitManualGimbal('start');assert.equal(requests.length,1);assert.equal(requests[0].value.tilt_deg,10);
});

test('submit button tracks invalid blank and changed-range drafts',()=>{
 const {h,topics,nodes}=harness();h.renderManualGimbal();assert.equal(nodes.gimbalManualStart.disabled,true);
 h.$('gimbalManualPan').value='35';h.renderManualGimbal();assert.equal(nodes.gimbalManualStart.disabled,false);
 topics.actuators.axis_limits_deg[0]=[-20,20];h.renderManualGimbal();assert.equal(nodes.gimbalManualStart.disabled,true);
 assert.equal(nodes.gimbalManualPan.value,'35');assert.match(nodes.gimbalManualReason.textContent,/当前车载范围/);
 for(const value of ['','NaN','Infinity']){h.$('gimbalManualPan').value=value;h.renderManualGimbal();assert.equal(nodes.gimbalManualStart.disabled,true);}
});
test('explicit one-shot angle submit has no heartbeat or automatic retry',async()=>{
 const {h,requests}=harness();h.$('gimbalManualPan').value='-35.5';await h.submitManualGimbal('start');
 assert.equal(requests.length,1);assert.equal(requests[0].url,'/api/gimbal-manual');assert.equal(requests[0].value.pan_deg,-35.5);
 assert.equal('ttl_s' in requests[0].value,false);assert.equal('tilt_deg' in requests[0].value,false);
 h.request=async()=>{requests.push({failed:true});throw Error('connection');};await h.submitManualGimbal('start');
 assert.equal(requests.length,2);assert.equal(h.state.gimbalManualPending,false);
});
test('stop remains submit-able with stale state, active pending start and no angle values',async()=>{
 const {h,flags,nodes,requests}=harness();flags.fresh=false;flags.online=false;h.state.gimbalManualPending=true;
 h.renderManualGimbal();assert.equal(nodes.gimbalManualStart.disabled,true);assert.equal(nodes.gimbalManualStop.disabled,false);
 await h.submitManualGimbal('stop');assert.equal(requests.length,1);
 assert.equal(JSON.stringify(requests[0].value),JSON.stringify({action:'stop',request_id:'manual-1'}));
});
test('new verified tilt range can be used only when live and finite',()=>{
 const {h,topics,flags,nodes}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];h.renderManualGimbal();
 assert.equal(nodes.gimbalManualTilt.disabled,false);h.$('gimbalManualPan').value='0';h.$('gimbalManualTilt').value='39';
 assert.equal(h.manualGimbalPayload().tilt_deg,39);topics.actuators.axis_limits_deg[1]=[-20,20];assert.throws(()=>h.manualGimbalPayload());
 flags.fresh=false;h.renderManualGimbal();assert.equal(nodes.gimbalManualPanActual.textContent,'—');assert.throws(()=>h.manualGimbalPayload());
});
test('FD8 shows mechanical travel, dynamic reachable endpoints and step without changing the entered target',()=>{
 const {h,topics,nodes,requests}=harness();
 topics.gimbal_manual.axis_reachable_limits_deg=[[-89.967041015625,89.132958984375],[-30,40]];
 topics.gimbal_manual.axis_step_deg=[.9,null];h.$('gimbalManualPan').value='90';h.renderManualGimbal();
 assert.match(nodes.gimbalManualPanRange.textContent,/机械范围 -90° ～ 90°（总行程 180°）/);
 assert.match(nodes.gimbalManualPanRange.textContent,/本次可达端点 -89.967° ～ 89.133°/);
 assert.match(nodes.gimbalManualPanRange.textContent,/指令分辨率 0.9°/);
 assert.equal(nodes.gimbalManualPan.min,'-90');assert.equal(nodes.gimbalManualPan.max,'90');
 assert.equal(nodes.gimbalManualPan.value,'90');assert.equal(h.manualGimbalPayload().pan_deg,90);assert.equal(requests.length,0);
 topics.gimbal_manual.axis_reachable_limits_deg[0]=[-89.5,89.6];h.renderManualGimbal();
 assert.match(nodes.gimbalManualPanRange.textContent,/本次可达端点 -89.5° ～ 89.6°/);
});
test('requested and realizable targets remain distinct and boundary refusal is readable',()=>{
 const {h,topics,nodes}=harness();const m=topics.gimbal_manual;
 m.axis_reachable_limits_deg=[[-89.967,89.133],[-30,40]];m.axis_step_deg=[.9,null];
 m.target_deg=[25,7];m.realizable_target_deg=[25.232958984375,7];m.last_request_id='r';
 m.last_result='rejected';m.last_request_reason='FD8 requested boundary is not reachable on the 0.9 degree lattice within tolerance';
 h.renderManualGimbal();
 assert.match(nodes.gimbalManualTarget.textContent,/请求目标 水平 25°/);
 assert.match(nodes.gimbalManualTarget.textContent,/可达目标 水平 25.233°/);
 assert.match(nodes.gimbalManualReceipt.textContent,/离散步距/);assert.match(nodes.gimbalManualReceipt.textContent,/-89.967° ～ 89.133°/);
 assert.doesNotMatch(nodes.gimbalManualReceipt.textContent,/not reachable/);
 delete m.realizable_target_deg;h.renderManualGimbal();assert.doesNotMatch(nodes.gimbalManualTarget.textContent,/可达目标/);
});
test('missing, invalid and stale lattice feedback is not displayed as current reachability',()=>{
 const {h,topics,nodes,flags}=harness();const m=topics.gimbal_manual;
 m.axis_reachable_limits_deg=[null,[-30,40]];m.axis_step_deg=[.9,null];m.can_start=false;m.ready_reason='fd8_no_reachable_lattice_point';
 h.renderManualGimbal();assert.equal(nodes.gimbalManualStart.disabled,true);
 assert.match(nodes.gimbalManualReason.textContent,/没有可达的 FD8 角度点/);
 m.axis_reachable_limits_deg[0]=[-91,90];h.renderManualGimbal();assert.doesNotMatch(nodes.gimbalManualPanRange.textContent,/-91/);
 m.axis_reachable_limits_deg[0]=[-89,89];flags.fresh=false;h.renderManualGimbal();
 assert.doesNotMatch(nodes.gimbalManualPanRange.textContent,/-89/);assert.equal(nodes.gimbalManualStop.disabled,false);
});

test('standby is an explicit non-submit software button alongside arbitrary angle and stop controls',()=>{
 const html=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
 assert.match(html,/<button type="button" id="gimbalManualStandby"[^>]*disabled>回待机位（0° \/ 30°）<\/button>/);
 assert.match(html,/<button type="submit" id="gimbalManualStart"/);
 assert.match(html,/<button type="button" id="gimbalManualStop"/);
 assert.match(html,/沿用已确认参考，不重新清零/);
});

test('standby DOM click fills 0 and 30 then sends exactly one normal real angle request',async()=>{
 const {h,topics,nodes,requests}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];
 h.$('gimbalManualPan').value='-25';h.$('gimbalManualTilt').value='9';h.renderManualGimbal();
 assert.equal(nodes.gimbalManualStandby.disabled,false);
 await nodes.gimbalManualStandby.click();
 assert.equal(nodes.gimbalManualPan.value,'0');assert.equal(nodes.gimbalManualTilt.value,'30');
 assert.equal(JSON.stringify(requests),JSON.stringify([{url:'/api/gimbal-manual',value:{action:'start',request_id:'manual-1',pan_deg:0,tilt_deg:30}}]));
 h.renderManualGimbal();assert.equal(requests.length,1);
 h.$('gimbalManualPan').value='8';h.$('gimbalManualTilt').value='11';await h.submitManualGimbal('start');
 assert.equal(requests.length,2);assert.equal(requests[1].value.pan_deg,8);assert.equal(requests[1].value.tilt_deg,11);
});

test('standby validates the preset independently of an incomplete or invalid manual draft',()=>{
 const {h,topics,nodes}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];
 h.$('gimbalManualPan').value='1000';h.$('gimbalManualTilt').value='';h.renderManualGimbal();
 assert.equal(nodes.gimbalManualStart.disabled,true);assert.equal(nodes.gimbalManualStandby.disabled,false);
 assert.equal(nodes.gimbalManualPan.value,'1000');assert.equal(nodes.gimbalManualTilt.value,'');
});

test('standby requires both axis permissions, current reference, real enabled feedback and preset limits',async()=>{
 const conditions=[
  (t,f)=>{f.online=false;},(t,f)=>{f.fresh=false;},
  t=>{t.actuators.enabled=false;},t=>{t.actuators.dry_run=true;},
  t=>{t.actuators.reference_confirmed_this_start=false;},
  t=>{t.actuators.status_fresh=false;},t=>{t.actuators.feedback_fresh=false;},
  t=>{t.gimbal_manual.can_start=false;},t=>{t.gimbal_manual.motion_axes=['pan'];},
  t=>{t.gimbal_manual.motion_axes=['tilt'];},
  t=>{t.actuators.axis_limits_deg[0]=[1,90];},
  t=>{t.actuators.axis_limits_deg[1]=[0,20];},
  t=>{t.actuators.axis_limits_deg[1]=[NaN,60];}
 ];
 for(const change of conditions){
  const {h,topics,flags,nodes,requests}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];
  h.$('gimbalManualPan').value='4';h.$('gimbalManualTilt').value='12';change(topics,flags);h.renderManualGimbal();
  assert.equal(nodes.gimbalManualStandby.disabled,true);assert.ok(nodes.gimbalManualStandby.title);
  await nodes.gimbalManualStandby.click();await h.submitManualGimbalStandby();
  assert.equal(requests.length,0);assert.equal(nodes.gimbalManualPan.value,'4');assert.equal(nodes.gimbalManualTilt.value,'12');
  assert.equal(nodes.gimbalManualStop.disabled,false);
 }
});

test('standby click rechecks fresh state even before a stale UI has rerendered',async()=>{
 const {h,topics,flags,nodes,requests}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];h.renderManualGimbal();
 assert.equal(nodes.gimbalManualStandby.disabled,false);flags.fresh=false;
 await nodes.gimbalManualStandby.click();assert.equal(requests.length,0);assert.equal(nodes.gimbalManualStandby.disabled,true);
 flags.fresh=true;h.renderManualGimbal();assert.equal(nodes.gimbalManualStandby.disabled,false);assert.equal(requests.length,0);
});

test('standby pending request suppresses repeat clicks while stop is still dispatched',async()=>{
 const {h,topics,nodes,requests}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];h.renderManualGimbal();
 let resolveStart;h.request=async(url,value)=>{requests.push({url,value});if(value.action==='start')return new Promise(resolve=>{resolveStart=resolve;});return {command_id:'stop-1'};};
 const pending=nodes.gimbalManualStandby.click();assert.equal(nodes.gimbalManualStandby.disabled,true);
 await nodes.gimbalManualStandby.click();await h.submitManualGimbalStandby();assert.equal(requests.length,1);
 assert.equal(nodes.gimbalManualStop.disabled,false);await nodes.gimbalManualStop.click();assert.equal(requests.length,2);assert.equal(requests[1].value.action,'stop');
 resolveStart({command_id:'start-1'});await pending;assert.equal(requests.length,2);assert.equal(h.state.gimbalManualPending,false);
});

test('failed standby has no retry on reconnect or render and preserves the entered destination',async()=>{
 const {h,topics,flags,nodes,requests}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];h.renderManualGimbal();
 h.request=async(url,value)=>{requests.push({url,value});throw Error('timeout');};
 await nodes.gimbalManualStandby.click();assert.equal(requests.length,1);assert.equal(h.state.gimbalManualPending,false);
 flags.online=false;flags.fresh=false;h.renderManualGimbal();flags.online=true;flags.fresh=true;h.renderManualGimbal();
 assert.equal(requests.length,1);assert.equal(nodes.gimbalManualPan.value,'0');assert.equal(nodes.gimbalManualTilt.value,'30');
 assert.equal(nodes.gimbalManualStop.disabled,false);
});

test('current actuator fault exposes exact measured angle, count and limits with editable drafts and no motion',async()=>{
 const {h,topics,nodes,requests}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];
 const detail='tilt measured position outside calibrated limits/reference: actual_deg=-5.4, counts=-983, limits_deg=[0.0,60.0], feedback_limit_tolerance_deg=0.2';
 Object.assign(topics.actuators,{error:detail,status_fresh:false,feedback_fresh:false});
 h.$('gimbalManualPan').value='0';h.$('gimbalManualTilt').value='30';h.renderManualGimbal();
 assert.ok(nodes.gimbalManualReason.textContent.includes(detail));assert.equal(nodes.gimbalManualReason.dataset.state,'warn');
 assert.equal(nodes.gimbalManualPhase.textContent,'运动已中止 · 可手动恢复');assert.equal(nodes.gimbalManualPanActual.textContent,'—');
 assert.equal(nodes.gimbalManualPan.disabled,false);assert.equal(nodes.gimbalManualTilt.disabled,false);
 assert.equal(nodes.gimbalManualPan.value,'0');assert.equal(nodes.gimbalManualTilt.value,'30');
 assert.equal(nodes.gimbalManualStart.disabled,true);assert.equal(nodes.gimbalManualStandby.disabled,true);assert.equal(nodes.gimbalManualStop.disabled,false);
 await h.submitManualGimbal('start');await h.submitManualGimbalStandby();assert.equal(requests.length,0);
 // A true can_start or leftover fresh booleans cannot hide an explicit current fault.
 Object.assign(topics.actuators,{status_fresh:true,feedback_fresh:true});h.renderManualGimbal();assert.equal(nodes.gimbalManualStart.disabled,true);
});

test('stale actuator fault is not presented as current and whole-vehicle disconnect takes precedence',()=>{
 const {h,topics,nodes,flags}=harness();topics.actuators.error='actual_deg=-5.4, limits_deg=[0,60]';
 flags.fresh=false;h.renderManualGimbal();assert.doesNotMatch(nodes.gimbalManualReason.textContent,/actual_deg/);assert.equal(nodes.gimbalManualReason.dataset.state,'idle');
 flags.fresh=true;flags.online=false;h.renderManualGimbal();
 assert.match(nodes.gimbalManualReason.textContent,/整车连接已失效/);assert.doesNotMatch(nodes.gimbalManualReason.textContent,/actual_deg|正式云台故障/);
 assert.equal(nodes.gimbalManualPhase.textContent,'整车连接失效');assert.equal(nodes.gimbalManualReason.dataset.state,'offline');
 assert.equal(nodes.gimbalManualPan.disabled,false);assert.equal(nodes.gimbalManualStop.disabled,false);
});

test('HTTP connection alone cannot enable movement when the ROS bridge or control subscriber is absent',async()=>{
 for(const ros of [{available:false,gimbal_manual_ready:false},{available:true,gimbal_manual_ready:false}]){
  const {h,topics,nodes,requests}=harness();h.state.data.ros=ros;topics.gimbal_manual.motion_axes=['pan','tilt'];
  h.$('gimbalManualPan').value='0';h.$('gimbalManualTilt').value='30';h.renderManualGimbal();
  assert.equal(nodes.gimbalManualConnection.textContent,'网页服务已连接');
  assert.equal(nodes.gimbalManualNode.textContent,ros.available?'未发现控制节点':'ROS 未就绪');
  assert.equal(nodes.gimbalManualPan.disabled,false);assert.equal(nodes.gimbalManualTilt.disabled,false);
  assert.equal(nodes.gimbalManualStart.disabled,true);assert.equal(nodes.gimbalManualStandby.disabled,true);
  assert.match(nodes.gimbalManualReason.textContent,ros.available?/未发现正式云台控制节点/:/ROS 通信尚未就绪/);
  await h.submitManualGimbal('start');await h.submitManualGimbalStandby();assert.equal(requests.length,0);
 }
});

test('missing and stale manual state are distinct while live encoder and reference details remain visible',async()=>{
 const {h,topics,flags,nodes,requests,envelopes}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];
 h.$('gimbalManualPan').value='20';h.$('gimbalManualTilt').value='10';flags.staleKeys.add('gimbal_manual');
 h.renderManualGimbal();assert.match(nodes.gimbalManualReason.textContent,/角度控制状态尚未收到/);
 assert.equal(nodes.gimbalManualStatus.textContent,'尚未收到');
 assert.equal(nodes.gimbalManualFeedback.textContent,'双轴回读正常');assert.equal(nodes.gimbalManualReference.textContent,'本次已记录');
 assert.equal(nodes.gimbalManualPanActual.textContent,'12.0°');assert.equal(nodes.gimbalManualStart.disabled,true);
 envelopes.gimbal_manual={received_at:1};h.renderManualGimbal();assert.match(nodes.gimbalManualReason.textContent,/角度控制状态已过期/);
 await h.submitManualGimbal('start');assert.equal(requests.length,0);
 flags.staleKeys.clear();h.renderManualGimbal();assert.equal(nodes.gimbalManualStatus.textContent,'正常更新');
 assert.equal(nodes.gimbalManualStart.disabled,false);assert.equal(requests.length,0);assert.equal(nodes.gimbalManualPan.value,'20');
});

test('missing actuator feedback does not masquerade as an unrecorded mechanical zero',()=>{
 const {h,flags,nodes,envelopes}=harness();flags.staleKeys.add('actuators');h.renderManualGimbal();
 assert.match(nodes.gimbalManualReason.textContent,/执行器状态尚未收到/);assert.equal(nodes.gimbalManualReference.textContent,'等待状态');
 envelopes.actuators={received_at:1};h.renderManualGimbal();assert.match(nodes.gimbalManualReason.textContent,/执行器状态已过期/);
 assert.doesNotMatch(nodes.gimbalManualReason.textContent,/本次开机零位尚未记录/);
});

test('editing during a pending submit preserves the next draft without changing or repeating the sent target',async()=>{
 const {h,topics,nodes,requests}=harness();topics.gimbal_manual.motion_axes=['pan','tilt'];
 let resolveStart;h.request=async(url,value)=>{requests.push({url,value});return new Promise(resolve=>{resolveStart=resolve;});};
 h.$('gimbalManualPan').value='10';h.$('gimbalManualTilt').value='20';const pending=h.submitManualGimbal('start');
 assert.equal(nodes.gimbalManualPan.disabled,false);assert.equal(nodes.gimbalManualTilt.disabled,false);
 nodes.gimbalManualPan.value='15';nodes.gimbalManualTilt.value='25';h.renderManualGimbal();
 assert.equal(nodes.gimbalManualStart.disabled,true);assert.equal(nodes.gimbalManualStandby.disabled,true);
 assert.equal(requests[0].value.pan_deg,10);assert.equal(requests[0].value.tilt_deg,20);
 resolveStart({command_id:'sent-1'});await pending;
 assert.equal(requests.length,1);assert.equal(nodes.gimbalManualPan.value,'15');assert.equal(nodes.gimbalManualTilt.value,'25');
 assert.equal(nodes.gimbalManualStart.disabled,false);
});

function unreferenced(harnessValue){
 const {topics}=harnessValue;
 Object.assign(topics.actuators,{dry_run:false,enabled:true,reference_confirmed_this_start:false,
  actual_angles_deg:null,status_fresh:false,feedback_fresh:false,raw_feedback_fresh:true,
  motor_counts:[-7,27200],motor_rpm:[0,0],error:'',estop:false});
 Object.assign(topics.gimbal_manual,{can_start:false,reference_capture_ready:true,reference_capture_reason:'',ready_reason:'reference_required'});
 return harnessValue;
}

test('real raw feedback stays visible without inventing joint angles or permitting motion',async()=>{
 const {h,nodes,requests}=unreferenced(harness());h.renderManualGimbal();
 assert.equal(nodes.gimbalManualPanActual.textContent,'—');assert.equal(nodes.gimbalManualTiltActual.textContent,'—');
 assert.equal(nodes.gimbalManualFeedback.textContent,'真实串口回读正常');assert.match(nodes.gimbalManualRaw.textContent,/-7.*27200/);
 assert.equal(nodes.gimbalReferenceConfirm.disabled,false);assert.equal(nodes.gimbalReferenceCapture.disabled,true);
 assert.equal(nodes.gimbalManualStart.disabled,true);assert.equal(nodes.gimbalManualStandby.disabled,true);
 await h.submitManualGimbalReference();assert.equal(requests.length,0);
});

test('operator checkbox sends one reference-only request and never angle, enable or reset commands',async()=>{
 const {h,nodes,requests}=unreferenced(harness());h.renderManualGimbal();
 nodes.gimbalReferenceConfirm.checked=true;h.renderManualGimbal();assert.equal(nodes.gimbalReferenceCapture.disabled,false);
 await nodes.gimbalReferenceCapture.click();
 assert.equal(JSON.stringify(requests),JSON.stringify([{url:'/api/gimbal-manual',value:{
  action:'confirm_reference',request_id:'manual-1',operator_zero_confirmed:true,supported_load:true,stationary_confirmed:true}}]));
 assert.equal(nodes.gimbalReferenceConfirm.checked,false);assert.equal(nodes.gimbalReferenceCapture.disabled,true);
 h.renderManualGimbal();assert.equal(requests.length,1);assert.equal(nodes.gimbalManualStart.disabled,true);
});

test('reference draft acknowledgement is revoked on disconnect, moving, stale or fault feedback',async()=>{
 for(const change of [v=>{v.flags.online=false;},v=>{v.flags.staleKeys.add('actuators');},
  v=>{v.flags.staleKeys.add('gimbal_manual');},v=>{v.topics.actuators.motor_rpm=[0,1];},
  v=>{v.topics.actuators.motor_counts=[NaN,2];},v=>{v.topics.actuators.raw_feedback_fresh=false;},
  v=>{v.topics.actuators.error='UART fault';},v=>{v.topics.actuators.estop=true;},
  v=>{v.topics.gimbal_manual.reference_capture_ready=false;}]){
  const v=unreferenced(harness()),{h,nodes,requests}=v;h.renderManualGimbal();nodes.gimbalReferenceConfirm.checked=true;
  change(v);h.renderManualGimbal();assert.equal(nodes.gimbalReferenceCapture.disabled,true);assert.equal(nodes.gimbalReferenceConfirm.checked,false);
  await h.submitManualGimbalReference();assert.equal(requests.length,0);
 }
});

test('reference timeout neither retries nor restores confirmation after reconnect',async()=>{
 const {h,nodes,requests,flags}=unreferenced(harness());h.renderManualGimbal();nodes.gimbalReferenceConfirm.checked=true;
 h.request=async(url,value)=>{requests.push({url,value});throw Error('timeout');};await h.submitManualGimbalReference();
 assert.equal(h.state.gimbalManualPending,false);assert.equal(nodes.gimbalReferenceConfirm.checked,false);
 flags.online=false;h.renderManualGimbal();flags.online=true;h.renderManualGimbal();
 assert.equal(requests.length,1);assert.equal(nodes.gimbalReferenceCapture.disabled,true);
});

test('reference request pending prevents other submits but preserves editable angles and stop',async()=>{
 const {h,nodes,requests}=unreferenced(harness());h.renderManualGimbal();nodes.gimbalReferenceConfirm.checked=true;
 let resolveReference;h.request=async(url,value)=>{requests.push({url,value});if(value.action==='confirm_reference')return new Promise(resolve=>{resolveReference=resolve;});return {command_id:'stop-1'};};
 const pending=h.submitManualGimbalReference();assert.equal(nodes.gimbalReferenceCapture.disabled,true);
 assert.equal(nodes.gimbalManualPan.disabled,false);assert.equal(nodes.gimbalManualStop.disabled,false);
 await h.submitManualGimbalReference();assert.equal(requests.length,1);
 await nodes.gimbalManualStop.click();assert.equal(requests.length,2);assert.equal(requests[1].value.action,'stop');
 resolveReference({command_id:'reference-1'});await pending;
 assert.notEqual(h.state.gimbalManualRequestId,'reference-1');assert.equal(nodes.gimbalReferenceConfirm.checked,false);
});

test('already recorded mechanical zero can be replaced repeatedly with a new acknowledgement each time',async()=>{
 const {h,nodes,topics,requests}=unreferenced(harness());
 Object.assign(topics.actuators,{reference_confirmed_this_start:true,reference_capture_supported:true});
 let sequence=0;h.uuid=()=>`repeat-${++sequence}`;h.renderManualGimbal();
 assert.match(nodes.gimbalReferenceCapture.textContent,/重新记录/);assert.equal(nodes.gimbalReferenceConfirm.disabled,false);
 for(let index=0;index<2;index++){
  nodes.gimbalReferenceConfirm.checked=true;h.renderManualGimbal();await nodes.gimbalReferenceCapture.click();
  assert.equal(requests.length,index+1);assert.equal(nodes.gimbalReferenceConfirm.checked,false);
  assert.equal(nodes.gimbalReferenceCapture.disabled,true);
 }
 assert.notEqual(requests[0].value.request_id,requests[1].value.request_id);
});

test('reference-only recovery permission permits recapture without enabling angle movement',async()=>{
 const {h,nodes,topics,requests}=unreferenced(harness());const fault='tilt measured position outside calibrated limits/reference: actual_deg=-8.8';
 Object.assign(topics.actuators,{error:fault,reference_capture_supported:true,reference_recovery_permitted:true,reference_recovery_error:fault});
 h.renderManualGimbal();assert.equal(nodes.gimbalReferenceConfirm.disabled,false);
 assert.equal(nodes.gimbalManualStart.disabled,true);assert.equal(nodes.gimbalManualStandby.disabled,true);
 nodes.gimbalReferenceConfirm.checked=true;await h.submitManualGimbalReference();assert.equal(requests.length,1);
 topics.actuators.error='UART fault';h.renderManualGimbal();assert.equal(nodes.gimbalReferenceConfirm.disabled,true);
 nodes.gimbalReferenceConfirm.checked=true;await h.submitManualGimbalReference();assert.equal(requests.length,1);
 topics.actuators.error=fault;topics.actuators.estop=true;h.renderManualGimbal();assert.equal(nodes.gimbalReferenceConfirm.disabled,true);
});

test('persistent zero reports recovery without sending motion or another reference request',()=>{
 const {h,nodes,topics,requests}=unreferenced(harness());
 Object.assign(topics.actuators,{absolute_reference_saved:true,absolute_reference_restore_state:'waiting'});
 h.renderManualGimbal();assert.equal(nodes.gimbalManualReference.textContent,'正在读取绝对位置');
 Object.assign(topics.actuators,{absolute_reference_restore_state:'restored',reference_confirmed_this_start:true});
 h.renderManualGimbal();assert.equal(nodes.gimbalManualReference.textContent,'固定零点已恢复');
 assert.match(nodes.gimbalReferenceReason.textContent,/只有要更改零点时/);
 topics.actuators.absolute_reference_restore_state='failed';h.renderManualGimbal();
 assert.equal(nodes.gimbalManualReference.textContent,'固定零点恢复失败');assert.equal(requests.length,0);
});


test('manual recovery stays available without angle feedback and never submits a cached target',async()=>{
 const {h,topics,flags,nodes,requests}=harness();
 Object.assign(topics.actuators,{error:'ZDT encoder left the commanded angle interval',feedback_fresh:false,status_fresh:false,serial_connected:false});
 topics.gimbal_manual.can_start=false;
 h.$('gimbalManualPan').value='45';h.renderManualGimbal();
 assert.equal(nodes.gimbalManualRecover.disabled,false);assert.equal(nodes.gimbalManualStart.disabled,true);
 await nodes.gimbalManualRecover.click();
 assert.deepEqual(JSON.parse(JSON.stringify(requests)),[{url:'/api/gimbal-manual',value:{action:'recover',request_id:'manual-1'}}]);
 h.renderManualGimbal();assert.equal(requests.length,1);assert.equal(nodes.gimbalManualPan.value,'45');
 topics.actuators.error='';topics.actuators.status_fresh=true;topics.actuators.feedback_fresh=true;topics.gimbal_manual.can_start=true;
 h.renderManualGimbal();assert.equal(requests.length,1);assert.equal(nodes.gimbalManualStart.disabled,false);
 flags.online=false;h.renderManualGimbal();assert.equal(nodes.gimbalManualRecover.disabled,true);
});

test('fresh diagnostic angles remain visible during interrupted motion without enabling a move',()=>{
 const {h,topics,nodes}=harness();
 Object.assign(topics.actuators,{error:'motion interrupted',status_fresh:false,feedback_fresh:false,
  diagnostic_feedback_fresh:true,diagnostic_angles_deg:[-3.2,20.1]});
 h.renderManualGimbal();assert.equal(nodes.gimbalManualPanActual.textContent,'-3.2°');assert.equal(nodes.gimbalManualTiltActual.textContent,'20.1°');
 assert.equal(nodes.gimbalManualStart.disabled,true);assert.match(nodes.gimbalManualFeedback.textContent,/诊断回读正常/);
 topics.actuators.diagnostic_feedback_fresh=false;h.renderManualGimbal();assert.equal(nodes.gimbalManualPanActual.textContent,'—');
});

test('stale recovery-active flags do not lock out an explicit reconnect to a live subscriber',async()=>{
 const {h,topics,flags,nodes,requests}=harness();
 topics.gimbal_manual.recovery_active=true;topics.actuators.recovery_active=true;
 h.renderManualGimbal();assert.equal(nodes.gimbalManualRecover.disabled,true);
 flags.fresh=false;h.renderManualGimbal();assert.equal(nodes.gimbalManualRecover.disabled,false);
 await nodes.gimbalManualRecover.click();assert.equal(requests.length,1);assert.equal(requests[0].value.action,'recover');
});
