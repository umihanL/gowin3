'use strict';
// Exercise the production motor UI without any browser, network or hardware.
const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const fragment=source.slice(source.indexOf('  const motorProfiles='),source.indexOf('  // MOTOR_TEST_EVENTS:'));
function harness(){
  const elements=new Map(),calls=[],beacons=[],paneCalls=[];let now=1000,live=true,actuators={};
  const element=id=>{if(!elements.has(id))elements.set(id,{disabled:false,dataset:{},textContent:'',listeners:{},addEventListener(event,handler){this.listeners[event]=handler;}});return elements.get(id);};
  const actions=['connect','read','enable','run_positive','run_negative','step_positive','step_negative','return_origin','stop','disconnect'];
  const buttons=[];for(const id of ['motor1','motor2'])for(const action of actions){const e=element(id+':'+action);e.dataset={motorId:id,motorAction:action};buttons.push(e);}
  const state={connected:true,view:'motor',devicePane:'motors',received:1000,data:{server_time:1000,control:{motor_test_ready:true},ros:{motor_test_ready:true}}};
  const motors=Object.fromEntries(['motor1','motor2'].map((id,i)=>[id,{motor_id:id,stamp:1000,protocol:i?'zdt_xs':'pd42',firmware_mode:i?'emm':null,
    connected:true,feedback_fresh:true,position_deg:0,position_counts:47360,reference_counts:47360,speed_rpm:0,run_state:1,enabled:true,
    voltage_v:11.86,can_enable:true,can_move:false,can_run:false,busy:false,motion_configuration_valid:true,continuous_running:false}]));
  let top={stamp:1000,motors,recent_requests:[]},sender=async(url,payload)=>({command_id:'receipt-'+payload.request_id});
  const document={hidden:false,querySelectorAll:()=>buttons};
  const context=vm.createContext({state,$:element,labels:{},performance:{now:()=>now},connected:()=>state.connected,fresh:()=>live,data:key=>key==='actuators'?actuators:top,
    finite:Number.isFinite,num:(x,n)=>Number(x).toFixed(n),text:(id,value)=>{element(id).textContent=value;},reason:x=>x,document,toast(){},uuid:()=>String(calls.length+beacons.length+1),
    navigator:{sendBeacon:(url,blob)=>{beacons.push({url,blob});return true;}},Blob,
    selectDevicePane:name=>paneCalls.push(name),request:async(url,payload)=>{calls.push({url,payload});return sender(url,payload);},fetch:()=>{throw Error('unexpected fetch fallback');}});
  vm.runInContext(fragment+'\nglobalThis.feature={motorUI,renderMotorTest,submitMotorTest,stopAllMotors,maintainMotorRuns,endMotorRuns,motorState};',context);
  const routeLine=source.split('\n').find(line=>line.includes("$('motorOpenGimbal').addEventListener"));vm.runInContext(routeLine,context);
  return {...context.feature,state,motors,element,calls,beacons,paneCalls,document,button:(id,a)=>element(id+':'+a),setLive:x=>{live=x;},setActuators:x=>{actuators=x;},
    advance:ms=>{now+=ms;},refresh:()=>{for(const m of Object.values(motors))m.stamp=1000+(now-1000)/1000;},setSender:x=>{sender=x;},
    receipt:(id,result,reason='node-result')=>{const u=context.feature.motorUI[id];top.recent_requests.push({request_id:u.requestId,action:u.requestAction,motor_id:id,result,reason,stamp:motors[id].stamp});},
    setTop:x=>{top=x;},confirmRun:id=>{const m=motors[id];Object.assign(m,{continuous_running:true,continuous_pending:false,run_id:context.feature.motorUI[id].lease.runId,speed_rpm:5,run_state:2,continuous_direction:1,keepalive_remaining_s:4.8});}};
}
function enableRun(h,id){Object.assign(h.motors[id],{can_enable:false,can_run:true,can_move:true});}

test('production ownership is visible above the bench and card reasons without enabling any test request',async()=>{
 const h=harness();h.setActuators({enabled:true,dry_run:false});
 Object.assign(h.state.data.control,{motor_test_ready:false,motor_test_reason:'motor_test_production_actuator_active'});
 for(const m of Object.values(h.motors))Object.assign(m,{connected:false,reason:'motor_test_not_connected'});
 h.renderMotorTest();
 assert.equal(h.element('motorControlNotice').hidden,false);assert.match(h.element('motorControlDetail').textContent,/正式云台控制占用两路串口.*空载调试页/);
 assert.match(h.element('motorServiceBadge').textContent,/正式云台或水泵控制已启用/);
 for(const id of ['motor1','motor2']){
  assert.match(h.element(id+'Reason').textContent,/正式云台或水泵控制已启用/);assert.equal(h.element(id+'Link').textContent,'空载测试未连接');
  assert.equal(h.button(id,'connect').disabled,true);assert.equal(h.button(id,'enable').disabled,true);
  await h.submitMotorTest(id,'connect');
 }
 assert.equal(h.calls.length,0);h.element('motorOpenGimbal').listeners.click();
 assert.deepEqual(h.paneCalls,['gimbal']);assert.equal(h.calls.length,0);assert.equal(h.beacons.length,0);
 const html=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
 assert.match(html,/<div id="motorControlNotice"[^>]*role="status"/);assert.match(html,/<button type="button" id="motorOpenGimbal"/);
});

test('whole-vehicle disconnect overrides stale production and motor-specific errors',()=>{
 const h=harness();h.setActuators({enabled:true,dry_run:false});
 Object.assign(h.state.data.control,{motor_test_ready:false,motor_test_reason:'motor_test_production_actuator_active'});h.state.connected=false;
 h.renderMotorTest();assert.equal(h.element('motorControlTitle').textContent,'整车连接失效');
 assert.match(h.element('motorControlDetail').textContent,/整车连接已失效/);assert.doesNotMatch(h.element('motorControlDetail').textContent,/控制占用/);
 assert.equal(h.element('motorServiceBadge').textContent,'整车连接失效');
 for(const id of ['motor1','motor2']){assert.equal(h.element(id+'Link').textContent,'整车连接失效');assert.match(h.element(id+'Reason').textContent,/整车连接已失效/);assert.match(h.button(id,'connect').title,/整车连接已失效/);}
});

test('current global interlock stays visible and stale actuator data cannot claim ownership',()=>{
 const h=harness();h.setActuators({enabled:true,dry_run:false});h.setLive(false);
 Object.assign(h.state.data.control,{motor_test_ready:false,motor_test_reason:'motor_test_system_status_stale'});h.renderMotorTest();
 assert.equal(h.element('motorControlTitle').textContent,'空载调试暂不可用');assert.doesNotMatch(h.element('motorControlDetail').textContent,/控制占用/);
 assert.match(h.element('motorServiceBadge').textContent,/状态尚未更新/);assert.match(h.element('motor1Reason').textContent,/状态尚未更新/);
 h.setLive(true);h.setActuators({enabled:false,dry_run:true});Object.assign(h.state.data.control,{motor_test_ready:true,motor_test_reason:''});h.renderMotorTest();
 assert.equal(h.element('motorControlNotice').hidden,true);assert.equal(h.element('motorServiceBadge').textContent,'电机测试节点在线');
 assert.equal(h.calls.length,0);assert.equal(h.beacons.length,0);
});

test('observed ZDT position profile has truthful small-step labels without continuous permission',()=>{
  const h=harness();Object.assign(h.motors.motor2,{motion_configuration_valid:false,
    motion_configuration_reason:'zdt_undocumented_options',motion_configuration_detail:'OPTIONS 0x0426: bit10 unknown',
    limited_position_only:true,motion_profile:'zdt_vendor_fd8_bench',step_deg:.9,limit_deg:.9,
    position_test_ready:true,can_enable:true,can_move:false,can_run:false});
  h.renderMotorTest();
  assert.match(h.element('motor2Configuration').textContent,/已适配 0.9°.*连续旋转未开放/);
  assert.match(h.element('motor2Configuration').title,/bit10 unknown/);
  assert.equal(h.button('motor2','enable').disabled,false);
  assert.equal(h.button('motor2','step_positive').disabled,true);
  assert.equal(h.button('motor2','step_positive').textContent,'正向 0.9° 点动');
  assert.equal(h.button('motor1','step_positive').textContent,'正向 5° 点动');
  Object.assign(h.motors.motor2,{can_enable:false,can_move:true});h.renderMotorTest();
  assert.equal(h.button('motor2','step_positive').disabled,false);
  assert.equal(h.button('motor2','run_positive').disabled,true);
  assert.match(h.button('motor2','run_positive').title,/连续旋转未开放/);
  h.motors.motor2.stamp=995;h.renderMotorTest();
  assert.equal(h.button('motor2','step_positive').disabled,true);
});

test('two cards have separate protocol, pins and motor-shaft degree evidence',()=>{
  const h=harness();Object.assign(h.motors.motor2,{position_counts:65536,position_deg:12.34});h.renderMotorTest();
  assert.match(h.element('motor1Protocol').textContent,/PD42.*ttyS1/);assert.match(h.element('motor2Protocol').textContent,/ZDT_XS.*EMM.*ttyS3/);
  assert.equal(h.element('motor2Angle').textContent,'12.34°');assert.match(h.element('motor2Raw').textContent,/ZDT 原始位置 65536/);
  const html=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
  assert.match(html,/id="motor1Card"/);assert.match(html,/id="motor2Card"/);assert.match(html,/PIN 3 TX/);assert.match(html,/PIN 8 TX/);
  assert.match(html,/停止空载测试并失能/);assert.doesNotMatch(html,/id="motorSelect"/);
});

test('one motor pending or stale does not disable the other motor',async()=>{
  const h=harness();enableRun(h,'motor1');enableRun(h,'motor2');let resolve;
  h.setSender((url,payload)=>new Promise(r=>{resolve=r;}));
  const pending=h.submitMotorTest('motor1','run_positive');h.renderMotorTest();
  assert.equal(h.button('motor1','run_positive').disabled,true);assert.equal(h.button('motor2','run_positive').disabled,false);
  h.motors.motor1.stamp=995;h.renderMotorTest();assert.equal(h.element('motor1Angle').textContent,'—');assert.equal(h.element('motor2Angle').textContent,'0.00°');
  resolve({command_id:'run-1'});await pending;
});

test('continuous and jog buttons require explicit per-motor capabilities',()=>{
  const h=harness();h.renderMotorTest();
  for(const id of ['motor1','motor2'])for(const a of ['run_positive','run_negative','step_positive'])assert.equal(h.button(id,a).disabled,true);
  enableRun(h,'motor1');h.renderMotorTest();assert.equal(h.button('motor1','run_positive').disabled,false);assert.equal(h.button('motor2','run_positive').disabled,true);
  h.motors.motor1.continuous_running=true;h.renderMotorTest();assert.equal(h.button('motor1','run_negative').disabled,true);assert.equal(h.button('motor1','stop').disabled,false);
});

test('both motors start independently with explicit identities and no heartbeat before measured running',async()=>{
  const h=harness();enableRun(h,'motor1');enableRun(h,'motor2');
  await Promise.all([h.submitMotorTest('motor1','run_positive'),h.submitMotorTest('motor2','run_negative')]);
  assert.deepEqual(h.calls.map(x=>x.payload.motor_id),['motor1','motor2']);assert.ok(h.motorUI.motor1.lease);assert.ok(h.motorUI.motor2.lease);
  await h.maintainMotorRuns();assert.equal(h.calls.length,2);
  h.confirmRun('motor1');h.confirmRun('motor2');await h.maintainMotorRuns();
  assert.deepEqual(h.calls.slice(2).map(x=>x.url),['/api/motor-heartbeat','/api/motor-heartbeat']);
  assert.equal(h.calls[2].payload.run_id,h.motorUI.motor1.lease.runId);assert.equal(h.calls[3].payload.run_id,h.motorUI.motor2.lease.runId);
  await h.maintainMotorRuns();assert.equal(h.calls.length,4);
  h.advance(1000);h.refresh();await h.maintainMotorRuns();assert.equal(h.calls.length,6);
});

test('heartbeat from another browser or a reloaded page is never adopted',async()=>{
  const h=harness();Object.assign(h.motors.motor1,{continuous_running:true,run_id:'other-browser',speed_rpm:5});
  await h.maintainMotorRuns();assert.equal(h.calls.length,0);assert.equal(h.beacons.length,0);
  h.renderMotorTest();assert.match(h.element('motor1Lease').textContent,/其他控制端启动.*不续租/);
});

test('HTTP failure or missing command ID does not establish a lease or retry movement',async()=>{
  for(const response of [null,{}]){
    const h=harness();enableRun(h,'motor1');h.setSender(async()=>{if(response===null)throw Error('network');return response;});
    await h.submitMotorTest('motor1','run_positive');assert.equal(h.motorUI.motor1.lease,null);assert.equal(h.calls.length,1);assert.equal(h.beacons.length,1);
    h.advance(1000);h.refresh();await h.maintainMotorRuns();assert.equal(h.calls.length,1);
  }
});

test('accepted command plus pending speed is not called confirmed continuous running',async()=>{
  const h=harness();enableRun(h,'motor1');await h.submitMotorTest('motor1','run_positive');h.receipt('motor1','accepted');
  Object.assign(h.motors.motor1,{continuous_pending:true,run_id:h.motorUI.motor1.lease.runId});h.renderMotorTest();
  assert.equal(h.element('motor1RunState').textContent,'等待实测转速');await h.maintainMotorRuns();assert.equal(h.calls.length,1);
  h.advance(5001);h.refresh();await h.maintainMotorRuns();assert.equal(h.motorUI.motor1.lease,null);assert.equal(h.beacons.length,1);
});

test('all-stop sends both stop requests even if first request fails, clearing both leases first',async()=>{
  const h=harness();enableRun(h,'motor1');enableRun(h,'motor2');
  await h.submitMotorTest('motor1','run_positive');await h.submitMotorTest('motor2','run_negative');
  h.setSender(async(url,payload)=>{assert.equal(h.motorUI.motor1.lease,null);assert.equal(h.motorUI.motor2.lease,null);if(payload.motor_id==='motor1')throw Error('failure');return {command_id:'stop-2'};});
  await h.stopAllMotors();assert.deepEqual(h.calls.slice(2).map(x=>[x.payload.motor_id,x.payload.action]),[['motor1','stop'],['motor2','stop']]);
});

test('single stop clears local lease before a failed HTTP result and does not stop other lease',async()=>{
  const h=harness();enableRun(h,'motor1');enableRun(h,'motor2');await h.submitMotorTest('motor1','run_positive');await h.submitMotorTest('motor2','run_positive');
  h.setSender(async()=>{assert.equal(h.motorUI.motor1.lease,null);throw Error('network');});
  await h.submitMotorTest('motor1','stop');assert.equal(h.motorUI.motor1.lease,null);assert.ok(h.motorUI.motor2.lease);
});

test('late start response after stop cannot restore continuous lease',async()=>{
  const h=harness();enableRun(h,'motor1');let resolve;
  h.setSender(async(url,payload)=>payload.action==='stop'?{command_id:'stop-id'}:new Promise(r=>{resolve=r;}));
  const pending=h.submitMotorTest('motor1','run_positive');await h.submitMotorTest('motor1','stop');resolve({command_id:'late-run'});await pending;
  assert.equal(h.motorUI.motor1.requestId,'stop-id');assert.equal(h.motorUI.motor1.lease,null);
});

test('leaving, hiding or disconnecting cancels only runs owned by this UI without automatic resumption',async()=>{
  for(const mode of ['leave','hide','offline']){
    const h=harness();enableRun(h,'motor1');await h.submitMotorTest('motor1','run_positive');
    if(mode==='leave')h.state.view='voice';if(mode==='hide')h.document.hidden=true;if(mode==='offline')h.state.connected=false;
    await h.maintainMotorRuns();assert.equal(h.motorUI.motor1.lease,null);assert.equal(h.beacons.length,1);
    const body=JSON.parse(await h.beacons[0].blob.text());assert.equal(body.action,'stop');assert.equal(body.motor_id,'motor1');
    h.state.view='motor';h.document.hidden=false;h.state.connected=true;await h.maintainMotorRuns();assert.equal(h.calls.length,1);
  }
});

test('leaving while HTTP start is pending invalidates its eventual acknowledgement',async()=>{
  const h=harness();enableRun(h,'motor1');let resolve;h.setSender(()=>new Promise(r=>{resolve=r;}));
  const pending=h.submitMotorTest('motor1','run_positive');h.endMotorRuns('已切换页面');resolve({command_id:'late'});await pending;
  assert.equal(h.motorUI.motor1.lease,null);assert.equal(h.beacons.length,1);
});

test('heartbeat rejection stops only that run and is not retried',async()=>{
  const h=harness();enableRun(h,'motor1');enableRun(h,'motor2');await h.submitMotorTest('motor1','run_positive');await h.submitMotorTest('motor2','run_positive');h.confirmRun('motor1');h.confirmRun('motor2');
  h.setSender(async(url,payload)=>{if(payload.motor_id==='motor1')throw Error('409');return {};});
  await h.maintainMotorRuns();assert.equal(h.motorUI.motor1.lease,null);assert.ok(h.motorUI.motor2.lease);assert.equal(h.beacons.length,1);
  h.advance(1000);h.refresh();await h.maintainMotorRuns();assert.equal(h.calls.filter(x=>x.url.includes('heartbeat')&&x.payload.motor_id==='motor1').length,1);
});

test('heartbeat needs matching run ID and independent fresh feedback',async()=>{
  const h=harness();enableRun(h,'motor1');await h.submitMotorTest('motor1','run_positive');
  Object.assign(h.motors.motor1,{continuous_running:true,run_id:'different'});await h.maintainMotorRuns();assert.equal(h.calls.length,1);
  h.confirmRun('motor1');h.motors.motor1.stamp=998;await h.maintainMotorRuns();assert.equal(h.motorUI.motor1.lease,null);assert.equal(h.beacons.length,1);
});

test('confirmed run stopping or a matching rejection clears its continuation',async()=>{
  const h=harness();enableRun(h,'motor1');await h.submitMotorTest('motor1','run_positive');h.confirmRun('motor1');await h.maintainMotorRuns();
  h.motors.motor1.continuous_running=false;await h.maintainMotorRuns();assert.equal(h.motorUI.motor1.lease,null);
});

test('wrong motor receipt never confirms the other card and terminal result outranks old acceptance',async()=>{
  const h=harness();enableRun(h,'motor1');await h.submitMotorTest('motor1','step_positive');
  const id=h.motorUI.motor1.requestId;
  h.setTop({stamp:1000,motors:h.motors,recent_requests:[{request_id:id,motor_id:'motor2',action:'step_positive',result:'completed'}]});h.renderMotorTest();
  assert.match(h.element('motor1Receipt').textContent,/等待电机节点/);
  h.setTop({stamp:1000,motors:h.motors,recent_requests:[{request_id:id,motor_id:'motor1',action:'step_positive',result:'completed'},{request_id:id,motor_id:'motor1',action:'step_positive',result:'accepted'}]});h.renderMotorTest();
  assert.match(h.element('motor1Receipt').textContent,/已完成/);
});

test('ZDT protocol mode and blockers are shown without converting raw position as PD42',()=>{
  const h=harness();Object.assign(h.motors.motor2,{firmware_mode:'x',position_counts:65536,position_deg:null,motion_configuration_valid:false,motion_configuration_reason:'zdt_configuration_changed_reconnect',can_enable:false,can_run:false});h.renderMotorTest();
  assert.match(h.element('motor2Protocol').textContent,/X 模式/);assert.equal(h.element('motor2Angle').textContent,'—');assert.match(h.element('motor2Configuration').textContent,/参数已变化，请断开后重新连接/);
  assert.equal(h.button('motor2','enable').disabled,true);assert.equal(h.button('motor2','run_positive').disabled,true);assert.equal(h.button('motor2','read').disabled,false);
});

test('legacy flat state displays only its actual motor and never invents second readings',()=>{
  const h=harness();h.setTop({...h.motors.motor1});h.renderMotorTest();assert.equal(h.element('motor1Angle').textContent,'0.00°');assert.equal(h.element('motor2Angle').textContent,'—');
});

test('unavailable or unsupported actions never send a motor command',async()=>{
  const h=harness();await h.submitMotorTest('motor3','run_positive');await h.submitMotorTest('motor1','raw');await h.submitMotorTest('motor1','run_positive');assert.equal(h.calls.length,0);
  h.state.connected=false;await h.submitMotorTest('motor1','stop');assert.equal(h.calls.length,0);
});
