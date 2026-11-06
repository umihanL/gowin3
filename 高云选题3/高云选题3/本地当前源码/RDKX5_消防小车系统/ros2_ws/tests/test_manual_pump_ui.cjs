'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const test=require('node:test'),assert=require('node:assert/strict');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const begin=source.indexOf('  function manualPumpReadiness('),end=source.indexOf('  function toast(',begin);
function harness(){
 const nodes={},calls=[],flags={online:true,fresh:true};
 const topics={pump_manual:{can_start:true,phase:'idle',actual_pump_on:false},actuators:{dry_run:true,enabled:false},fire_proposal:{calibration_ready:false}};
 const h={state:{},connected:()=>flags.online,fresh:()=>flags.fresh,data:k=>topics[k]||{},reason:x=>x,
  $:id=>nodes[id]||=( {checked:false,disabled:false,value:id==='pumpManualDuration'?'400':''} ),
  text:(id,v)=>{h.$(id).textContent=v;},uuid:()=> 'manual-pulse-1',toast(){},
  request:async(url,payload)=>{calls.push({url,payload});return {status:'queued'};}};
 vm.runInNewContext(source.slice(begin,end),h);return {h,nodes,calls,topics,flags};
}
test('manual pulse is independently ready without calibration or real gimbal enable',()=>{
 const {h,nodes}=harness();assert.equal(h.manualPumpReadiness(),'');h.renderManualPump();
 assert.equal(nodes.pumpManualStart.disabled,true);nodes.pumpManualCheck.checked=true;h.renderManualPump();assert.equal(nodes.pumpManualStart.disabled,false);
});
test('explicit click submits exactly one finite pulse and clears confirmation',async()=>{
 const {h,nodes,calls}=harness();h.$('pumpManualCheck').checked=true;
 await h.submitManualPump('start');assert.equal(calls.length,1);assert.equal(calls[0].url,'/api/pump-manual');
 assert.equal(JSON.stringify(calls[0].payload),JSON.stringify({action:'start',request_id:'manual-pulse-1',duration_ms:400,operator_confirmed:true}));
 assert.equal(nodes.pumpManualCheck.checked,false);await h.submitManualPump('start');assert.equal(calls.length,1);
});
test('stale state and invalid durations prevent start',async()=>{
 const {h,calls,flags}=harness();h.$('pumpManualCheck').checked=true;flags.fresh=false;await h.submitManualPump('start');assert.equal(calls.length,0);
 flags.fresh=true;for(const value of ['0','99','1001','Infinity','100.1']){h.$('pumpManualDuration').value=value;await h.submitManualPump('start');}
 assert.equal(calls.length,0);
});
test('stop can be sent during pending start without confirmation or fresh node feedback',async()=>{
 const {h,calls,flags}=harness();flags.fresh=false;h.state.pumpManualPending=true;
 await h.submitManualPump('stop');assert.equal(calls.length,1);assert.equal(JSON.stringify(calls[0].payload),JSON.stringify({action:'stop',request_id:'manual-pulse-1'}));
});
test('failed submission never retries and unknown feedback is not shown LOW',async()=>{
 const {h,nodes,calls,topics}=harness();h.$('pumpManualCheck').checked=true;
 h.request=async()=>{calls.push(1);throw Error('timeout');};await h.submitManualPump('start');
 assert.equal(calls.length,1);assert.match(nodes.pumpManualReceipt.textContent,/提交未确认/);assert.match(nodes.pumpManualReceipt.textContent,/不会自动重试/);
 topics.pump_manual.actual_pump_on=null;h.renderManualPump();assert.equal(nodes.pumpManualActual.textContent,'B13：反馈未知');
});
test('actual backend phases and correlated rejections are readable',()=>{
 const {h,topics,nodes}=harness();topics.pump_manual.phase='waiting_off';h.renderManualPump();assert.match(nodes.pumpManualPhase.textContent,/关水/);
 h.state.pumpManualRequestId='mine';topics.pump_manual.last_request_id='someone';topics.pump_manual.last_result='rejected';topics.pump_manual.last_request_reason='busy';
 h.renderManualPump();assert.equal(nodes.pumpManualReceipt,undefined);
 topics.pump_manual.last_request_id='mine';h.renderManualPump();assert.match(nodes.pumpManualReceipt.textContent,/未执行：busy/);
});
test('gimbal dry-run does not hide real manual pump feedback',()=>{
 const h={reason:x=>x};vm.runInNewContext(source.slice(source.indexOf('  function pumpOutputText('),source.indexOf('  function renderFire(')),h);
 const a={dry_run:true,manual_pump_output_enabled:true,pump_backend:'fpga_uart',pump_on:true};
 assert.match(h.pumpOutputText(a,true),/B13.*开启/);assert.match(h.pumpOutputText(a,false),/状态未知/);
 a.pump_on=false;assert.match(h.pumpOutputText(a,true),/B13.*关闭/);
});

test('typed stop is prioritized even when dialogue is unavailable',()=>{
 const calls=[],handlers={},nodes={commandText:{value:'停止试喷'}};
 const h={$:id=>nodes[id]||={addEventListener:(event,fn)=>{handlers[event]=fn;}},
  normalizedCommand:x=>x.replace(/[。！!]/g,''),dialogueInputReady:()=>false,
  sendCommand:(command,options)=>calls.push({command,options}),toast(){}};
 vm.runInNewContext(source.slice(source.indexOf("  $('commandForm').addEventListener('submit'"),source.indexOf("  $('dialogueMessages').addEventListener('scroll'")),h);
 handlers.submit({preventDefault(){}});assert.equal(calls.length,1);assert.equal(calls[0].options.stop,true);
 nodes.commandText.value='手动试喷';handlers.submit({preventDefault(){}});assert.equal(calls.length,1);
});
