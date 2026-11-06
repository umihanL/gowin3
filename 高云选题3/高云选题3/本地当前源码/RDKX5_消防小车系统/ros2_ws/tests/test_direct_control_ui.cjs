'use strict';
// Production control functions in an isolated VM: no browser, network or robot.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const test=require('node:test');
const vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
function block(start,end){return source.slice(source.indexOf(start),source.indexOf(end,source.indexOf(start)));}
function harness(){
  const elements=new Map(),calls=[];
  const element=id=>{if(!elements.has(id))elements.set(id,{disabled:false,dataset:{},value:'A',querySelector:()=>({disabled:false})});return elements.get(id);};
  const task=element('task'),stop=element('stop'),target=element('target'),auto=element('auto');
  stop.dataset.stop='true';target.dataset.selectable='true';
  const state={connected:true,pending:false,data:{control:{command_ready:true,listen_ready:true},ros:{available:true,listen_ready:true}}};
  const context=vm.createContext({state,$:element,connected:()=>state.connected,data:()=>({}),
    zebraSwitchReadiness:()=>({ready:false,reason:"offline"}),trafficDriveReadiness:()=>({ready:false,reason:"offline"}),extensionMotionReadiness:()=>({ready:false,reason:"offline"}),navigationReady:()=>false,pumpReady:()=>false,aimingReady:()=>false,selectionIdentity:()=>null,
    document:{querySelectorAll:selector=>selector==='[data-command],[data-control]'?[task,stop,auto]:selector==='[data-target]'?[target]:[auto]},
    text:(id,value)=>{element(id).textContent=value;},reason:value=>value,renderAudioTest(){},renderPointControls(){},dialogueInputReady:()=>state.connected,aimingBlockReason:()=> 'hardware_unavailable',targetSelectionReason:()=> 'mission_unavailable',selectedRoundReason:()=> 'hardware_unavailable',
    fetch:async(url,options)=>{calls.push({url,options});return {ok:true,json:async()=>({status:'queued'})};},
    AbortSignal,
  });
  vm.runInContext(block('  const canControl =','  const navigationReady =')+
    block('  async function request(', '  function selectView(')+
    block('  function renderControls()', '  function dialogueInputReady()')+
    '\nglobalThis.feature={renderControls,request};',context);
  return {...context.feature,state,element,calls};
}
test('connected task and voice controls work without token, while hardware gates stay active',()=>{
  const h=harness();h.renderControls();
  assert.equal(h.element('task').disabled,false);
  assert.equal(h.element('listenStart').disabled,false);
  assert.equal(h.element('streamToggle').disabled,false);
  assert.equal(h.element('controlBadge').textContent,'实车控制就绪');
  for(const id of ['navigateBtn','sprayBtn','target','auto'])assert.equal(h.element(id).disabled,true,id);
});
test('lost connection blocks controls; connected stop stays available when router is unavailable',()=>{
  const h=harness();h.state.data.control.command_ready=false;h.renderControls();
  assert.equal(h.element('task').disabled,true);assert.equal(h.element('stop').disabled,false);
  h.state.connected=false;h.renderControls();
  for(const id of ['task','stop','listenStart','listenStop','streamToggle'])assert.equal(h.element(id).disabled,true,id);
});
test('request is sent directly without Authorization, and never sent while disconnected',async()=>{
  const h=harness();await h.request('/api/command',{text:'寻找火焰',request_id:'unit-test'});
  assert.equal(h.calls.length,1);assert.equal(h.calls[0].options.headers.Authorization,undefined);
  assert.equal(h.calls[0].options.headers['Content-Type'],'application/json');
  h.state.connected=false;await assert.rejects(h.request('/api/command',{}),/未连接/);
  assert.equal(h.calls.length,1);
});
