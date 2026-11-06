'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const test=require('node:test'),assert=require('node:assert/strict');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const block=(start,end)=>source.slice(source.indexOf(start),source.indexOf(end,source.indexOf(start)));
function harness(response){
  const toasts=[],calls=[],state={pending:false,localMessages:[]},input={value:'去C区域搜索火焰'};
  const context={state,connected:()=>true,canControl:()=>true,dangerKind:()=>null,reason:x=>x,AbortSignal,
    $:()=>input,renderControls(){},renderDialogue(){},uuid:()=> 'one-request',toast:x=>toasts.push(x),
    fetch:async(...args)=>{calls.push(args);if(response instanceof Error)throw response;return response;}};
  vm.runInNewContext(block('  async function request(', '  function selectView(')+
    block('  async function sendCommand(', '  function normalizedCommand('),context);
  return {context,toasts,calls,state};
}
test('explicit command rejection reports not executed and never queues or resends',async()=>{
  const h=harness({ok:false,status:409,json:async()=>({error:'region_real_gimbal_reference_required',synthetic:false})});
  await h.context.sendCommand('在C区域搜寻火源并执行灭火');
  assert.match(h.toasts.at(-1),/^未执行：region_real_gimbal_reference_required/);
  assert.equal(h.state.localMessages.length,0);assert.equal(h.calls.length,1);assert.equal(h.state.pending,false);
});
test('ambiguous connection loss or server response never claims command rejection or retries',async()=>{
  for(const response of [new Error('timeout'),{ok:false,status:502,json:async()=>({error:'upstream'})},
    {ok:false,status:409,json:async()=>{throw Error('invalid JSON');}}]){
    const h=harness(response);await h.context.sendCommand('去C区域搜索火焰');
    assert.match(h.toasts.at(-1),/^提交未确认：/);assert.match(h.toasts.at(-1),/不会自动重发/);
    assert.equal(h.calls.length,1);assert.equal(h.state.localMessages.length,0);assert.equal(h.state.pending,false);
  }
});
test('accepted request still waits for vehicle execution receipt',async()=>{
  const h=harness({ok:true,status:202,json:async()=>({status:'queued',command_id:'region-1',action:'search_region'})});
  await h.context.sendCommand('去C区域搜索火焰');
  assert.equal(h.state.localMessages.length,1);assert.equal(h.state.localMessages[0].status,'queued');
  assert.match(h.toasts.at(-1),/不代表动作完成/);assert.equal(h.calls.length,1);
});
