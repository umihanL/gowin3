'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const test=require('node:test'),assert=require('node:assert/strict');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const block=(a,b)=>source.slice(source.indexOf(a),source.indexOf(b,source.indexOf(a)));
function harness(){
  const topics={mission:{mode:'inspect',vehicle_stopped:true},actuators:{dry_run:true,enabled:false},fire_targets:{},
    fire_selection:{valid:true,target_id:2,selection_generation:4,registry_session:'boot-a'},fire_proposal:{valid:true,calibration_ready:true}};
  const stale=new Set(),elements={},views=[{dataset:{devicePane:'gimbal'}},{dataset:{devicePane:'motors'}}],tabs=[];
  const h={state:{devicePane:'motors'},data:k=>stale.has(k)?{}:topics[k]||{},fresh:k=>!stale.has(k),finite:Number.isFinite,reason:x=>x,
    aimingBlockReason:()=>topics.actuators.enabled===true?'':'not_enabled',pumpReady:()=>topics.actuators.pump_enabled===true,
    regionSearchState:()=>topics.extension?.region_search||{},connected:()=>true,endMotorRuns:()=>{},
    $:id=>elements[id]||=( {checked:false,disabled:false,showModal(){this.open=true;},close(){this.open=false;}} ),
    text:(id,value)=>{h.$(id).textContent=value;},canControl:()=>true,toast:()=>{},
    document:{querySelectorAll:selector=>selector==='[data-device-pane]'?views:tabs}};
  for(const el of views)el.classList={toggle:(name,active)=>{el.active=active;}};
  vm.runInNewContext(block('  function targetSelectionReason(', '  function toast(')+
    block('  function selectDevicePane(', '  const mobileTabs =')+
    block('  function normalizedCommand(', '  async function handleCommandButton(')+
    block('  function regionFireStatisticsText(', '  function renderFireSetup(')+
    block('  function fireRoundProgress(', '  function renderPointNavigation('),h);
  return {h,topics,stale,elements,views};
}
test('target remains selectable without motor readiness in both observation modes',()=>{
  const {h,topics,stale}=harness();assert.equal(h.targetSelectionReason(),'');
  topics.mission.mode='fire';assert.equal(h.targetSelectionReason(),'');
  topics.mission.vehicle_stopped=false;assert.match(h.targetSelectionReason(),/停/);
  topics.mission.vehicle_stopped=true;stale.add('fire_targets');assert.match(h.targetSelectionReason(),/过期/);
});
test('gimbal page is reachable instead of silently returning to motor bench',()=>{
  const {h,views}=harness();h.selectDevicePane('gimbal');
  assert.equal(h.state.devicePane,'gimbal');assert.equal(views[0].active,true);assert.equal(views[1].active,false);
});
test('round may begin before aligned, while retaining output and calibration gates',()=>{
  const {h,topics,stale}=harness();assert.equal(h.selectedRoundReason(),'not_enabled');
  topics.actuators={enabled:true,pump_enabled:true};topics.mission.gimbal_ready=false;
  assert.equal(h.selectedRoundReason(),'');
  topics.fire_proposal.calibration_ready=false;assert.match(h.selectedRoundReason(),/标定/);
  topics.fire_proposal.calibration_ready=true;stale.add('fire_selection');assert.match(h.selectedRoundReason(),/编号/);
});
test('confirmation binds the chosen identity and recognizes only explicit finite-round phrases',()=>{
  const {h,topics}=harness();
  for(const phrase of ['对选定火焰自动瞄准喷水并关水','对选定火焰自动瞄准喷水关水','一键瞄准喷水关水','一键瞄准喷水并关水','对选定火焰执行一轮灭火'])assert.equal(h.dangerKind(phrase),'selected-round');
  assert.equal(h.dangerKind('不要一键瞄准喷水关水'),null);
  h.confirmAction('对选定火焰自动瞄准喷水并关水');
  const locked=h.state.pendingAction;assert.equal(locked.identity,'2:4:boot-a');
  assert.equal(JSON.stringify(locked.targetExpectation),JSON.stringify({target_id:2,registry_session:'boot-a',selection_generation:4}));
  topics.fire_selection.selection_generation=5;assert.notEqual(locked.identity,h.selectionIdentity());
  assert.equal(locked.targetExpectation.selection_generation,4);
});
test('statistics distinguish current evidence, retained records and short sprays',()=>{
  const {h}=harness(),r={command_id:'region-1',fire_statistics:{current_fresh:true,current_visible:2,tracked_observations_count:5,mapped_unique_count:3,rounds_completed:1}};
  const text=h.regionFireStatisticsText(r,true);
  assert.match(text,/当前可见 2/);assert.match(text,/累计跟踪记录 5/);assert.match(text,/已定位火源 3/);assert.match(text,/完成短喷 1/);assert.match(text,/不代表火已熄灭/);
  assert.match(h.regionFireStatisticsText(r,false),/历史统计 · 当前可见 —/);
  r.fire_statistics.current_fresh=false;assert.match(h.regionFireStatisticsText(r,true),/当前可见 —/);
});
test('setup readiness never treats stale depth or guessed geometry as measured',()=>{
  const {h,topics,stale}=harness();topics.stereo={gate:'ready'};
  assert.equal(h.fireSetupItems()[3][1],true);stale.add('stereo');assert.equal(h.fireSetupItems()[3][1],false);
  assert.equal(h.fireSetupItems()[2][1],false);assert.equal(h.fireSetupItems()[5][1],false);
});
test('real AutoFire snapshot command_id exposes aiming and burst progress',()=>{
  const {h}=harness();assert.equal(h.fireRoundProgress({}), '');
  assert.match(h.fireRoundProgress({command_id:'round-1',active:true,phase:'AIM'}),/实际角度到位/);
  assert.match(h.fireRoundProgress({command_id:'round-1',active:true,phase:'BURST'}),/开关水反馈/);
});
test('explicit manual pulse text asks for direction confirmation without a selected flame',()=>{
  const {h,topics,elements}=harness();topics.fire_selection={};
  for(const phrase of ['手动试喷','手动试喷400毫秒','手动试喷四百毫秒'])assert.equal(h.dangerKind(phrase),'manual-pump');
  assert.equal(h.dangerKind('不要手动试喷'),null);assert.equal(h.dangerKind('手动试喷吗'),null);
  h.confirmAction('手动试喷四百毫秒');assert.equal(h.state.pendingAction.spray,false);
  assert.match(elements.actionTitle.textContent,/当前喷嘴方向/);assert.match(elements.actionTarget.textContent,/不要求火焰标定/);
});
