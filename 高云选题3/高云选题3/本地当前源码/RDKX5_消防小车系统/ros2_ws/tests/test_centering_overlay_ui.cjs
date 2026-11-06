'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict'),test=require('node:test');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const code=source.slice(source.indexOf('  function exactFrameRegistry('),source.indexOf('  // CENTERING_OVERLAY_MODEL_END'));
const helper=source.slice(source.indexOf('  function searchPumpRecoveryText('),source.indexOf('  function checkpointAlignmentView('));
const h={reason:value=>value};vm.runInNewContext(helper+code,h);
const frame=()=>({token:'9:1790000000:1:left',coordinate_frame:'left',width:1280,height:1088,
 fire_registry:{frame_token:'9:1790000000:1:left',registry_session:'live-session',coordinate_frame:'left',image_size_wh:[1280,1088],
 targets:[{target_id:7,bbox_xyxy:[400,200,500,400],confidence:.95,selectable:true},
 {target_id:9,bbox_xyxy:[700,200,800,400],confidence:.97,selectable:true}]}});
const input=()=>({extensionFresh:true,frameFresh:true,elapsed:0,extension:{task:'semantic',real_uart:true,status_fresh:true,
 fpga_host:{selected:true,scope:true,link_ok:true,fresh:true},region_search:{active:true,phase:'CENTER_MOVING',
 centering:{enabled:true,active:true,phase:'CENTER_MOVING',center_tolerance_norm:.04,
 current_target_id:7,current_registry_session:'live-session',move_direction:'reverse',completed_count:1,total_count:2,
 queue:[{target_id:7,registry_session:'live-session',bbox_xyxy:[1,1,2,2],status:'aligning'},
 {target_id:9,registry_session:'live-session',bbox_xyxy:[3,3,4,4],status:'completed'}]}}}});
function setPhase(i,phase){i.extension.region_search.phase=phase;i.extension.region_search.centering.phase=phase;}

test('centering overlay is exclusive to fresh permitted extension two',()=>{
 for(const task of ['vision','traffic','standby','idle','fire',undefined]){
  const i=input();i.extension.task=task;assert.equal(h.centeringOverlayModel(frame(),i).visible,false,task);
 }
 for(const task of ['dialogue','semantic']){const i=input();i.extension.task=task;assert.equal(h.centeringOverlayModel(frame(),i).visible,true);}
 for(const key of ['selected','scope','link_ok','fresh']){const i=input();i.extension.fpga_host[key]=false;assert.equal(h.centeringOverlayModel(frame(),i).visible,false,key);}
 for(const key of ['extensionFresh','frameFresh']){const i=input();i[key]=false;assert.equal(h.centeringOverlayModel(frame(),i).visible,false,key);}
 for(const key of ['real_uart','status_fresh']){const i=input();i.extension[key]=false;assert.equal(h.centeringOverlayModel(frame(),i).visible,false,key);}
});

test('current and completed numbered boxes use exact image geometry, never cached queue coordinates',()=>{
 const view=h.centeringOverlayModel(frame(),input());
 assert.equal(view.tolerance,.04);assert.equal(view.boxes.length,2);
 assert.deepEqual(Array.from(view.boxes[0].bbox),[400,200,500,400]);
 assert.equal(view.boxes[0].current,true);assert.match(view.boxes[0].label,/#7 当前/);
 assert.equal(view.boxes[1].current,false);assert.match(view.boxes[1].label,/#9 已处理/);
 assert.match(view.status,/后退对齐/);assert.match(view.status,/本批已处理 1\/2/);
 const i=input();i.extension.region_search.centering.move_direction='forward';
 assert.match(h.centeringOverlayModel(frame(),i).status,/前进对齐/);
});

test('missing or mismatched registry hides numbered boxes and never reuses last known rectangles',()=>{
 for(const change of [f=>delete f.fire_registry,f=>f.fire_registry.frame_token='9:1790000000:2:left',
 f=>f.fire_registry.registry_session='previous-session',f=>f.fire_registry.coordinate_frame='right',
 f=>f.fire_registry.image_size_wh=[640,544]]){
  const f=frame();change(f);const view=h.centeringOverlayModel(f,input());
  assert.equal(view.visible,true);assert.equal(view.boxes.length,0);assert.match(view.status,/等待同帧火焰编号/);
 }
});

test('one second stopped centered countdown is gated by active measured dwell, not a stop request',()=>{
 const i=input(),c=i.extension.region_search.centering;setPhase(i,'CENTER_DWELL');c.dwell_remaining_s=.9;i.elapsed=.2;
 const f=frame();f.fire_registry.targets[0].bbox_xyxy=[590,200,690,400];
 const view=h.centeringOverlayModel(f,i);assert.match(view.status,/已停稳居中.*居中剩余 0.7 秒/);
 setPhase(i,'CENTER_STOP');assert.doesNotMatch(h.centeringOverlayModel(frame(),i).status,/居中剩余/);
 setPhase(i,'CENTER_DWELL');i.extension.region_search.active=false;
 assert.equal(h.centeringOverlayModel(frame(),i).boxes.length,0);assert.doesNotMatch(h.centeringOverlayModel(frame(),i).status,/居中剩余|本批已处理/);
});

test('new pixels outside the center band suppress an older dwell snapshot immediately',()=>{
 const i=input(),c=i.extension.region_search.centering;setPhase(i,'CENTER_DWELL');c.dwell_remaining_s=.8;
 let view=h.centeringOverlayModel(frame(),i);
 assert.match(view.status,/当前画面偏离中线/);assert.doesNotMatch(view.status,/已停稳居中|居中剩余/);
 const f=frame();delete f.fire_registry;view=h.centeringOverlayModel(f,i);
 assert.match(view.status,/等待当前目标同帧居中证据/);assert.doesNotMatch(view.status,/居中剩余/);
});
test('elapsed one-second dwell cannot imply completion after the current bbox vanishes',()=>{
 const i=input(),c=i.extension.region_search.centering;setPhase(i,'CENTER_DWELL');
 c.dwell_remaining_s=0;i.elapsed=1.5;
 const f=frame();f.fire_registry.targets=f.fire_registry.targets.filter(row=>row.target_id!==c.current_target_id);
 const view=h.centeringOverlayModel(f,i);
 assert.match(view.status,/等待当前目标同帧居中证据/);
 assert.doesNotMatch(view.status,/已停稳居中|居中剩余|本批已处理 2\/2/);
 assert.match(view.status,/本批已处理 1\/2/);
 assert.equal(view.boxes.some(row=>row.current),false);
});

test('reacquire and original-view wait explain queued targets without claiming completion',()=>{
 const i=input(),c=i.extension.region_search.centering;setPhase(i,'CENTER_REACQUIRE');c.move_direction='reverse';
 let view=h.centeringOverlayModel(frame(),i);
 assert.match(view.status,/返回原观察位置重找队列火焰.*后退返回/);assert.doesNotMatch(view.status,/居中剩余/);
 setPhase(i,'CENTER_SELECT');c.reason='rechecking_queued_flame_at_original_view';
 assert.match(h.centeringOverlayModel(frame(),i).status,/已回原观察位置，等待队列火焰重现/);
 i.extension.task='vision';assert.equal(h.centeringOverlayModel(frame(),i).visible,false);
});

test('registry selection alone is not the task three-frame confirmation and disabled inference has no numbered boxes',()=>{
 const i=input(),f=frame();i.extension.region_search.centering.queue=[];
 f.fire_registry.targets[1].target_id=10;
 const view=h.centeringOverlayModel(f,i);assert.match(view.boxes[1].label,/待任务确认/);
 assert.doesNotMatch(view.boxes[1].label,/已确认|已处理/);
 f.detector_available=false;assert.equal(h.centeringOverlayModel(f,i).boxes.length,0);
});

test('skipped targets stay visibly different from completed targets and pending confirmation',()=>{
 const i=input(),c=i.extension.region_search.centering;c.current_target_id=null;c.active=false;
 c.queue[0].status='skipped';c.queue[1].status='completed';
 const f=frame(),view=h.centeringOverlayModel(f,i);
 assert.match(view.boxes[0].label,/#7 已跳过/);assert.match(view.boxes[1].label,/#9 已处理/);
 assert.doesNotMatch(view.boxes[0].label,/待任务确认|已处理/);
 const colors=[],ctx={save(){},restore(){},fillRect(){},setLineDash(){},beginPath(){},moveTo(){},lineTo(){},stroke(){},
  strokeRect(){colors.push(this.strokeStyle);},measureText(){return{width:40};},fillText(){}};
 const d={context:ctx,$:()=>null};vm.runInNewContext(source.slice(source.indexOf('  function drawCenteringOverlay('),source.indexOf('  function showFrameState()')),d);
 d.drawCenteringOverlay(f,view);assert.equal(colors.length,2);assert.notEqual(colors[0],colors[1]);
});

test('cooldown reports elapsed duration and rejects low confidence or malformed boxes',()=>{
 const f=frame(),i=input();i.extension.region_search.fire_retrigger={cooldown_remaining_s:2.8};i.elapsed=.3;
 f.fire_registry.targets.push({target_id:10,bbox_xyxy:[300,200,400,400],confidence:.89,selectable:true});
 f.fire_registry.targets.push({target_id:11,bbox_xyxy:[-1,200,400,400],confidence:.99,selectable:true});
 f.fire_registry.targets.push({target_id:12,bbox_xyxy:[300,200,400,400],confidence:.99,selectable:false});
 const view=h.centeringOverlayModel(f,i);assert.equal(view.boxes.length,2);assert.match(view.status,/新目标冷却 2.5 秒/);
});

test('current region pause overrides an inactive completed batch and never shows old completion counts',()=>{
 const i=input(),r=i.extension.region_search,c=r.centering;
 r.phase='WAIT_STOP';c.phase='COMPLETE';c.active=false;c.completed_count=2;c.total_count=2;
 const view=h.centeringOverlayModel(frame(),i);
 assert.match(view.status,/停车准备对齐，尚未完成/);
 assert.doesNotMatch(view.status,/沿路线巡检|本批已处理|居中剩余/);
 r.phase='UNKNOWN';assert.match(h.centeringOverlayModel(frame(),i).status,/任务阶段待更新/);
});

test('collecting shows candidates and fresh frame evidence instead of a misleading zero over zero completion',()=>{
 const i=input(),c=i.extension.region_search.centering;setPhase(i,'CENTER_COLLECT');
 c.current_target_id=null;c.queue=[{target_id:7,registry_session:'live-session',status:'candidate',batch_id:'batch-1'}];
 c.completed_count=0;c.total_count=0;c.collect_reason='waiting_post_stop_stable_flame_evidence';
 const view=h.centeringOverlayModel(frame(),i);
 assert.match(view.status,/尚未完成居中.*待稳定 1.*本批已确认 0.*本帧合格编号 2/);
 assert.doesNotMatch(view.status,/已处理 0\/0/);
 assert.match(view.boxes[0].label,/本批目标 1.*#7.*待任务确认/);
});
test('waiting for a new candidate follows the current region reason rather than a completed collection reason',()=>{
 const i=input(),r=i.extension.region_search,c=r.centering;setPhase(i,'CENTER_SELECT');
 r.reason='waiting_new_batch_candidate_confirmation';c.collect_reason='stable_flames_confirmed_after_stop';
 c.current_target_id=null;
 const view=h.centeringOverlayModel(frame(),i);
 assert.match(view.status,/等待本批新候选连续多帧确认/);
 assert.doesNotMatch(view.status,/stable_flames_confirmed_after_stop/);
});

test('stable batch label follows explicit aliases while pixels and current-target evidence remain exact-frame',()=>{
 const i=input(),c=i.extension.region_search.centering,f=frame();
 c.queue[0].batch_id='batch-1';c.queue[0].observed_id_aliases=[['live-session',7],['live-session',383]];
 f.fire_registry.targets[0].target_id=383;
 let view=h.centeringOverlayModel(f,i);
 assert.match(view.boxes[0].label,/本批目标 1.*#383/);
 assert.equal(view.boxes[0].current,false);assert.match(view.status,/当前目标未在本帧出现/);
 assert.deepEqual(Array.from(view.boxes[0].bbox),[400,200,500,400]);
 c.current_target_id=383;view=h.centeringOverlayModel(f,i);assert.equal(view.boxes[0].current,true);
 f.fire_registry.frame_token='previous-frame';assert.equal(h.centeringOverlayModel(f,i).boxes.length,0);
});

test('skip cause remains visible during return and is distinct from successful alignment',()=>{
 const i=input(),c=i.extension.region_search.centering;setPhase(i,'CENTER_RETURN');
 c.current_target_id=null;c.queue[0].status='skipped';c.queue[0].reason='centering_straight_corridor_exhausted';
 assert.match(h.centeringOverlayModel(frame(),i).status,/回到原路线暂停位置.*已跳过目标：centering_straight_corridor_exhausted/);
 assert.match(h.centeringOverlayModel(frame(),i).boxes[0].label,/已跳过/);
});

test('late metadata updates only the matching frame without decoding or replacing image pixels',()=>{
 const f=frame(),r=f.fire_registry;delete f.fire_registry;
 const next=h.applyFrameRegistryUpdate(f,r);assert.notEqual(next,f);assert.equal(next.token,f.token);
 assert.equal(next.fire_registry,r);assert.equal(f.fire_registry,undefined);
 assert.equal(h.applyFrameRegistryUpdate(f,{...r,frame_token:'old'}),f);
 const applySource=source.slice(source.indexOf('  function applyFrameRegistryUpdate'),source.indexOf('  function centeringOverlayModel'));
 assert.doesNotMatch(applySource,/createImageBitmap|drawImage|jpeg/);
 assert.match(source,/metadata\.fire_registry_update&&state\.frameToken===state\.frame\?\.token/);
});

test('task changes and stale video repaint the base frame to remove the previous layer',()=>{
 const render=source.slice(source.indexOf('  function renderAll()'),source.indexOf('  async function pollState()'));
 const status=source.slice(source.indexOf('  function showFrameState()'),source.indexOf('  function stopFrame('));
 assert.match(render,/refreshCenteringOverlay\(\)/);assert.match(status,/refreshCenteringOverlay\(\)/);
 const draw=source.slice(source.indexOf('  function drawFrame('),source.indexOf('  // CENTERING_OVERLAY_MODEL:'));
 assert.ok(draw.indexOf('context.drawImage')<draw.indexOf('drawCenteringOverlay'));
 assert.doesNotMatch(source,/drawSeekCenterGuide/);
 const html=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
 assert.equal((html.match(/id="centeringOverlayStatus"/g)||[]).length,1);
 assert.match(html,/class="centering-overlay-status hidden"/);
});
