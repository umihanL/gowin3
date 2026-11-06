'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict'),test=require('node:test');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const code=source.slice(source.indexOf('  function searchPumpRecoveryText('),source.indexOf('  function renderRegionSearch('));
const context={};vm.runInNewContext(code,context);
const sample=()=>({command_id:'region-live',checkpoint_alignment:{enabled:true,active:true,phase:'ALIGNING',
  forward_image_side:'right',confirm_frames:3,center_confirm_frames:3,center_tolerance_norm:.03,dwell_s:2,
  confirmed_count:3,processed_count:0,remaining_count:3,current_target_id:7,center_error_norm:.08,
  queue:[{target_id:7,status:'aligning'},{target_id:2,status:'queued'},{target_id:9,status:'queued'}]}});

test('checkpoint queue uses actual backend order and reports alignment without a hit claim',()=>{
 const view=context.checkpointAlignmentView(sample(),true);
 assert.match(view.rule,/右半幅 → 中线/);assert.match(view.rule,/连续 3 帧/);assert.match(view.rule,/到位停 2 秒/);assert.match(view.rule,/±3%画宽/);
 assert.match(view.progress,/当前 #7/);assert.match(view.progress,/中线偏差 \+8\.0%画宽/);
 assert.match(view.queue,/#7 对齐中 → #2 待对齐 → #9 待对齐/);
 assert.doesNotMatch(view.progress+view.queue,/已灭火|命中|近距离|停车完成/);
});

test('semantic confidence and temporary stop feedback wait are visible without ending the task',()=>{
 const r=sample();r.checkpoint_alignment.min_confidence=.9;
 r.checkpoint_alignment.feedback_wait_reason='checkpoint_waiting_measured_stop';
 const view=context.checkpointAlignmentView(r,true);
 assert.match(view.rule,/置信度 ≥90%/);
 assert.match(view.progress,/等待停车与位姿回包一致，保留后续检查点/);
 assert.doesNotMatch(view.progress,/任务结束|搜索完成/);
 assert.doesNotMatch(context.checkpointAlignmentView(r,false).progress,/停车与位姿回包一致/);
});

test('dwell countdown appears only during fresh verified dwell state',()=>{
 const r=sample();r.checkpoint_alignment.dwell_remaining_s=1.6;
 assert.doesNotMatch(context.checkpointAlignmentView(r,true).progress,/停留剩余/);
 r.checkpoint_alignment.phase='ALIGN_DWELL';
 assert.match(context.checkpointAlignmentView(r,true).progress,/停留剩余 1\.6 秒/);
 for(const value of [-1,NaN,Infinity,'1.6']){
  r.checkpoint_alignment.dwell_remaining_s=value;
  assert.doesNotMatch(context.checkpointAlignmentView(r,true).progress,/停留剩余/);
 }
 r.checkpoint_alignment.dwell_remaining_s=1.6;
 const stale=context.checkpointAlignmentView(r,false);
 assert.doesNotMatch(stale.progress,/#7|1\.6|已停稳/);assert.equal(stale.queue,'');
});

test('unconfigured direction or disabled workflow cannot suggest active motion',()=>{
 const r=sample();r.checkpoint_alignment.forward_image_side='unset';
 let view=context.checkpointAlignmentView(r,true);assert.match(view.rule,/未设置/);assert.equal(view.queue,'');
 r.checkpoint_alignment.forward_image_side='left';assert.match(context.checkpointAlignmentView(r,true).rule,/左半幅 → 中线/);
 r.checkpoint_alignment.enabled=false;view=context.checkpointAlignmentView(r,true);
 assert.doesNotMatch(view.progress,/当前 #|前进对齐|停留剩余/);assert.equal(view.queue,'');
});

test('finished checkpoint keeps queue evidence but does not show an active old target',()=>{
 const r=sample();Object.assign(r.checkpoint_alignment,{active:false,phase:'DONE',processed_count:3,remaining_count:0});
 const view=context.checkpointAlignmentView(r,true);
 assert.match(view.progress,/继续剩余检测点/);assert.match(view.progress,/已处理 3 \/ 剩余 0/);
 assert.doesNotMatch(view.progress,/当前 #7|中线偏差/);assert.equal(view.state,'ok');
});

test('untrusted target identifiers and unknown queue labels are not rendered as HTML',()=>{
 const r=sample();r.checkpoint_alignment.current_target_id='<img src=x>';
 r.checkpoint_alignment.queue=[{target_id:'<script>bad</script>',status:'queued'},{target_id:0,status:'queued'},
  {target_id:2,status:'<b>complete</b>'}];
 const view=context.checkpointAlignmentView(r,true);
 assert.equal(view.queue,'本点队列：#2 待回报');assert.doesNotMatch(view.progress+view.queue,/<|>/);
});

test('checkpoint status remains inside the existing region panel without a new navigation workspace',()=>{
 const html=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
 assert.equal((html.match(/id="regionAlignment"/g)||[]).length,1);
 assert.ok(html.indexOf('id="regionAlignment"')>html.indexOf('class="region-search-panel"'));
 assert.ok(html.indexOf('id="regionAlignment"')<html.indexOf('id="regionProgress"'));
 assert.match(html,/看到火焰不代表喷嘴已瞄准或水流能够命中/);
 assert.match(html,/经过途经点不停，全程开启火焰检测/);
 assert.doesNotMatch(html,/到检测点后连续确认火焰|检测点逐个|逐个对齐中线/);
});

test('continuous patrol displays pass-through navigation and the new bidirectional one-second queue',()=>{
 const r=sample();Object.assign(r,{continuous_route:true,active:true,phase:'NAVIGATING',execute_fire:false,
  fire_statistics:{min_confidence:.9},target_id:null});
 const view=context.checkpointAlignmentView(r,true);
 assert.match(view.rule,/全程火焰检测.*置信度 ≥90%.*连续 3 帧确认.*前后对齐中线 ±4%.*每团停稳居中 1 秒/);
 assert.match(view.progress,/沿路线行驶，途经点不停/);
 assert.match(view.queue,/仅搜索：本批观察后返回原路线，不喷水/);
 assert.doesNotMatch(view.rule+view.progress+view.queue,/本点队列|#7|停靠|回正|减速/);
 r.phase='CENTER_DWELL';r.centering={current_target_id:9,dwell_remaining_s:.7,
 queue:[{target_id:9,status:'centered'},{target_id:8,status:'queued'}]};
 const stopped=context.checkpointAlignmentView(r,true);
 assert.match(stopped.progress,/当前火焰 #9.*居中停留剩余 0\.7 秒/);
 assert.match(stopped.queue,/#9 居中观察 → #8 待对齐/);
 r.phase='WAIT_STOP';assert.doesNotMatch(context.checkpointAlignmentView(r,true).progress,/停留剩余/);
 r.execute_fire=true;r.phase='WAIT_PUMP_OFF';
 assert.match(context.checkpointAlignmentView(r,true).progress,/等待实际关水后继续原路线/);
 assert.match(context.checkpointAlignmentView(r,true).queue,/搜索并灭火/);
 assert.doesNotMatch(context.checkpointAlignmentView(r,false).progress,/#9|0\.7/);
 r.phase='EXITING';assert.match(context.checkpointAlignmentView(r,true).progress,/继续检测火焰/);
 r.active=false;r.phase='COMPLETE';
 assert.match(context.checkpointAlignmentView(r,true).progress,/出口已停稳/);
});

test('departure cooldown preserves pending queue and disappears with stale or ended task feedback',()=>{
 const r=sample();r.active=true;
 r.fire_retrigger={cooldown_s:3,cooldown_remaining_s:2.4,queued_targets_exempt:true};
 const view=context.checkpointAlignmentView(r,true);
 assert.match(view.progress,/新目标停车冷却 2\.4 秒/);
 assert.match(view.progress,/已确认队列继续处理，已处理目标不重复排队/);
 assert.match(view.queue,/#2 待对齐 → #9 待对齐/);
 assert.doesNotMatch(context.checkpointAlignmentView(r,false).progress,/冷却|2\.4/);
 r.active=false;assert.doesNotMatch(context.checkpointAlignmentView(r,true).progress,/冷却/);
 r.active=true;
 for(const value of [0,-1,NaN,Infinity,'2.4',31]){
  r.fire_retrigger.cooldown_remaining_s=value;
  assert.doesNotMatch(context.checkpointAlignmentView(r,true).progress,/冷却/);
 }
});
