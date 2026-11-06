'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict'),test=require('node:test');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const code=source.slice(source.indexOf('  function exactFrameRegistry('),source.indexOf('  // CENTERING_OVERLAY_MODEL_END'));
const helper=source.slice(source.indexOf('  function searchPumpRecoveryText('),source.indexOf('  function checkpointAlignmentView('));
const h={reason:value=>value};vm.runInNewContext(helper+code,h);
test('search visual dropout reports route recovery without claiming centered completion',()=>{
 const frame={token:'f1',coordinate_frame:'left',width:1280,height:1088};
 const input={extensionFresh:true,frameFresh:true,elapsed:0,extension:{task:'semantic',real_uart:true,status_fresh:true,
  fpga_host:{selected:true,scope:true,link_ok:true,fresh:true},region_search:{active:true,execute_fire:false,
   phase:'CENTER_RETURN',visual_degraded:true,centering:{enabled:true,active:true,phase:'CENTER_RETURN',
    completed_count:0,total_count:1,queue:[{target_id:1,status:'skipped',reason:'search_only_visual_unavailable_skip_observation'}]}}}};
 const result=h.centeringOverlayModel(frame,input);
 assert.match(result.status,/视觉暂缺.*恢复原路线/);
 assert.doesNotMatch(result.status,/已停稳居中|居中剩余/);
 assert.equal(result.boxes.length,0);
 input.extension.task='vision';assert.equal(h.centeringOverlayModel(frame,input).visible,false);
});

test('a live bounded pump readback hold preserves the task instead of announcing cancellation',()=>{
 const r={active:true,execute_fire:false,phase:'NAVIGATING',pump_readback_recovery:{active:true},centering:{enabled:true}};
 const input={extensionFresh:true,frameFresh:true,extension:{task:'semantic',real_uart:true,status_fresh:true,
  fpga_host:{selected:true,scope:true,link_ok:true,fresh:true},region_search:r}};
 const result=h.centeringOverlayModel({token:'f1',coordinate_frame:'left',width:1280,height:1088},input);
 assert.match(result.status,/关水回读.*原路线保留.*自动继续/);
 assert.doesNotMatch(result.status,/沿路线巡检|任务已取消|任务已完成/);
 r.phase='CENTER_MOVING';r.centering.move_direction='forward';
 assert.doesNotMatch(h.centeringOverlayModel({token:'f1'},input).status,/前进对齐/);
 r.phase='NAVIGATING';
 r.pump_readback_recovery.active=false;
 assert.match(h.centeringOverlayModel({token:'f1'},input).status,/沿路线巡检/);
 r.active=false;r.pump_readback_recovery.active=true;
 assert.equal(h.searchPumpRecoveryText(r),'');
});
