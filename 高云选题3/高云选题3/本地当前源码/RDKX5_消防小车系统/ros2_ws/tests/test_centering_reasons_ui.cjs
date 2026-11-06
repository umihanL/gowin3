'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict'),test=require('node:test');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
const assignments=Array.from(source.matchAll(/Object\.assign\(reasons,\{[\s\S]*?\}\);/g),m=>m[0]).join('\n');
const h={};vm.runInNewContext('const reasons={};\n'+assignments+'\n'+source.slice(source.indexOf('  const reason ='),source.indexOf('  const state ='))+'\nglobalThis.explain=reason;',h);

test('old FPGA firmware gives the missing jog protocol and a concrete upgrade action',()=>{
 for(const code of ['semantic_region_bidirectional_centering_firmware_required','centering_retained_jog_firmware_required']){
  const message=h.explain(code);assert.match(message,/FPGA.*0x3C\/0x3D.*烧录.*固件/);assert.doesNotMatch(message,/firmware_required/);
 }
});
test('corridor and return budget explain why a flame is skipped without claiming extinguishing',()=>{
 assert.match(h.explain('centering_straight_corridor_exhausted'),/弯道或路线边界.*直线微调距离/);
 assert.match(h.explain('centering_distance_budget_reserved_for_return'),/保留返回原暂停位置/);
 assert.match(h.explain('fire_centering_target_lost'),/已跳过此目标.*继续处理本批其他目标/);
 assert.doesNotMatch(h.explain('fire_centering_target_lost'),/已扑灭|已灭火|全部完成/);
});
test('nested failure prefixes retain the actual Chinese cause',()=>{
 assert.match(h.explain('fire_centering_rejected:centering_begin_requires_paused_route_and_pump_off'),/火焰居中请求未执行：等待原导航路线暂停.*实际停稳.*水泵确认关闭/);
 assert.match(h.explain('fire_centering_anchor_return_unavailable:centering_return_requires_fresh_pose'),/无法恢复原暂停位置，任务已停止：.*新鲜惯导位姿.*FPGA 回包/);
 assert.match(h.explain('region_leg_rejected:fire_centering_rejected:centering_end_requires_measured_original_anchor'),/区域航段未启动：火焰居中请求未执行：尚未确认回到原暂停位置并停稳/);
});
test('timeouts and mismatched position feedback explain the reason for stopping',()=>{
 assert.match(h.explain('fire_centering_batch_timeout'),/超过 180 秒.*任务已停止.*重新下达任务/);
 assert.match(h.explain('centering_jog_ack_timeout'),/未确认接收.*检查串口/);
 assert.match(h.explain('centering_same_route_pose_lost'),/同一路线版本与进度.*停止微调/);
 assert.match(h.explain('centering_jog_endpoint_not_measured'),/惯导尚未确认到达/);
 assert.equal(h.explain('fire_centering_rejected:centering_fpga_jog_rejected_2'),'火焰居中请求未执行：FPGA 拒绝本次前后微调，返回码 2');
 assert.equal(h.explain('centering_fpga_aborted_5'),'FPGA 已中止本次前后微调，返回原因码 5');
});
test('reacquiring and one second observation are distinct from target completion',()=>{
 assert.match(h.explain('returning_to_anchor_to_reacquire_queued_flame'),/返回原观察位置.*重找/);
 assert.match(h.explain('rechecking_queued_flame_at_original_view'),/等待新鲜多帧/);
 assert.match(h.explain('centered_stationary_one_second_required'),/正在累计连续 1 秒/);
 assert.match(h.explain('flame_centered_and_stopped_for_one_second'),/本目标.*1 秒.*后续目标/);
 assert.equal(h.explain('fpga_command_rejected_3'),'FPGA 拒绝运动指令，返回码 3');
 assert.equal(h.explain(''),'等待反馈');
});
test('post-stop candidate collection explains that stopping is preparation, not alignment completion',()=>{
 assert.match(h.explain('collecting_post_stop_confirmations'),/新的连续画面.*停车不表示已居中完成/);
 assert.match(h.explain('waiting_post_stop_stable_flame_evidence'),/单帧高置信度还不能开始对齐/);
 assert.match(h.explain('no_flame_visible_after_verified_stop'),/未重新确认火焰.*恢复原路线/);
 assert.match(h.explain('flame_batch_confirmation_timeout'),/未在时限内稳定确认.*恢复原路线/);
 assert.match(h.explain('waiting_new_batch_candidate_confirmation'),/新候选.*连续多帧/);
});
test('actual retained corridor failures have specific Chinese explanations without claiming motion or completion',()=>{
 for(const [code,pattern] of [
  ['centering_corridor_static_obstacle',/地图障碍或道路边界/],
  ['centering_corridor_live_barrier',/路障相交/],
  ['centering_live_barrier_invalid',/路障数据无效/],
  ['centering_anchor_at_route_branch',/路线分叉/],
  ['centering_anchor_outside_retained_route_corridor',/不在原路线/],
  ['centering_retained_straight_span_ended',/沿当前车头可直线移动的边界/],
  ['centering_local_distance_limit',/局部微调距离上限/],
  ['centering_traversability_invalid',/栅格数据不可用/],
  ['centering_traversability_map_identity_mismatch',/当前 FPGA 地图不一致/],
  ['centering_traversability_run_invalid',/无效区段/],
  ['centering_traversability_overlapping_runs',/区段重叠/],
  ['centering_traversability_hash_mismatch',/校验不一致/],
  ['centering_route_graph_identity_mismatch',/地图版本不一致/],
  ['centering_route_graph_invalid',/连接关系无效/]]){
   assert.match(h.explain(code),pattern);assert.doesNotMatch(h.explain(code),/已居中|已灭火|centering_/);
  }
 assert.match(h.explain('fire_centering_anchor_return_unavailable:centering_corridor_live_barrier'),/无法恢复原暂停位置.*路障相交/);
});
