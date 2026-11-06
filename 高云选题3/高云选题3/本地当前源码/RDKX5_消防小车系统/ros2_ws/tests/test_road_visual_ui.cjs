'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs'),path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../web/presentation.js'),'utf8');
function harness(){
  const scope={finite:Number.isFinite,connected:()=>true,state:{stream:true},roadView:{status:'live',frame:{},age:.1,at:1000},performance:{now:()=>1000}};
  vm.runInNewContext(source.slice(source.indexOf('  function roadFrameFresh()'),source.indexOf('  function drawRoadFrame()')),scope);
  return scope;
}
function frame(){
  const identity={valid:true,capture_ros_s:1000,frame_id:'physical-left:1000000000000'};
  return {...identity,width:640,height:544,coordinate_frame:'raw_physical_left',road:{...identity,detected:true,bbox:[48,102,432,306],image_size:[480,408],stripe_count:6},traffic:{...identity,status:'green',fixture_bbox:[100,30,180,200],image_size:[640,544]}};
}
test('only traffic boxes are shown, even if an old peer still includes a road result',()=>{
  const h=harness(),boxes=h.roadOverlayBoxes(frame());assert.equal(boxes.length,1);
  assert.deepEqual(JSON.parse(JSON.stringify(boxes[0])),{x:100,y:30,w:80,h:170,label:'绿灯',color:'#30e49b'});
});
test('rectified image, mismatched captures, invalid observations and synthetic results never get boxes',()=>{
  const h=harness();
  for(const mutate of [f=>f.coordinate_frame='rectified',f=>f.synthetic=true,f=>{f.road.capture_ros_s--;f.traffic.frame_id='other';},f=>{f.road.valid=false;f.traffic.synthetic=true;}]){
    const f=frame();mutate(f);assert.equal(h.roadOverlayBoxes(f).length,0);
  }
});
test('negative detections and malformed coordinates do not fabricate a target',()=>{
  const h=harness(),f=frame();f.road.detected=false;f.traffic.status='unknown';assert.equal(h.roadOverlayBoxes(f).length,0);
  f.road.detected=true;f.road.bbox=[1,2,NaN,20];assert.equal(h.roadOverlayBoxes(f).length,0);
});
test('expiry, disabled images, stale transport and lost connection hide live results',()=>{
  const h=harness();assert.ok(h.roadFrameFresh());h.performance.now=()=>2501;assert.ok(!h.roadFrameFresh());
  h.performance.now=()=>1000;h.state.stream=false;assert.ok(!h.roadFrameFresh());h.state.stream=true;
  h.roadView.status='stale';assert.ok(!h.roadFrameFresh());h.roadView.status='live';h.connected=()=>false;assert.ok(!h.roadFrameFresh());
});
test('text cards also reject mismatched frame identities and empty frames',()=>{
  const h=harness(),f=frame();assert.equal(h.roadFrameResult(f,'traffic').status,'green');
  f.traffic.frame_id='old';assert.equal(Object.keys(h.roadFrameResult(f,'traffic')).length,0);assert.equal(Object.keys(h.roadFrameResult(null,'traffic')).length,0);
});

test('current page exposes traffic command and no removed crosswalk widgets',()=>{
  const page=fs.readFileSync(path.join(__dirname,'../web/presentation.html'),'utf8');
  assert.doesNotMatch(page,/crosswalkLabel|road-box-key/);
  assert.match(page,/data-command="开启交通任务"/);
  assert.match(source,/125-\(performance\.now\(\)-start\)/);
});
