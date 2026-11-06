'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const C=require('../web/fire_commissioning.js');
const near=(actual,expected,tolerance=1e-6)=>assert.ok(Math.abs(actual-expected)<tolerance,`${actual} != ${expected}`);
const sample=(id='A')=>({measurement_id:id,measured:false,nominal_pan_deg:0,nominal_tilt_deg:10,nozzle_distance_m:.7,angle_correction_deg:[0,1],measured_impact_error_m:[0,.002,.003],measured_encoder_error_deg:[.01,.02]});

test('embedded schema snapshots match all four existing templates exactly',()=>{
  for(const key of Object.keys(C.TEMPLATES))assert.deepEqual(C.TEMPLATES[key],JSON.parse(fs.readFileSync(path.join(__dirname,'../config',key+'.template.json'),'utf8')));
});
test('blank draft has no invented measured geometry, stereo or water values',()=>{
  const d=C.blank();assert.equal(d.configs.fixed_camera_aim.R_camera_to_gimbal,null);assert.equal(d.configs.stereo.K1,null);assert.deepEqual(d.configs.fixed_camera_landing.measurements,[]);assert.deepEqual(C.validate(d),[]);assert.ok(C.validate(d,true).length>0);
});
test('preset places camera in front of and above nozzle, with unverified flags',()=>{
  const d=C.preset(),g=d.configs.fixed_camera_aim;
  assert.deepEqual(g.t_camera_origin_in_gimbal_m,[.05,0,.08]);assert.deepEqual(g.axes.pan.limits_deg,[-90,90]);assert.deepEqual(g.axes.tilt.limits_deg,[0,60]);assert.equal(g.calibrated,false);assert.equal(g.axis_reference_verified,false);assert.equal(d.configs.stereo.calibrated,false);assert.deepEqual(d.configs.fixed_camera_landing.measurements,[]);
  const result=C.simulate(g,[0,0,.7]);near(result.actuator_angles_deg[0],0);near(result.actuator_angles_deg[1],Math.atan2(.08,.75)*180/Math.PI);near(result.nozzle_distance_m,Math.hypot(.75,.08));assert.equal(result.pump_allowed,false);assert.equal(result.physical_hit_verified,false);
});
test('optical target right requires negative pan, target high requires positive tilt',()=>{
  const g=C.preset().configs.fixed_camera_aim;
  assert.ok(C.simulate(g,[.1,0,.7]).actuator_angles_deg[0]<0);
  assert.ok(C.simulate(g,[0,-.1,.7]).actuator_angles_deg[1]>C.simulate(g,[0,0,.7]).actuator_angles_deg[1]);
});
test('rotation helper uses explicit base XYZ convention and proper optical mapping',()=>{
  assert.deepEqual(C.rotation(0,0,0),[[0,0,1],[-1,0,0],[0,-1,0]]);
  const d=C.preset();d.configs.fixed_camera_aim.R_camera_to_gimbal=C.rotation(7,-3,12);assert.deepEqual(C.validate(d),[]);
  assert.throws(()=>C.rotation(null,0,0),/有限数字/);
});
test('non-orthogonal/reflected matrices and non-unit nozzle direction are rejected',()=>{
  const d=C.preset();d.configs.fixed_camera_aim.R_camera_to_gimbal=[[1,0,0],[0,1,0],[0,0,-1]];assert.throws(()=>C.normalize(d),/旋转矩阵/);
  d.configs.fixed_camera_aim.R_camera_to_gimbal=C.rotation(0,0,0);d.configs.fixed_camera_aim.nozzle_direction_in_tilt=[2,0,0];assert.throws(()=>C.normalize(d),/单位方向/);
});
test('zero offset and reversed motor directions produce correct actuator coordinates',()=>{
  const g=C.preset().configs.fixed_camera_aim;
  g.axes.pan={zero_angle_deg:5,positive_direction:-1,limits_deg:[-90,90]};g.axes.tilt={zero_angle_deg:30,positive_direction:-1,limits_deg:[0,60]};
  const r=C.simulate(g,[.1,0,.7]);assert.ok(r.actuator_angles_deg[0]>5);assert.ok(r.actuator_angles_deg[1]<30);
  const f=C.forward(g,r.actuator_angles_deg);near(Math.hypot(...r.target_gimbal_m.map((v,i)=>v-f.origin[i]-r.nozzle_distance_m*f.direction[i])),0,1e-5);
});
test('noncoincident pivots and nozzle exit are honored by inverse rehearsal',()=>{
  const g=C.preset().configs.fixed_camera_aim;g.tilt_origin_in_pan_m=[.02,0,.03];g.nozzle_origin_in_tilt_m=[.04,0,0];
  const q=[20,15],f=C.forward(g,q),target=f.origin.map((x,i)=>x+.7*f.direction[i]),relative=target.map((x,i)=>x-g.t_camera_origin_in_gimbal_m[i]);
  const optical=[-relative[1],-relative[2],relative[0]],result=C.simulate(g,optical);
  near(result.actuator_angles_deg[0],q[0],.001);near(result.actuator_angles_deg[1],q[1],.001);near(result.nozzle_distance_m,.7,1e-5);
});
test('incomplete, behind-camera, out-of-range and mechanically unreachable targets fail closed',()=>{
  assert.throws(()=>C.simulate(C.blank().configs.fixed_camera_aim,[0,0,.7]),/尚未填写/);
  const g=C.preset().configs.fixed_camera_aim;
  assert.throws(()=>C.simulate(g,[0,0,-.7]),/相机前方/);assert.throws(()=>C.simulate(g,[0,0,3]),/距离范围/);assert.throws(()=>C.simulate(g,[0,.5,.7]),/没有满足/);
});
test('import never propagates calibrated, verified, measured, or enabled flags',()=>{
  const d=C.preset();d.calibrated=true;d.pump_allowed=true;d.hardware_written=true;d.configs.fixed_camera_aim.calibrated=true;d.configs.fixed_camera_aim.axis_reference_verified=true;d.configs.stereo_runtime.enabled=true;d.configs.stereo_runtime.metric_depth_verified=true;d.configs.fixed_camera_landing.measurements=[{...sample(),measured:true}];
  const out=C.importDocument(JSON.stringify(d));assert.equal(out.calibrated,false);assert.equal(out.pump_allowed,false);assert.equal(out.hardware_written,false);assert.equal(out.configs.fixed_camera_aim.calibrated,false);assert.equal(out.configs.fixed_camera_aim.axis_reference_verified,false);assert.equal(out.configs.stereo_runtime.enabled,false);assert.equal(out.configs.stereo_runtime.metric_depth_verified,false);assert.equal(out.configs.fixed_camera_landing.measurements[0].measured,false);assert.equal(d.calibrated,true);
});
test('individual stereo tool result imports measured numbers but remains disabled',()=>{
  const stereo={...C.TEMPLATES.stereo,calibrated:true,calibration_id:'actual-import',image_size:[1280,1088],K1:[[800,0,640],[0,800,544],[0,0,1]],K2:[[800,0,640],[0,800,544],[0,0,1]],D1:[0,0,0,0,0],D2:[0,0,0,0,0],R:[[1,0,0],[0,1,0],[0,0,1]],T:[-.06,0,0]};
  const out=C.importDocument(JSON.stringify(stereo),C.preset());assert.deepEqual(out.configs.stereo.T,[-.06,0,0]);assert.equal(out.configs.stereo.calibrated,false);assert.equal(out.configs.fixed_camera_aim.t_camera_origin_in_gimbal_m[2],.08);
});
test('import rejects unknown shape, prototype keys, nonfinite numbers and oversized input',()=>{
  assert.throws(()=>C.importDocument('{}'),/请选择/);assert.throws(()=>C.importDocument('{"__proto__":{"polluted":true}}'),/不支持/);assert.throws(()=>C.importDocument('{"mode":"fixed_camera_3d","max_speed_deg_s":1e999}'),/非有限/);assert.throws(()=>C.importDocument(' '.repeat(2*1024*1024+1)),/2MB/);assert.equal({}.polluted,undefined);
});
test('measurement rows require errors, unique IDs and explicit validation command angles',()=>{
  C.validateRow(sample(),'measurements');assert.throws(()=>C.validateRow({...sample(),measured_impact_error_m:null},'measurements'),/误差/);assert.throws(()=>C.validateRow(sample(),'independent_validation'),/角度/);
  const d=C.blank();d.configs.fixed_camera_landing.measurements=[sample(),sample()];assert.throws(()=>C.normalize(d),/编号重复/);
});
test('grid completeness catches missing corners and duplicate coordinates',()=>{
  const l=C.blank().configs.fixed_camera_landing;let i=0;
  for(const p of [-10,10])for(const t of [10,20])for(const d of [.5,1])l.measurements.push({...sample(String(i++)),nominal_pan_deg:p,nominal_tilt_deg:t,nozzle_distance_m:d});
  assert.equal(C.gridSummary(l).complete,true);assert.equal(C.gridSummary(l).cells,1);l.measurements.pop();assert.equal(C.gridSummary(l).complete,false);l.measurements.push({...l.measurements[0],measurement_id:'dup'});assert.equal(C.gridSummary(l).complete,false);
});
test('in-memory edits do not mutate templates or other newly created drafts',()=>{
  const d=C.preset();d.configs.fixed_camera_aim.axes.pan.limits_deg[0]=-20;assert.equal(C.preset().configs.fixed_camera_aim.axes.pan.limits_deg[0],-90);assert.equal(C.TEMPLATES.fixed_camera_aim.axes.pan.limits_deg,null);
});
test('original water evidence and reported measured status survive draft export and reimport',()=>{
  const original={...C.TEMPLATES.fixed_camera_landing,calibrated:true,measurements:[{...sample('real-001'),measured:true}]};
  const text=JSON.stringify(original),out=C.importDocument(text,C.preset());
  assert.equal(out.configs.fixed_camera_landing.measurements[0].measured,false);
  assert.equal(out.source_records[0].original_json,text);
  assert.equal(JSON.parse(out.source_records[0].original_json).measurements[0].measured,true);
  assert.equal(out.measurement_claims[0].source_claimed_measured,true);
  const again=C.importDocument(JSON.stringify(out));assert.deepEqual(again,out);assert.equal(again.source_records.length,1);
});
test('import preserves unrelated geometry notes, values and saved water records',()=>{
  const d=C.preset();d.evidence.geometry='人工记录：尺量8.2cm';d.configs.fixed_camera_aim.t_camera_origin_in_gimbal_m=[.051,0,.082];d.configs.fixed_camera_landing.measurements=[sample('saved')];
  const out=C.importDocument(JSON.stringify(C.TEMPLATES.stereo),d);
  assert.equal(out.evidence.geometry,d.evidence.geometry);assert.deepEqual(out.configs.fixed_camera_aim,d.configs.fixed_camera_aim);assert.deepEqual(out.configs.fixed_camera_landing.measurements,d.configs.fixed_camera_landing.measurements);
});
test('unsafe imported tolerances are rejected before simulation',()=>{
  const d=C.preset();d.configs.fixed_camera_aim.line_tolerance_m=2;assert.throws(()=>C.normalize(d),/line_tolerance_m/);
  d.configs.fixed_camera_aim.line_tolerance_m=null;assert.throws(()=>C.normalize(d),/不能为空/);
});
test('partial geometry import preserves user fields omitted from the source',()=>{
  const d=C.preset();d.configs.fixed_camera_aim.calibration_id='typed-id';d.configs.fixed_camera_aim.tilt_origin_in_pan_m=[.02,0,.03];
  const out=C.importDocument('{"mode":"fixed_camera_3d","t_camera_origin_in_gimbal_m":[0.051,0,0.082]}',d);
  assert.equal(out.configs.fixed_camera_aim.calibration_id,'typed-id');assert.deepEqual(out.configs.fixed_camera_aim.tilt_origin_in_pan_m,[.02,0,.03]);assert.deepEqual(out.configs.fixed_camera_aim.t_camera_origin_in_gimbal_m,[.051,0,.082]);
});
