/* Local measurement notebook. No network, actuator, or configuration-write API. */
(function (root, factory) {
  'use strict';
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else { root.FireCommissioning = api; api.mount(document); }
})(typeof globalThis === 'object' ? globalThis : this, function () {
  'use strict';
  const SCHEMA = 'fire_commissioning_draft/v1';
  const TEMPLATES = {"fixed_camera_aim":{"mode":"fixed_camera_3d","calibrated":false,"calibration_id":null,"stereo_calibration_id":null,"source_frame":"undistorted_left_optical","image_size_wh":null,"translation_unit":"m","R_camera_to_gimbal":null,"t_camera_origin_in_gimbal_m":null,"axis_model":"serial_pan_z_tilt_minus_y_with_offsets","tilt_origin_in_pan_m":null,"nozzle_origin_in_tilt_m":null,"nozzle_direction_in_tilt":null,"actuator_reference_id":null,"axis_reference_verified":false,"axes":{"pan":{"zero_angle_deg":null,"positive_direction":null,"limits_deg":null},"tilt":{"zero_angle_deg":null,"positive_direction":null,"limits_deg":null}},"valid_range_m":null,"max_speed_deg_s":12.0,"max_step_deg":0.6,"angle_tolerance_deg":0.3,"line_tolerance_m":0.002,"min_forward_ray_m":0.05,"stable_lock_s":0.35,"max_frame_age_s":0.5,"max_encoder_age_s":0.2,"max_target_shift_m":0.02,"notes":"UNMEASURED: fixed camera on vehicle, only nozzle rotates. Base x forward/y left/z up, pan +z, positive tilt about -y. p_gimbal=R_camera_to_gimbal*p_camera+t_camera_origin_in_gimbal_m. At geometric pan=tilt=0, explicitly measure tilt pivot offset and nozzle exit/direction; zero offsets are valid ONLY when physically measured. Geometric angle=(actuator angle-zero_angle_deg)*positive_direction. Match actuator_reference_id to servo.reference_id and exact limits in actuator status. Match source optical frame, image size and measured stereo calibration ID. No pixel-centering or fixed-distance fallback. Straight nozzle geometry can lock encoder angles but never authorizes water without measured impact calibration."},"fixed_camera_landing":{"mode":"fixed_camera_measured_angular_grid","calibrated":false,"calibration_id":null,"measurement_session_id":null,"geometry_calibration_id":null,"stereo_calibration_id":null,"actuator_reference_id":null,"pump_setting_id":null,"camera_nozzle_mount_id":null,"angle_convention":"actuator_joint_delta_deg","distance_reference":"nominal_nozzle_forward_ray_m","measurement_unit":"m","acceptance_radius_m":null,"water_impact_angle_tolerance_deg":null,"measurements":[],"independent_validation":[],"notes":"UNMEASURED. Separate from moving-camera pixel tables. Measure angular corrections at a complete nominal-pan/nominal-tilt/nozzle-distance grid (at least 2 values per axis). Every row requires a unique measurement_id, measured=true, nominal_pan_deg, nominal_tilt_deg, nozzle_distance_m, angle_correction_deg=[pan_delta,tilt_delta] and actual measured_impact_error_m=[x,y,z]. Every interpolation cell also needs a distinct strictly-interior validation measurement with commanded_angles_deg equal to the interpolated target and actual measured_impact_error_m within the declared acceptance radius. These are actual water tests at the same installation and pump setting; no extrapolation or pressure/gravity guessing. Bind all calibration/profile/reference IDs after measurement. No sample values are supplied because none have been physically measured."},"stereo":{"calibrated":false,"calibration_id":null,"camera_model":"pinhole","image_size":null,"translation_unit":"m","K1":null,"D1":null,"K2":null,"D2":null,"R":null,"T":null,"notes":"Use measured GS130W calibration from tools/calibrate_stereo.py. Never replace null with guessed parameters. image_size is single-eye [width,height]. T maps left optical coordinates to right optical coordinates and uses metres."},"stereo_runtime":{"enabled":false,"physical_eye_mapping_verified":false,"synchronization_verified":false,"metric_depth_verified":false,"verification_id":null,"physical_left_topic":"/image_right_raw","physical_right_topic":"/image_left_raw","input_rotation_deg":90.0,"process_fps":5.0,"max_frame_age_s":0.5,"max_pair_skew_s":0.012,"queue_size":16,"opencv_threads":2,"require_fire_mode":true,"matcher":{},"notes":"Only mark verified after checking eye order, raw image rotation, exposure synchronization and held-out measured distances. Timestamp pairing alone is not synchronization. Fire mode bounds CPU work; image resolution and the YOLO/preview pipeline remain unchanged."}};
  const clone = x => JSON.parse(JSON.stringify(x));
  const finite = x => typeof x === 'number' && Number.isFinite(x);
  const vector = (x, n) => Array.isArray(x) && x.length === n && x.every(finite);
  const norm = a => Math.hypot(...a);
  const add = (a, b) => a.map((v, i) => v + b[i]);
  const sub = (a, b) => a.map((v, i) => v - b[i]);
  const dot = (a, b) => a.reduce((v, x, i) => v + x * b[i], 0);
  const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const mv = (a, b) => a.map(row => dot(row, b));
  const mm = (a, b) => a.map(row => b[0].map((_, j) => dot(row, b.map(r => r[j]))));
  const rad = x => x*Math.PI/180;
  const OPTICAL = [[0,0,1],[-1,0,0],[0,-1,0]];
  function rotation(roll, pitch, yaw) {
    if (![roll,pitch,yaw].every(finite)) throw Error('安装角必须是有限数字');
    const [a,b,c] = [roll,pitch,yaw].map(rad), ca=Math.cos(a),sa=Math.sin(a),cb=Math.cos(b),sb=Math.sin(b),cc=Math.cos(c),sc=Math.sin(c);
    return mm(mm(mm([[cc,-sc,0],[sc,cc,0],[0,0,1]],[[cb,0,sb],[0,1,0],[-sb,0,cb]]),[[1,0,0],[0,ca,-sa],[0,sa,ca]]),OPTICAL);
  }
  function blank() {
    return {schema:SCHEMA, simulation_only:true, calibrated:false, hardware_written:false,
      pump_allowed:false, physical_hit_verified:false, configs:clone(TEMPLATES),
      evidence:{geometry:'',axes:'',stereo:'',water:''}, assumptions:[], source_records:[], measurement_claims:[]};
  }
  function preset() {
    const draft=blank(), g=draft.configs.fixed_camera_aim;
    Object.assign(g,{R_camera_to_gimbal:clone(OPTICAL),t_camera_origin_in_gimbal_m:[.05,0,.08],
      tilt_origin_in_pan_m:[0,0,0],nozzle_origin_in_tilt_m:[0,0,0],nozzle_direction_in_tilt:[1,0,0],
      image_size_wh:[1280,1088],valid_range_m:[.2,2],axes:{pan:{zero_angle_deg:0,positive_direction:1,limits_deg:[-90,90]},tilt:{zero_angle_deg:0,positive_direction:1,limits_deg:[0,60]}}});
    draft.assumptions=['水平−90°..90°、俯仰0°..60°为用户确认的工作范围，仍需核对实际机械与执行器限位。',
      '假设喷嘴轴心在相机后5cm、下8cm，所以相机在云台中为[0.05,0,0.08]m。',
      '假设两轴与喷嘴出口原点重合、相机正向对齐、喷嘴方向+x、执行器零点0°和正向+1。',
      '1280×1088与0.2..2m只供演示；双目、水流、轴向和安装均未实测。'];
    draft.evidence.geometry=draft.assumptions.slice(1).join('\n');
    draft.evidence.axes=draft.assumptions[0]+'\n零点0°、正向+1为仿真假设，未实测。';
    return draft;
  }
  function safeTree(value, depth=0) {
    if (depth>30) throw Error('JSON层级过深');
    if (typeof value==='number' && !finite(value)) throw Error('JSON包含非有限数字');
    if (!value || typeof value!=='object') return;
    for (const key of Object.keys(value)) {
      if (['__proto__','constructor','prototype'].includes(key)) throw Error('JSON包含不支持的字段');
      safeTree(value[key],depth+1);
    }
  }
  function forceUnverified(value) {
    if (!value || typeof value!=='object') return;
    for (const key of Object.keys(value)) {
      if (/^(calibrated|enabled|measured|hardware_written|pump_allowed|pump_candidate)$/.test(key) || /_verified$/.test(key)) value[key]=false;
      else forceUnverified(value[key]);
    }
  }
  function mergeShape(base, value) {
    if (!value || typeof value!=='object' || Array.isArray(value)) throw Error('配置必须是JSON对象');
    const out=clone(base);
    for (const key of Object.keys(value)) {
      if (value[key] && typeof value[key]==='object' && !Array.isArray(value[key]) && out[key] && typeof out[key]==='object' && !Array.isArray(out[key])) out[key]=mergeShape(out[key],value[key]);
      else out[key]=clone(value[key]);
    }
    return out;
  }
  function normalize(input) {
    safeTree(input);
    if (!input || typeof input!=='object' || Array.isArray(input) || input.schema!==SCHEMA || !input.configs) throw Error('不支持的完整草稿格式');
    const out=blank();
    for (const key of Object.keys(TEMPLATES)) out.configs[key]=mergeShape(TEMPLATES[key],input.configs[key]||{});
    for (const key of Object.keys(out.evidence)) if (typeof input.evidence?.[key]==='string') out.evidence[key]=input.evidence[key];
    if (Array.isArray(input.assumptions) && input.assumptions.every(x=>typeof x==='string')) out.assumptions=clone(input.assumptions);
    if (input.source_records !== undefined) {
      if (!Array.isArray(input.source_records) || !input.source_records.every(x=>x && typeof x.original_json==='string' && typeof x.imported_at_utc==='string' && typeof x.source_kind==='string')) throw Error('原始资料追溯记录格式错误');
      out.source_records=clone(input.source_records);
    }
    if (input.measurement_claims !== undefined) {
      if (!Array.isArray(input.measurement_claims) || !input.measurement_claims.every(x=>x && typeof x.measurement_id==='string' && typeof x.source_claimed_measured==='boolean')) throw Error('实测声明记录格式错误');
      out.measurement_claims=clone(input.measurement_claims);
    }
    forceUnverified(out);
    const errors=validate(out,false);
    if (errors.length) throw Error(errors.join('\n'));
    return out;
  }
  function importDocument(text, current=blank()) {
    if (text.length>2*1024*1024) throw Error('JSON请小于2MB');
    const doc=JSON.parse(text);safeTree(doc);
    if (doc?.schema===SCHEMA) {
      const out=normalize(doc);
      const claimed=value=>value && typeof value==='object' && Object.entries(value).some(([k,v])=>((k==='measured'||k==='calibrated'||k==='enabled'||k.endsWith('_verified'))&&v===true)||claimed(v));
      if(claimed(doc.configs))out.source_records.push({source_kind:'original_draft_with_claims',imported_at_utc:new Date().toISOString(),original_json:text});
      return out;
    }
    let key;
    if (doc?.mode==='fixed_camera_3d') key='fixed_camera_aim';
    else if (doc?.mode==='fixed_camera_measured_angular_grid') key='fixed_camera_landing';
    else if (doc && Object.hasOwn(doc,'K1') && Object.hasOwn(doc,'K2')) key='stereo';
    else if (doc && Object.hasOwn(doc,'physical_left_topic') && Object.hasOwn(doc,'physical_right_topic')) key='stereo_runtime';
    else throw Error('请选择本页草稿或现有固定相机/落点/双目JSON');
    const out=clone(current);out.configs[key]=mergeShape(out.configs[key],doc);
    out.source_records.push({source_kind:key,imported_at_utc:new Date().toISOString(),original_json:text});
    if(key==='fixed_camera_landing')for(const kind of ['measurements','independent_validation'])for(const row of doc[kind]||[]) {
      out.measurement_claims=out.measurement_claims.filter(x=>x.measurement_id!==row.measurement_id);
      out.measurement_claims.push({measurement_id:row.measurement_id,source_claimed_measured:row.measured===true,source:'imported_original'});
    }
    return normalize(out);
  }
  function validate(draft, complete=false) {
    const e=[],g=draft.configs.fixed_camera_aim,l=draft.configs.fixed_camera_landing;
    const check=(value, test, name) => { if (value===null) {if(complete)e.push(name+'：尚未填写');} else if(!test(value))e.push(name+'：格式或范围错误'); };
    const R=g.R_camera_to_gimbal;
    check(R,x=>Array.isArray(x)&&x.length===3&&x.every(r=>vector(r,3))&&
      x.every((row,i)=>x.every((r,j)=>Math.abs(dot(row,r)-(i===j?1:0))<1e-6))&&Math.abs(dot(x[0],cross(x[1],x[2]))-1)<1e-6,'相机旋转矩阵');
    for(const key of ['t_camera_origin_in_gimbal_m','tilt_origin_in_pan_m','nozzle_origin_in_tilt_m'])check(g[key],x=>vector(x,3)&&norm(x)<=(key==='t_camera_origin_in_gimbal_m'?5:1),key);
    check(g.nozzle_direction_in_tilt,x=>vector(x,3)&&Math.abs(norm(x)-1)<1e-6,'喷嘴单位方向');
    check(g.image_size_wh,x=>Array.isArray(x)&&x.length===2&&x.every(v=>Number.isInteger(v)&&v>=32),'单眼尺寸');
    check(g.valid_range_m,x=>vector(x,2)&&x[0]>=.05&&x[1]<=10&&x[0]<x[1],'距离范围');
    for(const name of ['pan','tilt']) {
      const axis=g.axes?.[name];if(!axis){e.push(name+'轴参数缺失');continue;}
      check(axis.zero_angle_deg,x=>finite(x)&&Math.abs(x)<=180,name+'零点');
      check(axis.positive_direction,x=>x===1||x===-1,name+'方向');
      check(axis.limits_deg,x=>vector(x,2)&&x[0]>=-180&&x[1]<=180&&x[0]<x[1],name+'限位');
      if(vector(axis.limits_deg,2)&&finite(axis.zero_angle_deg)&&(axis.zero_angle_deg<axis.limits_deg[0]||axis.zero_angle_deg>axis.limits_deg[1]))e.push(name+'零点必须位于限位内');
    }
    if(g.mode!=='fixed_camera_3d'||g.translation_unit!=='m'||g.axis_model!=='serial_pan_z_tilt_minus_y_with_offsets')e.push('固定相机模式、米制单位或轴模型不匹配');
    if(!['undistorted_left_optical','rectified_left_optical','left_camera_optical'].includes(g.source_frame))e.push('不支持的光学来源帧');
    for(const [key,lo,hi] of [['max_speed_deg_s',.1,30],['max_step_deg',.01,2],['angle_tolerance_deg',.01,1],['line_tolerance_m',.00001,.01],['min_forward_ray_m',.01,.5],['stable_lock_s',.1,2],['max_frame_age_s',.05,.5],['max_encoder_age_s',.03,.4],['max_target_shift_m',.001,.1]]) {
      if(g[key]===null)e.push(key+'：运行阈值不能为空');else check(g[key],x=>finite(x)&&x>=lo&&x<=hi,key);
    }
    if(l.mode!=='fixed_camera_measured_angular_grid'||l.angle_convention!=='actuator_joint_delta_deg'||l.distance_reference!=='nominal_nozzle_forward_ray_m'||l.measurement_unit!=='m')e.push('水流记录坐标约定不匹配');
    check(l.acceptance_radius_m,x=>finite(x)&&x>=.0001&&x<=.1,'落点接受半径');
    check(l.water_impact_angle_tolerance_deg,x=>finite(x)&&x>=.001&&x<=.3,'水流角度容差');
    const ids=new Set();
    for(const kind of ['measurements','independent_validation']) {
      if(!Array.isArray(l[kind])||l[kind].length>4096){e.push(kind+'必须为不超过4096条的数组');continue;}
      l[kind].forEach((row,i)=>{
        const prefix=kind+' 第'+(i+1)+'条';
        try {validateRow(row,kind);}catch(err){e.push(prefix+'：'+err.message);return;}
        if(ids.has(row.measurement_id))e.push(prefix+'：记录编号重复');ids.add(row.measurement_id);
      });
    }
    const stereo=draft.configs.stereo;
    for(const key of ['K1','K2','R'])check(stereo[key],x=>Array.isArray(x)&&x.length===3&&x.every(r=>vector(r,3)),'双目 '+key);
    for(const key of ['D1','D2'])check(stereo[key],x=>Array.isArray(x)&&x.length>=4&&x.length<=14&&x.every(finite),'双目 '+key);
    check(stereo.T,x=>vector(x,3),'双目 T');
    if(stereo.translation_unit!=='m')e.push('双目平移单位必须为m');
    return e;
  }
  function validateRow(row,kind) {
    if(!row||typeof row!=='object'||typeof row.measurement_id!=='string'||!row.measurement_id.trim())throw Error('缺少唯一记录编号');
    if(![row.nominal_pan_deg,row.nominal_tilt_deg].every(x=>finite(x)&&Math.abs(x)<=180)||!finite(row.nozzle_distance_m)||row.nozzle_distance_m<.05||row.nozzle_distance_m>10)throw Error('名义角/喷嘴距离越界或缺失');
    const key=kind==='measurements'?'angle_correction_deg':'commanded_angles_deg';
    if(!vector(row[key],2)||row[key].some(x=>Math.abs(x)>(kind==='measurements'?20:180)))throw Error('角度数组格式或范围错误');
    if(!vector(row.measured_impact_error_m,3)||!vector(row.measured_encoder_error_deg,2))throw Error('必须填写实测落点与编码器误差向量，不能把空白当0');
  }
  function gridSummary(landing) {
    const rows=landing.measurements;
    const axes=['nominal_pan_deg','nominal_tilt_deg','nozzle_distance_m'].map(k=>[...new Set(rows.map(r=>r[k]))].sort((a,b)=>a-b));
    const keys=new Set(rows.map(r=>[r.nominal_pan_deg,r.nominal_tilt_deg,r.nozzle_distance_m].join(',')));
    const expected=axes.reduce((n,a)=>n*a.length,1), complete=axes.every(a=>a.length>=2)&&keys.size===rows.length&&keys.size===expected;
    return {rows:rows.length,unique:keys.size,axis_sizes:axes.map(a=>a.length),expected,complete,
      cells:axes.every(a=>a.length>=2)?axes.reduce((n,a)=>n*(a.length-1),1):0,validation_rows:landing.independent_validation.length};
  }
  function forward(g,q) {
    const angles=['pan','tilt'].map((name,i)=>rad((q[i]-g.axes[name].zero_angle_deg)*g.axes[name].positive_direction));
    const [p,t]=angles,cp=Math.cos(p),sp=Math.sin(p),ct=Math.cos(t),st=Math.sin(t);
    const rp=[[cp,-sp,0],[sp,cp,0],[0,0,1]],rt=[[ct,0,-st],[0,1,0],[st,0,ct]];
    return {origin:mv(rp,add(g.tilt_origin_in_pan_m,mv(rt,g.nozzle_origin_in_tilt_m))),direction:mv(rp,mv(rt,g.nozzle_direction_in_tilt))};
  }
  function simulate(g,target) {
    const d=blank();d.configs.fixed_camera_aim=clone(g);
    // Only geometry is required for the explicitly assumed straight-ray rehearsal.
    const errors=validate(d,false);if(errors.length)throw Error(errors.join('\n'));
    for(const key of ['R_camera_to_gimbal','t_camera_origin_in_gimbal_m','tilt_origin_in_pan_m','nozzle_origin_in_tilt_m','nozzle_direction_in_tilt','valid_range_m'])if(g[key]===null)throw Error(key+'尚未填写');
    for(const name of ['pan','tilt'])if(Object.values(g.axes[name]).some(x=>x===null))throw Error(name+'轴尚未填写');
    if(!vector(target,3)||target[2]<=0)throw Error('目标必须是相机前方的有限三维点');
    if(norm(target)<g.valid_range_m[0]||norm(target)>g.valid_range_m[1])throw Error('目标超出填写的距离范围');
    const point=add(mv(g.R_camera_to_gimbal,target),g.t_camera_origin_in_gimbal_m),limits=['pan','tilt'].map(n=>g.axes[n].limits_deg);
    const clamp=q=>q.map((v,i)=>Math.min(limits[i][1],Math.max(limits[i][0],v)));
    const physical=[Math.atan2(point[1],point[0]),Math.atan2(point[2],Math.hypot(point[0],point[1]))].map(x=>x*180/Math.PI);
    const guess=clamp(['pan','tilt'].map((n,i)=>g.axes[n].zero_angle_deg+physical[i]*g.axes[n].positive_direction));
    const residual=q=>{const f=forward(g,q);return cross(f.direction,sub(point,f.origin));};
    const seeds=[guess,limits.map(x=>(x[0]+x[1])/2),...limits[0].flatMap(p=>limits[1].map(t=>[p,t]))];
    let best=null;
    for(let q of seeds) {
      for(let iteration=0;iteration<60;iteration++) {
        const err=residual(q);if(norm(err)<1e-8)break;
        const jac=[0,1].map(i=>{const a=q.slice(),b=q.slice();a[i]+=.001;b[i]-=.001;return sub(residual(a),residual(b)).map(v=>v/.002);});
        const a=dot(jac[0],jac[0])+1e-12,b=dot(jac[0],jac[1]),c=dot(jac[1],jac[1])+1e-12,u=-dot(jac[0],err),v=-dot(jac[1],err),det=a*c-b*b;
        if(det<=1e-25)break;
        const step=[(u*c-b*v)/det,(a*v-b*u)/det].map(x=>Math.max(-15,Math.min(15,x)));
        let improved=false;for(const scale of [1,.5,.25,.1]){const next=clamp(q.map((x,i)=>x+scale*step[i]));if(norm(residual(next))<norm(err)-1e-12){q=next;improved=true;break;}}if(!improved)break;
      }
      const f=forward(g,q),miss=norm(residual(q)),distance=dot(f.direction,sub(point,f.origin));
      if(miss<=g.line_tolerance_m&&distance>=g.min_forward_ray_m&&(!best||miss<best.ray_error_m))best={actuator_angles_deg:q,target_gimbal_m:point,nozzle_distance_m:distance,ray_error_m:miss};
    }
    if(!best)throw Error('当前限位内没有满足直线射线误差门槛的解；未夹紧目标冒充成功');
    return {...best,simulation_only:true,calibrated:false,pump_allowed:false,hardware_written:false,physical_hit_verified:false};
  }
  function mount(doc) {
    const $=id=>doc.getElementById(id);let draft=blank(),editing=null;
    const message=text=>{$('message').textContent=text;};
    const parse=(value,kind,name)=>{if(!value.trim())return null;let result;if(kind==='json'){try{result=JSON.parse(value);}catch{throw Error(name+'：JSON格式错误');}}else result=kind==='number'?Number(value):value.trim();if(kind==='number'&&!finite(result))throw Error(name+'必须是有限数字');return result;};
    const set=(object,path,value)=>{const keys=path.split('.');let o=object;for(const k of keys.slice(0,-1))o=o[k];o[keys.at(-1)]=value;};
    const get=(object,path)=>path.split('.').reduce((o,k)=>o[k],object);
    function read() {
      const next=clone(draft);
      doc.querySelectorAll('[data-path]').forEach(el=>set(next.configs,el.dataset.path,parse(el.value,el.dataset.kind,el.dataset.path)));
      doc.querySelectorAll('[data-evidence]').forEach(el=>{next.evidence[el.dataset.evidence]=el.value;});
      next.configs.stereo=mergeShape(TEMPLATES.stereo,JSON.parse($('stereoJson').value));
      next.configs.stereo_runtime=mergeShape(TEMPLATES.stereo_runtime,JSON.parse($('stereoRuntimeJson').value));
      draft=normalize(next);return draft;
    }
    function review() {
      const missing=validate(draft,true),s=gridSummary(draft.configs.fixed_camera_landing);
      $('review').textContent='状态：未标定 / 未验证 / 输出关闭\n'+(missing.length?'待补充：\n'+missing.join('\n'):'数值结构已填齐；仍需实测证据与后端独立验收。')+'\n\n原始导入资料 '+draft.source_records.length+' 份，已在完整草稿 source_records 保留原文。\n几何、双目、落点与本次编码器参考ID仍需交叉核验。\n检查完整性不等于实测通过。';
      $('gridStatus').textContent=`网格角点 ${s.rows} 条；三轴取值数 ${s.axis_sizes.join(' × ')}；${s.complete?'组合齐全（仅结构检查）':'组合尚不完整或存在重复坐标'}。需要 ${s.cells} 个单元的独立内部验证，已录入 ${s.validation_rows} 条（尚未核验覆盖、插值或误差）。`;
    }
    function renderRows() {
      const tbody=$('rows');tbody.replaceChildren();
      for(const kind of ['measurements','independent_validation'])draft.configs.fixed_camera_landing[kind].forEach((row,index)=>{
        const tr=doc.createElement('tr');
        for(const value of [row.measurement_id+' / '+(kind==='measurements'?'角点':'独立验证'),`${row.nominal_pan_deg}°, ${row.nominal_tilt_deg}° / ${row.nozzle_distance_m} m`,JSON.stringify(row.angle_correction_deg||row.commanded_angles_deg),JSON.stringify(row.measured_impact_error_m)+' / '+JSON.stringify(row.measured_encoder_error_deg)]){const td=doc.createElement('td');td.textContent=value;tr.appendChild(td);}
        const cell=doc.createElement('td');cell.textContent='未验收 ';
        const edit=doc.createElement('button');edit.textContent='编辑';edit.onclick=()=>{editing={kind,index};$('rowType').value=kind;$('measurementId').value=row.measurement_id;$('nominalPan').value=row.nominal_pan_deg;$('nominalTilt').value=row.nominal_tilt_deg;$('distance').value=row.nozzle_distance_m;$('rowAngles').value=JSON.stringify(row.angle_correction_deg||row.commanded_angles_deg);$('impactError').value=JSON.stringify(row.measured_impact_error_m);$('encoderError').value=JSON.stringify(row.measured_encoder_error_deg);$('rowClaim').value=String(draft.measurement_claims.find(x=>x.measurement_id===row.measurement_id)?.source_claimed_measured===true);$('addRow').textContent='保存编辑';};cell.appendChild(edit);
        const remove=doc.createElement('button');remove.textContent='删除';remove.onclick=()=>guard(()=>{read();draft.configs.fixed_camera_landing[kind].splice(index,1);clearRow();renderRows();review();});cell.appendChild(remove);tr.appendChild(cell);tbody.appendChild(tr);
      });
    }
    function render() {
      doc.querySelectorAll('[data-path]').forEach(el=>{const v=get(draft.configs,el.dataset.path);el.value=v===null?'':typeof v==='object'?JSON.stringify(v):String(v);});
      doc.querySelectorAll('[data-evidence]').forEach(el=>{el.value=draft.evidence[el.dataset.evidence];});
      $('stereoJson').value=JSON.stringify(draft.configs.stereo,null,2);$('stereoRuntimeJson').value=JSON.stringify(draft.configs.stereo_runtime,null,2);renderRows();review();
    }
    function clearRow(){editing=null;for(const id of ['measurementId','nominalPan','nominalTilt','distance','rowAngles','impactError','encoderError'])$(id).value='';$('rowClaim').value='false';$('addRow').textContent='保留此条记录';}
    function guard(fn){try{fn();}catch(err){message(err.message);}}
    function download(name,value){const url=URL.createObjectURL(new Blob([JSON.stringify(value,null,2)+'\n'],{type:'application/json'})),a=doc.createElement('a');a.href=url;a.download=name;doc.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);}
    $('preset').onclick=()=>guard(()=>{const current=read(),p=preset();current.configs.fixed_camera_aim=p.configs.fixed_camera_aim;current.assumptions=p.assumptions;current.evidence.geometry=p.evidence.geometry;current.evidence.axes=p.evidence.axes;draft=current;clearRow();render();message('已填入仿真几何预设。现有双目/水流资料保留；全部未验证。');});
    $('rotation').onclick=()=>guard(()=>{const values=['roll','pitch','yaw'].map(id=>parse($(id).value,'number',id));const R=rotation(...values);doc.querySelector('[data-path="fixed_camera_aim.R_camera_to_gimbal"]').value=JSON.stringify(R);message('已根据辅助安装角生成 R；这不代表完成外参实测。');});
    $('check').onclick=()=>guard(()=>{read();review();message('录入已保留在本页内存。请下载草稿；所有验收状态保持false。');});
    $('saveDraft').onclick=()=>guard(()=>{read();download('fire_commissioning.draft.json',draft);message('已生成完整未验收草稿下载。');});
    $('exportConfig').onclick=()=>guard(()=>{read();const kind=$('exportKind').value;download(kind+'.draft.json',draft.configs[kind]);message('已生成未启用配置下载：'+kind+'.draft.json');});
    $('importFile').onchange=async()=>{const file=$('importFile').files[0];if(!file)return;try{if(file.size>2*1024*1024)throw Error('JSON请小于2MB');const next=importDocument(await file.text(),read());draft=next;clearRow();render();message('资料已导入；运行配置副本全部降为未验收。原始JSON与实测声明保存在完整草稿的追溯区，原文件未修改。');}catch(err){message('导入失败：'+err.message);}finally{$('importFile').value='';}};
    $('addRow').onclick=()=>guard(()=>{
      read();const kind=$('rowType').value,row={measurement_id:$('measurementId').value.trim(),measured:false,nominal_pan_deg:parse($('nominalPan').value,'number','名义水平角'),nominal_tilt_deg:parse($('nominalTilt').value,'number','名义俯仰角'),nozzle_distance_m:parse($('distance').value,'number','喷嘴距离'),measured_impact_error_m:parse($('impactError').value,'json','落点误差'),measured_encoder_error_deg:parse($('encoderError').value,'json','编码器误差')};
      row[kind==='measurements'?'angle_correction_deg':'commanded_angles_deg']=parse($('rowAngles').value,'json','角度');validateRow(row,kind);
      const next=clone(draft);if(editing)next.configs.fixed_camera_landing[editing.kind].splice(editing.index,1);next.configs.fixed_camera_landing[kind].push(row);
      next.measurement_claims=next.measurement_claims.filter(x=>x.measurement_id!==row.measurement_id);next.measurement_claims.push({measurement_id:row.measurement_id,source_claimed_measured:$('rowClaim').value==='true',source:'operator_entry'});
      draft=normalize(next);clearRow();renderRows();review();message('记录及记录者实测声明已保留，仍为未验收；请下载完整草稿备份。');
    });
    $('cancelRow').onclick=clearRow;
    $('simulate').onclick=()=>{ $('simResult').textContent='本次尚未得到有效射线解；失败原因见页面上方。';guard(()=>{read();const result=simulate(draft.configs.fixed_camera_aim,parse($('simTarget').value,'json','假设目标'));$('simResult').textContent=`仅仿真：水平 ${result.actuator_angles_deg[0].toFixed(3)}°，俯仰 ${result.actuator_angles_deg[1].toFixed(3)}°；名义喷嘴距离 ${result.nozzle_distance_m.toFixed(4)} m；直线几何残差 ${(result.ray_error_m*1000).toFixed(4)} mm。\n未检查实际电机格点、编码器、双目尺度和水流落点；实际命中未验证，水泵输出关闭。`;message('已完成浏览器内直线射线计算。');});};
    render();
  }
  return {SCHEMA,TEMPLATES,blank,preset,rotation,normalize,importDocument,validate,validateRow,gridSummary,forward,simulate,mount};
});
