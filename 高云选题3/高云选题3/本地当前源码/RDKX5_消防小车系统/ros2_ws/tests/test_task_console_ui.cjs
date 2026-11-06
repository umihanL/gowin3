'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict'),test=require('node:test');
const root=path.resolve(__dirname,'../..'),source=fs.readFileSync(path.join(root,'ros2_ws/web/presentation.js'),'utf8'),html=fs.readFileSync(path.join(root,'ros2_ws/web/presentation.html'),'utf8');
const block=(a,b)=>source.slice(source.indexOf(a),source.indexOf(b,source.indexOf(a)));
test('three main workspaces retain every previous control identity once',()=>{
 const old=fs.readFileSync(path.join(root,'reports/hmi_task_console_20261031/baseline/presentation.html'),'utf8');
 const ids=[...html.matchAll(/\sid="([^"]+)"/g)].map(m=>m[1]),previous=[...old.matchAll(/\sid="([^"]+)"/g)].map(m=>m[1]);
 assert.equal(new Set(ids).size,ids.length);assert.deepEqual(previous.filter(id=>!ids.includes(id)),[]);
 const nav=html.match(/<nav aria-label="主导航">([\s\S]+?)<\/nav>/)[1];
 assert.deepEqual([...nav.matchAll(/data-view="([^"]+)"/g)].map(m=>m[1]),['overview','fire','motor']);
 assert.equal((html.match(/class="view(?: active)?"/g)||[]).length,3);
});
test('map telemetry requires fresh real feedback and never derives pose from point selection',()=>{
 const h={finite:Number.isFinite};vm.runInNewContext(block('  function mapSnapshot(', '  function selectMapPoint('),h);
 const value={live:{supported:true,fresh:true,pose:{valid:true,fresh:true,x_cm:25,y_cm:159,heading_deg:90},route:{valid:true,fresh:true,points_cm:[[25,159],[25,35]]}}};
 assert.equal(h.mapSnapshot(value,true,1).pose.x_cm,25);
 for(const [online,age] of [[false,1],[true,2500],[true,-1]]){assert.equal(h.mapSnapshot(value,online,age).pose,null);assert.equal(h.mapSnapshot(value,online,age).route,null);}
 value.live.fresh=false;assert.equal(h.mapSnapshot(value,true,1).pose,null);
 value.live.fresh=true;value.live.pose.fresh=false;assert.equal(h.mapSnapshot(value,true,1).pose,null);
 assert.equal(h.mapSnapshot({reference:{confirmed:true,point_id:2}},true,1).pose,null);
});
test('map selection only prepares a sentence and does not submit navigation',()=>{
 const nodes={commandText:{value:'',focus(){}}},h={state:{},$:id=>nodes[id],validPointId:x=>Number.isInteger(x)&&x>=1&&x<=9,renderPointNavigation(){},renderPointControls(){},toast(){}};
 vm.runInNewContext(block('  function selectMapPoint(', '  function renderFpgaMap('),h);h.selectMapPoint(6);
 assert.equal(h.state.pointTarget,6);assert.equal(nodes.commandText.value,'去6号点');
 h.selectMapPoint(0);assert.equal(h.state.pointTarget,6);
});
test('free map start keeps image/world coordinates distinct and repeat clicks rotate counterclockwise',()=>{
 const h={finite:Number.isFinite};vm.runInNewContext(block('  function normalizedMapHeading(', '  function renderFpgaMap('),h);
 const samples=[{node:0,cell:[12,79],xy_q8:[25*256,159*256],heading_deg:90},{node:1,cell:[52,140],xy_q8:[105*256,281*256],heading_deg:37}];
 let start=h.nextMapStart(null,25,159,samples);assert.equal(start.heading,90);
 const directions=[-180,-90,0,90];
 for(const expected of directions){start=h.nextMapStart(start,25,159,samples);assert.equal(start.heading,expected);assert.equal(start.x,25);assert.equal(start.y,159);}
 assert.equal(h.mapReferenceCommand(start,320),'确认起点X25.00厘米Y161.00厘米朝向90度');
 const moved=h.nextMapStart(start,104,280,samples);assert.equal(moved.heading,37);assert.equal(moved.x,104);
 assert.equal(h.mapReferenceCommand(moved,320),'确认起点X104.00厘米Y40.00厘米朝向37度');
 assert.throws(()=>h.mapReferenceCommand({...moved,heading:NaN},320));
});
test('start direction follows exact ROM Manhattan snapping and tie order, never a guessed cardinal',()=>{
 const h={finite:Number.isFinite};vm.runInNewContext(block('  function normalizedMapHeading(', '  function renderFpgaMap('),h);
 // Query rounds to the 2 cm cell centre (21,21). L1 selects node 1;
 // Euclidean distance would select node 0. Exact Q8 matters inside one cell.
 const samples=[{node:0,cell:[12,12],xy_q8:[24*256,24*256],heading_deg:37},
   {node:1,cell:[10,13],xy_q8:[21*256,26*256],heading_deg:127},
   {node:2,cell:[10,8],xy_q8:[21*256,16*256],heading_deg:-45}];
 assert.equal(h.nearestMapStartHeading(20.1,20.9,samples),127);
 assert.equal(h.nearestMapStartHeading(20.1,20.9,[samples[0],samples[2],samples[1]]),-45);
 const sameCell=[{node:0,cell:[10,10],xy_q8:[20*256,20*256],heading_deg:10},
   {node:1,cell:[10,10],xy_q8:[21*256,21*256],heading_deg:20}];
 assert.equal(h.nearestMapStartHeading(20.1,20.9,sameCell),20);
 const missing=h.nextMapStart(null,35,35,[]);assert.equal(missing.heading,null);
 assert.throws(()=>h.mapReferenceCommand(missing,320));
 assert.equal(h.nextMapStart(missing,35,35,[]).heading,null);
 const catalog=JSON.parse(fs.readFileSync(path.join(root,'ros2_ws/config/region_traversability.json'),'utf8'));
 for(const [x,y,heading] of [[35,35,37],[35,285,127],[317,285,-135],[317,35,-45]]){
   assert.equal(h.nearestMapStartHeading(x,y,catalog.start_heading_samples),heading);
 }
});
test('missing start-heading catalog disables the real confirmation control',()=>{
 const nodes={},state={mapPickMode:'start',mapStart:{x:35,y:35,heading:37}},payload={geometry:{width_cm:352,height_cm:320,road_runs:[]},points:[]};
 const extension={real_uart:true,protocol_ready:true,status_fresh:true,vehicle_stopped:true,can_move:true,capabilities:{route_turn:true}};
 const h={state,mapData:payload,mapReceived:0,mapStaticKey:'',performance:{now:()=>0},finite:Number.isFinite,
   $:id=>nodes[id]||=( {disabled:false,dataset:{},classList:{add(){},remove(){},toggle(){}},setAttribute(){},querySelectorAll:()=>[]} ),
   html(){},text:(id,v)=>{h.$(id).textContent=v;},data:k=>k==='extension'?extension:{},fresh:()=>true,
   canControl:()=>true,connected:()=>true,regionSearchState:()=>({}),pointStartReference:()=>null,validPointId:x=>x>=1&&x<=9,num:String,reason:x=>x};
 vm.runInNewContext(block('  function mapSnapshot(', '  function selectMapPoint(')+block('  function normalizedMapHeading(', "  $('mapSetStart').addEventListener"),h);
 h.renderFpgaMap();assert.equal(nodes.mapConfirmStart.disabled,true);assert.match(nodes.mapStartHint.textContent,/缺少道路朝向目录/);
 payload.start_heading_samples=[{node:0,cell:[17,17],xy_q8:[35*256,35*256],heading_deg:37}];
 h.renderFpgaMap();assert.equal(nodes.mapConfirmStart.disabled,false);
 assert.match(nodes.mapConfirmStart.title,/不会启动车辆/);
 // A draft is not a pose. Confirmation must work before the first pose exists.
 assert.match(nodes.mapTelemetry.textContent,/没有新鲜 FPGA 位姿/);
 extension.real_uart=false;extension.error='Write timeout';h.renderFpgaMap();
 assert.equal(nodes.mapConfirmStart.disabled,true);
 assert.match(nodes.mapConfirmStart.title,/串口写入超时.*Write timeout/);
 assert.equal(nodes.mapStartHint.textContent.startsWith(nodes.mapConfirmStart.title),true);
});
function mapStartGateHarness(){
 const extension={real_uart:true,protocol_ready:true,status_fresh:true,vehicle_stopped:true,can_move:true,
   capabilities:{route_turn:true},reason:'ready',fpga_status:{mode_selected:true}};
 const events={},sent=[],notices=[],h={extension,events,sent,notices,finite:Number.isFinite,
   state:{mapPickMode:'start',mapStart:{x:35,y:35,heading:37}},
   mapData:{geometry:{height_cm:320},start_heading_samples:[{node:0,cell:[17,17],xy_q8:[35*256,35*256],heading_deg:37}]},
   data:k=>k==='extension'?extension:{dry_run:true},fresh:()=>true,connected:()=>true,canControl:()=>true,
   $:id=>({addEventListener:(event,handler)=>{events[id+':'+event]=handler;}}),
   reason:x=>x,toast:message=>notices.push(message),sendCommand:command=>sent.push(command)};
 vm.runInNewContext(block('  function normalizedMapHeading(', '  function renderFpgaMap(')+
   block("  $('mapConfirmStart').addEventListener",'  async function pollFpgaMap('),h);
 return h;
}
test('map start readiness follows extension UART, stop and mode evidence without an existing pose',()=>{
 const h=mapStartGateHarness();assert.equal(h.mapStartReadiness().ready,true);
 for(const [patch,expected] of [
   [{real_uart:false},'real_extension_uart_required'],
   [{protocol_ready:false},'extension_firmware_or_feedback_unavailable'],
   [{status_fresh:false},'extension_firmware_or_feedback_unavailable'],
   [{capabilities:{}},'fpga_capability_not_implemented_route_turn'],
   [{can_move:false,reason:'fpga_physical_host_mode_required'},'fpga_physical_host_mode_required'],
   [{can_move:false,busy:true,reason:'extension_movement_in_progress'},'extension_movement_in_progress'],
   [{vehicle_stopped:false},'measured_stop_required']]){
   const c=mapStartGateHarness();Object.assign(c.extension,patch);
   assert.equal(c.mapStartReadiness().ready,false);assert.equal(c.mapStartReadiness().reason,expected);
 }
 h.fresh=()=>false;assert.equal(h.mapStartReadiness().reason,'extension_status_unavailable');
 h.fresh=()=>true;h.extension.real_uart=false;h.extension.error='Write timeout';
 assert.match(h.mapStartReadiness().reason,/串口写入超时/);
 h.extension.real_uart=true;h.extension.error='';h.state.pending=true;h.canControl=()=>false;
 assert.match(h.mapStartReadiness().reason,/上一条指令正在提交/);
 h.canControl=()=>true;h.state.mapStart.heading=NaN;assert.equal(h.mapStartReadiness().ready,false);
});
test('map start click rechecks live readiness and never submits after UART loss or motion',()=>{
 const h=mapStartGateHarness(),click=h.events['mapConfirmStart:click'];
 click();assert.deepEqual(h.sent,['确认起点X35.00厘米Y285.00厘米朝向37度']);
 h.extension.vehicle_stopped=false;click();assert.equal(h.sent.length,1);assert.equal(h.notices.at(-1),'measured_stop_required');
 h.extension.vehicle_stopped=true;h.extension.real_uart=false;h.extension.error='Write timeout';click();
 assert.equal(h.sent.length,1);assert.match(h.notices.at(-1),/串口写入超时/);
});
test('map polling downloads heading ROM once and invalidates it when the signature or heading hash changes',async()=>{
 const sample={node:0,cell:[17,17],xy_q8:[35*256,35*256],heading_deg:37},urls=[];
 const responses=[{schema_version:1,map_signature:1,start_heading_samples_sha256:'a',start_heading_samples:[sample]},
   {schema_version:1,map_signature:1,start_heading_samples_sha256:'a'},
   {schema_version:1,map_signature:1,start_heading_samples_sha256:'b'},
   {schema_version:1,map_signature:1,start_heading_samples_sha256:'b',start_heading_samples:[sample]},
   {schema_version:1,map_signature:2,start_heading_samples_sha256:'b'}];
 const h={mapData:null,mapReceived:0,state:{mapStart:{x:35,y:35,heading:37}},finite:Number.isFinite,performance:{now:()=>10},
   fetch:async url=>{urls.push(url);return {ok:true,json:async()=>responses.shift()};},AbortSignal:{timeout:()=>null},
   renderFpgaMap(){},setTimeout(){},document:{hidden:false}};
 vm.runInNewContext(block('  function normalizedMapHeading(', '  function nearestMapStartHeading(')+block('  async function pollFpgaMap(', '  function renderGimbalFeedback('),h);
 await h.pollFpgaMap();await h.pollFpgaMap();assert.equal(h.mapData.start_heading_samples.length,1);
 await h.pollFpgaMap();assert.equal(h.mapData.start_heading_samples,undefined);assert.equal(h.state.mapStart,null);
 await h.pollFpgaMap();assert.equal(h.mapData.start_heading_samples.length,1);
 h.state.mapStart={x:35,y:35,heading:37};await h.pollFpgaMap();assert.equal(h.mapData.start_heading_samples,undefined);assert.equal(h.state.mapStart,null);
 assert.deepEqual(urls,['/api/fpga-map?include_headings=1','/api/fpga-map','/api/fpga-map','/api/fpga-map?include_headings=1','/api/fpga-map']);
});
test('map regions come from the configured catalog and unknown HTML labels are never interpolated',()=>{
 const h={finite:Number.isFinite};vm.runInNewContext(block('  function normalizedMapHeading(', '  function renderFpgaMap('),h);
 const svg=h.regionMapSvg({regions:{A:{label:'<script>bad</script>',bounds_image_cm:[0,0,176,160],waypoints:[{image_cm:[35,35]}]},E:{bounds_image_cm:[0,0,10,10]}}});
 assert.match(svg,/data-map-region="A"/);assert.match(svg,/translate\(35 35\)/);assert.doesNotMatch(svg,/<script|data-map-region="E"/);
});

test('region route display follows the board order rather than reordering by label',()=>{
 const h={finite:Number.isFinite,num:(v,n)=>v.toFixed(n),reason:v=>v};
 vm.runInNewContext(block('  function regionRouteSummary(', '  function renderRegionSearch('),h);
 const r={command_id:'r1',waypoints:[{id:'C3'},{id:'C1'},{id:'C4'},{id:'C2'}],visited_waypoint_ids:['C3'],
  exit:{cell:[90,120],confirmed_stopped:false},route_policy:{strategy:'directed_graph_region_first',estimated_distance_cm:346.2}};
 assert.match(h.regionRouteSummary(r,true),/✓ C3 → C1 → C4 → C2 → 出口/);
 assert.match(h.regionRouteSummary(r,true),/区域内优先/);assert.doesNotMatch(h.regionRouteSummary(r,true),/出口已停稳/);
 r.exit.confirmed_stopped=true;assert.match(h.regionRouteSummary(r,true),/出口已停稳/);
 assert.doesNotMatch(h.regionRouteSummary(r,false),/C3|出口已停稳/);
 r.route_policy.fallback_reason='region_detour_required';assert.match(h.regionRouteSummary(r,true),/需借道/);
 r.route_policy.continuous_route=true;
 assert.match(h.regionRouteSummary(r,true),/整条路线一次规划/);
 assert.doesNotMatch(h.regionRouteSummary(r,true),/沿原路线续行/);
 r.navigation={retained_route:true};
 assert.match(h.regionRouteSummary(r,true),/途经点不停 · 仅确认火焰后暂停，再沿原路线续行/);
 assert.doesNotMatch(h.regionRouteSummary(r,false),/一次规划|原路线续行/);
 r.route_policy.continuous_route=false;
 assert.doesNotMatch(h.regionRouteSummary(r,true),/整条路线一次规划|沿原路线续行/);
 r.passed_waypoint_ids=['C3','C1'];
 assert.match(h.regionRouteSummary(r,true),/✓ C3 → ✓ C1 → C4 → C2/);
});

test('exit marker requires fresh validated map coordinates and never inserts arbitrary labels',()=>{
 const h={};vm.runInNewContext(block('  function regionExitSvg(', '  function renderFpgaMap('),h);
 const r={command_id:'r1',exit:{id:'<script>bad</script>',cell:[91,80]}};
 assert.match(h.regionExitSvg(r,true),/translate\(183 161\)/);assert.doesNotMatch(h.regionExitSvg(r,true),/<script/);
 assert.equal(h.regionExitSvg(r,false),'');
 for(const cell of [[176,0],[-1,20],[2.1,3],[1,160],[0,NaN]])assert.equal(h.regionExitSvg({...r,exit:{cell}},true),'');
});

test('continuous route predownload reports fresh bounded progress before departure',()=>{
 const h={reason:v=>v};
 vm.runInNewContext(block('  function regionRouteSummary(', '  function renderRegionSearch('),h);
 const r={command_id:'r1',continuous_route:true,route_policy:{continuous_route:true},
  navigation:{route_download:{active:true,received:128,total:360}}};
 assert.match(h.regionRouteSummary(r,true),/完整路线回读 128\/360 点，完成后出发/);
 assert.doesNotMatch(h.regionRouteSummary(r,false),/128\/360|完成后出发/);
 r.continuous_route=false;assert.doesNotMatch(h.regionRouteSummary(r,true),/完整路线回读/);
 r.continuous_route=true;r.navigation.route_download.active=false;
 assert.doesNotMatch(h.regionRouteSummary(r,true),/完整路线回读/);
 r.navigation.route_download.active=true;
 for(const received of [-1,NaN,'128',361]){
  r.navigation.route_download.received=received;
  assert.doesNotMatch(h.regionRouteSummary(r,true),/完整路线回读/);
 }
 assert.match(source,/checkpoint_fire_out_of_range_on_retained_route:'本点火焰不在有效喷水距离，已记录并继续巡检'/);
});
test('changing away from the motor tab ends any continuous motor lease',()=>{
 const calls=[],h={state:{devicePane:'motors'},endMotorRuns:s=>calls.push(s),document:{querySelectorAll:()=>[]}};
 vm.runInNewContext(block('  function selectDevicePane(', '  const mobileTabs'),h);h.selectDevicePane('evidence');
 assert.equal(calls.length,1);assert.equal(h.state.devicePane,'evidence');h.selectDevicePane('advanced');assert.equal(calls.length,1);
});

test('first device visit routes fresh production control to gimbal while preserving explicit later pane choices',()=>{
 const flags={online:true,fresh:true},actuators={enabled:true,dry_run:false};
 const panes=['motors','gimbal','advanced','evidence'].map(name=>({dataset:{devicePane:name},classList:{toggle(){}}}));
 const h={state:{view:'overview',frameEpoch:0},data:()=>actuators,connected:()=>flags.online,fresh:()=>flags.fresh,
  document:{querySelectorAll:selector=>selector==='[data-device-pane]'?panes:[]},$:()=>null,
  history:{replaceState(){}},endMotorRuns(){},wantsYoloFrames:()=>false,stopFrame(){},stopRoadFrames(){},scheduleRoadFrame(){}};
 vm.runInNewContext(block('  function selectView(', '  const mobileTabs ='),h);
 h.selectView('motor');assert.equal(h.state.devicePane,'gimbal');
 h.selectDevicePane('motors');h.selectView('motor');assert.equal(h.state.devicePane,'motors');
 // New status does not switch a pane during any render; this chooser is used only on view entry.
 h.selectDevicePane('evidence');h.selectView('motor');assert.equal(h.state.devicePane,'evidence');
 for(const staleCase of ['offline','stale','dry_run','disabled']){
  delete h.state.devicePane;flags.online=staleCase!=='offline';flags.fresh=staleCase!=='stale';
  actuators.dry_run=staleCase==='dry_run';actuators.enabled=staleCase!=='disabled';
  h.selectView('motor');assert.equal(h.state.devicePane,'motors',staleCase);
 }
});
test('measured dual-axis readout and self-test cannot be enabled from commands or old feedback',()=>{
 const nodes={},topics={},flags={fresh:true},h={data:k=>topics[k]||{},fresh:()=>flags.fresh,finite:Number.isFinite,num:(x,n)=>x.toFixed(n),text:(id,v)=>{(nodes[id]||={}).textContent=v;},$:id=>nodes[id]||=( {} ),canControl:()=>true,connected:()=>true,reason:x=>x};
 vm.runInNewContext(block('  function renderGimbalFeedback(', '  async function submitGimbalTest('),h);
 topics.actuators={dry_run:false,status_fresh:true,feedback_fresh:true,actual_angles_deg:[12,34],target_deg:[15,35]};topics.gimbal_self_test={can_start:true};
 h.renderGimbalFeedback();assert.equal(nodes.aimPanActual.textContent,'12.0°');assert.equal(nodes.gimbalTestStart.disabled,false);
 topics.actuators.feedback_fresh=false;topics.gimbal_self_test.can_start=false;h.renderGimbalFeedback();assert.equal(nodes.aimPanActual.textContent,'—');assert.equal(nodes.gimbalTestStart.disabled,true);
 flags.fresh=false;topics.gimbal_self_test.can_start=true;h.renderGimbalFeedback();assert.equal(nodes.gimbalTestStart.disabled,true);
});
