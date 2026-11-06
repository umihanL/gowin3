'use strict';
const el=id=>document.getElementById(id);
let token='',connected=false,lastFrameUrl=null,polling=false,lastResult='';
const text=(id,value)=>{el(id).textContent=String(value??'未知')};
const labels={idle:'空闲',inspect:'观察',fire:'火焰瞄准',patrol:'巡航',unknown:'未知',observed:'发现火焰',not_observed:'本视野未检出',RED:'红灯等待',GREEN:'绿灯确认',SEARCH:'搜索信号灯',WAIT_RED:'红灯等待',SCANNING:'左右扫描',WAITING:'等待',not_ready:'尚未就绪'};
const tr=value=>labels[value]||value||'未收到状态';
function healthy(st){return st && !st.stale;}
function switchConnection(ok){connected=ok;el('connection').className='pill'+(ok?'':' off');text('connection',ok?'● 已连接网页服务器':'● 未连接服务器');document.querySelectorAll('[data-action],#sendText').forEach(b=>b.disabled=!ok)}
async function api(path,options={}){const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),2500);try{const response=await fetch(path,{...options,headers:{'X-Rescue-Token':token,...(options.headers||{})},signal:controller.signal,cache:'no-store'});if(!response.ok)throw new Error(response.status===401?'令牌无效或未填写':`请求失败 ${response.status}`);return response;}finally{clearTimeout(timeout)}}
function stateText(st){if(!st)return '未收到状态';const d=st.data;const val=d.mode||d.state||d.status||d.reason||'已收到状态';return (st.stale?'已过期 · ':'')+tr(val)}
function render(data){const s=data.states||{},scene=s.scene,valid=healthy(scene)&&scene.data.valid;const demo=data.mode==='recorded_demo';
 text('notice',demo?'演示模式 / 既有合成图片的真实算法报告回放，未连接相机、FPGA 或小车。按钮只记录请求，不会驱动硬件。':'已连接网页服务。设备是否在线请查看 FPGA 链路；请求受车端任务管理和执行条件约束。');
 const report=scene?.data||{},view=valid?report:(healthy(scene)?report.unassigned_current_view:null);
 text('building',healthy(scene)&&report.building_number?`${report.building_number} ${report.building_name||'号楼'}`:'未确认');
 text('buildingSub',valid?'同帧楼牌与楼面关联已确认':healthy(scene)&&report.building_number?'楼牌已读到，目标归属未确认':'需要楼牌与楼面关联证据');
 const fire=view?(valid?view.fire_status:(view.fire_observed?'observed':'not_observed')):'unknown';text('fire',tr(fire));el('fire').className=fire==='observed'?'danger':'';
 text('scope',valid?'已关联楼面 / 当前视野':view?'全画面观测 / 楼栋归属未确认':'当前视野 / 非全楼统计');
 text('people',view?.visible_person_count??'—');text('firefighters',view?.visible_firefighter_count??'—');
 text('sceneAge',scene?`报告接收于 ${scene.age_s.toFixed(1)} 秒前${scene.stale?' · 已过期':''}`:'尚无场景报告');
 for(const key of ['fpga','traffic','navigation','mission']){const st=s[key];text(key,stateText(st));text(key+'Age',st?`${st.age_s.toFixed(1)} 秒前${st.stale?' · 已过期':''}`:'尚无数据');el(key).className=st?.stale?'stale':'';}
 if(healthy(s.fpga)){const d=s.fpga.data;text('fpga',d.dry_run?'演练模式 / 未打开串口':d.status_fresh?'UART 状态新鲜':d.error?'UART 异常：'+d.error:d.connected===false?'UART 未连接':'未收到新鲜 FPGA 状态')}
 const events=(data.events||[]).slice().reverse();el('events').replaceChildren();
 if(!events.length){const li=document.createElement('li');li.className='empty';li.textContent='尚无语音或告警。';el('events').append(li)}
 events.forEach(ev=>{const li=document.createElement('li'),date=document.createElement('small');date.textContent=new Date(ev.stamp*1000).toLocaleTimeString('zh-CN')+' / '+(ev.kind==='voice'?'语音':ev.kind==='interaction'?'任务回复':'告警');li.append(date,document.createTextNode(ev.data.text||ev.data.transcript||ev.data.message||ev.data.error||ev.data.reason||ev.data.status||JSON.stringify(ev.data)));el('events').append(li)});
 if(s.interaction){const signature=JSON.stringify(s.interaction.data);if(signature!==lastResult){lastResult=signature;const d=s.interaction.data;text('commandResult','车端回复：'+(d.message||d.reason||d.error||d.speech||d.status||(d.accepted?'已接受任务请求，执行状态见下方。':'请求未被接受')))}}
 text('raw',JSON.stringify(data,null,2));text('clock',new Date().toLocaleString('zh-CN'));
 text('imageCaption',demo?'既有合成地图画面，仅用于演示；不是实时摄像头取景。':'图像丢失或超过 3 秒将隐藏，避免误认为实时画面。');
 return data.preview_age_s!==null&&data.preview_age_s<=3;
}
function hideFrame(){el('preview').style.display='none';el('placeholder').style.display='block';text('cameraBadge','无新鲜画面');if(lastFrameUrl){URL.revokeObjectURL(lastFrameUrl);lastFrameUrl=null}}
async function poll(){if(!token||polling)return;polling=true;try{const data=await(await api('/api/state')).json();switchConnection(true);const hasFrame=render(data);if(hasFrame){const blob=await(await api('/api/frame')).blob();const url=URL.createObjectURL(blob);el('preview').src=url;el('preview').style.display='block';el('placeholder').style.display='none';if(lastFrameUrl)URL.revokeObjectURL(lastFrameUrl);lastFrameUrl=url;text('cameraBadge',data.mode==='recorded_demo'?'合成地图录制画面':'低频实时预览')}else hideFrame();}catch(e){switchConnection(false);hideFrame();text('notice',e.message+'。上次画面已隐藏；下方保留数据不是实时状态。');for(const id of ['fpga','traffic','navigation','mission'])text(id,'连接中断 / 状态未知');text('building','未确认');text('buildingSub','连接中断，不能确认当前位置');text('fire','未知');text('people','—');text('firefighters','—');text('sceneAge','服务器连接中断')}finally{polling=false}}
function commandId(){if(globalThis.crypto?.randomUUID)return crypto.randomUUID();const bytes=new Uint8Array(16);crypto.getRandomValues(bytes);return Array.from(bytes,b=>b.toString(16).padStart(2,'0')).join('')}
async function command(action,commandText){if(!connected)return;const cmd={command_id:commandId(),source:'web',action,text:commandText,issued_at:Date.now()/1000,ttl_s:10};try{const value=await(await api('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(cmd)})).json();text('commandResult',value.accepted?'已加入车端命令队列，等待任务管理确认；这不表示已执行。':'此命令已接收过，本次未重复提交。')}catch(e){text('commandResult',e.message)}}
el('connect').addEventListener('click',()=>{token=el('token').value.trim();poll()});
el('token').addEventListener('keydown',e=>{if(e.key==='Enter')el('connect').click()});
document.querySelectorAll('[data-action]').forEach(b=>b.addEventListener('click',()=>command(b.dataset.action,b.dataset.text)));
el('sendText').addEventListener('click',()=>{const t=el('commandText').value.trim();if(!t){text('commandResult','请先填写指令');return}let action='navigate';if(['停车','停止'].includes(t))action='stop';else if(['报告场景','播报场景'].includes(t))action='report';else if(['继续','恢复'].includes(t))action='resume';else if(['观察','观察当前场景'].includes(t))action='inspect';command(action,t)});
switchConnection(false);setInterval(poll,1000);
