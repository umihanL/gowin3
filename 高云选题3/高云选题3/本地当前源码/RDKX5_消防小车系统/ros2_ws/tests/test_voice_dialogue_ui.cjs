'use strict';

// Production rendering only; no browser, network, audio, ROS or robot commands.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../web/presentation.js'), 'utf8');
const start = source.indexOf('  function speechPlaybackStatus(');
const end = source.indexOf('  function renderTimeline()', start);
assert.ok(start >= 0 && end > start, 'Find the actual production voice/dialogue renderers');
const fragment = source.slice(start, end);

function harness() {
  const elements = new Map();
  const topics = { voice: {}, dialogue_status: {} };
  const freshness = { voice: true, dialogue_status: true };
  const events = [];
  let now = 1000, online = true;
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      textContent: '', innerHTML: '', dataset: {}, style: {},
      scrollHeight: 0, scrollTop: 0, clientHeight: 0,
    });
    return elements.get(id);
  };
  const state = { received: now, localMessages: [], data: { server_time: 100, commands: [] } };
  const context = vm.createContext({
    state, performance: { now: () => now }, $: element,
    data: key => freshness[key] ? topics[key] || {} : {},
    fresh: key => freshness[key] === true,
    finite: value => typeof value === 'number' && Number.isFinite(value),
    num: (value, digits = 0) => typeof value === 'number' ? value.toFixed(digits) : '—',
    text: (id, value) => { element(id).textContent = value; },
    reason: value => value, clock: value => String(value),
    connected: () => online, liveEvents: () => events,
    labels: { interaction: '任务回执' },
    topic: key => ({ topic: key === 'interaction' ? '/interaction/result' : '/unused', data: topics[key], received_at: 100 }),
    esc: value => String(value ?? '').replaceAll('&', '&amp;').replaceAll('<', '&lt;').replaceAll('>', '&gt;'),
  });
  vm.runInContext(fragment + '\nglobalThis.feature={renderVoiceCapture,renderDialogueActivity,voiceReceiptMeta,renderDialogue,collectDialogueTurns,dialogueTurnStatus,speechPlaybackStatus};', context);
  return { ...context.feature, topics, freshness, events, element, state,
    advance: seconds => { now += seconds * 1000; },
    disconnect: () => { online = false; },
    recording: fields => { topics.voice = { state: 'listening', listening: true,
      capture_mode: 'command', capture_active: true, elapsed_s: 4.5, duration_s: 15,
      remaining_s: 10.5, capture_started_at: 95.5, level_stamp: 100,
      level_dbfs: -18, peak_dbfs: -9, ...fields }; },
  };
}

test('formal capture defaults to 15 seconds and shows actual reported progress', () => {
  const h = harness();
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureProgress').max, 15);
  assert.match(h.element('voiceCaptureTime').textContent, /默认收音 15 秒，可手动结束/);
  h.recording({});
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureProgress').value, 4.5);
  assert.match(h.element('voiceCaptureTime').textContent, /4\.5 \/ 15 秒/);
  assert.match(h.element('micNote').textContent, /说完提交/);
  h.recording({ elapsed_s: 18 });
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureProgress').value, 15);
});

test('delayed answers stay directly below their own questions, even when a later count answers first', () => {
  const h=harness();
  h.state.localMessages=[{id:'slow',text:'解释避障原理',at:95},{id:'fast',text:'现在有几个人',at:96}];
  h.events.push({id:1,at:97,kind:'scene_answer',data:{command_id:'fast',text:'当前没有观察到人物',accepted:true}});
  h.topics.dialogue_status={command_id:'slow',state:'thinking',elapsed_s:3};
  h.renderDialogue();
  let html=h.element('dialogueMessages').innerHTML;
  assert.ok(html.indexOf('解释避障原理')<html.indexOf('正在思考'));
  assert.ok(html.indexOf('正在思考')<html.indexOf('现在有几个人'));
  h.events.push({id:2,at:99,kind:'scene_answer',data:{command_id:'slow',text:'依据障碍距离规划安全通路',accepted:true}});
  h.renderDialogue();html=h.element('dialogueMessages').innerHTML;
  assert.ok(html.indexOf('解释避障原理')<html.indexOf('依据障碍距离规划安全通路'));
  assert.ok(html.indexOf('依据障碍距离规划安全通路')<html.indexOf('现在有几个人'));
  assert.equal((html.match(/class="dialogue-turn"/g)||[]).length,2);
  assert.doesNotMatch(html,/正在思考/);
});

test('reload restores command questions and stored answers; local and event copies do not duplicate them', () => {
  const h=harness();
  const answer={command_id:'q1',stamp:96,text:'支持场景分析',accepted:true};
  h.state.data.commands=[{command_id:'q1',request_id:'r1',kind:'command',action:'dialogue',text:'你能做什么',queued_at:95,answer}];
  h.renderDialogue();
  assert.match(h.element('dialogueMessages').innerHTML,/你能做什么/);
  h.state.localMessages.push({id:'q1',text:'你能做什么',at:95});
  h.events.push({id:1,at:96,kind:'scene_answer',data:answer},{id:2,at:97,kind:'scene_answer',data:answer});
  h.renderDialogue();
  const html=h.element('dialogueMessages').innerHTML;
  assert.equal((html.match(/你能做什么/g)||[]).length,1);
  assert.equal((html.match(/支持场景分析/g)||[]).length,1);
});

test('identical repeated questions remain separate turns by command ID', () => {
  const h=harness();
  h.state.localMessages=[{id:'a',text:'有几团火焰',at:90},{id:'b',text:'有几团火焰',at:95}];
  h.events.push({id:1,at:96,kind:'scene_answer',data:{command_id:'b',accepted:true,text:'第二次观察'}});
  const turns=h.collectDialogueTurns();
  assert.equal(turns.length,2);assert.equal(turns[0].answer,undefined);assert.equal(turns[1].answer.text,'第二次观察');
});

test('late original voice text replaces a recovered router rewrite, preserving what the user actually said', () => {
  const h=harness();
  h.events.push({id:1,at:95,kind:'interaction',data:{command_id:'scene',action:'scene',accepted:true,question:{text:'请分析当前场景'}}});
  assert.equal(h.collectDialogueTurns()[0].text,'请分析当前场景');
  h.events.push({id:2,at:96,kind:'transcript',data:{command_id:'scene',action:'scene',submitted:true,text:'分析当前火场'}});
  assert.equal(h.collectDialogueTurns()[0].text,'分析当前火场');
});

test('latest topic recovers an answer outside the event ring; duplicate-question rejection cannot replace success', () => {
  const h=harness();
  h.state.data.commands=[{command_id:'q1',text:'问题',queued_at:90,action:'dialogue'}];
  h.topics.scene_answer={command_id:'q1',stamp:95,accepted:true,text:'成功回答'};
  assert.equal(h.collectDialogueTurns()[0].answer.text,'成功回答');
  h.topics.scene_answer={command_id:'q1',stamp:100,accepted:false,reason:'duplicate_question'};
  assert.equal(h.collectDialogueTurns()[0].answer.text,'成功回答');
});

test('answer-before-transcript pairs later voice text; listen control IDs are excluded', () => {
  const h=harness();
  h.state.data.commands=[{command_id:'listen-start',text:null,kind:'listen',queued_at:90}];
  h.events.push({id:2,at:96,kind:'scene_answer',data:{command_id:'voice',accepted:true,text:'已观察到一个目标'}});
  h.renderDialogue();assert.match(h.element('dialogueMessages').innerHTML,/原问题未保留/);
  h.events.push({id:3,at:97,kind:'transcript',data:{command_id:'voice',text:'看到了什么',action:'dialogue',submitted:true}});
  h.renderDialogue();const html=h.element('dialogueMessages').innerHTML;
  assert.equal((html.match(/class="dialogue-turn"/g)||[]).length,1);
  assert.ok(html.indexOf('看到了什么')<html.indexOf('已观察到一个目标'));
  assert.match(html,/车载语音/);assert.doesNotMatch(html,/原问题未保留|listen-start/);
});

test('router question and thinking status recover missing original text without nearest-question guessing', () => {
  const h=harness();
  h.events.push({id:1,at:90,kind:'interaction',data:{command_id:'router',accepted:true,action:'dialogue',question:{text:'为什么需要双目'}}});
  h.topics.dialogue_status={command_id:'thinking',question:'解释灭火流程',stamp:100,elapsed_s:2,state:'thinking'};
  h.events.push({id:2,at:99,kind:'scene_answer',data:{command_id:'unknown',accepted:true,text:'未知原问题的回答'}});
  const turns=h.collectDialogueTurns();
  assert.equal(turns.find(t=>t.id==='router').text,'为什么需要双目');
  assert.equal(turns.find(t=>t.id==='thinking').text,'解释灭火流程');
  assert.equal(turns.find(t=>t.id==='unknown').text,undefined);
});

test('history survives the event ring advancing and is limited by complete turns', () => {
  const h=harness();
  for(let i=0;i<27;i++){
    h.events.push({id:i*2,at:i,kind:'transcript',data:{command_id:'q'+i,text:'问题'+i,submitted:true}});
    h.events.push({id:i*2+1,at:i+.5,kind:'scene_answer',data:{command_id:'q'+i,accepted:true,text:'回答'+i}});
  }
  let turns=h.collectDialogueTurns();assert.equal(turns.length,25);assert.equal(turns[0].id,'q2');
  h.events.length=0;turns=h.collectDialogueTurns();
  assert.equal(turns.length,25);assert.ok(turns.every(t=>t.text&&t.answer));
});

test('pending, rejection and disconnection are honest per-turn states; task acceptance is not completion', () => {
  const h=harness();
  h.topics.dialogue_status={command_id:'other',state:'thinking'};
  assert.equal(h.dialogueTurnStatus({id:'q',at:99,action:'dialogue'}).state,'waiting');
  h.topics.dialogue_status={command_id:'q',state:'thinking'};
  assert.equal(h.dialogueTurnStatus({id:'q',at:99}).state,'thinking');
  h.freshness.dialogue_status=false;
  assert.equal(h.dialogueTurnStatus({id:'q',at:99}).state,'waiting');
  const receipt=h.dialogueTurnStatus({id:'cmd',at:99,action:'navigate',receipt:{accepted:true}});
  assert.equal(receipt.state,'receipt');assert.match(receipt.text,/执行进度以实车反馈为准/);
  assert.equal(h.dialogueTurnStatus({id:'q',receipt:{accepted:false,reason:'denied'}}).state,'error');
  h.disconnect();assert.match(h.dialogueTurnStatus({id:'q',at:99}).text,/连接已断开/);
});

test('messages render untrusted text literally, and reading history does not force the scroll to bottom', () => {
  const h=harness();
  h.state.localMessages=[{id:'safe',text:'<img src=x onerror=alert(1)>',at:99}];
  h.events.push({id:1,at:100,kind:'scene_answer',data:{command_id:'safe',text:'<script>alert(2)</script>',accepted:true}});
  const box=h.element('dialogueMessages');box.scrollHeight=2000;box.clientHeight=200;box.scrollTop=150;
  h.renderDialogue();assert.equal(box.scrollTop,150);
  assert.doesNotMatch(box.innerHTML,/<img|<script/);assert.match(box.innerHTML,/&lt;script&gt;/);
  box.scrollTop=1800;h.events[0].data.text='更新回答';h.renderDialogue();assert.equal(box.scrollTop,2000);
});

test('unchanged chat content is not rewritten every poll even when browser HTML serialization differs', () => {
  const h=harness();
  h.state.localMessages=[{id:'q',text:'解释“火焰”识别',at:99}];
  const box=h.element('dialogueMessages');let writes=0;
  Object.defineProperty(box,'innerHTML',{get:()=>'<browser-normalized-html>',set:()=>{writes++;}});
  h.renderDialogue();h.renderDialogue();assert.equal(writes,1);
});

test('manual stop enters transcription without showing continued recording or a fabricated answer', () => {
  const h = harness();
  h.recording({}); h.renderVoiceCapture();
  h.recording({ state: 'transcribing', capture_active: false, ended_by: 'manual_stop', elapsed_s: 4.5 });
  h.renderVoiceCapture(); h.renderDialogueActivity();
  assert.equal(h.element('voiceCaptureTime').textContent, '收音已结束，正在识别文字');
  assert.match(h.element('micNote').textContent, /麦克风已停止采集/);
  assert.equal(h.element('voiceCaptureProgress').value, 0);
  assert.equal(h.element('voiceCaptureLevel').textContent, '待收音');
  assert.match(h.element('dialogueActivity').textContent, /先完成收音与转写/);
  assert.equal(h.element('dialogueActivity').dataset.state, 'idle');
});

test('End click waits for actual microphone closure, then shows transcription separately', () => {
  const h=harness();h.recording({stop_requested_at:0});
  h.state.voiceControl={stop:{requestId:'end',baselineStopAt:0,error:''}};
  h.state.data.commands=[{request_id:'end',status:'queued'}];
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureTime').textContent,'结束请求待车载确认');
  h.state.data.commands[0].status='submitted';h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureTime').textContent,'结束请求待车载确认');
  h.recording({state:'stopping',stop_requested:true,stop_requested_at:101,capture_stop_confirmed:false});
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureTime').textContent,'正在结束车载收音');
  h.recording({state:'transcribing',capture_active:false,stop_requested:true,stop_requested_at:101,capture_stop_confirmed:true});
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureTime').textContent,'收音已结束，正在识别文字');
  h.topics.dialogue_status={state:'thinking',elapsed_s:2};h.renderDialogueActivity();
  assert.match(h.element('dialogueActivity').textContent,/Qwen 正在思考/);
});

test('preparation cancellation, queue failures and stale status cannot claim microphone stopped', () => {
  const h=harness();h.state.voiceControl={stop:{requestId:'end',baselineStopAt:90,error:''}};
  h.topics.voice={state:'loading_asr',listening:true,stop_requested:true,stop_requested_at:101,capture_stop_confirmed:false,capture_active:false};
  h.renderVoiceCapture();assert.equal(h.element('voiceCaptureTime').textContent,'正在结束语音准备');
  h.freshness.voice=false;h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureTime').textContent,'等待麦克风停止确认');
  h.freshness.voice=true;
  h.state.data.commands=[{request_id:'end',status:'expired',reason:'queue_ttl_exceeded'}];
  h.renderVoiceCapture();assert.equal(h.element('voiceCaptureTime').textContent,'结束请求未确认');
  assert.match(h.element('micNote').textContent,/queue_ttl_exceeded/);
  h.state.data.commands=[];h.state.voiceControl.stop=null;
  h.recording({stop_requested:false,stop_requested_at:0});h.renderVoiceCapture();
  assert.match(h.element('voiceCaptureTime').textContent,/4\.5 \/ 15 秒/);
});

test('each repeated End waits for its own sequence, even when the source timestamp has not advanced', () => {
  const h=harness();
  h.state.voiceControl={stop:{requestId:'again',baselineStopAt:100,baselineStopSeq:5,error:''}};
  h.topics.voice={state:'transcribing',listening:true,capture_active:false,stop_requested:true,
    stop_requested_at:100,stop_request_seq:5,stop_request_accepted:true,capture_stop_confirmed:true};
  h.renderVoiceCapture();assert.equal(h.element('voiceCaptureTime').textContent,'结束请求待车载确认');
  h.topics.voice.stop_request_seq=6;h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureTime').textContent,'收音已结束，正在识别文字');
});

test('ending formal capture cannot report independent diagnostic microphone stopped', () => {
  const h=harness();
  h.state.voiceControl={stop:{requestId:'end',baselineStopSeq:4,error:''}};
  h.topics.voice={state:'listening',capture_mode:'audio_test',listening:true,capture_active:true,
    stop_request_seq:5,stop_request_accepted:false,stop_request_reason:'audio_test_requires_own_stop'};
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureTime').textContent,'结束请求未执行');
  assert.match(h.element('micNote').textContent,/收声测试.*测试未被中断/);
});

test('terminal capture errors remain errors instead of perpetual stopping or false closure', () => {
  const h=harness();
  h.state.voiceControl={stop:{requestId:'end',baselineStopSeq:0,error:''}};
  h.topics.voice={state:'asr_unavailable_or_rejected',listening:false,capture_active:false,
    stop_requested:true,stop_request_seq:1,stop_request_accepted:true,capture_stop_confirmed:false,
    reason:'capture_shutdown_failed'};
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureTime').textContent,'语音处理未完成');
  assert.match(h.element('micNote').textContent,/capture_shutdown_failed/);
  assert.doesNotMatch(h.element('micNote').textContent,/采集已停止|麦克风已停止/);
});

test('immediate End cancels the in-flight Start ID without waiting for its HTTP response', async () => {
  const calls=[],pending=[],toasts=[],state={};let serial=0;
  const begin=source.indexOf('  async function submitListen('),end=source.indexOf('  for(const [id,enabled]',begin);
  const context=vm.createContext({state,uuid:()=>`id-${++serial}`,data:()=>({stop_requested_at:12}),
    renderControls:()=>{},renderVoiceCapture:()=>{},toast:message=>toasts.push(message),
    request:(url,payload)=>{calls.push({url,payload});return new Promise((resolve,reject)=>pending.push({resolve,reject}));}});
  vm.runInContext(source.slice(begin,end)+'\nglobalThis.submit=submitListen;',context);
  const start=context.submit(true),stop=context.submit(false);
  assert.equal(calls.length,2);
  assert.equal(calls[1].payload.enabled,false);
  assert.equal(calls[1].payload.cancel_start_request_id,calls[0].payload.request_id);
  await context.submit(false);assert.equal(calls.length,2,'repeated End while pending is coalesced');
  pending[1].resolve({command_id:'end'});await stop;
  pending[0].reject(new Error('superseded_by_listen_stop'));await start;
  assert.equal(toasts.length,1);assert.match(toasts[0],/结束请求/);
  assert.equal(state.voiceControl.pendingStart,null);assert.equal(state.voiceControl.pendingStop,null);
});

test('consecutive recordings and failed requests never reuse prior stop state or leave buttons pending', async () => {
  const calls=[],pending=[],state={};let serial=0,seq=0;
  const begin=source.indexOf('  async function submitListen('),end=source.indexOf('  for(const [id,enabled]',begin);
  const context=vm.createContext({state,uuid:()=>`round-${++serial}`,
    data:()=>({stop_request_seq:seq,stop_requested_at:seq}),renderControls:()=>{},renderVoiceCapture:()=>{},toast:()=>{},
    request:(url,payload)=>{calls.push(payload);return new Promise((resolve,reject)=>pending.push({resolve,reject}));}});
  vm.runInContext(source.slice(begin,end)+'\nglobalThis.submit=submitListen;',context);
  for(let round=0;round<3;round++){
    const start=context.submit(true),startId=calls.at(-1).request_id;
    assert.equal(state.voiceControl.stop,null,'new recording clears previous End receipt');
    pending.at(-1).resolve({command_id:startId});await start;
    const stop=context.submit(false),stopId=calls.at(-1).request_id;
    assert.equal(calls.at(-1).cancel_start_request_id,startId);
    assert.equal(state.voiceControl.stop.baselineStopSeq,seq);
    pending.at(-1).resolve({command_id:stopId});await stop;seq++;
    assert.equal(state.voiceControl.pendingStart,null);assert.equal(state.voiceControl.pendingStop,null);
  }
  const failed=context.submit(true);pending.at(-1).reject(new Error('request_timeout'));await failed;
  assert.equal(state.voiceControl.pendingStart,null);
  const retry=context.submit(true);pending.at(-1).resolve({command_id:'retry'});await retry;
  assert.equal(state.voiceControl.pendingStart,null);assert.equal(state.voiceControl.stop,null);
  assert.equal(new Set(calls.map(v=>v.request_id)).size,calls.length);
});

test('a recovered second recording replaces the first transcription error rather than looking frozen', () => {
  const h=harness();
  h.topics.voice={state:'idle',listening:false,capture_active:false,
    last_recognition:{state:'asr_unavailable_or_rejected',reason:'transcription_timeout'}};
  h.renderVoiceCapture();assert.equal(h.element('voiceCaptureTime').textContent,'语音处理未完成');
  h.recording({elapsed_s:1,last_recognition:h.topics.voice.last_recognition});
  h.state.voiceControl={stop:null};h.renderVoiceCapture();
  assert.match(h.element('voiceCaptureTime').textContent,/已收音 1.0/);
  assert.doesNotMatch(h.element('micNote').textContent,/transcription_timeout|语音处理未完成/);
});

test('fresh status heartbeats cannot refresh stale or future audio levels', () => {
  const h = harness();
  h.recording({}); h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureLevel').textContent, '-18 dBFS');
  h.advance(2);
  h.topics.voice.stamp = 102; // Heartbeat fresh, actual sample still from 100.
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureLevel').textContent, '正在采集');
  h.topics.voice.level_stamp = 103;
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureLevel').textContent, '正在采集');
  h.freshness.voice = false;
  h.renderVoiceCapture();
  assert.equal(h.element('voiceCaptureLevel').textContent, '待收音');
});

test('independent microphone diagnostics never appear as formal command capture', () => {
  const h = harness();
  h.recording({ capture_mode: 'audio_test' });
  h.renderVoiceCapture(); h.renderDialogueActivity();
  assert.equal(h.element('voiceCaptureProgress').value, 0);
  assert.equal(h.element('voiceCaptureTime').textContent, '正在独立收声测试');
  assert.equal(h.element('voiceCaptureLevel').textContent, '仅测试');
  assert.doesNotMatch(h.element('dialogueActivity').textContent, /先完成收音与转写/);
});

test('Qwen thinking reflects fresh node state and exposes errors without an invented answer', () => {
  const h = harness();
  for (const state of ['thinking', 'generating', 'processing']) {
    h.topics.dialogue_status = { state, elapsed_s: 3 };
    h.renderDialogueActivity();
    assert.equal(h.element('dialogueActivity').dataset.state, 'thinking');
    assert.match(h.element('dialogueActivity').textContent, /本地 Qwen 正在思考 · 3 秒/);
  }
  h.freshness.dialogue_status = false;
  h.renderDialogueActivity();
  assert.equal(h.element('dialogueActivity').dataset.state, 'idle');
  assert.doesNotMatch(h.element('dialogueActivity').textContent, /正在思考/);
  h.freshness.dialogue_status = true;
  h.topics.dialogue_status = { state: 'timeout', reason: 'edge_model_timeout' };
  h.renderDialogueActivity();
  assert.equal(h.element('dialogueActivity').dataset.state, 'error');
  assert.match(h.element('dialogueActivity').textContent, /edge_model_timeout/);
});

test('voice command receipts distinguish submission, routing acceptance and rejection by command ID', () => {
  const h = harness();
  const transcript = { text: '前进二十厘米', command_id: 'voice-1', action: 'manual_move', submitted: true };
  assert.match(h.voiceReceiptMeta(transcript), /已提交，等待任务回执/);
  h.events.push({ kind: 'interaction', data: { command_id: 'another', accepted: true } });
  assert.match(h.voiceReceiptMeta(transcript), /等待任务回执/);
  h.events.push({ topic: '/interaction/result', data: { command_id: 'voice-1', accepted: false, reason: 'manual_motion_not_enabled' } });
  assert.match(h.voiceReceiptMeta(transcript), /未执行：manual_motion_not_enabled/);
  h.events.push({ kind: 'interaction', data: { command_id: 'voice-1', accepted: true } });
  assert.match(h.voiceReceiptMeta(transcript), /已接收，动作进度以实车反馈为准/);
  assert.match(h.voiceReceiptMeta({ ...transcript, action: 'dialogue' }), /已接收，等待回答/);
  assert.match(h.voiceReceiptMeta({ ...transcript, submitted: false, route_error: 'wake_word_required' }), /未提交：wake_word_required/);
});

test('the actual dialogue renderer visibly preserves rejected voice text and its reason', () => {
  const h = harness();
  h.events.push({ id: 1, at: 100, kind: 'transcript', topic: '/voice/transcript',
    data: { text: '前进二十厘米', command_id: 'voice-1', action: 'manual_move', submitted: true } });
  h.events.push({ id: 2, at: 101, kind: 'interaction', topic: '/interaction/result',
    data: { command_id: 'voice-1', accepted: false, reason: 'manual_requires_confirmed_stop' } });
  h.renderDialogue();
  const html = h.element('dialogueMessages').innerHTML;
  assert.match(html, /前进二十厘米/);
  assert.match(html, /未执行：manual_requires_confirmed_stop/);
  assert.doesNotMatch(html, /动作完成/);
});

test('Qwen navigation understanding is pending, then one grounded reply stays below its original voice', () => {
  const h=harness(),original='你现在在8号点位，车头朝左，请导航前往至4号点位';
  h.events.push({kind:'transcript',at:95,data:{command_id:'nav',text:original,action:'semantic_navigate',submitted:true}});
  h.events.push({kind:'interaction',at:96,data:{command_id:'nav',stamp:96,accepted:false,status:'understanding',
    semantic:{status:'understanding',original_text:original,model:'qwen3-1.7b',model_called:true}}});
  h.renderDialogue();
  let html=h.element('dialogueMessages').innerHTML;
  assert.match(html,/本地 Qwen 正在理解点位和车头方向/);
  assert.doesNotMatch(html,/未执行：|未接受/);
  h.events.push({kind:'interaction',at:98,data:{command_id:'nav',stamp:98,action:'navigate_point',accepted:true,
    speech:'Qwen理解为：8号点到4号点，车头朝左（180°）；执行以FPGA回执为准。',
    semantic:{status:'ready',original_text:original,model:'qwen3-1.7b',model_called:true,elapsed_s:2}}});
  h.renderDialogue();html=h.element('dialogueMessages').innerHTML;
  assert.ok(html.indexOf(original)<html.indexOf('Qwen理解为'));
  assert.match(html,/qwen3-1.7b · 2.0 秒/);
  assert.equal((html.match(/class="dialogue-turn"/g)||[]).length,1);
  assert.doesNotMatch(html,/正在理解点位/);
});

test('semantic clarification displays the actual missing information instead of a generic rejection', () => {
  const h=harness();
  h.events.push({kind:'interaction',at:100,data:{command_id:'clarify',stamp:100,accepted:false,
    speech:'请补充车头当前朝向，向地图左侧还是右侧？',reason:'heading_missing',
    semantic:{status:'clarification',original_text:'从8号点去4号点',model:'qwen3-1.7b',model_called:true}}});
  h.renderDialogue();const html=h.element('dialogueMessages').innerHTML;
  assert.match(html,/请补充车头当前朝向/);assert.match(html,/未下发导航/);
  assert.doesNotMatch(html,/这次问题未接受|已请求FPGA/);
  assert.equal(h.collectDialogueTurns()[0].text,'从8号点去4号点');
});

test('late pending snapshot cannot replace the completed semantic result', () => {
  const h=harness();
  const final={command_id:'nav',stamp:99,accepted:true,speech:'已理解并提交',
    semantic:{status:'ready',original_text:'去4号点',model_called:true}};
  h.events.push({kind:'interaction',at:99,data:final});h.collectDialogueTurns();
  h.events.push({kind:'interaction',at:101,data:{command_id:'nav',stamp:97,accepted:false,
    semantic:{status:'understanding',original_text:'去4号点',model_called:true}}});
  let turn=h.collectDialogueTurns()[0];
  assert.equal(turn.semantic.status,'ready');assert.equal(turn.answer.text,'已理解并提交');
  h.events.length=0;
  h.state.data.commands=[{command_id:'nav',text:'去4号点',queued_at:96,receipt:final}];
  turn=h.collectDialogueTurns()[0];assert.equal(turn.answer.text,'已理解并提交');
});

test('semantic navigation text and model reply are escaped and cancellation never claims execution', () => {
  const h=harness();
  h.events.push({kind:'interaction',at:100,data:{command_id:'nav',stamp:100,accepted:false,
    speech:'<b>已取消</b>',reason:'semantic_navigation_cancelled_by_command',
    semantic:{status:'cancelled',original_text:'去4号点<script>',model_called:true}}});
  h.renderDialogue();const html=h.element('dialogueMessages').innerHTML;
  assert.match(html,/&lt;b&gt;已取消&lt;\/b&gt;/);assert.match(html,/&lt;script&gt;/);
  assert.doesNotMatch(html,/<script>|已请求FPGA|任务已提交/);
});

test('rule navigation replies under the original voice without waiting for Qwen', () => {
  const h=harness(),original='你现在在8号点位，车头朝左，请导航前往至4号点位';
  h.events.push({kind:'transcript',at:95,data:{command_id:'rule-nav',text:original,action:'navigate_point',submitted:true}});
  h.events.push({kind:'interaction',at:96,data:{command_id:'rule-nav',stamp:96,action:'navigate_point',accepted:true,
    navigation_parser:'rules',speech:'规则解析：8号点到4号点，车头朝左；已提交FPGA，等待执行反馈。',
    rule_navigation:{original_text:original,start_point:8,target_point:4,start_heading_deg:180,model_called:false}}});
  h.renderDialogue();const html=h.element('dialogueMessages').innerHTML;
  assert.ok(html.indexOf(original)<html.indexOf('规则解析：8号点'));
  assert.match(html,/规则导航 · 文字与语音统一解析/);
  assert.match(html,/等待导航执行节点回执/);
  assert.doesNotMatch(html,/Qwen|正在思考|到达完成/);
  assert.equal((html.match(/class="dialogue-turn"/g)||[]).length,1);
});

test('rule rejection preserves missing-heading explanation and does not claim execution', () => {
  const h=harness();
  h.events.push({kind:'interaction',at:100,data:{command_id:'rule-heading',stamp:100,action:'navigate_point',accepted:false,
    speech:'请补充车头当前朝向，本次未下发导航。',reason:'numbered_start_heading_required',
    rule_navigation:{original_text:'你现在在8号点，请前往4号点',start_point:8,target_point:4,start_heading_deg:null,model_called:false}}});
  h.renderDialogue();const html=h.element('dialogueMessages').innerHTML;
  assert.match(html,/请补充车头当前朝向/);assert.match(html,/规则导航/);
  assert.doesNotMatch(html,/Qwen|等待回答|这次问题未接受|任务已提交/);
});

test('rules and real Qwen scene answers remain separate even when their receipts interleave', () => {
  const h=harness();
  h.state.localMessages=[{id:'scene',text:'分析当前火场',at:95,action:'scene'},
    {id:'route',text:'我现在在8号点车头朝左请前往4号点',at:96,action:'navigate_point'}];
  h.topics.dialogue_status={command_id:'scene',state:'thinking',elapsed_s:4};
  h.events.push({kind:'interaction',at:97,data:{command_id:'route',stamp:97,accepted:true,
    speech:'规则导航已提交',rule_navigation:{original_text:'我现在在8号点车头朝左请前往4号点',model_called:false}}});
  h.events.push({kind:'scene_answer',at:98,data:{command_id:'scene',stamp:98,accepted:true,
    text:'根据视觉结果，当前有一处火焰。',backend:'qwen',response_scope:'historical_snapshot_analysis'}});
  h.renderDialogue();const turns=h.collectDialogueTurns();
  assert.equal(turns.find(t=>t.id==='route').answer.backend,'rule_navigation');
  assert.equal(turns.find(t=>t.id==='scene').answer.backend,'qwen');
  const html=h.element('dialogueMessages').innerHTML;
  assert.match(html,/规则导航/);assert.match(html,/本地 Qwen/);
  assert.equal((html.match(/class="dialogue-turn"/g)||[]).length,2);
});


test('exact navigation executor rejection replaces premature router speech below original question', () => {
  const h=harness();
  h.state.localMessages=[{id:'nav',text:'我在8号点，请到4号点',at:95}];
  h.events.push({id:1,at:96,kind:'interaction',data:{command_id:'nav',action:'navigate_point',accepted:true,
    rule_navigation:{original_text:'我在8号点，请到4号点'},speech:'已提交FPGA规划与执行'}});
  h.events.push({id:2,at:97,kind:'extension_result',data:{command_id:'nav',stamp:97,accepted:false,
    completed:false,reason:'pump_off_fresh_actuator_feedback_required'}});
  h.renderDialogue();const html=h.element('dialogueMessages').innerHTML;
  assert.match(html,/导航未执行：pump_off_fresh_actuator_feedback_required/);
  assert.doesNotMatch(html,/已提交FPGA规划与执行|回执超时/);
  assert.ok(html.indexOf('我在8号点')<html.indexOf('导航未执行'));
});

test('voice executor outcome arriving before transcript retains exact ID without proximity guessing', () => {
  const h=harness();
  h.events.push({id:1,at:96,kind:'extension_result',data:{command_id:'voice',stamp:96,accepted:false,reason:'blocked'}});
  h.collectDialogueTurns();h.events.length=0;
  h.events.push({id:2,at:97,kind:'transcript',data:{command_id:'other',text:'另一条语音',submitted:true}},
    {id:3,at:98,kind:'transcript',data:{command_id:'voice',text:'前往4号点',action:'navigate_point',submitted:true}});
  const turns=h.collectDialogueTurns();
  assert.equal(turns.find(t=>t.id==='voice').execution.reason,'blocked');
  assert.equal(turns.find(t=>t.id==='voice').text,'前往4号点');
  assert.equal(turns.find(t=>t.id==='other').execution,undefined);
});

test('reload restores executor result and older accepted heartbeats cannot revive rejected navigation', () => {
  const h=harness();
  h.state.data.commands=[{command_id:'nav',kind:'command',text:'前往4号点',action:'navigate_point',queued_at:90,
    execution:{command_id:'nav',stamp:95,accepted:false,reason:'blocked'},
    receipt:{command_id:'nav',stamp:93,accepted:true,rule_navigation:{},speech:'已提交执行'}}];
  h.topics.extension={last_result:{command_id:'nav',stamp:94,accepted:true,completed:false,reason:'request_accepted_waiting_hardware'}};
  h.renderDialogue();assert.match(h.element('dialogueMessages').innerHTML,/导航未执行：blocked/);
  h.state.data.commands=[];h.topics.extension={last_result:{command_id:'nav',stamp:99,accepted:true,completed:false}};
  assert.equal(h.collectDialogueTurns()[0].execution.accepted,false);
});

test('unidentified and synthetic execution outcomes are never attached to navigation', () => {
  const h=harness();h.state.localMessages=[{id:'nav',text:'前往4号点',at:95}];
  h.events.push({at:96,kind:'extension_result',data:{stamp:96,accepted:false,reason:'no-id'}},
    {at:96,kind:'extension_result',synthetic:true,data:{command_id:'nav',stamp:96,accepted:false}},
    {at:96,kind:'extension_result',data:{command_id:'nav',stamp:96,accepted:false,synthetic:true}});
  h.topics.extension={synthetic:true,last_result:{command_id:'nav',stamp:96,accepted:false}};
  assert.equal(h.collectDialogueTurns()[0].execution,undefined);
});

test('executor acceptance is displayed as waiting hardware, never as confirmed movement', () => {
  const h=harness();
  const status=h.dialogueTurnStatus({id:'nav',action:'navigate_point',execution:{accepted:true,completed:false}});
  assert.match(status.text,/等待 FPGA 确认执行/);assert.match(status.text,/尚未确认车辆运动/);
});

test('TTS device loss persists alongside idle microphone; synthesis is distinct from playback', () => {
  const h=harness();
  const missing=h.speechPlaybackStatus({state:'idle',tts_enabled:true,tts_state:'unavailable',
    tts_error:'audio_output_device_missing: plughw:CARD=RescueSpeaker,DEV=0',tts_device_ready:false},true);
  assert.equal(missing.label,'USB 音箱未连接');assert.equal(missing.state,'error');
  assert.match(h.speechPlaybackStatus({tts_enabled:true,tts_state:'synthesizing'},true).detail,/尚未开始播放/);
  assert.equal(h.speechPlaybackStatus({tts_enabled:true,tts_state:'playing'},true).label,'正在音箱播报');
  assert.equal(h.speechPlaybackStatus({tts_state:'playing'},false).label,'等待音箱状态');
});


test('parser acceptance without executor outcome reports that specific timeout, including voice commands', () => {
  const h=harness();h.state.data.server_time=105;
  h.events.push({at:95,kind:'transcript',data:{command_id:'nav',text:'前往4号点',action:'navigate_point',submitted:true}},
    {at:96,kind:'interaction',data:{command_id:'nav',stamp:96,action:'navigate_point',accepted:true,
      rule_navigation:{},speech:'等待执行端确认'}});
  h.renderDialogue();
  assert.match(h.element('dialogueMessages').innerHTML,/导航指令已解析，但未收到执行节点回执/);
  assert.doesNotMatch(h.element('dialogueMessages').innerHTML,/等待执行端确认/);
  h.events.push({at:105,kind:'extension_result',data:{command_id:'nav',stamp:105,accepted:false,reason:'real-gate'}});
  h.renderDialogue();assert.match(h.element('dialogueMessages').innerHTML,/导航未执行：real-gate/);
  assert.doesNotMatch(h.element('dialogueMessages').innerHTML,/但未收到执行节点回执/);
});
