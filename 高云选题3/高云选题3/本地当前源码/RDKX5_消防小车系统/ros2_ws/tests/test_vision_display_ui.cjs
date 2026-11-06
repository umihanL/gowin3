'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const script = fs.readFileSync(path.join(__dirname, '../web/presentation.js'), 'utf8');
const first = script.indexOf('  function visionDisplayGeometry(');
const last = script.indexOf('  function setupVisionInspection()', first);
assert.ok(first >= 0 && last > first);
const scope = {};
vm.runInNewContext(script.slice(first, last), scope);
const calculate = scope.visionDisplayGeometry;

test('complete image keeps the source aspect ratio within a wide monitor panel', () => {
  const view = calculate('fit', 1280, 1088, 1000, 500, 1);
  assert.ok(view.width <= 1000 && view.height <= 500);
  assert.equal(view.height, 500);
  assert.equal(view.width / view.height, 1280 / 1088);
});

test('original-pixel mode maps each received pixel to one physical display pixel', () => {
  for (const ratio of [1, 1.25, 1.5, 2]) {
    const view = calculate('native', 1280, 1088, 600, 500, ratio);
    assert.equal(view.width * ratio, 1280);
    assert.equal(view.height * ratio, 1088);
  }
});

test('detail mode shows two screen pixels per received pixel without changing the source', () => {
  const view = calculate('detail', 1280, 1088, 600, 500, 1.25);
  assert.equal(view.width * 1.25, 2560);
  assert.equal(view.height * 1.25, 2176);
});

test('invalid pixel ratios fall back to ordinary physical pixel mapping', () => {
  for (const ratio of [0, -1, NaN, Infinity]) {
    const view = calculate('native', 1280, 1088, 600, 500, ratio);
    assert.equal(view.width, 1280);
    assert.equal(view.height, 1088);
  }
});

function statusHarness() {
  const elements = new Map(), topics = {}, stale = new Set();
  const $ = id => {
    if (!elements.has(id)) elements.set(id, {textContent:'',innerHTML:'',dataset:{},
      classList:{toggle(){}},parentElement:{style:{setProperty(){}}}});
    return elements.get(id);
  };
  const state={data:{ros:{available:true},control:{command_ready:true},server_time:100},frameAt:0};
  const context=vm.createContext({$,state,connected:()=>true,
    data:key=>stale.has(key)?{}:topics[key]||{},fresh:key=>Boolean(topics[key])&&!stale.has(key),
    document:{body:{dataset:{}},querySelectorAll:()=>[]},modeNames:{},
    performance:{now:()=>1000},clock:String,badge(){},esc:String,
    text:(id,value)=>{$(id).textContent=value;},html:(id,value)=>{$(id).innerHTML=value;},
    finite:Number.isFinite,num:(value,digits=1)=>Number.isFinite(value)?value.toFixed(digits):'—',
    reason:value=>value==='yielding_to_dialogue'?'正在让出算力给对话，识别稍后恢复':value,
    visibleCounts:()=>({available:false}),renderTaskWorkflow(){},
  });
  const status=script.slice(script.indexOf('  function renderStatus()'),script.indexOf('  function visibleCounts()'));
  const building=script.slice(script.indexOf('  function renderBuilding()'),script.indexOf('  function renderNavigation()'));
  vm.runInContext(status+building+'\nglobalThis.renderers={renderStatus,renderBuilding};',context);
  return {...context.renderers,$,topics,stale};
}

test('question-time frame cap appears only when fresh diagnostics actually report yielding', () => {
  const h=statusHarness();h.topics.perception_diagnostics={performance:{processing_fps:5.8,effective_process_fps:6,dialogue_yielding:true}};
  h.renderStatus();assert.equal(h.$('fpsDetail').textContent,'问答时上限 6 帧，完成自动恢复');
  h.topics.perception_diagnostics.performance.dialogue_yielding=false;h.renderStatus();
  assert.doesNotMatch(h.$('fpsDetail').textContent,/问答/);
  h.topics.perception_diagnostics.performance.dialogue_yielding=true;h.stale.add('perception_diagnostics');h.renderStatus();
  assert.doesNotMatch(h.$('fpsDetail').textContent,/问答/);
});

test('OCR yielding has an explicit Chinese explanation and disappears when evidence expires', () => {
  const h=statusHarness();h.topics.ocr_diagnostics={yielding_to_dialogue:true};
  h.renderBuilding();assert.equal(h.$('buildingName').textContent,'正在让出算力给对话，识别稍后恢复');
  h.stale.add('ocr_diagnostics');h.renderBuilding();assert.equal(h.$('buildingName').textContent,'等待有效楼牌观测');
});
