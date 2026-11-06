'use strict';

// Exercise the production UI in an isolated VM; no DOM, network, ROS or microphone.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../web/presentation.js'), 'utf8');
const start = source.indexOf('  const audioTestUI=');
const end = source.indexOf('  function renderControls()', start);
assert.ok(start >= 0 && end > start, 'The production audio-test block must be found');
const fragment = source.slice(start, end);

function harness() {
  const elements = new Map();
  const topics = { voice: {}, audio_test: null };
  const freshness = { voice: true, audio_test: true };
  const requests = [];
  let now = 1000;
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      textContent: '', style: {}, dataset: {}, attributes: {}, disabled: false,
      classList: { toggle() {} }, addEventListener() {},
      setAttribute(key, value) { this.attributes[key] = String(value); },
      showModal() { this.open = true; }, close() { this.open = false; this.closed = true; },
    });
    return elements.get(id);
  };
  const state = { received: now, data: {
    server_time: 100, commands: [], control: { audio_test_ready: true },
    ros: { available: true, audio_test_ready: true },
  } };
  const context = vm.createContext({
    state, performance: { now: () => now }, $: element,
    data: key => freshness[key] ? topics[key] || {} : {},
    seen: key => topics[key], fresh: key => freshness[key],
    connected: () => true,
    finite: value => typeof value === 'number' && Number.isFinite(value),
    num: (value, digits = 0) => typeof value === 'number' ? value.toFixed(digits) : '—',
    reason: value => value, clock: value => String(value),
    text: (id, value) => { element(id).textContent = value; },
    uuid: () => 'request-' + (requests.length + 1), toast() {},
    request: async (url, payload) => {
      requests.push({ url, ...payload });
      return { command_id: 'command-' + requests.length };
    },
  });
  vm.runInContext(fragment + '\nglobalThis.feature={audioTestUI,renderAudioTest,submitAudioTest,closeAudioTest};', context);
  return { ...context.feature, element, topics, freshness, requests, state,
    advance: seconds => { now += seconds * 1000; },
    feedback: fields => {
      topics.audio_test = { command_id: 'command-1', session_id: 'command-1',
        request_id: 'command-1', request_accepted: true, state: 'listening',
        capture_active: true, level_stamp: 100, level_dbfs: -18, peak_dbfs: -9,
        elapsed_s: 1, duration_s: 10, sampled_s: 1, ...fields };
    },
  };
}

test('accepted request keeps its session identity and hides another session result', async () => {
  const h = harness();
  await h.submitAudioTest(true);
  h.feedback({});
  h.renderAudioTest();
  assert.equal(h.audioTestUI.requestId, null);
  assert.equal(h.audioTestUI.sessionId, 'command-1');
  h.feedback({ command_id: 'other', session_id: 'other', request_id: 'other',
    state: 'completed', capture_active: false, text: '外部会话文字' });
  h.renderAudioTest();
  assert.equal(h.element('audioTestText').textContent, '等待本次识别文字');
  assert.equal(h.element('audioTestMeterFill').style.width, '0%');
});

test('rejected start does not relabel an earlier transcript as this test', async () => {
  const h = harness();
  await h.submitAudioTest(true);
  h.feedback({ command_id: 'previous', session_id: 'previous', request_id: 'command-1',
    request_accepted: false, request_rejected: true, request_reason: 'microphone_busy',
    state: 'completed', capture_active: false, text: '旧结果' });
  h.renderAudioTest();
  assert.match(h.element('audioTestHint').textContent, /microphone_busy/);
  assert.equal(h.element('audioTestText').textContent, '等待本次识别文字');
});

test('stop rejection is correlated and displayed while original session is preserved', async () => {
  const h = harness();
  await h.submitAudioTest(true);
  h.feedback({});
  h.renderAudioTest();
  await h.submitAudioTest(false);
  assert.equal(h.audioTestUI.requestId, 'command-2');
  h.feedback({ request_id: 'command-2', request_accepted: false, request_rejected: true,
    request_reason: 'expired_or_future_audio_test_request' });
  h.renderAudioTest();
  assert.match(h.element('audioTestHint').textContent, /expired_or_future_audio_test_request/);
  assert.equal(h.audioTestUI.requestId, null);
  assert.equal(h.audioTestUI.sessionId, 'command-1');
  assert.deepEqual(h.requests.map(r => [r.url, r.enabled]),
    [['/api/audio-test', true], ['/api/audio-test', false]]);
});

test('fresh heartbeat cannot keep an expired audio sample meter alive', () => {
  const h = harness();
  h.feedback({});
  h.renderAudioTest();
  assert.notEqual(h.element('audioTestMeterFill').style.width, '0%');
  h.advance(2);
  h.renderAudioTest();
  assert.equal(h.element('audioTestMeterFill').style.width, '0%');
  assert.equal(h.element('audioTestMeter').attributes['aria-valuenow'], '-96');
  assert.equal(h.element('audioTestPeak').textContent, '—');
  assert.equal(h.element('audioTestLevel').textContent, '没有实时采样');
});

test('closing a read-only view of an external active session does not stop it', () => {
  const h = harness();
  h.topics.voice = { audio_test_active: true, listening: true };
  h.feedback({ command_id: 'external', session_id: 'external', request_id: 'external' });
  h.closeAudioTest();
  assert.equal(h.requests.length, 0);
  assert.equal(h.element('audioTestDialog').closed, true);
});

test('closing this window\'s active session sends a diagnostic stop only', async () => {
  const h = harness();
  await h.submitAudioTest(true);
  h.feedback({});
  h.renderAudioTest();
  h.closeAudioTest();
  await Promise.resolve();
  assert.deepEqual(h.requests.map(r => [r.url, r.enabled]),
    [['/api/audio-test', true], ['/api/audio-test', false]]);
});
