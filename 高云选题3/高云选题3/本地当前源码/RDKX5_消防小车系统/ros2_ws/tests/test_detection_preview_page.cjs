'use strict';

// Production standalone preview, fake browser/network/time: no camera or ROS.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');
const page = fs.readFileSync(path.join(__dirname, '../web/detection-preview.html'), 'utf8');
const source = page.match(/<script>([\s\S]*?)<\/script>/)[1];

function harness(hidden = false) {
  const requests = [], timers = new Map(), events = new Map(), elements = new Map(), bitmaps = [];
  let serial = 0;
  const canvasContext = {drawImage() {}, measureText: () => ({width: 20}), strokeRect() {}, fillRect() {}, fillText() {}};
  const element = id => {
    if (!elements.has(id)) elements.set(id, {textContent: '', checked: true, disabled: false,
      classList: {add() {}}, parentElement: {style: {}}, width: 1280, height: 1088,
      getContext: () => canvasContext, addEventListener(type, callback) {this[type] = callback;}});
    return elements.get(id);
  };
  const doc = {hidden, getElementById: element, addEventListener: (name, fn) => events.set(name, fn)};
  const context = vm.createContext({document: doc,
    window: {addEventListener: (name, fn) => events.set(name, fn)},
    performance: {now: () => 1000}, TextDecoder, Blob, AbortController, AbortSignal,
    setTimeout: (fn, ms) => {timers.set(++serial, {fn, ms}); return serial;},
    clearTimeout: id => timers.delete(id),
    fetch: (url, options) => new Promise((resolve, reject) => requests.push({url, options, resolve, reject})),
    createImageBitmap: async () => {const bitmap = {width:1280,height:1088,closed:false,close(){this.closed=true;}};bitmaps.push(bitmap);return bitmap;},
  });
  vm.runInContext(source + '\nglobalThis.feature={poll,view:()=>({requested,polling,token,timer,state,bitmap,controlEpoch,toggleBusy})};', context);
  const settle = () => new Promise(resolve => setImmediate(resolve));
  return {...context.feature, requests, timers, bitmaps, element, settle,
    visibility(value) {doc.hidden=value;events.get('visibilitychange')();},
    event(name) {events.get(name)();},
    tick() {const id=context.feature.view().timer;if(timers.has(id)){const task=timers.get(id);timers.delete(id);task.fn();}},
    async finish(index, fields={}) {
      const metadata = {transport_version:1, state:'waiting', stream:{enabled:true,active:true,generation:1},
        frame:null,frame_age_s:0,update_age_s:0,...fields};
      const json=Buffer.from(JSON.stringify(metadata)),prefix=Buffer.alloc(4);prefix.writeUInt32BE(json.length);
      const packet=Buffer.concat([prefix,json,...(metadata.frame?[Buffer.from('fakejpeg')]:[])]);
      requests[index].resolve({ok:true,status:200,json:async()=>metadata,
        arrayBuffer:async()=>packet.buffer.slice(packet.byteOffset,packet.byteOffset+packet.length)});
      await settle();
    },
  };
}

test('loading hidden does not request or renew the stream; visible polling stays single-flight', async () => {
  const h=harness(true);assert.equal(h.requests.length,0);assert.equal(h.timers.size,0);
  h.visibility(false);h.tick();await h.poll();
  assert.equal(h.requests.length,1);assert.match(h.requests[0].url,/^\/api\/frame/);
  await h.finish(0);assert.equal(h.view().polling,false);assert.equal(h.timers.size,1);
});

test('hiding aborts the current GET, discards its late frame, and sends no global switch command', async () => {
  const h=harness();assert.equal(h.requests.length,1);
  h.visibility(true);assert.equal(h.requests[0].options.signal.aborted,true);
  await h.finish(0,{state:'live',frame:{token:'late',width:1280,height:1088,detections:[]}});
  assert.equal(h.bitmaps.length,0);assert.equal(h.view().token,'');assert.equal(h.timers.size,0);
  assert.equal(h.view().requested,true);assert.equal(h.requests.length,1);
  assert.equal(h.requests.some(item=>item.options.method==='POST'),false);
});

test('hide/show before the old fetch settles resumes once without parallel requests', async () => {
  const h=harness();h.visibility(true);h.visibility(false);h.tick();
  assert.equal(h.requests.length,1);assert.equal(h.view().polling,true);
  h.requests[0].reject(new Error('aborted'));await h.settle();
  h.tick();await h.poll();assert.equal(h.requests.length,2);
  await h.finish(1);assert.equal(h.view().polling,false);assert.equal(h.timers.size,1);
});

test('explicitly disabled transmission remains disabled after hiding and returning', async () => {
  const h=harness();await h.finish(0,{stream:{enabled:false,active:false,generation:2},state:'paused'});
  assert.equal(h.view().requested,false);h.visibility(true);h.visibility(false);h.tick();
  assert.match(h.requests[1].url,/^\/api\/state\?images=0/);
  assert.equal(h.element('streamToggle').checked,false);assert.equal(h.view().state,'paused');
  await h.finish(1,{stream:{enabled:false,active:false,generation:2},state:'paused'});
  assert.equal(h.requests.some(item=>item.options.method==='POST'),false);
});

test('pagehide releases the bitmap and a restored page fetches a new frame', async () => {
  const h=harness();await h.finish(0,{state:'live',frame:{token:'shown',sequence:1,width:1280,height:1088,detections:[]}});
  assert.equal(h.view().token,'shown');h.event('pagehide');
  assert.equal(h.bitmaps[0].closed,true);assert.equal(h.view().bitmap,null);
  assert.equal(h.timers.size,0);assert.equal(h.view().requested,true);
  h.event('pageshow');h.tick();assert.equal(h.requests[1].url,'/api/frame?after=');
});

test('a pending explicit switch does not restart hidden polling or lose the user choice', async () => {
  const h=harness();await h.finish(0);
  h.element('streamToggle').checked=false;const changing=h.element('streamToggle').change();
  assert.equal(h.requests[1].url,'/api/stream');h.visibility(true);
  await h.finish(1,{stream:{enabled:false,active:false,generation:2},state:'paused'});await changing;
  assert.equal(h.view().requested,false);assert.equal(h.timers.size,0);
  h.visibility(false);h.tick();assert.match(h.requests[2].url,/^\/api\/state\?images=0/);
});
