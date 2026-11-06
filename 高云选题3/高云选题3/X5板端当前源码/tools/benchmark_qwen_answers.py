#!/usr/bin/env python3
"""Read-only real HMI questions; retain answers, latency and vision workload evidence.

Run on X5. This never records audio, changes mission mode or submits motion commands.
The report contains genuine current scene observations, not synthetic detections.
"""
import argparse
import json
from pathlib import Path
import time
import urllib.error
import urllib.request
import uuid

QUESTIONS = [
    '为什么只用陀螺仪不能长期准确定位小车？请用两句话说明原因和改进方法。',
    '如何区分相关性和因果关系？举一个简短的例子。',
    '一辆小车匀速每秒0.2米，要走3米，已经走了5秒，还要多久？请给出计算。',
    '分析当前场景，应该优先复核什么？',
    '分析一下场景里有几团火焰',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    base = 'http://127.0.0.1:8090'

    def state():
        return json.load(opener.open(base+'/api/state', timeout=3))

    results = []
    for q in QUESTIONS:
        s = state()
        if s['topics']['voice']['data'].get('listening') or s['topics']['dialogue_status']['data'].get('busy'):
            raise RuntimeError('User audio or dialogue is busy; benchmark stopped')
        if not s['control']['command_ready']:
            raise RuntimeError('HMI command endpoint not ready')
        start = time.monotonic()
        body = json.dumps({'text': q, 'request_id': 'audit-'+uuid.uuid4().hex}).encode()
        req = urllib.request.Request(base+'/api/command', data=body, headers={
            'Content-Type': 'application/json', 'Origin': base})
        try:
            ack = json.load(opener.open(req, timeout=5))
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f'HMI rejected read-only test: {exc.code} {exc.read(2000).decode()}') from None
        cid = ack['command_id']
        samples = []
        while time.monotonic()-start < 90:
            time.sleep(.25)
            topics = state()['topics']
            answer = topics.get('scene_answer', {}).get('data', {})
            p = topics.get('perception_diagnostics', {}).get('data', {})
            samples.append({k: p.get(k) for k in ('processing_fps', 'dialogue_yielding',
                'effective_process_fps', 'image_age_s', 'preview_subscribers')})
            if answer.get('command_id') == cid:
                item = dict(question=q, elapsed_s=time.monotonic()-start, answer=answer, samples=samples)
                results.append(item)
                output.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
                print(json.dumps(dict(question=q, elapsed_s=item['elapsed_s'], text=answer.get('text'),
                    backend=answer.get('backend'), model_error=answer.get('model_error')), ensure_ascii=False), flush=True)
                break
        else:
            raise RuntimeError('Answer timeout')
        time.sleep(1.)  # Allow status to return idle before the next independent question.


if __name__ == '__main__':
    main()
