#!/usr/bin/env python3
"""Passive, monotonic live preview diagnostics (no camera/model started)."""
import argparse
import json
from pathlib import Path
import statistics
import time
import urllib.request


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', default='http://192.168.127.10:8086')
    p.add_argument('--seconds', type=int, default=25)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    samples = []
    for _ in range(args.seconds):
        started = time.monotonic()
        with opener.open(args.url + '/api/state?images=0', timeout=3) as response:
            sample = json.load(response)
        sample['benchmark_monotonic'] = started
        samples.append(sample)
        time.sleep(max(0, 1-(time.monotonic()-started)))
    active = [s for s in samples if s.get('performance')]
    median = lambda values: statistics.median(values) if values else None
    summary = dict(samples=len(samples), elapsed_s=time.monotonic()-samples[0]['benchmark_monotonic'],
        processing_fps_median=median([s['performance']['processing_fps'] for s in active]),
        processing_fps_min=min((s['performance']['processing_fps'] for s in active), default=None),
        preview_fps_median=median([s['fps'] for s in active]),
        frame_age_s_median=median([s['frame_age_s'] for s in active if s.get('frame_age_s') is not None]),
        stage_ms_median={k: median([s['performance']['stage_ms'][k] for s in active])
                         for k in active[0]['performance']['stage_ms']} if active else {},
        note='Passive diagnostics: keep a preview browser open for transmission-on measurements.')
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(dict(summary=summary, samples=samples), ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
