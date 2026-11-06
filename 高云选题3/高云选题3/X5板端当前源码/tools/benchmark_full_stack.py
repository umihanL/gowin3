#!/usr/bin/env python3
"""Passive monotonic board benchmark; leaves all mission/hardware states unchanged."""
import argparse
import json
import os
from pathlib import Path
import statistics
import time
import urllib.request

import rclpy
from std_msgs.msg import String


def processes():
    result = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            text = path.read_text()
            tail = text[text.rfind(')')+2:].split()
            cmd = (path.parent/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if '/rescue/' in cmd or 'mipi_cam/mipi_cam' in cmd:
                result[path.parent.name] = dict(ticks=int(tail[11])+int(tail[12]),
                    rss_kib=int(tail[21])*os.sysconf('SC_PAGE_SIZE')//1024,
                    name=text[text.find('(')+1:text.rfind(')')], command=cmd)
        except (OSError, ValueError, IndexError):
            pass
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=30)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('full_stack_passive_benchmark')
    latest={}
    def receive(topic,msg):
        try:latest[topic]=json.loads(msg.data)
        except ValueError:pass
    for topic in ('/mission/state','/actuators/state','/fpga/link','/diagnostics/building_ocr',
                  '/diagnostics/perception','/scene/answer'):
        node.create_subscription(String,topic,lambda msg,t=topic:receive(t,msg),10)
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    samples=[];start=time.monotonic();previous=processes();last=start
    try:
        while time.monotonic()-start<args.seconds:
            until=time.monotonic()+1
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.03)
            now=time.monotonic();current=processes();cpu={}
            for pid,p in current.items():
                if pid in previous:
                    cpu[pid]=dict(name=p['name'],rss_kib=p['rss_kib'],
                        cpu_pct=round((p['ticks']-previous[pid]['ticks'])/os.sysconf('SC_CLK_TCK')/(now-last)*100,2))
            previous,last=current,now
            with opener.open('http://192.168.127.10:8086/api/state?images=0',timeout=3) as r:
                preview=json.load(r)
            samples.append(dict(elapsed_s=now-start,cpu=cpu,preview=preview,topics=dict(latest),
                thermal=[p.read_text().strip() for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')]))
        perf=[s['topics'].get('/diagnostics/perception') or s['preview'].get('performance') or {} for s in samples[3:]]
        median=lambda xs:statistics.median(xs) if xs else None
        summary=dict(samples=len(samples),elapsed_s=time.monotonic()-start,
            median={k:median([p[k] for p in perf if k in p]) for k in
                    ('processing_fps','capture_fps','geometry_fps','image_age_s','processing_latency_ms','queue_wait_ms')},
            stages_ms={k:median([p['stage_ms'][k] for p in perf if k in p.get('stage_ms',{})])
                       for k in ('decode','lens','preprocess','inference','postprocess','publish','jpeg','total')},
            aggregate_cpu_pct=median([sum(p['cpu_pct'] for p in s['cpu'].values()) for s in samples[3:]]),
            final_topics=latest)
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(dict(summary=summary,samples=samples),ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in summary.items() if k!='final_topics'},ensure_ascii=False),flush=True)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
