#!/usr/bin/env python3
"""Read-only HTTP and /proc benchmark: no ROS participant or control commands."""
import argparse,json,time,os,statistics,urllib.request,pathlib,threading,http.client,struct
p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=25);p.add_argument('--output',required=True);p.add_argument('--stream',action='store_true');args=p.parse_args()
def proc():
    out={}
    for path in pathlib.Path('/proc').glob('[0-9]*/stat'):
        try:
            raw=path.read_text();cols=raw[raw.rfind(')')+2:].split()
            cmd=(path.parent/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            if '/rescue/' in cmd or 'mipi_cam/mipi_cam' in cmd or 'presentation_server.py' in cmd or 'detection_preview.py' in cmd:
                out[path.parent.name]={'name':raw[raw.find('(')+1:raw.rfind(')')],'ticks':int(cols[11])+int(cols[12]),'threads':int(cols[17]),'rss_kib':int(cols[21])*os.sysconf('SC_PAGE_SIZE')//1024}
        except (OSError,ValueError,IndexError):pass
    return out
stream_stop=threading.Event();stream_stats={'requests':0,'frames':0,'errors':0,'bytes':0}
def stream_reader():
    connection=http.client.HTTPConnection('127.0.0.1',8090,timeout=4);last_token=''
    try:
        while not stream_stop.is_set():
            started_frame=time.monotonic()
            try:
                connection.request('GET','/api/frame?after='+last_token)
                response=connection.getresponse();body=response.read();stream_stats['requests']+=1;stream_stats['bytes']+=len(body)
                if response.status==200 and len(body)>=4:
                    length=struct.unpack('>I',body[:4])[0];meta=json.loads(body[4:4+length]);frame=meta.get('frame') or {};new_token=str(frame.get('token') or '')
                    if len(body)>4+length and new_token and new_token!=last_token:stream_stats['frames']+=1;last_token=new_token
                else:stream_stats['errors']+=1
            except (OSError,ValueError,http.client.HTTPException):
                stream_stats['errors']+=1;connection.close()
            stream_stop.wait(max(0,1/30-(time.monotonic()-started_frame)))
    finally:connection.close()
thread=threading.Thread(target=stream_reader,daemon=True)
if args.stream:thread.start()
samples=[];started=last=time.monotonic();previous=proc()
while time.monotonic()-started<args.seconds:
    tick=time.monotonic()
    with urllib.request.urlopen('http://127.0.0.1:8090/api/state',timeout=4) as f:s=json.load(f)
    now=time.monotonic();current=proc();cpu={}
    for pid,v in current.items():
        if pid in previous:cpu[pid]=dict(v,cpu_pct=round((v['ticks']-previous[pid]['ticks'])/os.sysconf('SC_CLK_TCK')/max(.01,now-last)*100,2))
    previous,last=current,now
    topics=s['topics'];perf=(topics['perception_diagnostics']['data'] or {})
    samples.append({'elapsed':now-started,'cpu':cpu,'performance':perf,'control':s['control'],'preview':s['preview'],'ocr':topics['ocr_diagnostics']['data'],'road':topics['road_diagnostics']['data'],'voice':topics['voice']['data'],'mission':topics['mission']['data']})
    time.sleep(max(0,1-(time.monotonic()-tick)))
stream_stop.set()
if args.stream:thread.join(timeout=5)
data=samples[3:]
med=lambda vals:statistics.median(vals) if vals else None
summary={'stream_requested':args.stream,'stream_stats':stream_stats,'samples':len(data),'seconds':time.monotonic()-started,'medians':{k:med([x['performance'][k] for x in data if k in x['performance']]) for k in ['processing_fps','capture_fps','geometry_fps','image_age_s','processing_latency_ms','queue_wait_ms','preview_encode_fps','observation_subscribers']},'stage_ms':{k:med([x['performance']['stage_ms'][k] for x in data if k in x['performance'].get('stage_ms',{})]) for k in ['decode','lens','preprocess','inference','postprocess','publish','jpeg','total','observation_publish']},'total_cpu_pct':med([sum(v['cpu_pct'] for v in x['cpu'].values()) for x in data]),'command_ready_samples':sum(x['control']['command_ready'] for x in data),'listen_ready_samples':sum(x['control']['listen_ready'] for x in data),'microphone_open_samples':sum((x['voice'] or {}).get('capture_active') is True for x in data)}
path=pathlib.Path(args.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps({'summary':summary,'samples':samples},ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(summary,ensure_ascii=False,indent=2))
