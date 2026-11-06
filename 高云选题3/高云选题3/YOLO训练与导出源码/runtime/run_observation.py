"""Run saved YOLO Detect weights on images/video/camera, then observation policy.

Produces annotated previews and decision JSONL, never motor/pump commands.
Live telemetry must be fresh; without it, detections are shown but ready=false.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from runtime.observation import Policy, Detection, Observer, iou

MAX_INPUT_AGE_S = .25


def fresh_telemetry(value, now, frame_id):
    try:
        age=now-float(value['timestamp_unix_s'])
        if (not 0 <= age <= MAX_INPUT_AGE_S
                or ('frame_id' in value and value['frame_id'] != frame_id)):
            return {}
        return value
    except (ValueError,TypeError,KeyError):
        return {}


def telemetry_at(path, now, frame_id):
    if path is None:
        return {}
    try:
        value=json.loads(Path(path).read_text(encoding='utf-8'))
        return fresh_telemetry(value,now,frame_id)
    except (OSError,ValueError,TypeError,KeyError):
        return {}


def decide_observation(observer, detections, telemetry, *, timestamp, frame_id,
                       width, height, decision_stamp, decision_wall):
    """Recheck captured inputs after inference; never refresh an old image's pose."""
    telemetry=fresh_telemetry(telemetry,decision_wall,frame_id)
    age=decision_stamp-timestamp
    frame_fresh=math.isfinite(age) and 0 <= age <= MAX_INPUT_AGE_S
    def number(key,default=None):
        val=telemetry.get(key,default)
        return float(val) if isinstance(val,(float,int)) and not isinstance(val,bool) and math.isfinite(val) else default
    rig_valid=all(number(key) is not None for key in ('pan_deg','tilt_deg','speed_mps'))
    decision=observer.update(detections,timestamp=timestamp,frame_id=frame_id,width=width,height=height,
        pan_deg=number('pan_deg',0.),tilt_deg=number('tilt_deg',0.),speed_mps=number('speed_mps'),
        safety_clear=frame_fresh and rig_valid and telemetry.get('safety_clear') is True,
        at_observation_point=telemetry.get('at_observation_point') is True)
    if not frame_fresh:
        decision['reason']='stale_source_frame'
    elif not rig_valid:
        decision['reason']='await_fresh_rig_telemetry'
    decision.update(telemetry_fresh=bool(telemetry),frame_fresh=frame_fresh,
                    input_age_s=age,decision_wall_time=decision_wall)
    return decision


def attach_ranges(detections, readings):
    """A measurement can support one spatially associated box, not all detections."""
    pairs=[]
    for i,d in enumerate(detections):
        for j,r in enumerate(readings):
            try:
                if not isinstance(r,dict):
                    continue
                box=tuple(map(float,r['bbox_xyxy']));distance=float(r['distance_m'])
                if (len(box)!=4 or not all(math.isfinite(v) for v in box)
                        or box[2]<=box[0] or box[3]<=box[1] or not math.isfinite(distance)
                        or distance<=0 or not isinstance(r['source'],str) or not r['source']):
                    continue
                overlap=iou(d.bbox,box)
                if overlap>=.5:
                    pairs.append((overlap,i,j,distance))
            except (KeyError,TypeError,ValueError):
                continue
    used_d=set();used_r=set()
    for _,i,j,distance in sorted(pairs,reverse=True):
        if i in used_d or j in used_r:
            continue
        # Prefer calibrated stereo when available.
        if detections[i].distance_m is None:
            detections[i].distance_m=distance
            detections[i].distance_source=readings[j]['source']
            angle=readings[j].get('obliquity_deg')
            if isinstance(angle,(int,float)) and math.isfinite(angle):
                detections[i].obliquity_deg=angle
        used_d.add(i);used_r.add(j)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True,help='Image, video path, or camera device integer')
    parser.add_argument('--weights',type=Path,default=ROOT/'runs/wallpaper_yolo11n_people_v2_20261008/weights/best.pt')
    parser.add_argument('--policy',type=Path,default=ROOT/'config/observation_policy.json')
    parser.add_argument('--layout',choices=['mono','sbs'],default='mono')
    parser.add_argument('--eye',choices=['left','right'],default='left')
    parser.add_argument('--target-class',choices=['all','fire','firefighter','person'],default='all')
    parser.add_argument('--stereo-calibration',type=Path)
    parser.add_argument('--telemetry',type=Path,help='Fresh rig/safety/range JSON; no telemetry means no ready authorization')
    parser.add_argument('--roi',help='Optional native eye image crop x,y,width,height; camera FOV remains unchanged')
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--imgsz',type=int,default=640)
    parser.add_argument('--output',type=Path,required=True,help='New output directory (never overwritten)')
    parser.add_argument('--display',action='store_true')
    parser.add_argument('--max-frames',type=int,default=0)
    parser.add_argument('--save-every',type=int,default=15)
    args=parser.parse_args()
    if args.imgsz<32 or args.imgsz%32 or args.save_every<1 or args.max_frames<0:
        parser.error('Invalid image size, frame count, or save cadence')
    if not args.weights.is_file():
        parser.error('Saved Detect weights not found')
    if not args.source.isdecimal() and not Path(args.source).is_file():
        parser.error('Source must be an existing local image/video or camera index')
    if args.output.exists():
        parser.error('Use a new output directory')
    import os
    os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'.ultralytics'))
    os.environ.setdefault('YOLO_AUTOINSTALL','false')
    import torch
    if args.device=='cpu':
        torch.set_num_threads(2)
    import cv2
    from ultralytics import YOLO
    import numpy as np
    policy=Policy.load(args.policy);observer=Observer(policy)
    stereo=None
    if args.stereo_calibration:
        if args.layout!='sbs' or args.eye!='left':
            parser.error('Calibrated stereo mode uses SBS left view')
        from runtime.stereo_depth import CalibratedStereo
        stereo=CalibratedStereo(args.stereo_calibration)
        policy.hfov_deg=stereo.hfov_deg
    model=YOLO(str(args.weights),task='detect')
    if model.task!='detect' or dict(model.names)!={0:'fire',1:'firefighter',2:'person'}:
        parser.error('Expected the saved three-class Detect model')
    image_path=Path(args.source)
    is_image=image_path.suffix.lower() in ('.png','.jpg','.jpeg','.bmp','.webp') and image_path.is_file()
    capture=None
    if not is_image:
        capture=cv2.VideoCapture(int(args.source) if args.source.isdecimal() else args.source)
        if not capture.isOpened():
            raise RuntimeError('Cannot open camera/video')
    args.output.mkdir(parents=True)
    (args.output/'policy.json').write_text(json.dumps(asdict(policy),indent=2),encoding='utf-8')
    frames=0;ready_frames=0;total_detections=0;started=time.time()
    try:
        with (args.output/'decisions.jsonl').open('w',encoding='utf-8') as output:
            while True:
                if is_image:
                    full=cv2.imdecode(np.fromfile(str(image_path),dtype=np.uint8),cv2.IMREAD_COLOR)
                    ok=full is not None
                else:
                    ok,full=capture.read()
                if not ok:
                    if frames==0:raise RuntimeError('No readable frames')
                    break
                wall=time.time();stamp=time.monotonic()
                telemetry=telemetry_at(args.telemetry,wall,frames)
                depth=None
                if args.layout=='sbs':
                    if full.shape[1]%2:
                        raise ValueError('SBS frame width must be even')
                    half=full.shape[1]//2;left,right=full[:,:half],full[:,half:]
                    if stereo:left,right,depth=stereo.process(left,right)
                    full=left if args.eye=='left' else right
                h,w=full.shape[:2];x=y=0;cw=w;ch=h
                if args.roi:
                    x,y,cw,ch=map(int,args.roi.split(','))
                    if min(x,y)<0 or min(cw,ch)<=0 or x+cw>w or y+ch>h:
                        raise ValueError('ROI is outside actual eye frame')
                result=model.predict(full[y:y+ch,x:x+cw],imgsz=args.imgsz,conf=policy.confidence,
                                     device=args.device,verbose=False)[0]
                det=[]
                for box,conf,cls in zip(result.boxes.xyxy.cpu().tolist(),result.boxes.conf.cpu().tolist(),result.boxes.cls.cpu().tolist()):
                    bbox=(max(0,box[0]+x),max(0,box[1]+y),min(w,box[2]+x),min(h,box[3]+y))
                    d=Detection(int(cls),float(conf),bbox,detail_scale=min(1.,args.imgsz/max(cw,ch)))
                    if depth is not None:
                        d.distance_m=stereo.detection_range(depth,bbox)
                        if d.distance_m is not None:d.distance_source='calibrated_stereo_roi_median'
                    det.append(d)
                readings=telemetry.get('ranges',[])
                attach_ranges(det,readings if isinstance(readings,list) else [])
                active_detections=[d for d in det if args.target_class=='all' or model.names[d.class_id]==args.target_class]
                decision=decide_observation(observer,active_detections,telemetry,
                    timestamp=stamp,frame_id=frames,width=w,height=h,
                    decision_stamp=time.monotonic(),decision_wall=time.time())
                decision.update(detections=[asdict(d) for d in det],
                    source=args.source,wall_time=wall,actuator_commands_sent=False,
                    detector='saved_YOLO11n_Detect',rectified=stereo is not None,target_class=args.target_class)
                output.write(json.dumps(decision,ensure_ascii=False,allow_nan=False)+'\n');output.flush()
                preview=full.copy()
                for d in det:
                    x0,y0,x1,y1=map(int,d.bbox);color=[(40,140,255),(80,210,60),(240,190,70)][d.class_id]
                    cv2.rectangle(preview,(x0,y0),(x1,y1),color,1)
                    cv2.putText(preview,f'{model.names[d.class_id]} {d.confidence:.2f}',(x0,max(12,y0-4)),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
                cv2.putText(preview,f"{decision['state']}: {decision['reason']} [{decision['votes']}/{policy.window_frames}]",(8,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                if frames%args.save_every==0:
                    ok,encoded=cv2.imencode('.jpg',preview)
                    if not ok:raise RuntimeError('Preview encoding failed')
                    encoded.tofile(str(args.output/f'frame_{frames:06d}.jpg'))
                if args.display:
                    cv2.imshow('YOLO observation (no actuators)',preview)
                    if cv2.waitKey(1)&255 in (27,ord('q')):break
                frames+=1;total_detections+=len(det);ready_frames+=int(decision['ready'])
                if is_image or (args.max_frames and frames>=args.max_frames):break
    finally:
        if capture:capture.release()
        if args.display:cv2.destroyAllWindows()
    summary=dict(frames=frames,detections=total_detections,ready_frames=ready_frames,
                 weights=str(args.weights.resolve()),device=args.device,seconds=time.time()-started,
                 no_training=True,no_actuators=True,calibrated_stereo=stereo is not None,
                 boundary='PyTorch/OpenCV reference runner; not an RDK X5 BPU binary or a hardware driver. Single image cannot establish temporal confirmation.')
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':
    main()
