"""Fire target association and image-space lock; no actuator or pump I/O.

A YOLO bounding-box centre is an aiming proxy, not a measured flame root.
Angles are available only with explicitly calibrated camera intrinsics.
"""
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import json
import math
import cv2
import numpy as np


@dataclass
class AimConfig:
    confidence: float = .4
    window: int = 5
    min_hits: int = 3
    lock_radius_px: float = 6.0
    max_jitter_px: float = 3.0
    dwell_s: float = .35
    lost_timeout_s: float = .4
    max_frame_age_s: float = .25
    smoothing_alpha: float = .55
    association_radius_px: float = 45.0
    ambiguity_margin_px: float = 6.0
    min_target_short_side_px: float = 6.0
    min_border_margin_px: float = 2.0
    target_fraction_x: float = .5
    target_fraction_y: float = .5
    reference_reset_radius_px: float = 2.0


class CameraCalibration:
    def __init__(self, document):
        self.doc = document
        self.ready = document.get('camera_calibrated') is True
        if self.ready:
            self.K = np.asarray(document['camera_matrix'],dtype=np.float64)
            self.dist = np.asarray(document['distortion_coefficients'],dtype=np.float64)
            self.size = tuple(document['image_size_wh'])
            if self.K.shape!=(3,3) or not np.isfinite(self.K).all() or self.K[0,0]<=0 or self.K[1,1]<=0:
                raise ValueError('Invalid calibrated camera matrix')
            if not np.isfinite(self.dist).all() or self.dist.size not in (4,5,8,12,14):
                raise ValueError('Invalid lens distortion parameters')
            if not document.get('calibration_id'):
                raise ValueError('Provide a calibration ID for traceable intrinsics')
            if document.get('eye') not in ('left','right'):
                raise ValueError('Name the physical camera eye used for calibration')

    @classmethod
    def load(cls,path):
        return cls(json.loads(Path(path).read_text(encoding='utf-8-sig')))

    def angle_error(self, point, reference, size):
        if not self.ready:
            return None
        if tuple(size)!=self.size:
            raise ValueError('Frame dimensions differ from calibrated single-eye size')
        xy=cv2.undistortPoints(np.array([point,reference],dtype=np.float64).reshape(-1,1,2),self.K,self.dist).reshape(-1,2)
        yaw=np.arctan(xy[:,0])
        pitch=np.arctan2(-xy[:,1],np.sqrt(1+xy[:,0]**2))
        return dict(yaw_right_positive=float(np.degrees(yaw[0]-yaw[1])),pitch_up_positive=float(np.degrees(pitch[0]-pitch[1])))


def bbox_iou(a,b):
    a,b=np.asarray(a),np.asarray(b)
    wh=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2]))
    inter=np.prod(wh)
    return float(inter/max(1e-9,np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2])-inter))


class FireAimTracker:
    def __init__(self, config=None, calibration=None):
        self.cfg=config or AimConfig()
        c=self.cfg
        if not (1<=c.min_hits<=c.window and 0<c.smoothing_alpha<=1 and c.dwell_s>=0 and c.lost_timeout_s>0 and 0<c.confidence<1 and c.lock_radius_px>0):
            raise ValueError('Invalid aim configuration')
        if not math.isfinite(c.reference_reset_radius_px) or not 0 <= c.reference_reset_radius_px < c.lock_radius_px:
            raise ValueError('Reference reset radius must be finite and smaller than lock radius')
        self.calibration=calibration or CameraCalibration({'camera_calibrated':False})
        self.track_id=0;self.box=None;self.point=None;self.last_seen=None
        self.last_frame=None;self.last_capture=None;self.last_now=None;self.stable_since=None
        self.hits=deque(maxlen=c.window);self.points=deque(maxlen=c.window)
        self.target_reference=None

    def reset(self):
        self.box=None;self.point=None;self.last_seen=None;self.stable_since=None
        self.hits.clear();self.points.clear()

    def update(self, detections, image_size_wh, frame_id, capture_time_s, now_s, aim_pixel=None):
        """Times are monotonic seconds in the same clock domain.

        aim_pixel may be the image point measured by a nozzle landing-point
        calibration at the current range. Without that, it defaults to image
        centre and provides visual alignment only.
        """
        w,h=image_size_wh
        if w<=0 or h<=0 or not all(math.isfinite(v) for v in [capture_time_s,now_s]):
            raise ValueError('Invalid dimensions/time')
        reference=np.asarray(aim_pixel if aim_pixel is not None else [w/2,h/2],dtype=float)
        if reference.shape!=(2,) or not np.isfinite(reference).all() or not (0<=reference[0]<w and 0<=reference[1]<h):
            raise ValueError('Aim point must be inside the single-eye image')
        result=dict(frame_id=frame_id,state='SEARCH',image_locked=False,track_id=None,target_pixel=None,
                    aim_pixel=reference.tolist(),pixel_error=None,angle_error_deg=None,position_xyz_m=None,
                    gimbal_command=None,pump_command=None,physical_hit_verified=False,
                    coordinate_space='selected single-eye original pixels',target_method='smoothed YOLO box centre/fraction')
        def unavailable(state,reason):
            self.stable_since=None
            result.update(state=state,reason=reason)
            return result
        if self.last_now is not None and now_s<self.last_now:
            self.reset();return unavailable('INVALID_TIME','Monotonic clock moved backwards')
        self.last_now=now_s
        if frame_id==self.last_frame or self.last_capture is not None and capture_time_s<=self.last_capture:
            return unavailable('STALE','Repeated or out-of-order frame; no lock or correction')
        self.last_frame=frame_id;self.last_capture=capture_time_s
        if now_s-capture_time_s>self.cfg.max_frame_age_s or now_s<capture_time_s:
            self.hits.append(False);self.points.clear()
            return unavailable('STALE','Image is too old or timestamp is in the future')
        if self.last_seen is not None and now_s-self.last_seen>self.cfg.lost_timeout_s:
            self.reset()
        # Small stereo noise must not reset the dwell on every frame. Compare
        # against an anchor, so accumulated drift still clears the lock. Both
        # current raw/smoothed target errors remain checked below every frame.
        if self.target_reference is None or np.linalg.norm(reference-self.target_reference)>self.cfg.reference_reset_radius_px:
            self.stable_since=None;self.points.clear()
            self.target_reference=reference.copy()
        candidates=[]
        for d in detections:
            if d.get('class_id')!=0 or d.get('confidence',0)<self.cfg.confidence:continue
            b=np.asarray(d['bbox_xyxy'],dtype=float)
            if b.shape!=(4,) or not np.isfinite(b).all() or not (0<=b[0]<b[2]<=w and 0<=b[1]<b[3]<=h):continue
            if min(b[2:]-b[:2])<self.cfg.min_target_short_side_px:continue
            if min(b[0],b[1],w-b[2],h-b[3])<self.cfg.min_border_margin_px:continue
            point=b[:2]+(b[2:]-b[:2])*[self.cfg.target_fraction_x,self.cfg.target_fraction_y]
            if self.box is not None:
                distance=float(np.linalg.norm(point-self.point))
                # Both gates must hold: an unrelated nearby flame must not
                # silently inherit the old target's temporal evidence.
                if distance>self.cfg.association_radius_px or bbox_iou(b,self.box)<.1:continue
            else:distance=float(np.linalg.norm(point-reference))
            candidates.append((distance,b,point,float(d['confidence'])))
        candidates.sort(key=lambda x:x[0])
        if len(candidates)>1 and candidates[1][0]-candidates[0][0]<self.cfg.ambiguity_margin_px:
            self.hits.append(False);self.points.clear()
            return unavailable('AMBIGUOUS','Multiple similar fire candidates; target identity unresolved')
        if not candidates:
            self.hits.append(False);self.points.clear()
            return unavailable('LOST' if self.box is not None else 'SEARCH','No matching fresh, sufficiently sized, non-border fire detection')
        _,box,raw,confidence=candidates[0]
        if self.box is None:
            self.track_id+=1;self.hits.clear();self.points.clear();self.point=raw.copy()
        else:self.point=self.cfg.smoothing_alpha*raw+(1-self.cfg.smoothing_alpha)*self.point
        self.box=box;self.last_seen=now_s;self.hits.append(True);self.points.append(raw)
        error=self.point-reference
        jitter=float(np.linalg.norm(np.std(np.asarray(self.points),axis=0))) if len(self.points)>1 else 0.
        # Raw as well as smoothed coordinates must be on target: smoothing
        # must never conceal a newly displaced flame.
        centered=np.linalg.norm(error)<=self.cfg.lock_radius_px and np.linalg.norm(raw-reference)<=self.cfg.lock_radius_px
        stable=sum(self.hits)>=self.cfg.min_hits and len(self.points)>=self.cfg.min_hits and jitter<=self.cfg.max_jitter_px and centered
        if stable:
            if self.stable_since is None:self.stable_since=now_s
        else:self.stable_since=None
        locked=stable and now_s-self.stable_since>=self.cfg.dwell_s
        result.update(state='IMAGE_LOCKED' if locked else 'ALIGNING' if sum(self.hits)>=self.cfg.min_hits else 'ACQUIRING',
                      image_locked=bool(locked),track_id=self.track_id,target_pixel=self.point.tolist(),raw_target_pixel=raw.tolist(),
                      bbox_xyxy=box.tolist(),confidence=confidence,pixel_error=dict(x_right=float(error[0]),y_down=float(error[1])),
                      angle_error_deg=self.calibration.angle_error(self.point,reference,image_size_wh),
                      hits_in_window=sum(self.hits),jitter_px=jitter,
                      reason='Image alignment only; nozzle landing-point calibration and actuator feedback determine physical aiming')
        return result
