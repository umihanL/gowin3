"""Optional calibrated OpenCV stereo depth. No default/fabricated intrinsics."""
import json
from pathlib import Path
import cv2
import numpy as np


class CalibratedStereo:
    def __init__(self, calibration):
        data = json.loads(Path(calibration).read_text(encoding='utf-8-sig'))
        if data.get('translation_unit') != 'm' or data.get('calibrated') is not True:
            raise ValueError('Use measured calibration with T in metres; provisional settings cannot supply depth')
        self.size = tuple(map(int, data['image_size']))
        if len(self.size) != 2 or min(self.size) < 160:
            raise ValueError('Invalid calibrated image size')
        arrays = {k: np.asarray(data[k], dtype=np.float64) for k in ['K_left','D_left','K_right','D_right','R','T']}
        if not all(np.all(np.isfinite(a)) for a in arrays.values()):
            raise ValueError('Non-finite camera calibration')
        k1, d1, k2, d2, r, t = (arrays[k] for k in arrays)
        if (k1.shape != (3,3) or k2.shape != (3,3) or r.shape != (3,3) or t.size != 3
                or min(k1[0,0],k1[1,1],k2[0,0],k2[1,1]) <= 0
                or not .005 < np.linalg.norm(t) < .5):
            raise ValueError('Calibration shapes/focal length/baseline invalid')
        t=t.reshape(3,1)
        d1,d2=d1.reshape(-1,1),d2.reshape(-1,1)
        r1,r2,p1,p2,self.q,roi1,roi2 = cv2.stereoRectify(k1,d1,k2,d2,self.size,r,t,flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
        if abs(p2[0,3]) < 1e-6 or abs(p2[1,3]) > 1e-5:
            raise ValueError('This adapter supports horizontal rectified stereo only')
        if p2[0,3] >= 0:
            raise ValueError('Left/right order must give positive left disparity; check calibration and device order')
        self.fx = float(p1[0,0]); self.fy = float(p1[1,1])
        self.cx, self.cy = float(p1[0,2]), float(p1[1,2])
        self.hfov_deg = float(np.degrees(2*np.arctan(self.size[0]/(2*self.fx))))
        self.fx_baseline = float(-p2[0,3])
        self.maps_l=cv2.initUndistortRectifyMap(k1,d1,r1,p1,self.size,cv2.CV_32FC1)
        self.maps_r=cv2.initUndistortRectifyMap(k2,d2,r2,p2,self.size,cv2.CV_32FC1)
        block=5; disparities=min(128,max(16,(self.size[0]//4//16)*16))
        self.matcher=cv2.StereoSGBM_create(minDisparity=0,numDisparities=disparities,blockSize=block,
            P1=8*block*block,P2=32*block*block,disp12MaxDiff=1,uniquenessRatio=12,
            speckleWindowSize=60,speckleRange=2,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def process(self, left, right):
        if (left.shape[1],left.shape[0]) != self.size or left.shape != right.shape:
            raise ValueError('Actual stereo frames do not match the calibrated dimensions')
        left=cv2.remap(left,*self.maps_l,cv2.INTER_LINEAR)
        right=cv2.remap(right,*self.maps_r,cv2.INTER_LINEAR)
        disparity=self.matcher.compute(cv2.cvtColor(left,cv2.COLOR_BGR2GRAY),cv2.cvtColor(right,cv2.COLOR_BGR2GRAY)).astype(np.float32)/16
        depth=np.full(disparity.shape,np.nan,dtype=np.float32)
        mask=disparity>1
        depth[mask]=self.fx_baseline/disparity[mask]
        depth[(depth<.1)|(depth>5)]=np.nan
        return left,right,depth

    def detection_range(self, depth, box):
        x0,y0,x1,y1=box; dx=(x1-x0)*.2;dy=(y1-y0)*.2
        crop=depth[max(0,int(y0+dy)):min(depth.shape[0],int(y1-dy)),
                   max(0,int(x0+dx)):min(depth.shape[1],int(x1-dx))]
        valid=crop[np.isfinite(crop)]
        if len(valid)<30 or len(valid)<crop.size*.45:
            return None
        median=float(np.median(valid));mad=float(np.median(np.abs(valid-median)))
        # Reject inconsistent ROI depth, not just a plausible single disparity.
        if mad > max(.03,.08*median):
            return None
        # Compare camera-to-target Euclidean range, not optical Z, to the policy.
        ray_x=((x0+x1)/2-self.cx)/self.fx
        ray_y=((y0+y1)/2-self.cy)/self.fy
        return median*float(np.sqrt(1+ray_x*ray_x+ray_y*ray_y))
