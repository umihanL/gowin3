"""Measured nozzle-landing pixels vs range, for a rigid camera/nozzle gimbal."""
import math
from copy import deepcopy
import numpy as np


class LandingPointTable:
    def __init__(self,document):
        self.doc=deepcopy(document)
        document=self.doc
        if document.get('landing_calibrated') is not True or not document.get('calibration_id'):
            raise ValueError('Measured landing calibration is required')
        rows=document.get('measurements',[])
        if len(rows)<2:raise ValueError('At least two measured distances are required')
        a=np.asarray([[r['distance_m'],*r['landing_pixel']] for r in rows],dtype=float)
        if a.shape!=(len(rows),3) or not np.isfinite(a).all():raise ValueError('Invalid landing table')
        if np.any(a[:,0]<=0) or np.any(np.diff(a[:,0])<=0):raise ValueError('Distances must be positive, unique, and sorted')
        self.width,self.height=document['image_size_wh']
        if np.any(a[:,1]<0) or np.any(a[:,1]>=self.width) or np.any(a[:,2]<0) or np.any(a[:,2]>=self.height):
            raise ValueError('Landing pixels outside calibrated single-eye image')
        if document.get('eye') not in ('left','right'):raise ValueError('Calibrated physical camera eye must be named')
        self.values=a

    def aim_pixel(self,distance_m,image_size_wh):
        if tuple(image_size_wh)!=(self.width,self.height):raise ValueError('Landing calibration resolution mismatch')
        if not math.isfinite(distance_m) or not self.values[0,0]<=distance_m<=self.values[-1,0]:
            raise ValueError('Range outside measured landing table; extrapolation is not allowed')
        return [float(np.interp(distance_m,self.values[:,0],self.values[:,i])) for i in (1,2)]
