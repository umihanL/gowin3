"""One owner for an explicitly selected side-by-side camera device."""
import math
import cv2
from cv_bridge import CvBridge
from .node_common import Node
from rescue_interfaces.msg import StereoPair
from std_msgs.msg import String
from .node_common import IMAGE_QOS,param,publish_json,spin


class StereoCamera(Node):
    def __init__(self):
        super().__init__('stereo_camera')
        device=param(self,'device','');self.enabled=param(self,'enabled',False)
        self.verified=param(self,'synchronization_verified',False)
        self.swap=param(self,'swap_eyes',False);self.frame_id=param(self,'frame_id','stereo_rig')
        self.width=int(param(self,'capture_width',1280));self.height=int(param(self,'capture_height',480))
        fps=float(param(self,'fps',10.0))
        if not 0<fps<=120:raise ValueError('fps must be positive')
        self.bridge=CvBridge();self.cap=None;self.sequence=0
        self.pub=self.create_publisher(StereoPair,'/stereo/pair',IMAGE_QOS)
        self.status=self.create_publisher(String,'/diagnostics/camera',10)
        if self.enabled:
            if not device:raise ValueError('Select the actual GS130W SBS V4L2 device')
            self.cap=cv2.VideoCapture(int(device) if device.isdigit() else device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,self.width);self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.height)
            self.cap.set(cv2.CAP_PROP_FPS,fps);self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
            if not self.cap.isOpened():raise RuntimeError('Cannot open camera')
        self.create_timer(1/fps,self.tick)
    def tick(self):
        if self.cap is None:return
        ok,frame=self.cap.read();stamp=self.get_clock().now().to_msg()
        if not ok:publish_json(self.status,dict(ok=False,reason='capture_failed'));return
        if frame.shape[:2]!=(self.height,self.width) or self.width%2:
            publish_json(self.status,dict(ok=False,reason='SBS dimensions differ from configuration',shape=list(frame.shape)));return
        l,r=frame[:,:self.width//2],frame[:,self.width//2:]
        if self.swap:l,r=r,l
        msg=StereoPair();msg.header.stamp=stamp;msg.header.frame_id=self.frame_id;msg.sequence=self.sequence
        msg.left=self.bridge.cv2_to_imgmsg(l,encoding='bgr8');msg.right=self.bridge.cv2_to_imgmsg(r,encoding='bgr8')
        msg.left.header.stamp=stamp;msg.left.header.frame_id='left_camera_optical'
        msg.right.header.stamp=stamp;msg.right.header.frame_id='right_camera_optical'
        msg.synchronization_verified=self.verified;msg.skew_s=0.0 if self.verified else float('nan')
        self.pub.publish(msg);self.sequence+=1
    def destroy_node(self):
        if self.cap is not None:self.cap.release()
        return super().destroy_node()

def main():spin(StereoCamera)
