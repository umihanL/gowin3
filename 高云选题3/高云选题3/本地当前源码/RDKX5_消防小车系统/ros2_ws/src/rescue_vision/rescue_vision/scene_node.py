"""Match OCR and detections from the same image before assigning a building."""
import json
from concurrent.futures import ThreadPoolExecutor
from cv_bridge import CvBridge
from .node_common import Node
from std_msgs.msg import String
from rescue_interfaces.msg import StereoPair,DetectionArray
from .node_common import IMAGE_QOS,param,read_json,age,seconds,publish_json,spin
from .building_ocr import OCRReader,BuildingTracker
from .semantics import SceneMemory


class Scene(Node):
    def __init__(self):
        super().__init__('scene');self.bridge=CvBridge();self.frames={};self.detections={};self.last_sequence=None
        self.mission={};self.mission_time=0.
        self.pose_generation=0;self.worker=ThreadPoolExecutor(max_workers=1);self.pending=None
        config=read_json(param(self,'buildings_config',''))
        self.tracker=BuildingTracker(config);self.memory=SceneMemory(config,ttl_s=3.0)
        self.reader=OCRReader(param(self,'ocr_models','')) if param(self,'ocr_enabled',False) else None
        roi_file=param(self,'facade_config','');self.roi=read_json(roi_file) if roi_file else {}
        self.report_pub=self.create_publisher(String,'/scene/report',10)
        self.ocr_pub=self.create_publisher(String,'/scene/ocr',10)
        self.goal_pub=self.create_publisher(String,'/navigation/semantic_goal',10)
        self.create_subscription(StereoPair,'/vision/frame',self.frame,IMAGE_QOS)
        self.create_subscription(DetectionArray,'/perception/detections',self.objects,IMAGE_QOS)
        self.create_subscription(String,'/semantic/request',self.request,10)
        self.create_subscription(String,'/mission/state',self.mission_state,10)
        self.create_timer(float(param(self,'ocr_interval_s',.75)),self.tick)
    def frame(self,msg):
        self.frames[msg.sequence]=msg
        while len(self.frames)>6:del self.frames[next(iter(self.frames))]
    def objects(self,msg):
        self.detections[msg.sequence]=msg
        while len(self.detections)>6:del self.detections[next(iter(self.detections))]
    def request(self,msg):
        try:
            data=json.loads(msg.data)
            now=self.get_clock().now().nanoseconds/1e9
            goal=self.memory.route_request(data['building_id'],now) if 'building_id' in data else self.memory.resolve_request(data['text'],now)
            publish_json(self.goal_pub,goal)
        except (ValueError,KeyError,TypeError) as exc:publish_json(self.goal_pub,dict(can_execute=False,error=str(exc)))
    def mission_state(self,msg):
        try:
            incoming=json.loads(msg.data)
            if not isinstance(incoming,dict):raise ValueError('Mission state must be an object')
            keys=('mode','vehicle_stopped','gimbal_ready','gimbal_target_deg','observation_pose_id','traffic_active')
            if any(incoming.get(k)!=self.mission.get(k) for k in keys):self.pose_generation+=1
            self.mission=incoming;self.mission_time=self.get_clock().now().nanoseconds/1e9
        except ValueError:self.mission={}
    def current(self):publish_json(self.report_pub,self.memory.current(self.get_clock().now().nanoseconds/1e9))
    def tick(self):
        if self.pending is not None:
            if not self.pending.done():self.current();return
            try:self.pending.result()
            except Exception as exc:
                self.get_logger().error('OCR processing failed: '+str(exc))
                publish_json(self.report_pub,dict(valid=False,fire_status='unknown',reason='ocr_processing_failed'))
            self.pending=None
        ids=self.frames.keys()&self.detections.keys()
        if self.reader is None:
            publish_json(self.report_pub,dict(valid=False,reason='ocr_disabled',fire_status='unknown'));return
        if not ids:self.current();return
        key=max(ids)
        if key==self.last_sequence:
            publish_json(self.report_pub,self.memory.current(self.get_clock().now().nanoseconds/1e9));return
        self.last_sequence=key;frame=self.frames[key];det=self.detections[key]
        if frame.header!=det.header or not 0<=age(self,frame.header.stamp)<=1.5:self.current();return
        self.pending=self.worker.submit(self.process,frame,det,self.pose_generation)
    def process(self,frame,det,pose_generation):
        key=frame.sequence
        image=self.bridge.imgmsg_to_cv2(frame.left,'bgr8');stamp=seconds(frame.header.stamp);frame_id=str(key)
        readings=self.reader.predict(image)
        building=self.tracker.update(readings,stamp,frame_id)
        publish_json(self.ocr_pub,dict(stamp=stamp,sequence=key,readings=readings,building=building))
        boxes=[dict(class_name=d.class_name,confidence=d.confidence,bbox=list(d.bbox_xyxy)) for d in det.detections]
        roi=self.roi.get('roi_xyxy');verified=self.roi.get('verified',False)
        verified=verified and self.roi.get('image_size_wh')==[det.width,det.height]
        verified=verified and self.roi.get('coordinate_frame')==det.header.frame_id
        now=self.get_clock().now().nanoseconds/1e9
        verified=verified and 0<=now-self.mission_time<=.5 and self.mission.get('vehicle_stopped') and self.mission.get('gimbal_ready')
        verified=verified and self.mission.get('mode')=='inspect' and bool(self.roi.get('observation_pose_id'))
        verified=verified and self.roi.get('observation_pose_id')==self.mission.get('observation_pose_id')
        verified=verified and self.roi.get('gimbal_target_deg')==self.mission.get('gimbal_target_deg')
        verified=verified and pose_generation==self.pose_generation
        if not det.detector_available:verified=False
        report=self.memory.observe(building,boxes,stamp,frame_id,roi,verified)
        if age(self,frame.header.stamp)>3.0:report=self.memory.current(self.get_clock().now().nanoseconds/1e9)
        report['detector_available']=det.detector_available
        # Independent whole-view counts do not assert ownership by a building.
        if det.detector_available and 0<=age(self,frame.header.stamp)<=3.0:
            names=[b['class_name'] for b in boxes if b['confidence']>=.4]
            report['unassigned_current_view']=dict(fire_observed='fire' in names,visible_person_count=names.count('person'),
                visible_firefighter_count=names.count('firefighter'),casualty_count=None,building_association_verified=bool(report['valid']))
        publish_json(self.report_pub,report)
    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()

def main():spin(Scene)
