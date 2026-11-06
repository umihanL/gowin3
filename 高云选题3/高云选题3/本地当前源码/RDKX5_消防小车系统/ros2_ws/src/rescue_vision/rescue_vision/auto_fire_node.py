"""ROS adapter for a bounded automatic fire round; Mission owns all gimbal output."""
import json
from .node_common import Node
from std_msgs.msg import String
from .node_common import param, publish_json, spin
from .auto_fire import AutoFireRound


class AutoFire(Node):
    def __init__(self):
        super().__init__('auto_fire')
        self.controller = AutoFireRound(timeout_s=float(param(self,'timeout_s',90.)),
            fixed_camera_observation=bool(param(self,'fixed_camera_observation',False)))
        self.evidence = {}
        self.pubs = {key:self.create_publisher(String,topic,10) for key,topic in (
            ('mission','/mission/request'),('actuator','/actuators/request'),
            ('select','/fire/select'),('speech','/speech/say'),('question','/scene/question'))}
        self.state_pub = self.create_publisher(String,'/fire/auto/state',10)
        self.create_subscription(String,'/fire/auto/request',self.request,10)
        self.create_subscription(String,'/fire/select',self.manual_selection,10)
        for key,topic in (('mission','/mission/state'),('targets','/fire/targets'),
                          ('selection','/fire/selection'),('proposal','/fire/proposal'),
                          ('actuator','/actuators/state')):
            self.create_subscription(String,topic,lambda msg,k=key:self.receive(k,msg),10)
        self.create_timer(.1,self.tick)

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def receive(self,key,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('expected object')
            self.evidence[key]=value
        except (ValueError,TypeError):self.evidence.pop(key,None)

    def emit(self,outputs):
        for key,value in outputs:
            if key=='speech':self.pubs[key].publish(String(data=value))
            else:
                if key=='mission':value=dict(value,auto_run_id=self.controller.run_id)
                publish_json(self.pubs[key],value)
        publish_json(self.state_pub,self.controller.snapshot(self.now()))

    def request(self,msg):
        try:self.emit(self.controller.request(json.loads(msg.data),self.now()))
        except (ValueError,TypeError) as exc:self.get_logger().warning(str(exc))

    def manual_selection(self,msg):
        try:self.emit(self.controller.manual_selection(json.loads(msg.data),self.now()))
        except (ValueError,TypeError):pass

    def tick(self):self.emit(self.controller.tick(self.now(),**self.evidence))


def main():spin(AutoFire)
