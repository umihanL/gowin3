"""ROS wrapper: plans and proposes; mission remains the single hardware command owner."""
import json
from .node_common import Node
from std_msgs.msg import String
from .node_common import param,read_json,publish_json,spin
from .navigation import Navigator


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.core=Navigator(read_json(param(self,'navigation_map','')),read_json(param(self,'navigation_config','')))
        self.last_request_result=None
        self.state_pub=self.create_publisher(String,'/navigation/state',10)
        self.proposal_pub=self.create_publisher(String,'/navigation/proposal',10)
        self.create_subscription(String,'/navigation/request',self.request,10)
        self.create_subscription(String,'/fpga/telemetry',self.telemetry,20)
        self.create_subscription(String,'/navigation/landmark',self.landmark,10)
        self.create_subscription(String,'/navigation/blocked_edges',self.blocked,10)
        self.create_timer(.1,self.tick)

    def now(self):return self.get_clock().now().nanoseconds/1e9
    def parse(self,msg):
        data=json.loads(msg.data)
        if not isinstance(data,dict):raise ValueError('Expected object')
        return data
    def request(self,msg):
        try:
            request=self.parse(msg)
            self.core.request(request,self.now())
            self.last_request_result=dict(accepted=True,command_id=request.get('command_id'),stamp=self.now())
        except (ValueError,KeyError,TypeError) as exc:
            self.get_logger().warning('Navigation request rejected: '+str(exc))
            # Keep rejection visible in the periodic state; do not resume an older goal
            # after a new navigation request was rejected.
            self.core.paused=True;self.core.reason='request_rejected: '+str(exc)
            self.last_request_result=dict(accepted=False,reason=str(exc),stamp=self.now())
        self.tick()
    def telemetry(self,msg):
        try:self.core.update_telemetry(self.parse(msg),self.now())
        except (ValueError,KeyError,TypeError) as exc:
            self.core.odom.valid=False;self.core.odom.reason='malformed_telemetry'
            self.get_logger().warning(str(exc))
    def landmark(self,msg):
        try:
            if not self.core.odom.pose_fixture(self.parse(msg),self.now()):self.get_logger().warning('Landmark rejected; OCR text alone is not localization')
        except (ValueError,KeyError,TypeError) as exc:self.get_logger().warning(str(exc))
    def blocked(self,msg):
        try:
            data=self.parse(msg)
            if not 0<=self.now()-data['stamp']<=1.:raise ValueError('Stale blockage event')
            self.core.block_edges(data['edge_ids'],self.now())
        except (ValueError,KeyError,TypeError) as exc:
            self.core.paused=True;self.core.reason='blockage_replan_failed'
            self.get_logger().warning(str(exc))
    def tick(self):
        now=self.now()
        state=self.core.state(now);state['last_request_result']=self.last_request_result
        publish_json(self.state_pub,state)
        publish_json(self.proposal_pub,self.core.proposal(now))


def main():spin(Navigation)
