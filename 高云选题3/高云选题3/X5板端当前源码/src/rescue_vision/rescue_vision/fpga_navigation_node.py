"""ROS semantic-goal handoff; this node never plans a path or drives a segment."""
import json
from .node_common import Node
from std_msgs.msg import String
from .node_common import param, read_json, publish_json, spin
from .fpga_navigation import FpgaNavigator
from .manual_motion import start_guard, runtime_guard


class FpgaNavigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.core = FpgaNavigator(read_json(param(self, 'buildings_config', '')),
                                  manual_motion_enabled=bool(param(self, 'manual_motion_enabled', False)))
        self.result = None
        self.bridge = {}
        self.safety = {'mission': {}, 'actuator': {}, 'telemetry': {}}
        self.state_pub = self.create_publisher(String, '/navigation/state', 10)
        self.proposal_pub = self.create_publisher(String, '/navigation/proposal', 10)
        self.command_pub = self.create_publisher(String, '/fpga/navigation_command', 10)
        self.create_subscription(String, '/navigation/request', self.request, 10)
        self.create_subscription(String, '/fpga/navigation_status', self.status, 20)
        self.create_subscription(String, '/fpga/link', self.link, 10)
        for key, topic in (('mission', '/mission/state'), ('actuator', '/actuators/state'), ('telemetry', '/fpga/telemetry')):
            self.create_subscription(String, topic, lambda msg, k=key: self.safety_update(k, msg), 10)
        self.create_timer(.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def parse(msg):
        value = json.loads(msg.data)
        if not isinstance(value, dict):
            raise ValueError('Expected JSON object')
        return value

    def request(self, msg):
        request = {}
        try:
            request = self.parse(msg)
            if request.get('action') == 'manual_move':
                reason = start_guard(self.now(), **self.safety, link=self.bridge, navigation=self.core.state(self.now()))
                if reason:
                    raise ValueError(reason)
            self.core.request(request, self.now())
            self.result = dict(accepted=True, command_id=request.get('command_id'), stamp=self.now())
        except (ValueError, TypeError, KeyError) as exc:
            self.result = dict(accepted=False, command_id=request.get('command_id'),
                               reason=str(exc), stamp=self.now())
            self.get_logger().warning('Semantic goal rejected: '+str(exc))
        self.tick()

    def status(self, msg):
        try:
            self.core.observe(self.parse(msg), self.now())
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning('Navigation status rejected: '+str(exc))

    def link(self, msg):
        try:
            self.bridge = self.parse(msg)
        except (ValueError, TypeError):
            self.bridge = {}

    def safety_update(self, key, msg):
        try:
            self.safety[key] = self.parse(msg)
        except (ValueError, TypeError):
            self.safety[key] = {}

    def tick(self):
        now = self.now()
        if self.core.active and self.core.active.get('motion_kind') == 'manual' and self.core.busy():
            reason = runtime_guard(now, **self.safety)
            if reason:
                self.core.cancel_manual(now, reason)
        command = self.core.next_command(now)
        if command:
            publish_json(self.command_pub, command)
        state = self.core.state(now)
        state.update(last_request_result=self.result, uart_dry_run=self.bridge.get('dry_run'),
                     uart_link_error=self.bridge.get('error'))
        publish_json(self.state_pub, state)
        publish_json(self.proposal_pub, self.core.proposal(now))


def main():
    spin(FpgaNavigation)
