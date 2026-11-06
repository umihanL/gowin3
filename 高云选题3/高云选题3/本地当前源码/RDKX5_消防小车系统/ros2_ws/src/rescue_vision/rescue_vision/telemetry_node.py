"""ROS executor owns all publishers; HTTP workers only update a bounded queue."""
from pathlib import Path
import json
import time
import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from .node_common import Node
from std_msgs.msg import String
from rescue_interfaces.msg import StereoPair
from .node_common import IMAGE_QOS, param, read_json, publish_json, spin, age
from .telemetry import DashboardServer, TelemetryStore, TOPICS, runtime_token


class Telemetry(Node):
    def __init__(self):
        super().__init__('telemetry')
        share = Path(get_package_share_directory('rescue_vision'))
        cfg = read_json(param(self, 'telemetry_config', str(share/'config/telemetry.json')))
        buildings = read_json(param(self, 'buildings_config', str(share/'config/buildings.json')))
        self.store = TelemetryStore([b['building_id'] for b in buildings['buildings']])
        self.bridge = CvBridge()
        self.last_preview = 0.
        self.preview_hz = min(2., max(.1, float(cfg.get('preview_hz', 1.))))
        self.preview_width = min(800, max(160, int(cfg.get('preview_max_width', 640))))
        token_path = cfg.get('token_file', '~/.local/state/rescue_robot/dashboard.token')
        self.server = DashboardServer(self.store, share/'web', runtime_token(token_path),
            bind=cfg.get('bind', '127.0.0.1'), port=cfg.get('port', 8088),
            allowed_hosts=cfg.get('allowed_hosts', ['127.0.0.1', 'localhost']))
        self.publisher = self.create_publisher(String, '/interaction/command', 10)
        for key, topic in TOPICS.items():
            self.create_subscription(String, topic, lambda msg, k=key: self.state(k, msg), 10)
        self.create_subscription(StereoPair, '/vision/frame', self.frame, IMAGE_QOS)
        self.create_timer(.05, self.tick)
        self.server.start()
        self.get_logger().info('Dashboard listening at '+str(cfg.get('bind', '127.0.0.1'))+
            ':'+str(self.server.port)+'; token file: '+str(Path(token_path).expanduser()))

    def state(self, topic, msg):
        try:
            self.store.update(topic, json.loads(msg.data))
        except (ValueError, TypeError):
            self.get_logger().warning('Dropped invalid '+topic+' telemetry')

    def frame(self, msg):
        now = time.monotonic()
        if now-self.last_preview < 1/self.preview_hz or not 0 <= age(self, msg.header.stamp) <= .5:
            return
        self.last_preview = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg.left, 'bgr8')
            height, width = image.shape[:2]
            scale = min(1., self.preview_width/width, 600/height)
            if scale < 1:
                image = cv2.resize(image, (int(width*scale), int(height*scale)), interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if ok:
                self.store.set_frame(jpg.tobytes())
        except (ValueError, cv2.error) as exc:
            self.get_logger().warning('Preview unavailable: '+str(exc))

    def tick(self):
        for command in self.store.drain():
            publish_json(self.publisher, command)

    def destroy_node(self):
        self.server.close()
        return super().destroy_node()


def main():
    spin(Telemetry)
