import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks
from std_msgs.msg import String

IMAGE_QOS=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)


class Node(RclpyNode):
    """Avoid unused default QoS warning events in every executor wait set.

    Humble registers an incompatibility-warning event for each publisher and
    subscription by default. Rebuilding those waitables dominated the idle
    control nodes' Python execution. Message QoS and explicit event callbacks
    are preserved; application freshness checks and control leases are unchanged.
    """

    def __init__(self, node_name, *args, **kwargs):
        # The task launcher can omit six unused parameter RPC services per node.
        # Launch overrides and local parameter declarations still work, but
        # external `ros2 param` inspection is unavailable for those task nodes.
        # Keep ordinary development launches unchanged; setting the environment
        # to 1 restores the services. Explicit constructor choices take priority.
        # YOLO needs live mission_gating get/set, including during task shutdown.
        kwargs.setdefault('start_parameter_services',
                          node_name == 'fast_mipi_perception' or
                          os.environ.get('RESCUE_PARAMETER_SERVICES') != '0')
        super().__init__(node_name, *args, **kwargs)

    def create_publisher(self, *args, **kwargs):
        if 'event_callbacks' not in kwargs:
            kwargs['event_callbacks'] = PublisherEventCallbacks(use_default_callbacks=False)
        return super().create_publisher(*args, **kwargs)

    def create_subscription(self, *args, **kwargs):
        if 'event_callbacks' not in kwargs:
            kwargs['event_callbacks'] = SubscriptionEventCallbacks(use_default_callbacks=False)
        return super().create_subscription(*args, **kwargs)


def param(node,name,default):return node.declare_parameter(name,default).value
def read_json(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))
def seconds(stamp):return stamp.sec+stamp.nanosec/1e9
def age(node,stamp):return node.get_clock().now().nanoseconds/1e9-seconds(stamp)
def publish_json(publisher,data):
    msg=String();msg.data=json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',', ':'));publisher.publish(msg)
def spin(factory):
    rclpy.init();node=None
    try:node=factory();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        try:
            if node is not None:node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        except (KeyboardInterrupt,ExternalShutdownException):pass
