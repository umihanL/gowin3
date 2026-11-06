#!/usr/bin/env python3
"""Read-only typed/raw subscriber probe; run modes sequentially, not together.

This adds one image subscriber and therefore has observer overhead. It neither
starts nor changes a camera. It reports delivery rate separately from header
timestamp spacing, and uses monotonic time so an unset board RTC is harmless.
"""
import argparse
import json
import resource
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from rescue_vision.raw_image_cdr import decode_image_cdr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['typed', 'raw'], default='raw')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--topic', default='/image_right_raw')
    args = parser.parse_args()
    rclpy.init()
    node = Node('capture_probe_' + args.mode)
    stamps, received, callback_ms = [], [], []
    errors = []

    def callback(message):
        began = time.perf_counter()
        try:
            if args.mode == 'raw':
                message = decode_image_cdr(message)
            stamps.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
            received.append(time.monotonic())
        except Exception as exc:
            errors.append(str(exc))
        callback_ms.append((time.perf_counter() - began) * 1000)

    node.create_subscription(Image, args.topic, callback,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                             raw=args.mode == 'raw')
    usage_start = resource.getrusage(resource.RUSAGE_SELF)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        elapsed = time.monotonic() - started
        usage_end = resource.getrusage(resource.RUSAGE_SELF)
        source_gaps = [(b - a) * 1000 for a, b in zip(stamps, stamps[1:])]
        print(json.dumps(dict(mode=args.mode, elapsed_s=elapsed, received=len(received),
            received_fps=(len(received)-1)/(received[-1]-received[0]) if len(received)>1 else 0,
            source_span_fps=(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0,
            source_gap_ms_median=statistics.median(source_gaps) if source_gaps else None,
            source_gap_ms_max=max(source_gaps) if source_gaps else None,
            source_gaps_gt_50ms=sum(x>50 for x in source_gaps),
            callback_ms_median=statistics.median(callback_ms) if callback_ms else None,
            process_cpu_percent=100*((usage_end.ru_utime+usage_end.ru_stime)-
                                    (usage_start.ru_utime+usage_start.ru_stime))/elapsed,
            errors=errors[:5]), indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
