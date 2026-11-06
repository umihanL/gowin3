#!/usr/bin/env python3
"""Two-process lossless image transport experiment, restricted to ROS domain 83."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import zlib


def worker(args):
    import rclpy
    from sensor_msgs.msg import Image
    from rescue_vision.node_common import IMAGE_QOS
    from rescue_vision.raw_image_cdr import decode_image_cdr
    rclpy.init();node=rclpy.create_node('image_transport_benchmark_'+args.worker)
    size=1280*1088*3//2
    payload=bytes(range(256))*(size//256)
    crc=zlib.crc32(payload)
    count=0;invalid=0;times=[];ages=[]
    start=time.monotonic()
    try:
        if args.worker=='pub':
            pub=node.create_publisher(Image,'/isolated_image_transport',IMAGE_QOS)
            while not pub.get_subscription_count() and time.monotonic()-start<8:
                rclpy.spin_once(node,timeout_sec=.05)
            if not pub.get_subscription_count():raise RuntimeError('Subscriber discovery timed out')
            image=Image(width=1280,height=1088,encoding='nv12',step=1280)
            image.data.frombytes(payload)
            start=time.monotonic();deadline=start
            while time.monotonic()-start<args.seconds:
                image.header.stamp=node.get_clock().now().to_msg()
                image.header.frame_id=str(count)
                pub.publish(image);count+=1;times.append(time.monotonic())
                deadline+=1/30
                time.sleep(max(0,deadline-time.monotonic()))
        else:
            def callback(raw):
                nonlocal count,invalid
                f=decode_image_cdr(raw)
                count+=1;times.append(time.monotonic())
                ages.append(node.get_clock().now().nanoseconds/1e9-(f.header.stamp.sec+f.header.stamp.nanosec/1e9))
                invalid+=int(zlib.crc32(f.data)!=crc)
            node.create_subscription(Image,'/isolated_image_transport',callback,IMAGE_QOS,raw=True)
            while time.monotonic()-start<args.seconds+10:
                rclpy.spin_once(node,timeout_sec=.1)
                if times and time.monotonic()-times[-1]>2:break
        import statistics
        report=dict(worker=args.worker,frames=count,invalid=invalid,bytes_per_frame=size,
            fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0,
            median_age_ms=statistics.median(ages)*1000 if ages else None,
            xml=os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'))
        Path(args.output).write_text(json.dumps(report,indent=2),encoding='utf-8')
    finally:node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--worker',choices=['pub','sub'])
    p.add_argument('--seconds',type=int,default=15)
    p.add_argument('--subscriber-default',action='store_true',help='Keep the subscriber on its default transport profile')
    p.add_argument('--output',required=True)
    args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='83':raise RuntimeError('Isolated ROS domain 83 required')
    if args.worker:return worker(args)
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    children=[]
    try:
        for name in ('sub','pub'):
            environment=os.environ.copy()
            if name=='sub' and args.subscriber_default:
                environment.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
                environment.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
            children.append(subprocess.Popen([sys.executable,__file__,'--worker',name,'--seconds',str(args.seconds),
                '--output',str(output.with_suffix('.'+name+'.json'))],env=environment))
        for child in children:
            if child.wait(timeout=args.seconds+25):raise RuntimeError('Transport worker failed')
        result={name:json.loads(output.with_suffix('.'+name+'.json').read_text()) for name in ('pub','sub')}
        result['received_fraction']=result['sub']['frames']/max(result['pub']['frames'],1)
        output.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
    finally:
        for child in children:
            if child.poll() is None:child.terminate();child.wait(timeout=5)


if __name__=='__main__':main()
