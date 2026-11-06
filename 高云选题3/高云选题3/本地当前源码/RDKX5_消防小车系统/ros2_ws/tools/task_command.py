#!/usr/bin/env python3
"""Send one text command on the sourced robot ROS domain (no direct hardware IO)."""
import argparse
import json
import math
import time
import uuid
import rclpy
from std_msgs.msg import String
from rescue_vision.node_common import read_json
from rescue_vision.task_interaction import parse_task_text
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('text',nargs='?',help='例如：开始巡逻、开启识别、锁定火焰2、停止喷水、现在有几个人')
    parser.add_argument('--targets',action='store_true',help='Show current flame numbers instead of issuing a command')
    parser.add_argument('--wait-answer',action='store_true',help='Wait for the matching scene/LLM answer after the task receipt')
    parser.add_argument('--answer-timeout',type=float,default=90.,help='Answer wait in seconds (1–180; default 90)')
    args=parser.parse_args()
    if not args.targets and not args.text:parser.error('Provide a command or --targets')
    if not math.isfinite(args.answer_timeout) or not 1 <= args.answer_timeout <= 180:
        parser.error('--answer-timeout must be between 1 and 180 seconds')
    if args.targets and args.wait_answer:parser.error('--targets cannot be combined with --wait-answer')
    rclpy.init();node=rclpy.create_node('task_command_client');replies=[];answers=[]
    try:
        if args.targets:
            node.create_subscription(String,'/fire/targets',lambda m:replies.append(json.loads(m.data)),10)
            deadline=time.monotonic()+3
            while time.monotonic()<deadline and not replies:rclpy.spin_once(node,timeout_sec=.1)
        else:
            config=read_json(Path(__file__).resolve().parents[1]/'config/buildings.json')
            data=parse_task_text(args.text,config);cid=str(uuid.uuid4())
            if args.wait_answer and data['action'] not in ('dialogue','report','scene'):
                raise ValueError('--wait-answer is only for questions or scene reports; no command sent')
            def reply(message):
                value=json.loads(message.data)
                if value.get('command_id')==cid:replies.append(value)
            node.create_subscription(String,'/interaction/result',reply,10)
            def answer(message):
                value=json.loads(message.data)
                if value.get('command_id')==cid:answers.append(value)
            if args.wait_answer:
                # Subscribe before sending, including for immediately answered factual questions.
                node.create_subscription(String,'/scene/answer',answer,10)
            pub=node.create_publisher(String,'/interaction/command',10)
            deadline=time.monotonic()+3
            while not pub.get_subscription_count() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
            if not pub.get_subscription_count():raise RuntimeError('Task router unavailable: start run_task_modes.sh')
            data.update(command_id=cid,source='text',issued_at=node.get_clock().now().nanoseconds/1e9,ttl_s=3.)
            pub.publish(String(data=json.dumps(data,ensure_ascii=False)))
            deadline=time.monotonic()+3
            while time.monotonic()<deadline and not replies:rclpy.spin_once(node,timeout_sec=.1)
        if not replies:raise RuntimeError('No fresh task reply received')
        print(json.dumps(replies[-1],ensure_ascii=False,indent=2),flush=True)
        if args.wait_answer and replies[-1].get('accepted'):
            deadline=time.monotonic()+args.answer_timeout
            while time.monotonic()<deadline and not answers:rclpy.spin_once(node,timeout_sec=.1)
            if not answers:raise RuntimeError('No matching scene answer before --answer-timeout')
            print(json.dumps(answers[-1],ensure_ascii=False,indent=2),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
