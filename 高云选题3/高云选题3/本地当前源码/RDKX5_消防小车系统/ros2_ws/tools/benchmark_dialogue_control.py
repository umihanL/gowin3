#!/usr/bin/env python3
"""Real Qwen plus read-only facts/stop receipts; physical outputs must be disabled."""
import json
from pathlib import Path
import time
import uuid
import rclpy
from std_msgs.msg import String
from rescue_vision.task_interaction import parse_task_text


def main():
    root=Path(__file__).resolve().parents[1]
    rclpy.init();node=rclpy.create_node('dialogue_control_benchmark')
    latest={};messages={};arrivals={}
    def receive(topic,msg):
        value=json.loads(msg.data);latest[topic]=value
        cid=value.get('command_id')
        if cid:messages[topic,cid]=value;arrivals[topic,cid]=time.monotonic()
    topics=('/actuators/state','/fpga/link','/interaction/result','/scene/answer')
    for topic in topics:
        node.create_subscription(String,topic,lambda msg,t=topic:receive(t,msg),10)
    pub=node.create_publisher(String,'/interaction/command',10)
    def until(test,seconds=8):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if test():return
        raise RuntimeError('Timed out waiting for command/answer')
    config=json.loads((root/'config/buildings.json').read_text(encoding='utf-8-sig'))
    def send(text):
        cid='perf:'+uuid.uuid4().hex;request=parse_task_text(text,config)
        request.update(command_id=cid,source='test',issued_at=node.get_clock().now().nanoseconds/1e9,ttl_s=3.)
        start=time.monotonic();pub.publish(String(data=json.dumps(request,ensure_ascii=False)))
        until(lambda:('/interaction/result',cid) in messages)
        assert messages['/interaction/result',cid]['accepted']
        return cid,start
    result={'hardware_written':False}
    try:
        until(lambda:'/actuators/state' in latest and '/fpga/link' in latest
              and pub.get_subscription_count()>0 and all(node.count_publishers(t)>0 for t in topics),20)
        # Graph discovery precedes endpoint matching on Humble/Fast DDS. Let
        # this newly-created diagnostic client finish discovery before timing.
        ready_at=time.monotonic()+1.
        while time.monotonic()<ready_at:rclpy.spin_once(node,timeout_sec=.02)
        assert latest['/actuators/state']['dry_run'] is True and latest['/actuators/state']['enabled'] is False
        assert latest['/fpga/link']['dry_run'] is True
        llm,start=send('对当前火场进行分析')
        fact,ft=send('现在有几个人')
        until(lambda:('/scene/answer',fact) in messages)
        result['fact_answer_ms']=(arrivals['/scene/answer',fact]-ft)*1000
        result['fact_answer']=messages['/scene/answer',fact]
        stop,st=send('停止喷水')
        result['stop_receipt_ms']=(arrivals['/interaction/result',stop]-st)*1000
        until(lambda:('/scene/answer',llm) in messages,85)
        result['llm_answer_s']=arrivals['/scene/answer',llm]-start
        result['llm_answer']=messages['/scene/answer',llm]
        result['llm_was_pending_during_fact']=arrivals['/scene/answer',fact]<arrivals['/scene/answer',llm]
        assert latest['/actuators/state']['hardware_written'] is False
        assert latest['/actuators/state']['pump_on'] is False
        result['passed']=True
    except Exception as exc:
        result.update(passed=False,error=repr(exc));raise
    finally:
        (root/'logs/performance_20261011/dialogue_control.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k not in ('llm_answer','fact_answer')},ensure_ascii=False),flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
