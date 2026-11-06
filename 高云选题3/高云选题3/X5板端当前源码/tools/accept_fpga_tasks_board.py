"""Integration acceptance on the running board; requires UART/actuators disabled.

Reuses real camera/OCR. Only command processing and dry-run goal output are
tested; no synthetic sensor statuses are injected into the live robot domain.
"""
import json
import time
import uuid
from pathlib import Path
import rclpy
from std_msgs.msg import String
from rescue_vision.task_interaction import parse_task_text


def main():
    root=Path(__file__).resolve().parents[1]
    result={'test':'fpga_task_stack_20261011','physical_outputs_tested':False,'phases':[]}
    output=root/'logs/fpga_tasks_20261011/live_acceptance.json'
    rclpy.init();node=rclpy.create_node('fpga_task_acceptance')
    latest={};messages={}
    def receive(topic,msg):
        value=json.loads(msg.data)
        latest[topic]=value
        messages.setdefault(topic,[]).append(value)
        messages[topic]=messages[topic][-128:]
    topics=('/mission/state','/navigation/state','/fpga/tx','/fpga/link','/actuators/state',
            '/interaction/result','/scene/answer','/scene/report','/scene/ocr',
            '/diagnostics/building_ocr','/fire/auto/state','/voice/status')
    for topic in topics:
        node.create_subscription(String,topic,lambda m,t=topic:receive(t,m),20)
    pub=node.create_publisher(String,'/interaction/command',10)
    def until(predicate,seconds=5):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return
        raise AssertionError('Timeout waiting for '+str(predicate))
    def save(phase,**detail):
        result['phases'].append(dict(phase=phase,**detail))
        output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(phase,json.dumps(detail,ensure_ascii=False),flush=True)
    buildings=json.loads((root/'config/buildings.json').read_text(encoding='utf-8-sig'))
    def command(text):
        cid='accept:'+uuid.uuid4().hex
        data=parse_task_text(text,buildings)
        data.update(command_id=cid,source='test',issued_at=node.get_clock().now().nanoseconds/1e9,ttl_s=3.)
        pub.publish(String(data=json.dumps(data,ensure_ascii=False)))
        until(lambda: any(v.get('command_id')==cid for v in messages.get('/interaction/result',[])))
        reply=next(v for v in messages['/interaction/result'] if v.get('command_id')==cid)
        assert reply['accepted'],reply
        return cid,reply
    try:
        until(lambda: '/actuators/state' in latest and '/fpga/link' in latest,20)
        assert latest['/actuators/state']['dry_run'] is True
        assert latest['/actuators/state']['enabled'] is False
        assert latest['/fpga/link']['dry_run'] is True
        save('hardware_disabled_verified')
        cid,reply=command('去三号楼等待语音指令')
        until(lambda:any(v.get('kind')==0x85 and (v.get('metadata') or {}).get('building_number')=='03'
                         for v in messages.get('/fpga/tx',[])))
        save('semantic_goal_uart_dry_run',reply=reply,navigation=latest.get('/navigation/state'))
        command('到达后自动找火灭火')
        until(lambda:latest.get('/mission/state',{}).get('arrival_policy')=='auto')
        command('到达后等待语音指令')
        until(lambda:latest.get('/mission/state',{}).get('arrival_policy')=='manual')
        save('arrival_policy_switches_verified')
        command('寻找火焰')
        until(lambda:latest.get('/diagnostics/building_ocr',{}).get('counts',{}).get('completed',0)>=2
              and latest.get('/scene/ocr',{}).get('same_capture_verified') is True,30)
        ocr=latest['/diagnostics/building_ocr']
        assert ocr['reader_available'] is True,ocr
        assert ocr['counts']['invalid']==0,ocr
        assert ocr['counts']['completed']>ocr['counts']['expired'],ocr
        save('real_camera_ocr_processed',ocr_diagnostics=ocr,ocr_result=latest.get('/scene/ocr'),
             scene=latest.get('/scene/report'))
        cid,_=command('现在是几号楼')
        until(lambda:any(v.get('command_id')==cid for v in messages.get('/scene/answer',[])))
        save('building_question',answer=next(v for v in messages['/scene/answer'] if v.get('command_id')==cid))
        started=time.monotonic();cid,_=command('对当前火场进行分析')
        until(lambda:any(v.get('command_id')==cid for v in messages.get('/scene/answer',[])),85)
        answer=next(v for v in messages['/scene/answer'] if v.get('command_id')==cid)
        assert answer.get('accepted') is True,answer
        save('scene_analysis',elapsed_s=round(time.monotonic()-started,3),answer=answer)
        command('任务结束')
        until(lambda:latest.get('/mission/state',{}).get('mode')=='standby'
              and latest.get('/fire/auto/state',{}).get('active') is False)
        assert latest['/actuators/state']['pump_on'] is False
        assert latest['/actuators/state']['hardware_written'] is False
        save('returned_to_standby',mission=latest['/mission/state'])
        result['passed']=True
    except Exception as exc:
        result['passed']=False;result['error']=repr(exc)
        result['last_states']=latest
        raise
    finally:
        output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
