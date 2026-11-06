"""Real threading/queue tests with fake cores; no ROS or serial hardware."""
from collections import deque
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
source=ROOT/'src/rescue_vision/rescue_vision/pd42_test_node.py'
class NodeStub:
    def destroy_node(self):
        self.base_destroyed=True
        return True
spec=importlib.util.spec_from_file_location('rescue_vision._dual_worker_test_node',source)
module=importlib.util.module_from_spec(spec)
with patch.dict(sys.modules,{'rclpy.node':SimpleNamespace(Node=NodeStub),
        'std_msgs.msg':SimpleNamespace(String=object),
        'rescue_vision.node_common':SimpleNamespace(publish_json=lambda *args:None,spin=lambda *args:None)}):
    spec.loader.exec_module(module)


class FakeCore:
    def __init__(self,motor_id):
        self.motor_id=motor_id;self.calls=[];self.guard_probe=None
        self.entered=threading.Event();self.release=threading.Event();self.stopped=threading.Event()
        self.closed=threading.Event();self.raise_tick=False
        self.state=dict(connected=False,feedback_fresh=True,can_enable=True,can_move=True,can_run=True,
            busy=False,position_counts=0,position_deg=0.,speed_rpm=0,enabled=False,run_state=0,
            voltage_v=12.,last_request_id=None,last_action=None,last_motor_id=motor_id,last_result=None,
            reason='',continuous_running=False,run_id=None,keepalive_remaining_s=0.)
    def snapshot(self,now):return dict(self.state,stamp=now,motor_id=self.motor_id)
    def handle(self,request,now,evidence):
        self.calls.append(dict(request))
        if request['action']=='read':
            self.entered.set();self.release.wait(2.)
        if request['action']=='keepalive':return
        if request['action'] in ('stop','disconnect'):
            self.state.update(continuous_running=False,run_id=None,enabled=False)
            self.stopped.set()
        if request['action'].startswith('run_'):
            self.state.update(continuous_running=True,run_id=request['request_id'],enabled=True)
        self.state.update(last_request_id=request['request_id'],last_action=request['action'],
                          last_result='completed',reason='done')
    def tick(self,*args):
        if self.raise_tick:raise RuntimeError('test_uart_worker_failure')
    def close(self):self.state.update(connected=False,enabled=False);self.closed.set()


def wait_for(predicate,timeout=1.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.005)
    raise AssertionError('condition not reached')


class DualWorkerTests(unittest.TestCase):
    def setUp(self):
        self.cores={m:FakeCore(m) for m in ('motor1','motor2')};self.errors=[]
        self.workers={m:module.MotorWorker(m,c,time.time,lambda:{},self.errors.append)
                      for m,c in self.cores.items()}
        for w in self.workers.values():w.start()
    def tearDown(self):
        for c in self.cores.values():c.release.set()
        for w in self.workers.values():w.request_close()
        for w in self.workers.values():self.assertTrue(w.join(1.))
    def request(self,action,motor='motor1',request_id=None,**extra):
        return dict(action=action,motor_id=motor,request_id=request_id or action+'-'+motor,
                    issued_at=time.time(),ttl_s=1.,**extra)
    def test_startup_runs_workers_without_any_hardware_action(self):
        self.assertTrue(all(w.thread.is_alive() for w in self.workers.values()))
        self.assertTrue(all(c.calls==[] for c in self.cores.values()))
    def test_motor2_stop_completes_while_motor1_transaction_is_blocked(self):
        self.workers['motor1'].submit(self.request('read'))
        self.assertTrue(self.cores['motor1'].entered.wait(1.))
        self.workers['motor2'].submit(self.request('stop','motor2'))
        self.assertTrue(self.cores['motor2'].stopped.wait(.5))
        self.assertFalse(self.cores['motor1'].release.is_set())
        self.assertEqual([r['action'] for r in self.cores['motor1'].calls],['read'])
    def test_two_continuous_runs_have_independent_run_ids_and_stop(self):
        for motor in self.workers:self.workers[motor].submit(self.request('run_positive',motor))
        wait_for(lambda:all(c.state['continuous_running'] for c in self.cores.values()))
        self.assertNotEqual(self.cores['motor1'].state['run_id'],self.cores['motor2'].state['run_id'])
        self.workers['motor1'].submit(self.request('stop'))
        self.assertTrue(self.cores['motor1'].stopped.wait(.5))
        self.assertTrue(self.cores['motor2'].state['continuous_running'])
    def test_stop_preempts_only_same_motor_pending_and_clears_heartbeat(self):
        worker=self.workers['motor1'];worker.submit(self.request('read'))
        self.assertTrue(self.cores['motor1'].entered.wait(1.))
        for n in range(30):worker.submit(self.request('keepalive',request_id=str(n),run_id='run'))
        self.assertEqual(worker.keepalive['request_id'],'29')
        worker.submit(self.request('stop'))
        self.assertIsNone(worker.keepalive);self.assertEqual(worker.pending['action'],'stop')
        self.assertFalse(worker.submit(self.request('keepalive',run_id='run')))
        self.assertEqual(worker.pending['action'],'stop')
        self.cores['motor1'].release.set();self.assertTrue(self.cores['motor1'].stopped.wait(1.))
        self.assertNotIn('keepalive',[r['action'] for r in self.cores['motor1'].calls])
    def test_heartbeat_coalesces_without_replacing_operator_receipts(self):
        worker=self.workers['motor1'];worker.submit(self.request('read'))
        self.assertTrue(self.cores['motor1'].entered.wait(1.))
        for n in range(30):worker.submit(self.request('keepalive',request_id=str(n),run_id='run'))
        self.assertEqual(worker.keepalive['request_id'],'29')
        self.assertFalse(any(r['action']=='keepalive' for r in worker.receipts))
        self.cores['motor1'].release.set()
        wait_for(lambda:any(r['action']=='keepalive' for r in self.cores['motor1'].calls))
        calls=[r for r in self.cores['motor1'].calls if r['action']=='keepalive']
        self.assertEqual([r['request_id'] for r in calls],['29'])
        self.assertEqual(worker.snapshot()['last_action'],'read')
    def test_wrong_worker_rejection_does_not_touch_the_motor(self):
        self.assertFalse(self.workers['motor1'].submit(self.request('stop','motor2')))
        self.assertEqual(self.cores['motor1'].calls,[])
        self.assertEqual(self.workers['motor1'].snapshot()['reason'],'motor_test_wrong_worker')
    def test_stale_worker_snapshot_cannot_refresh_measurement_or_movement_buttons(self):
        worker=self.workers['motor1'];worker.submit(self.request('read'))
        self.assertTrue(self.cores['motor1'].entered.wait(1.))
        with worker.lock:worker.view['stamp']=time.time()-2.
        snapshot=worker.snapshot()
        self.assertFalse(snapshot['can_run']);self.assertFalse(snapshot['can_move'])
        self.assertFalse(snapshot['feedback_fresh']);self.assertIsNone(snapshot['position_deg'])
        self.assertGreater(snapshot['worker_snapshot_age_s'],.7)
        self.assertTrue(self.workers['motor2'].snapshot()['worker_alive'])
    def test_one_worker_failure_closes_only_it_and_other_stop_still_works(self):
        self.cores['motor1'].raise_tick=True;self.workers['motor1'].wake.set()
        self.assertTrue(self.cores['motor1'].closed.wait(1.))
        self.workers['motor2'].submit(self.request('stop','motor2'))
        self.assertTrue(self.cores['motor2'].stopped.wait(.5))
        self.assertFalse(self.workers['motor1'].snapshot()['worker_alive'])
        self.assertTrue(self.workers['motor2'].snapshot()['worker_alive'])
    def test_shutdown_signals_both_before_join(self):
        self.workers['motor1'].submit(self.request('read'));self.assertTrue(self.cores['motor1'].entered.wait(1.))
        for worker in self.workers.values():worker.request_close()
        self.assertTrue(self.cores['motor2'].closed.wait(.5))
        self.assertFalse(self.cores['motor1'].closed.is_set())
        self.cores['motor1'].release.set();self.assertTrue(self.cores['motor1'].closed.wait(1.))
    def test_ros_dispatch_and_aggregate_keep_motor_states_and_receipts_separate(self):
        node=module.PD42Test.__new__(module.PD42Test)
        node.workers=self.workers;node.lock=threading.Lock();node.invalid_requests=deque(maxlen=8)
        node.now=time.time
        for motor in self.workers:
            request=self.request('connect',motor)
            node.request(SimpleNamespace(data=json.dumps(request)))
        wait_for(lambda:all(c.calls for c in self.cores.values()))
        state=node.state();self.assertTrue(state['independent_workers']);self.assertEqual(state['motor_count'],2)
        self.assertEqual(set(state['motors']),{'motor1','motor2'})
        for motor,snapshot in state['motors'].items():
            self.assertEqual(snapshot['motor_id'],motor)
            self.assertTrue(all(r['motor_id']==motor for r in snapshot['recent_requests']))
        self.assertEqual(state['motor_id'],'motor1')
        invalid=self.request('stop','motor3')
        node.request(SimpleNamespace(data=json.dumps(invalid)))
        self.assertEqual(node.state()['recent_requests'][-1]['reason'],'invalid_motor_test_motor_id')
    def test_ros_shutdown_publish_error_still_closes_both_workers_and_destroys_node(self):
        node=module.PD42Test.__new__(module.PD42Test)
        node.workers=self.workers;node.publisher=object();node.state=lambda:{}
        with patch.object(module,'publish_json',side_effect=RuntimeError('ROS context invalid')):
            self.assertTrue(node.destroy_node())
        self.assertTrue(node.base_destroyed)
        for core in self.cores.values():self.assertTrue(core.closed.is_set())


if __name__=='__main__':unittest.main()
