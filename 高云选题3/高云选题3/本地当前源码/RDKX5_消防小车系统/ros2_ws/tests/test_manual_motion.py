"""Bounded, explicit movement contracts; no ROS or hardware access."""
import importlib.util
import pathlib
import struct
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.task_interaction import parse_task_text, TaskRouter
from rescue_vision.manual_motion import parameters, validate, start_guard, runtime_guard
from rescue_vision.fpga_navigation import FpgaNavigator
from rescue_vision import protocol as p


def context(now=10.):
    return dict(mission=dict(stamp=now,vehicle_stopped=True,motion_allowed=False,traffic_active=False),
        actuator=dict(stamp=now,pump_on=False,guard=dict(armed=False)),
        telemetry=dict(stamp=now,vehicle_stopped=True,estop=False),
        link=dict(stamp=now,dry_run=False,status_fresh=True,manual_motion_enabled=True),
        navigation=dict(stamp=now,mode='preview'))


class GrammarAndSafetyTests(unittest.TestCase):
    def test_exact_aliases_and_centimetres(self):
        for text,direction,dist in [('前进','forward',.2),('向前走30厘米','forward',.3),
                ('回退','reverse',.2),('往后退二十公分','reverse',.2),('后退50厘米','reverse',.5)]:
            out=parse_task_text(text)
            self.assertEqual(out['action'],'manual_move')
            self.assertEqual((out['direction'],out['distance_m']),(direction,dist))
        for text in ('不要前进','前进吗','前进然后喷水','先后退再前进','我想知道怎么前进','向前走一米','后退-10厘米'):
            self.assertEqual(parse_task_text(text)['action'],'dialogue',text)
        # The extension UART accepts up to 10 m; the legacy manual controller
        # retains its independent 50 cm bound (tested below).
        for text in ('前进0厘米','前进4厘米','后退1001厘米'):
            with self.assertRaises(ValueError):parse_task_text(text)

    def test_limits_cannot_be_increased_or_nonfinite(self):
        for value in (True,float('nan'),float('inf'),-.2,.51):
            with self.assertRaises(ValueError):parameters('forward',value)
        for key,value in [('speed_m_s',.2),('max_duration_s',100.)]:
            request=parameters('forward');request[key]=value
            with self.assertRaises(ValueError):validate(request)

    def test_start_requires_real_fresh_stopped_unarmed_and_clear_ownership(self):
        self.assertEqual(start_guard(10.,**context()),'')
        changes=[('mission','traffic_active',True),('mission','vehicle_stopped',False),
            ('mission','motion_allowed',True),('mission','stamp',9.),('link','dry_run',True),
            ('link','manual_motion_enabled',False),('telemetry','stamp',9.6),('telemetry','estop',True),
            ('actuator','pump_on',True),('actuator','guard',{'armed':True}),
            ('navigation','mode','execute')]
        for section,key,value in changes:
            state=context();state[section][key]=value
            if section=='navigation':state[section]['goal_id']=1
            self.assertTrue(start_guard(10.,**state),(section,key))

    def test_router_default_disabled_and_reparse_and_real_output(self):
        parsed=parse_task_text('回退')
        request=dict(parsed,command_id='one',source='web',issued_at=10.,ttl_s=3.)
        with self.assertRaisesRegex(ValueError,'not_enabled'):
            TaskRouter().route(request,10.,**context())
        out=TaskRouter(manual_motion_enabled=True).route(request,10.,**context())
        self.assertEqual(out['navigation']['direction'],'reverse')
        self.assertEqual(out['mission'],dict(mode='navigate',arrival_policy='manual'))
        self.assertEqual(out['actuator']['action'],'disarm')
        for changes in ({'direction':'forward'},{'text':'不要回退'},{'text':''}):
            with self.assertRaises(ValueError):
                TaskRouter(manual_motion_enabled=True).route(dict(request,**changes),10.,**context())


class ManualNavigatorTests(unittest.TestCase):
    def setUp(self):
        self.core=FpgaNavigator({'buildings':[{'number':3,'building_id':'C','name':'春晖楼'}]},
                                session_id=42,manual_motion_enabled=True)
    def request(self,now=10.,**changes):
        request=dict(parameters('reverse'),action='manual_move',command_id='one',mode='execute',
                     confirmed=True,issued_at=now,ttl_s=3.)
        request.update(changes)
        return self.core.request(request,now)
    def status(self,now=10.1,seq=1,**changes):
        status=dict(session_id=42,goal_id=self.core.active['goal_id'],building_number=0,
            motion_kind='manual',obstacle_clear=True,status_sequence=seq,state='accepted',
            vehicle_stopped=True,estop=False,stamp=now)
        status.update(changes)
        return self.core.observe(status,now)
    def test_default_disabled(self):
        self.core.manual_motion_enabled=False
        with self.assertRaisesRegex(ValueError,'not_enabled'):self.request()
    def test_acceptance_requires_direction_status_and_does_not_extend_deadline(self):
        self.request();deadline=self.core.active['deadline']
        a=self.core.next_command(10.);b=self.core.next_command(10.5)
        self.assertEqual(a['goal_id'],b['goal_id'])
        self.assertFalse(self.status(motion_kind='navigation'))
        self.assertTrue(self.status())
        self.assertTrue(self.core.proposal(10.1)['ready_to_move'])
        self.assertEqual(self.core.active['deadline'],deadline)
        with self.assertRaises(ValueError):self.request(now=10.2,command_id='repeat')
    def test_deadline_cancels_and_late_acceptance_cannot_restart(self):
        self.request();self.status()
        self.assertFalse(self.core.proposal(14.)['ready_to_move'])
        self.assertEqual(self.core.next_command(14.)['action'],'cancel')
        self.assertFalse(self.status(now=14.1,seq=2))
        self.assertTrue(self.status(now=14.2,seq=3,state='cancelled'))
        self.assertFalse(self.core.proposal(14.2)['ready_to_move'])
    def test_obstacle_stop_is_latched(self):
        self.request();self.assertTrue(self.status(obstacle_clear=False))
        self.assertEqual(self.core.next_command(10.1)['action'],'cancel')
        self.assertFalse(self.status(now=10.2,seq=2,obstacle_clear=True))
    def test_pause_never_resumes_manual_and_cancel_ack_releases_ownership(self):
        self.request();self.status()
        self.request(now=10.2,command_id='stop',action='pause')
        self.assertFalse(self.core.proposal(10.2)['ready_to_move'])
        self.status(now=10.3,seq=2,state='paused')
        with self.assertRaises(ValueError):self.request(now=10.4,command_id='resume',action='resume')
        with self.assertRaises(ValueError):self.request(now=10.4,command_id='next')
        self.request(now=10.4,command_id='cancel',action='cancel')
        self.status(now=10.5,seq=3,state='cancelled')
        self.request(now=10.6,command_id='next')
    def test_navigation_and_manual_are_exclusive(self):
        self.request()
        with self.assertRaises(ValueError):self.request(now=10.1,command_id='go3',action='navigate',building_id='C')
        self.core=FpgaNavigator({'buildings':[{'number':3,'building_id':'C','name':'春晖楼'}]},
                               session_id=42,manual_motion_enabled=True)
        self.request(action='navigate',building_id='C')
        with self.assertRaises(ValueError):self.request(now=10.1,command_id='manual')
    def test_wire_signed_reverse_and_reserved_clearance_validation(self):
        raw=p.manual_move_payload(42,1,parameters('reverse'))
        self.assertEqual(struct.unpack('<IIbHHHH',raw),(42,1,-1,100,200,4000,1000))
        self.assertEqual(p.Parser().feed(p.encode(p.MANUAL_MOVE,4,raw))[0].payload,raw)
        raw=struct.pack(p.NAV_STATUS_FORMAT,42,1,7,1,5,0,0)
        status=p.manual_status(p.Packet(p.MANUAL_STATUS,4,raw),10.)
        self.assertTrue(status['obstacle_clear']);self.assertTrue(status['vehicle_stopped'])
        for flags,reserved in ((8,0),(5,1)):
            with self.assertRaises(ValueError):p.manual_status(p.Packet(p.MANUAL_STATUS,4,
                struct.pack(p.NAV_STATUS_FORMAT,42,1,7,1,flags,reserved,0)),10.)


class ConsoleManualTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('manual_console_test',ROOT/'tools/presentation_server.py')
        cls.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.module)
    def state(self):
        state=self.module.PresentationState(monotonic=lambda:10.,wall=lambda:10.)
        state.ros.update(available=True,router_ready=True)
        for key,value in context().items():
            state.update({'actuator':'actuators','link':'fpga_link','telemetry':'fpga_telemetry'}.get(key,key),value)
        state.update('extension',dict(stamp=10.,real_uart=True,protocol_ready=True,status_fresh=True,
            can_move=True,vehicle_stopped=True,capabilities={'distance':True}))
        return state
    def test_manual_checked_before_enqueue_and_again_before_dispatch(self):
        state=self.state();state.update('extension',dict(stamp=10.,real_uart=False))
        with self.assertRaises(self.module.ApiError):state.enqueue(text='前进',request_id='one')
        state=self.state();state.enqueue(text='回退',request_id='two')
        state.update('extension',dict(stamp=10.,real_uart=True,protocol_ready=True,status_fresh=True,
            can_move=False,vehicle_stopped=False,capabilities={'distance':True},reason='vehicle_not_stopped'))
        # The same public execution gate is used by queue draining before ROS publication.
        ready,reason=state.execution_ready(parse_task_text('回退'))
        self.assertFalse(ready);self.assertIn('stopped',reason)
    def test_voice_meter_updates_do_not_flood_evidence(self):
        state=self.state()
        for i in range(100):state.update('voice',dict(stamp=10.,state='listening',level_dbfs=i,
            peak_dbfs=i,elapsed_s=i/4.,remaining_s=30.-i/4.,level_stamp=i))
        self.assertEqual(len(state.events),1)
        state.update('voice',dict(stamp=10.,state='transcribing'))
        self.assertEqual(len(state.events),2)
        state.update('dialogue_status',dict(stamp=10.,state='ready'))
        self.assertEqual(len(state.events),2)


if __name__=='__main__':unittest.main(verbosity=2)
