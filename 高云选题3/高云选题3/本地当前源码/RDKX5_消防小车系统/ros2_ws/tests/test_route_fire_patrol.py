"""Real route/core/protocol state machines, with simulated replies only."""
import unittest
import test_numbered_navigation as hw
from rescue_vision.task_interaction import parse_task_text


def actuator(now, **changes):
    return dict(dict(stamp=now, enabled=True, dry_run=False, status_fresh=True,
        feedback_fresh=True, reference_confirmed_this_start=True, pump_enabled=True,
        pump_on=False, pump_feedback_fresh=True, pump_ready=True, guard=dict(armed=False)), **changes)


def frame(now, target=None, session='camera-session'):
    return dict(capture_stamp=now, valid=target is not None, frame_valid=True, registry_session=session, image_size_wh=[640,480],
        targets=[] if target is None else [dict(target_id=target, selectable=True,bbox_xyxy=[100,100,180,220])])


def ready():
    c = hw.ready(); c.task = 'dialogue'; c.capabilities['pump_b13'] = True
    c.actuator = actuator(100.)
    c.mission_state = dict(stamp=100., mounting_verified=True, gimbal_transport='x5')
    c.auto_state = dict(stamp=100., active=False, phase='IDLE')
    c.observe_targets(frame(100.),100.)
    return c


def running():
    c = ready()
    # Legacy pause/resume primitive coverage only. Public patrol instructions
    # are retired; explicit region search is tested by test_region_* instead.
    text = '我现在在2号点，车头朝上，请前往3号点'
    c.request(hw.request('navigate_point',100.02,**{k:v for k,v in parse_task_text(text).items() if k!='action'}),100.02)
    c.numbered.active['patrol_fire']=True
    c.numbered.patrol.start(True,100.02)
    hw.ack(c,100.03); hw.ack(c,100.035,bytes((0,0x84,3,0)))
    hw.ack(c,100.04,bytes((0,3,1,c.numbered.active['task_id'])))
    hw.query(c,hw.wire.POINT_STATUS,bytes((0,3,2,0)),100.05)
    hw.query(c,hw.wire.HEARTBEAT,bytes((0,0,19,c.numbered.active['task_id'])),100.06)
    hw.ack(c,100.07); hw.telemetry(c,100.08)
    return c


def feedback(c,now,*,run=True,stopped=False,motion=2,**kwargs):
    c.actuator=dict(c.actuator,stamp=now)
    c.mission_state=dict(c.mission_state,stamp=now)
    c.auto_state=dict(c.auto_state,stamp=now)
    c.observe_targets(frame(now),now)
    hw.telemetry(c,now,run=run,stopped=stopped,motion=motion,**kwargs)


def detect(c,now,target=1):
    for offset in (0.,.05,.10):
        c.observe_targets(frame(now+offset,target),now+offset)


def pause(c,now=100.2,target=1):
    detect(c,now,target)
    a=c.numbered.active
    assert a['phase']=='pausing',a
    c.receive(hw.wire.Packet(a['pause_sequence'],hw.wire.STOP|0x80,bytes(4)),now+.11)
    feedback(c,now+.12,run=False,stopped=True)
    assert a['phase']=='paused',a
    return c.numbered.patrol.run_id


def finish_round(c,now):
    patrol=c.numbered.patrol
    c.auto_state=dict(stamp=now,command_id=patrol.run_id,active=False,phase='COMPLETE',
        reason='one_target_round_finished',burst_completed=True,confirmed_pump_off_at=now-.01)
    feedback(c,now,run=False,stopped=True)
    assert patrol.phase=='WAIT_PUMP_OFF',patrol.snapshot(now)
    c.auto_state.update(phase='CANCELLED',reason='operator_cancelled')
    feedback(c,now+.1,run=False,stopped=True)
    a=c.numbered.active
    assert a['phase']=='resuming',a
    c.receive(hw.wire.Packet(a['resume_sequence'],hw.wire.RESUME|0x80,bytes(4)),now+.12)
    feedback(c,now+.14)
    assert patrol.phase=='COOLDOWN',patrol.snapshot(now)


class RoutePatrolTests(unittest.TestCase):
    def test_two_fire_rounds_keep_task_route_then_arrive(self):
        c=running(); a=c.numbered.active; task=a['task_id']; cid=a['command_id']
        run1=pause(c)
        self.assertEqual(c.numbered.patrol.phase,'EXTINGUISHING')
        self.assertFalse(c.state(100.34)['can_move'])
        self.assertTrue(c.numbered.patrol.pump_allowed(100.34))
        auto=[v for k,v in c.effects if k=='auto' and v['action']=='start'][-1]
        self.assertEqual((auto['preferred_target_id'],auto['preferred_registry_session']),(1,'camera-session'))
        self.assertFalse(auto['return_to_standby'])
        self.assertFalse(c.numbered.state(100.34)['arrived'])
        finish_round(c,100.6)
        until=c.numbered.patrol.ignore_until
        # Fresh frames in cooldown remain visible but cannot trigger STOP.
        feedback(c,until-.2); detect(c,until-.15,target=2)
        self.assertEqual(a['phase'],'running')
        feedback(c,until+.02); run2=pause(c,until+.04,target=2)
        self.assertNotEqual(run1,run2)
        finish_round(c,until+.5)
        self.assertEqual(c.numbered.patrol.rounds,2)
        self.assertEqual((a['task_id'],a['command_id']),(task,cid))
        commands=[v[4] for v in c.outbox]
        self.assertEqual(commands.count(hw.wire.SET_START),1)
        self.assertEqual(commands.count(hw.wire.SELECT_POINT),1)
        self.assertEqual(commands.count(hw.wire.START),1)
        self.assertEqual(commands.count(hw.wire.RESUME),2)
        hw.done(c,until+.8); feedback(c,until+.85,run=False,stopped=True,motion=3)
        self.assertTrue(c.numbered.state(until+.87)['arrived'])
        self.assertEqual(c.numbered.patrol.phase,'COMPLETE')

    def test_repeated_frame_and_identity_switch_do_not_pause(self):
        c=running()
        for _ in range(5):c.observe_targets(frame(100.2,1),100.2)
        self.assertEqual(c.numbered.active['phase'],'running')
        c.observe_targets(frame(100.25,2),100.25)
        c.observe_targets(frame(100.3,1),100.3)
        self.assertEqual(c.numbered.active['phase'],'running')

    def test_destination_reached_while_pause_is_in_flight_wins_over_fire(self):
        c=running();detect(c,100.2)
        hw.done(c,100.31)
        feedback(c,100.32,run=False,stopped=True,motion=3)
        self.assertTrue(c.numbered.state(100.34)['arrived'])
        self.assertEqual(c.numbered.patrol.phase,'COMPLETE')
        self.assertFalse(any(k=='auto' and v['action']=='start' for k,v in c.effects))

    def test_pause_ack_is_not_stop_confirmation(self):
        c=running();detect(c,100.2)
        a=c.numbered.active
        c.receive(hw.wire.Packet(a['pause_sequence'],hw.wire.STOP|0x80,bytes(4)),100.31)
        feedback(c,100.32,run=False,stopped=False)
        self.assertEqual(a['phase'],'pausing')
        self.assertFalse(any(k=='auto' and v['action']=='start' for k,v in c.effects))

    def test_terminal_status_before_done_during_pause_waits_for_event(self):
        c=running();detect(c,100.2)
        feedback(c,100.32,run=False,stopped=True,motion=3)
        self.assertEqual(c.numbered.active['phase'],'pausing')
        self.assertFalse(any(k=='auto' and v['action']=='start' for k,v in c.effects))
        hw.done(c,100.34);feedback(c,100.36,run=False,stopped=True,motion=3)
        self.assertTrue(c.numbered.state(100.38)['arrived'])

    def test_wrong_paused_task_cannot_spray_or_resume(self):
        c=running(); pause(c)
        feedback(c,100.4,run=False,stopped=True,task=99)
        self.assertEqual(c.numbered.active['phase'],'stopping')
        self.assertFalse(c.numbered.patrol.pump_allowed(100.42))
        self.assertFalse(any(f[4]==hw.wire.RESUME for f in c.outbox))

    def test_abort_or_unconfirmed_burst_never_resumes(self):
        for changes in (dict(reason='pump_hardware_not_enabled'),dict(burst_completed=False),
                        dict(confirmed_pump_off_at=None),dict(phase='CANCELLED')):
            c=running();run=pause(c)
            c.auto_state=dict(stamp=100.6,command_id=run,active=False,phase='COMPLETE',
                reason='one_target_round_finished',burst_completed=True,confirmed_pump_off_at=100.59)
            c.auto_state.update(changes)
            feedback(c,100.6,run=False,stopped=True)
            self.assertEqual(c.numbered.active['phase'],'stopping',changes)
            self.assertFalse(any(f[4]==hw.wire.RESUME for f in c.outbox))

    def test_missing_or_high_pump_feedback_never_resumes(self):
        for on in (None,True):
            c=running();run=pause(c)
            c.auto_state=dict(stamp=100.6,command_id=run,active=False,phase='COMPLETE',
                reason='one_target_round_finished',burst_completed=True,confirmed_pump_off_at=100.59)
            feedback(c,100.6,run=False,stopped=True)
            c.actuator['pump_on']=on
            feedback(c,100.7,run=False,stopped=True)
            self.assertEqual(c.numbered.active['phase'],'paused')
            self.assertFalse(any(f[4]==hw.wire.RESUME for f in c.outbox))

    def test_cancel_discards_old_round_authority_and_no_delayed_completion_revives(self):
        c=running();run=pause(c)
        c.stop(100.4,'operator_stop')
        self.assertFalse(c.numbered.patrol.enabled)
        self.assertTrue(any(k=='auto' and v['action']=='cancel' for k,v in c.effects))
        c.auto_state=dict(stamp=100.5,command_id=run,active=False,phase='COMPLETE',
            reason='one_target_round_finished',burst_completed=True,confirmed_pump_off_at=100.49)
        feedback(c,100.5,run=False,stopped=True)
        self.assertEqual(c.numbered.active['phase'],'cancelled')
        self.assertFalse(any(f[4]==hw.wire.RESUME for f in c.outbox))

    def test_retired_patrol_authority_rejected_before_route_upload(self):
        c=ready();c.actuator['dry_run']=True;c.outbox.clear()
        with self.assertRaises(ValueError):
            parse_task_text('我现在在2号点，车头朝上，请前往3号点，沿途自动灭火')
        with self.assertRaisesRegex(ValueError,'along_route_fire_replaced'):
            c.request(dict(action='navigate_point',patrol_fire=True,command_id='invalid-start',
                source='web',issued_at=100.02,ttl_s=3.),100.02)
        self.assertIsNone(c.numbered.active)
        self.assertFalse(c.outbox)

    def test_yolo_loss_stops_patrol_but_normal_navigation_is_unchanged(self):
        c=running()
        hw.telemetry(c,101.3)
        self.assertEqual(c.numbered.active['reason'],'patrol_yolo_feedback_lost')
        normal=hw.running(); hw.telemetry(normal,101.3)
        self.assertEqual(normal.numbered.active['phase'],'running')

    def test_confirmed_pause_with_missing_pump_off_has_bounded_wait(self):
        c=running(); detect(c,100.2)
        c.actuator['pump_on']=None
        a=c.numbered.active
        c.receive(hw.wire.Packet(a['pause_sequence'],hw.wire.STOP|0x80,bytes(4)),100.31)
        feedback(c,100.32,run=False,stopped=True)
        self.assertEqual(c.numbered.patrol.phase,'WAIT_STOP')
        feedback(c,104.4,run=False,stopped=True)
        self.assertEqual(a['phase'],'stopping')
        self.assertEqual(a['reason'],'patrol_stop_not_confirmed')


if __name__=='__main__':unittest.main()
