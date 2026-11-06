"""No hardware: commissioning actions, measured arrival and stop boundaries."""
import pathlib
import importlib.util
import json
import struct
import sys
import threading
import unittest
from collections import deque
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.pd42_test import PD42Bench, bench_guard, validate_request
from rescue_vision.pd42_protocol import Command, degrees_to_counts


class Clock:
    value = 100.
    def __call__(self): return self.value
    def advance(self, seconds): self.value += seconds


class FakeTransport:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs
        self.writes = []; self.closed = False; self.position = 47360
        self.enabled = False; self.speed = 0; self.status = 1; self.stalled = False
        self.target = None; self.fail = None
    def exchange(self, request):
        self.writes.append(request); command = Command(request[2])
        if command == Command.ENABLE:
            self.enabled = request[3] == 0
            if not self.enabled: self.speed = 0; self.status = 0
        elif command == Command.ABSOLUTE_POSITION:
            direction, acceleration, rpm, magnitude = struct.unpack('>BBHI', request[3:-2])
            self.target = -magnitude if direction else magnitude
            self.speed = rpm; self.status = 2
        elif command == Command.VELOCITY:
            direction, acceleration, rpm = struct.unpack('>BBf', request[3:-2])
            self.speed = -rpm if direction else rpm; self.status = 2
        if command == self.fail:
            self.fail = None
            raise TimeoutError('lost_reply')
        values = {Command.READ_VERSION: {'firmware': 1.8, 'hardware': 1.0},
                  Command.READ_VOLTAGE: 11.86, Command.READ_POSITION: self.position,
                  Command.READ_SPEED: self.speed, Command.READ_STATUS: self.status,
                  Command.READ_ENABLED: self.enabled, Command.READ_STALL: self.stalled}
        return SimpleNamespace(value=values.get(command), ok=True)
    def arrive(self): self.position=self.target;self.speed=0;self.status=1
    def close(self): self.closed=True
    def commands(self): return [Command(r[2]) for r in self.writes]


class FakeZDTTransport:
    """Wire-format fixture: ZDT replies have no PD42 header or success byte."""
    def __init__(self, *args, **kwargs):
        self.args,self.kwargs=args,kwargs;self.writes=[];self.closed=False
        self.position=65536;self.enabled=False;self.speed=0;self.arrived=True
        self.stalled=False;self.target=None;self.fail=None;self.mode='emm'
        self.ack_mode=1;self.microsteps=16;self.scaled=False;self.closed_loop=True
        self.mode_mismatch=False;self.malformed=False
        self.extended_options=False
        self.firmware_raw=206;self.hardware_raw=0x0314
    def exchange(self, request):
        from rescue_vision import zdt_xs_protocol as z
        z.validate_request(request);self.writes.append(request);cmd=z.Command(request[1])
        if cmd==z.Command.ENABLE:
            self.enabled=bool(request[3])
            if not self.enabled:self.speed=0
        elif cmd in (z.Command.POSITION_EMM,z.Command.POSITION_X):
            magnitude=int.from_bytes(request[6:10] if cmd==z.Command.POSITION_EMM else request[5:9],'big')
            direction=-1 if request[2] else 1
            if cmd==z.Command.POSITION_EMM:
                angle=direction*magnitude*1.8/self.microsteps
                self.target=round(angle*65536/360)
            else:self.target=round(direction*magnitude/(10 if self.scaled else 1))
            self.speed=5 if self.mode=='emm' else 50;self.arrived=False
        elif cmd==z.Command.VELOCITY:
            direction=-1 if request[2] else 1
            if self.mode=='emm':
                velocity=int.from_bytes(request[3:5],'big')/(10 if self.scaled else 1)
            else:velocity=int.from_bytes(request[5:7],'big')
            self.speed=int(direction*velocity);self.arrived=False
        if cmd==self.fail:
            self.fail=None;raise TimeoutError('zdt lost reply')
        if self.malformed:raise z.ProtocolError('bad packet')
        if cmd in z.CONTROL_COMMANDS:frame=bytes((1,cmd,2,0x6B))
        elif cmd==z.Command.READ_CONFIG:
            if self.mode=='emm':
                f=bytearray(33);f[:4]=bytes.fromhex('01 42 21 15');f[4]=0x19;f[5]=2;f[6]=2
                f[9]=self.microsteps%256;f[18]=5;f[20]=1;f[22]=self.ack_mode
            else:
                f=bytearray(37);f[:4]=bytes.fromhex('01 42 25 18');f[5]=1;f[7]=2
                f[10]=self.microsteps%256;f[22]=5;f[25]=self.ack_mode;f[26]=int(self.scaled)
            f[-3:]=bytes.fromhex('00 08 6B');frame=bytes(f)
        elif cmd==z.Command.READ_OPTIONS:
            mode='x' if self.mode_mismatch else self.mode
            raw=(2 if mode=='emm' else 0)|(4 if self.closed_loop else 0)|(128 if self.scaled else 0)
            frame=bytes((1,cmd,0,raw,0x6B)) if self.extended_options else bytes((1,cmd,raw,0x6B))
        elif cmd==z.Command.READ_VERSION:
            frame=bytes((1,cmd))+self.firmware_raw.to_bytes(2,'big')+self.hardware_raw.to_bytes(2,'big')+b'\x6b'
        elif cmd==z.Command.READ_VOLTAGE:frame=bytes.fromhex('01 24 2E E0 6B')
        elif cmd in (z.Command.READ_POSITION,z.Command.READ_SPEED):
            raw=self.position if cmd==z.Command.READ_POSITION else self.speed
            n=4 if cmd==z.Command.READ_POSITION else 2
            frame=bytes((1,cmd,int(raw<0)))+abs(raw).to_bytes(n,'big')+b'\x6B'
        elif cmd==z.Command.READ_STATE:
            raw=int(self.enabled)|(2 if self.arrived else 0)|(4 if self.stalled else 0)
            frame=bytes((1,cmd,raw,0x6B))
        else:raise AssertionError('unexpected ZDT command')
        return z.parse_response(frame,1,cmd).require_success()
    def arrive(self):self.position=self.target;self.speed=0;self.arrived=True
    def close(self):self.closed=True
    def commands(self):return [r[1] for r in self.writes]


def evidence(now):
    return dict(mission=dict(stamp=now, mode='inspect',traffic_active=False,auto_fire_active=False,
                             emergency_stop=False,pump_armed=False),
                actuators=dict(stamp=now,dry_run=True,enabled=False,pump_on=False,
                               pump_enabled=False,estop=False,hardware_written=False),
                fpga_link=dict(stamp=now,dry_run=True))


class BenchTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.uart=FakeTransport();self.opens=0;self.n=0
        self.zdt=FakeZDTTransport()
        def factory(*args,**kwargs):
            self.opens+=1;self.uart.args=args;self.uart.kwargs=kwargs
            return self.uart
        def zdt_factory(*args,**kwargs):
            self.opens+=1;self.zdt.args=args;self.zdt.kwargs=kwargs
            return self.zdt
        self.core=PD42Bench(factory,self.clock,zdt_factory=zdt_factory)
    def request(self,action, **overrides):
        self.n+=1
        value=dict(action=action,request_id=str(self.n),issued_at=self.clock(),ttl_s=3.)
        value.update(overrides)
        self.core.handle(value,self.clock(),evidence(self.clock()))
        return value
    def connected(self,enabled=False):
        self.uart.enabled=enabled;self.request('connect')
        self.assertEqual(self.core.last_result,'completed')
    def ready(self):self.connected();self.request('enable')
    def tick(self):self.core.tick(self.clock(),evidence(self.clock()))
    def test_startup_connect_is_read_only_and_captures_stationary_reference(self):
        self.assertEqual(self.opens,0);self.assertEqual(self.uart.writes,[])
        self.connected(enabled=True)
        self.assertEqual(self.uart.args,('/dev/ttyS1',));self.assertEqual(self.uart.kwargs['baudrate'],115200)
        self.assertTrue(all(c.value<0xF0 for c in self.uart.commands()))
        s=self.core.snapshot(self.clock());self.assertTrue(s['enabled'])
        self.assertTrue(s['can_enable']);self.assertFalse(s['can_move'])
        self.assertEqual(s['position_deg'],0.);self.assertEqual(s['reference_counts'],47360)
        self.request('step_positive');self.assertEqual(self.core.reason,'motor_test_explicit_enable_required')
        self.request('enable');self.assertTrue(self.core.snapshot(self.clock())['can_move'])
    def test_connect_read_timeout_closes_without_any_write_command(self):
        self.uart.fail=Command.READ_POSITION;self.uart.enabled=True
        self.request('connect');self.assertTrue(self.uart.closed);self.assertIsNone(self.core.link)
        self.assertEqual(self.core.reason,'motor_test_connect_failed')
        self.assertTrue(all(c.value<0xF0 for c in self.uart.commands()))
    def test_connect_running_motor_does_not_capture_reference_or_disable_it(self):
        self.uart.enabled=True;self.uart.speed=10;self.uart.status=2
        self.request('connect');self.assertEqual(self.core.last_result,'rejected')
        self.assertIsNone(self.core.reference_counts);self.assertTrue(self.uart.closed)
        self.assertNotIn(Command.ENABLE,self.uart.commands())
    def test_move_is_absolute_bounded_and_ack_is_not_arrival(self):
        self.ready();self.request('step_positive')
        self.assertEqual(self.core.last_result,'accepted');self.assertEqual(self.uart.target,47360+degrees_to_counts(5))
        packet=self.uart.writes[-1];self.assertEqual(packet[2],Command.ABSOLUTE_POSITION)
        self.assertEqual(struct.unpack('>BBHI',packet[3:-2])[1:3],(10,5))
        self.tick();self.assertEqual(self.core.last_result,'accepted')
        self.uart.arrive();self.tick();self.assertEqual(self.core.reason,'motor_test_measured_arrival')
        self.assertEqual(self.core.last_result,'completed')
    def test_next_command_does_not_stack_while_moving(self):
        self.ready();self.request('step_positive');self.request('step_positive')
        self.assertEqual(self.core.reason,'motor_test_move_in_progress')
        self.assertEqual(self.uart.commands().count(Command.ABSOLUTE_POSITION),1)
    def test_range_caps_three_steps_and_return_uses_session_origin(self):
        self.ready()
        for _ in range(3):
            self.request('step_positive');self.assertEqual(self.core.last_result,'accepted')
            self.uart.arrive();self.tick()
        self.request('step_positive');self.assertEqual(self.core.reason,'motor_test_15_degree_limit')
        self.assertEqual(self.uart.commands().count(Command.ABSOLUTE_POSITION),3)
        self.request('return_origin');self.assertEqual(self.uart.target,47360)
        self.uart.arrive();self.tick();self.assertAlmostEqual(self.core.snapshot(self.clock())['position_deg'],0.)
    def test_negative_steps_cannot_exceed_negative_range(self):
        self.ready()
        for _ in range(3):
            self.request('step_negative');self.uart.arrive();self.tick()
        self.request('step_negative');self.assertEqual(self.core.reason,'motor_test_15_degree_limit')
    def test_duplicate_id_and_stale_input_never_repeat_motion(self):
        self.ready();request=self.request('step_positive');self.uart.arrive();self.tick()
        self.core.handle(request,self.clock(),evidence(self.clock()))
        self.assertEqual(self.core.reason,'duplicate_motor_test_request')
        self.request('step_negative',issued_at=90.)
        self.assertEqual(self.core.reason,'expired_motor_test_request')
        self.assertEqual(self.uart.commands().count(Command.ABSOLUTE_POSITION),1)
    def test_lost_motion_ack_sends_disable_once_not_another_move(self):
        self.ready();self.uart.fail=Command.ABSOLUTE_POSITION;self.request('step_positive')
        self.assertEqual(self.core.last_result,'error');self.assertTrue(self.core.fault)
        self.assertFalse(self.uart.enabled);self.assertEqual(self.uart.commands().count(Command.ABSOLUTE_POSITION),1)
        n=len(self.uart.writes);self.tick();self.assertEqual(len(self.uart.writes),n)
        self.request('enable');self.assertEqual(self.core.reason,'motor_test_fault_reconnect_required')
    def test_stalled_or_timeout_motion_is_disabled(self):
        for fault in ('timeout','stalled','undervoltage'):
            with self.subTest(fault=fault):
                self.setUp();self.ready();self.request('step_positive')
                if fault=='timeout':self.clock.advance(3.1)
                elif fault=='stalled':self.uart.stalled=True
                else:self.uart.status=5
                self.tick();self.assertFalse(self.uart.enabled);self.assertEqual(self.core.last_result,'error')
    def test_idle_enable_lease_expires_and_disables(self):
        self.ready();self.clock.advance(30.01);self.tick()
        self.assertFalse(self.uart.enabled);self.assertEqual(self.core.reason,'motor_test_enable_lease_expired')
        self.assertFalse(self.core.snapshot(self.clock())['can_move'])
    def test_loss_of_fresh_owner_or_mode_change_stops_and_closes(self):
        cases=[('mission','stamp',50.),('mission','mode','patrol'),('mission','emergency_stop',True),
               ('mission','traffic_active',True),('mission','auto_fire_active',True),
               ('actuators','dry_run',False),('actuators','enabled',True),('fpga_link','dry_run',False)]
        for source,key,value in cases:
            with self.subTest(source=source,key=key):
                self.setUp();self.ready();self.request('step_positive')
                data=evidence(self.clock());data[source][key]=value
                self.core.tick(self.clock(),data)
                self.assertFalse(self.uart.enabled);self.assertTrue(self.uart.closed)
                self.assertEqual(self.core.reason,'motor_test_interlock_lost')
    def test_stop_bypasses_mission_and_disconnect_loses_reference(self):
        self.ready();self.request('step_positive')
        self.core.handle(dict(action='stop',request_id='stop',issued_at=self.clock(),ttl_s=3.),self.clock(),{})
        self.assertFalse(self.uart.enabled);self.assertEqual(self.core.last_result,'completed')
        self.core.handle(dict(action='disconnect',request_id='close',issued_at=self.clock(),ttl_s=3.),self.clock(),{})
        self.assertTrue(self.uart.closed);self.assertIsNone(self.core.reference_counts)
    def test_stale_snapshot_never_looks_like_measured_angle(self):
        self.ready();self.clock.advance(.8);snapshot=self.core.snapshot(self.clock())
        self.assertFalse(snapshot['feedback_fresh']);self.assertIsNone(snapshot['position_deg'])
        self.assertIsNone(snapshot['enabled']);self.assertFalse(snapshot['can_move'])
    def test_close_disables_and_never_homes_or_brakes(self):
        self.ready();self.request('step_positive');self.core.close()
        self.assertFalse(self.uart.enabled);self.assertTrue(self.uart.closed)
        self.assertNotIn(Command.STOP,self.uart.commands());self.assertNotIn(Command.CLEAR_STATUS,self.uart.commands())
    def test_request_schema_and_guard_reject_smuggling_future_and_fake_state(self):
        base=dict(action='connect',request_id='a',issued_at=100.,ttl_s=3.)
        for extra in ({'port':'/dev/ttyS2'},{'degrees':360},{'issued_at':101.},{'ttl_s':True},{'action':'home'}):
            with self.subTest(extra=extra):
                with self.assertRaises(ValueError):validate_request(dict(base,**extra),100.)
        data=evidence(100.);self.assertEqual(bench_guard(data,100.),'')
        data['actuators']['synthetic']=True;self.assertEqual(bench_guard(data,100.),'actuators_state_stale')
    def test_live_interlock_rechecked_after_reads_before_motion_write(self):
        self.ready();self.core.guard_probe=lambda:'motor_test_mission_busy_or_estop'
        before=self.uart.commands().count(Command.ABSOLUTE_POSITION)
        self.request('step_positive')
        self.assertEqual(self.core.last_result,'rejected')
        self.assertEqual(self.core.reason,'motor_test_mission_busy_or_estop')
        self.assertEqual(self.uart.commands().count(Command.ABSOLUTE_POSITION),before)

    def test_motor_id_defaults_only_to_motor1_and_rejects_unknown_or_arbitrary_ports(self):
        base=dict(action='connect',request_id='a',issued_at=100.,ttl_s=3.)
        self.assertEqual(validate_request(base,100.)['motor_id'],'motor1')
        for motor in ('motor1','motor2'):
            self.assertEqual(validate_request(dict(base,motor_id=motor),100.)['motor_id'],motor)
        for motor in ('motor3','/dev/ttyS7',None,True,1,['motor1']):
            with self.subTest(motor=motor):
                with self.assertRaisesRegex(ValueError,'invalid_motor_test_motor_id'):
                    validate_request(dict(base,motor_id=motor),100.)

    def test_motor2_connect_opens_uart3_and_preserves_read_only_baseline(self):
        self.zdt.enabled=True
        self.request('connect',motor_id='motor2')
        self.assertEqual(self.core.last_result,'completed')
        self.assertEqual(self.zdt.args,('/dev/ttyS3',))
        snapshot=self.core.snapshot(self.clock())
        self.assertEqual((snapshot['motor_id'],snapshot['tx_pin'],snapshot['rx_pin']),('motor2',3,5))
        self.assertEqual(snapshot['last_motor_id'],'motor2')
        self.assertTrue(all(c<0xF0 for c in self.zdt.commands()))
        self.assertEqual(self.uart.writes,[])
        self.assertEqual(snapshot['protocol'],'zdt_xs');self.assertEqual(snapshot['firmware_mode'],'emm')
        self.assertFalse(snapshot['can_move'])
        self.request('enable',motor_id='motor2')
        self.request('step_negative',motor_id='motor2')
        self.assertEqual(self.core.last_result,'accepted')
        self.zdt.arrive();self.tick()
        snapshot=self.core.snapshot(self.clock())
        self.assertEqual(snapshot['last_result'],'completed')
        self.assertEqual(snapshot['last_motor_id'],'motor2')
        self.assertAlmostEqual(snapshot['position_deg'],-5.,delta=.1)

    def test_wrong_motor_action_never_writes_or_disables_the_connected_motor(self):
        self.ready();before=len(self.uart.writes)
        for action in ('connect','read','enable','step_positive','return_origin','stop','disconnect'):
            with self.subTest(action=action):
                self.request(action,motor_id='motor2')
                self.assertEqual(self.core.reason,'motor_test_different_motor_connected')
                self.assertEqual(len(self.uart.writes),before)
                self.assertEqual(self.opens,1);self.assertTrue(self.uart.enabled);self.assertFalse(self.uart.closed)
                snapshot=self.core.snapshot(self.clock())
                self.assertEqual(snapshot['motor_id'],'motor1')
                self.assertEqual(snapshot['last_motor_id'],'motor2')

    def test_motor_switch_requires_disconnect_and_records_new_reference_without_enable(self):
        self.ready();self.request('disconnect')
        self.assertTrue(self.uart.closed);self.assertIsNone(self.core.reference_counts)
        self.zdt.position=8123
        self.request('connect',motor_id='motor2')
        self.assertEqual(self.opens,2);self.assertEqual(self.zdt.args,('/dev/ttyS3',))
        self.assertEqual(self.core.reference_counts,8123)
        self.assertFalse(self.core.snapshot(self.clock())['can_move'])
        before=len(self.zdt.writes)
        self.request('enable')  # Legacy request means motor1, never the selected motor2.
        self.assertEqual(self.core.last_result,'rejected');self.assertEqual(len(self.zdt.writes),before)

    def test_move_completion_keeps_its_motor_identity_after_other_motor_rejection(self):
        self.ready();move=self.request('step_positive')
        self.request('stop',motor_id='motor2')
        self.assertEqual(self.core.last_motor_id,'motor2');self.assertTrue(self.uart.enabled)
        self.uart.arrive();self.tick()
        self.assertEqual(self.core.last_motor_id,'motor1')
        self.assertEqual(self.core.last_request_id,move['request_id'])
        self.assertEqual(self.core.reason,'motor_test_measured_arrival')

    def test_stale_other_motor_connect_cannot_change_selection_or_open_a_port(self):
        self.request('connect',motor_id='motor2',issued_at=90.)
        self.assertEqual(self.core.reason,'expired_motor_test_request')
        self.assertEqual(self.opens,0);self.assertEqual(self.core.motor_id,'motor1')


class ZDTBenchTests(unittest.TestCase):
    setUp=BenchTests.setUp
    request=BenchTests.request
    tick=BenchTests.tick

    def connect(self):
        self.request('connect',motor_id='motor2')
        self.assertEqual(self.core.last_result,'completed',self.core.error)
    def ready(self):
        self.connect();self.request('enable',motor_id='motor2')
        self.assertEqual(self.core.last_result,'completed',self.core.error)

    def test_emm_encoder_counts_are_converted_to_pulses_not_sent_as_counts(self):
        self.ready();self.request('step_positive',motor_id='motor2')
        # At 360 degrees, absolute 365 degrees is 3244 microstep pulses.
        self.assertEqual(self.zdt.writes[-1].hex(' '),'01 fd 00 00 05 0a 00 00 0c ac 01 00 6b')
        self.assertNotEqual(self.core.last_result,'completed')
        self.zdt.arrive();self.tick()
        self.assertEqual(self.core.last_result,'completed')
        self.assertAlmostEqual(self.core.snapshot(self.clock())['position_deg'],4.95,delta=.01)
    def test_x_mode_position_and_speed_use_tenths_not_emm_pulses(self):
        self.zdt.mode='x';self.zdt.position=3600;self.ready()
        self.request('step_positive',motor_id='motor2')
        self.assertEqual(self.zdt.writes[-1].hex(' '),'01 fb 00 00 32 00 00 0e 42 01 00 6b')
        self.tick();self.assertEqual(self.core.snapshot(self.clock())['speed_rpm'],5)
        self.zdt.arrive();self.tick();self.assertEqual(self.core.last_result,'completed')
    def test_scaled_x_command_changes_encoding_not_feedback_units(self):
        self.zdt.mode='x';self.zdt.scaled=True;self.zdt.position=3600;self.ready()
        self.request('step_positive',motor_id='motor2')
        self.assertEqual(self.zdt.writes[-1].hex(' '),'01 fb 00 00 32 00 00 8e 94 01 00 6b')
        self.zdt.arrive();self.tick();self.assertEqual(self.core.snapshot(self.clock())['position_deg'],5.)
    def test_emm_scaled_speed_is_still_five_rpm(self):
        self.zdt.scaled=True;self.ready();self.request('step_positive',motor_id='motor2')
        self.assertEqual(self.zdt.writes[-1][3:5],bytes.fromhex('00 32'))
    def test_missing_ack_mode_and_open_loop_remain_read_only(self):
        for field,value,reason in [('ack_mode',2,'zdt_command_ack_required'),('closed_loop',False,'zdt_closed_loop_required')]:
            with self.subTest(field=field):
                self.setUp();setattr(self.zdt,field,value);self.connect()
                self.assertFalse(self.core.snapshot(self.clock())['can_enable'])
                self.request('enable',motor_id='motor2');self.assertEqual(self.core.reason,reason)
                self.assertTrue(all(c<0xF0 for c in self.zdt.commands()))
    def test_mode_conflict_and_malformed_connect_never_send_control_commands(self):
        for attr in ('mode_mismatch','malformed'):
            with self.subTest(attr=attr):
                self.setUp();setattr(self.zdt,attr,True);self.request('connect',motor_id='motor2')
                self.assertEqual(self.core.last_result,'error');self.assertTrue(self.zdt.closed)
                self.assertTrue(all(c<0xF0 for c in self.zdt.commands()))
    def test_undocumented_extended_options_preserve_readback_without_motion_permission(self):
        self.zdt.extended_options=True;self.connect()
        s=self.core.snapshot(self.clock())
        self.assertTrue(s['connected']);self.assertTrue(s['feedback_fresh'])
        self.assertEqual(s['position_deg'],0.)
        self.assertFalse(s['motion_configuration_valid']);self.assertFalse(s['can_enable'])
        self.assertEqual(s['motion_configuration_reason'],'zdt_options_layout_unverified')
        self.request('enable',motor_id='motor2')
        self.assertTrue(all(c<0xF0 for c in self.zdt.commands()))
    def test_observed_extended_profile_connect_is_read_only_and_requires_explicit_enable(self):
        self.zdt.extended_options=True;self.zdt.hardware_raw=0x2314;self.connect()
        s=self.core.snapshot(self.clock())
        self.assertTrue(s['motion_configuration_valid']);self.assertTrue(s['can_enable'])
        self.assertFalse(s['can_move']);self.assertFalse(self.zdt.enabled)
        self.assertTrue(all(c<0xF0 for c in self.zdt.commands()))
        self.assertEqual(s['driver_options']['layout_profile'],'zdt_xs_fw206_hw2314_emm_u16be')
        self.request('step_positive',motor_id='motor2')
        self.assertEqual(self.core.reason,'motor_test_explicit_enable_required')
        self.assertTrue(all(c<0xF0 for c in self.zdt.commands()))
    def test_observed_extended_profile_preserves_actual_pulse_conversion(self):
        self.zdt.extended_options=True;self.zdt.hardware_raw=0x2314;self.ready()
        self.request('step_positive',motor_id='motor2')
        self.assertEqual(self.zdt.writes[-1].hex(' '),'01 fd 00 00 05 0a 00 00 0c ac 01 00 6b')
        self.zdt.arrive();self.tick();self.assertEqual(self.core.last_result,'completed')
    def test_device_version_change_after_connection_blocks_enable_without_control_write(self):
        self.zdt.extended_options=True;self.zdt.hardware_raw=0x2314;self.connect()
        self.zdt.firmware_raw=207
        self.request('enable',motor_id='motor2')
        self.assertEqual(self.core.reason,'zdt_configuration_changed_reconnect')
        self.assertFalse(self.core.snapshot(self.clock())['can_enable'])
        self.assertTrue(all(c<0xF0 for c in self.zdt.commands()))
    def test_extended_scale_change_before_movement_cannot_reinterpret_units(self):
        self.zdt.extended_options=True;self.zdt.hardware_raw=0x2314;self.ready()
        self.zdt.scaled=True
        self.request('step_positive',motor_id='motor2')
        self.assertEqual(self.core.reason,'zdt_configuration_changed_reconnect')
        self.assertNotIn(0xFD,self.zdt.commands())
    def test_menu_change_requires_reconnect_without_reinterpreting_origin(self):
        self.ready();self.zdt.microsteps=32
        self.request('step_positive',motor_id='motor2')
        self.assertEqual(self.core.reason,'zdt_configuration_changed_reconnect')
        self.assertFalse(self.core.snapshot(self.clock())['can_move'])
        self.assertNotIn(0xFD,self.zdt.commands())
    def test_arrived_flag_alone_and_position_alone_do_not_confirm_motion(self):
        self.ready();self.request('step_positive',motor_id='motor2');self.zdt.speed=0
        self.zdt.arrived=True;self.tick();self.assertEqual(self.core.last_result,'accepted')
        self.zdt.position=self.zdt.target;self.zdt.arrived=False;self.tick()
        self.assertEqual(self.core.last_result,'accepted')
        self.zdt.arrived=True;self.tick();self.assertEqual(self.core.last_result,'completed')
    def test_zdt_stop_is_explicit_disable_and_verified_from_status(self):
        self.ready();self.request('step_positive',motor_id='motor2');self.request('stop',motor_id='motor2')
        self.assertIn(bytes.fromhex('01 f3 ab 00 00 6b'),self.zdt.writes)
        self.assertFalse(self.zdt.enabled);self.assertEqual(self.core.reason,'motor_test_disabled_no_holding_torque')
    def test_lost_zdt_position_ack_disables_once_and_does_not_retry_motion(self):
        from rescue_vision.zdt_xs_protocol import Command as Z
        self.ready();self.zdt.fail=Z.POSITION_EMM;self.request('step_positive',motor_id='motor2')
        self.assertEqual(self.core.last_result,'error');self.assertFalse(self.zdt.enabled)
        self.assertEqual(self.zdt.commands().count(0xFD),1)
        count=len(self.zdt.writes);self.tick();self.assertEqual(len(self.zdt.writes),count)
    def test_large_global_coordinate_does_not_expand_session_limit(self):
        self.zdt.position=20*65536;self.ready()
        for _ in range(3):
            self.request('step_positive',motor_id='motor2');self.zdt.arrive();self.tick()
        self.request('step_positive',motor_id='motor2')
        self.assertEqual(self.core.reason,'motor_test_15_degree_limit')
        self.assertEqual(self.zdt.commands().count(0xFD),3)


# Dual-worker ROS queue tests are in test_pd42_dual_workers.py.

if __name__=='__main__':unittest.main()
