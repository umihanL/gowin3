#!/usr/bin/env python3
"""Explicit unloaded M2 Emm test; default only prints an offline plan.

Fixed ttyS3/address 1/FW206/HW2314, Emm 16 microsteps and 1.8-degree motor.
Keep the live worker disconnected and confirm this UART has no other owner.
Never enable, home, clear faults, write configuration or retry a command.
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision import zdt_xs_protocol as wire
from rescue_vision.zdt_xs_transport import ZDTSerialTransport

PORT,ADDRESS,BAUDRATE='/dev/ttyS3',1,115200
PULSE_DEG=1.8/16
TARGET_BOUND_DEG,EXCURSION_DEG,ERROR_DEG=1.02,1.7,.2
MOVING_ERROR_LIMIT_DEG=2.5
TRANSACTION_TIMEOUT_S,SEGMENT_TIMEOUT_S=.2,4.


class AcceptanceFault(RuntimeError):pass


def relative_plan(origin_counts,current_counts,desired_offset_deg):
    """Quantize measured displacement to microsteps, never encoder counts."""
    if any(type(v) is not int or abs(v)>0xffffffff for v in (origin_counts,current_counts)):
        raise AcceptanceFault('invalid_encoder_count')
    if desired_offset_deg not in (-1.,0.,1.):
        raise AcceptanceFault('unsupported_target_offset')
    current_offset=(current_counts-origin_counts)*360./65536
    if abs(current_offset)>EXCURSION_DEG:
        raise AcceptanceFault('excursion_exceeds_1_7_degrees')
    delta=desired_offset_deg-current_offset
    pulses=int(math.floor(abs(delta)/PULSE_DEG+.5))*(1 if delta>=0 else -1)
    expected=current_offset+pulses*PULSE_DEG
    # Rounding against a slightly offset measured start must not expand travel.
    if abs(expected)>TARGET_BOUND_DEG:
        pulses-=1 if expected>0 else -1
        expected=current_offset+pulses*PULSE_DEG
    if abs(expected)>TARGET_BOUND_DEG or abs(expected-desired_offset_deg)>.12:
        raise AcceptanceFault('quantized_target_outside_small_motion_bound')
    return dict(pulses=pulses,delta_degrees=pulses*PULSE_DEG,
                expected_offset_deg=expected,desired_offset_deg=desired_offset_deg,
                start_counts=current_counts)


def plan():
    return dict(port=PORT,address=ADDRESS,baudrate=BAUDRATE,
        expected_device=dict(firmware_raw=206,hardware_raw=0x2314,mode='emm',microsteps=16,motor_step_deg=1.8),
        desired_offsets_from_measured_start_deg=[1.,-1.,0.],absolute_targets=None,
        pulse_degrees=PULSE_DEG,motion_mode=2,speed_rpm=1,acceleration_selector=1,
        acceleration_note='Lowest nonzero Emm selector; 12.75 ms per 1 RPM increment, not 1 RPM/s.',
        target_bound_from_start_deg=TARGET_BOUND_DEG,maximum_measured_excursion_deg=EXCURSION_DEG,
        segment_timeout_s=SEGMENT_TIMEOUT_S,consecutive_stable_samples=3,error_limit_deg=ERROR_DEG,
        moving_following_error_hard_limit_deg=MOVING_ERROR_LIMIT_DEG,
        final_target_reads=['0x33 final input target','0x34 realtime set target'],
        unknown_option_bits='Reject; normalize_options and verified_motion_parameters must both pass.',
        normal_exit='Keep enabled; close UART. Report measured residual, never claim exact zero.',
        fault_exit='If any motion may have been sent, attempt one unloaded disable. No retry or return motion.')


class SmallMotionCheck:
    def __init__(self,transport,report,clock=time.monotonic,sleep=time.sleep):
        self.io,self.report,self.clock,self.sleep=transport,report,clock,sleep
        self.origin=None;self.started=clock();self.parameters=None;self.options_raw=None;self.config_raw=None
        report.update(transactions=[],samples=[],segments=[],motion_commands_sent=0,
                      enable_commands_sent=0,disable_commands_sent=0)

    def exchange(self,request,deadline=None,purpose='feedback'):
        now=self.clock()
        if deadline is not None and now>=deadline:raise AcceptanceFault('segment_deadline_expired')
        self.io.timeout_s=min(TRANSACTION_TIMEOUT_S,deadline-now) if deadline is not None else TRANSACTION_TIMEOUT_S
        record=dict(t_s=now-self.started,purpose=purpose,request_hex=request.hex(' '),command='0x%02X'%request[1])
        self.report['transactions'].append(record)
        if request[1]==wire.Command.POSITION_EMM:self.report['motion_commands_sent']+=1
        if request[1]==wire.Command.ENABLE:self.report['disable_commands_sent' if request[3]==0 else 'enable_commands_sent']+=1
        try:
            response=self.io.exchange(request)
            record.update(response_success=response.ok,raw_data_hex=response.data.hex(' '),decoded=response.value,
                          finished_t_s=self.clock()-self.started)
            response.require_success()
            if deadline is not None and self.clock()>deadline:raise AcceptanceFault('segment_deadline_expired')
            return response
        except (Exception,KeyboardInterrupt) as exc:
            record.update(error=type(exc).__name__+': '+str(exc),finished_t_s=self.clock()-self.started)
            raise

    def read(self,command,deadline=None):return self.exchange(wire.read_request(ADDRESS,command),deadline)

    def verify_configuration(self):
        version=self.read(wire.Command.READ_VERSION).value
        config_response=self.read(wire.Command.READ_CONFIG);config=config_response.value
        options_response=self.read(wire.Command.READ_OPTIONS)
        normalized=wire.normalize_options(version,config,options_response.value)
        self.report.update(version=version,configuration=config,normalized_options=normalized)
        if (version.get('firmware_raw'),version.get('hardware_raw'))!=(206,0x2314):
            raise AcceptanceFault('unexpected_firmware_or_hardware')
        parameters=wire.verified_motion_parameters(config,normalized)
        if normalized.get('layout_verified') is not True or type(normalized.get('raw')) is not int or normalized['raw']&~0x3b7:
            raise AcceptanceFault('unknown_or_unverified_option_bits')
        if (parameters['mode'],parameters['microsteps'],parameters['motor_step_deg'])!=('emm',16,1.8):
            raise AcceptanceFault('unexpected_motor_mode_or_microstep_configuration')
        if config.get('address')!=ADDRESS or config.get('baudrate')!=BAUDRATE:
            raise AcceptanceFault('unexpected_address_or_baudrate')
        if self.config_raw is not None and (config_response.data!=self.config_raw or options_response.data!=self.options_raw):
            raise AcceptanceFault('configuration_changed_during_acceptance')
        self.parameters=parameters;self.config_raw=config_response.data;self.options_raw=options_response.data

    def sample(self,deadline,stage):
        record=dict(stage=stage,t_s=self.clock()-self.started);self.report['samples'].append(record)
        counts=self.read(wire.Command.READ_POSITION,deadline).value
        position=wire.position_degrees(counts,'emm')
        record.update(position_counts=counts,position_deg=position)
        if self.origin is not None:
            record['offset_from_start_deg']=(counts-self.origin)*360./65536
            if abs(record['offset_from_start_deg'])>EXCURSION_DEG:raise AcceptanceFault('excursion_exceeds_1_7_degrees')
        record['speed_rpm']=wire.speed_rpm(self.read(wire.Command.READ_SPEED,deadline).value,'emm')
        state=self.read(wire.Command.READ_STATE,deadline).value;record['state']=state
        if state.get('enabled') is not True:raise AcceptanceFault('motor_not_enabled_no_auto_enable')
        if any(state.get(k) for k in ('stalled','stall_protection','left_limit','right_limit')):
            raise AcceptanceFault('driver_fault_or_limit')
        # READ_STATE bit6 is undocumented; bit7 is the documented power marker.
        if state.get('raw',0)&0x40:raise AcceptanceFault('unknown_driver_state_bit')
        record['position_error_deg']=wire.error_degrees(self.read(wire.Command.READ_POSITION_ERROR,deadline).value,'emm')
        if abs(record['position_error_deg'])>MOVING_ERROR_LIMIT_DEG:
            raise AcceptanceFault('position_following_error_exceeds_2_5_degree')
        for key,command in (('final_input_target',wire.Command.READ_TARGET_POSITION),
                            ('realtime_target',wire.Command.READ_REALTIME_TARGET_POSITION)):
            target=self.read(command,deadline).value
            record[key+'_counts']=target;record[key+'_deg']=wire.position_degrees(target,'emm')
        return record

    def stable_at(self,sample,offset):
        target=self.origin*360./65536+offset
        return (sample['speed_rpm']==0 and abs(sample['position_error_deg'])<=ERROR_DEG and
                abs(sample['position_deg']-target)<=ERROR_DEG and
                all(abs(sample[key+'_deg']-target)<=ERROR_DEG for key in ('final_input_target','realtime_target')))

    def run(self):
        self.verify_configuration()
        deadline=self.clock()+SEGMENT_TIMEOUT_S
        first=self.sample(deadline,'initial');self.origin=first['position_counts']
        self.report.update(initial_position_counts=self.origin,initial_position_deg=first['position_deg'])
        if not self.stable_at(first,0.):raise AcceptanceFault('initial_motion_or_unmatched_retained_target')
        for _ in range(2):
            self.sleep(.04)
            if not self.stable_at(self.sample(deadline,'initial'),0.):raise AcceptanceFault('initial_feedback_not_stable')
        for index,offset in enumerate((1.,-1.,0.),1):
            # Independent fresh configuration checks before every movement.
            self.verify_configuration()
            deadline=self.clock()+SEGMENT_TIMEOUT_S
            before=self.sample(deadline,'before_segment_%d'%index)
            previous_offset=0. if index==1 else self.report['segments'][-1]['expected_offset_deg']
            if not self.stable_at(before,previous_offset):raise AcceptanceFault('motor_or_retained_target_changed_between_segments')
            segment=relative_plan(self.origin,before['position_counts'],offset)
            segment.update(index=index,completed=False,stable_samples=0,started_t_s=self.clock()-self.started)
            self.report['segments'].append(segment)
            started=self.clock();deadline=started+SEGMENT_TIMEOUT_S
            if segment['pulses']:
                request=wire.position_request(ADDRESS,segment['delta_degrees'],1,
                    motion_mode=2,acceleration=1,**self.parameters)
                self.exchange(request,deadline,purpose='segment_%d_measured_relative_motion'%index)
            else:segment['no_motion_needed']=True
            while True:
                sample=self.sample(deadline,'segment_%d'%index)
                stable=(self.stable_at(sample,segment['expected_offset_deg']) and
                        (index!=3 or abs(sample['offset_from_start_deg'])<=ERROR_DEG))
                segment['stable_samples']=segment['stable_samples']+1 if stable else 0
                if segment['stable_samples']>=3:
                    segment.update(completed=True,elapsed_s=self.clock()-started,
                        final_position_counts=sample['position_counts'],measured_offset_deg=sample['offset_from_start_deg'],
                        final_input_target_counts=sample['final_input_target_counts'],realtime_target_counts=sample['realtime_target_counts'])
                    break
                remaining=deadline-self.clock()
                if remaining<=0:raise AcceptanceFault('segment_deadline_expired')
                self.sleep(min(.04,remaining))
        self.report.update(completed=True,final_enabled_verified=True,
            final_position_counts=sample['position_counts'],measured_return_error_deg=sample['offset_from_start_deg'],
            retained_targets_consistent=True)

    def fault_disable(self):
        cleanup=self.report['fault_cleanup']=dict(disable_ack=False,disabled_readback=False)
        try:
            self.exchange(wire.enable_request(ADDRESS,False),purpose='fault_unloaded_disable')
            cleanup['disable_ack']=True
        except (Exception,KeyboardInterrupt) as exc:cleanup['disable_error']=type(exc).__name__+': '+str(exc)
        try:
            state=self.read(wire.Command.READ_STATE).value
            cleanup.update(actual_state=state,disabled_readback=state.get('enabled') is False)
        except (Exception,KeyboardInterrupt) as exc:cleanup['readback_error']=type(exc).__name__+': '+str(exc)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--unloaded',action='store_true')
    parser.add_argument('--clearance-deg',type=float,help='At least 2 degrees of free travel on each side')
    parser.add_argument('--report',type=Path)
    args=parser.parse_args(argv)
    if args.execute and (not args.unloaded or args.clearance_deg is None or not math.isfinite(args.clearance_deg) or args.clearance_deg<2.):
        parser.error('Execution requires --execute --unloaded --clearance-deg 2 or greater on BOTH sides')
    now=datetime.datetime.now(datetime.timezone.utc)
    path=args.report or ROOT/'reports/zdt_small_motion'/(now.strftime('%Y%m%dT%H%M%S_%fZ')+'_'+str(os.getpid())+'.json')
    report=dict(created_utc=now.isoformat(),hardware_access_requested=args.execute,plan=plan(),
        completed=False,error=None,configuration_modified=False,homing_or_zero_reset=False,
        clear_status_sent=False,report_path=str(path))
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    connection,checker,old_term=None,None,None
    try:
        if args.execute:
            def terminate(_signum,_frame):raise KeyboardInterrupt('termination_requested')
            old_term=signal.signal(signal.SIGTERM,terminate)
            connection=ZDTSerialTransport(PORT,baudrate=BAUDRATE,timeout_s=TRANSACTION_TIMEOUT_S)
            checker=SmallMotionCheck(connection,report);checker.run()
        else:report['note']='Offline plan only; no UART opened, no actual positions or absolute targets invented.'
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=type(exc).__name__+': '+str(exc)
        if checker is not None and report.get('motion_commands_sent',0)>0:checker.fault_disable()
        else:report['fault_cleanup']=dict(skipped=True,reason='no_motion_issued_by_this_check')
    finally:
        if connection is not None:
            try:connection.close();report['uart_closed']=True
            except Exception as exc:report['uart_close_error']=str(exc)
        if old_term is not None:signal.signal(signal.SIGTERM,old_term)
        encoded=json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)
        path.write_text(encoded+'\n',encoding='utf-8');print(encoded)
    return 1 if report['error'] else 0


if __name__=='__main__':raise SystemExit(main())
