"""Capture the operator-placed zero without moving either motor; pump stays disabled."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import uuid

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE/'src/rescue_vision'))
from rescue_vision.local_actuator_io import _ExclusiveOwner
from rescue_vision.pd42_transport import PD42SerialTransport
from rescue_vision.zdt_xs_transport import ZDTSerialTransport
from rescue_vision import pd42_protocol as pd42, zdt_xs_protocol as zdt
from rescue_vision import zdt_vendor_bench as vendor
from rescue_vision.mixed_actuator_io import validate


def read_baseline():
    owner = _ExclusiveOwner()
    tilt = pan = None
    try:
        tilt = PD42SerialTransport('/dev/ttyS1',115200,.2)
        pan = ZDTSerialTransport('/dev/ttyS3',115200,.2)
        pread = lambda c: pan.exchange(zdt.read_request(1,c)).require_success().value
        tread = lambda c: tilt.exchange(pd42.read_request(1,c)).require_success().value
        version = pread(zdt.Command.READ_VERSION)
        cfg = pan.exchange(zdt.read_request(1,zdt.Command.READ_CONFIG)).require_success()
        options = pread(zdt.Command.READ_OPTIONS)
        if not vendor.matches(version,cfg.data,options):
            raise ValueError('Current ZDT differs from verified fixed-FD8 profile')
        rows=[]
        for _ in range(3):
            t=tread(pd42.Command.READ_SYSTEM)
            ts=tread(pd42.Command.READ_STATUS)
            p=dict(position_before_counts=pread(zdt.Command.READ_POSITION),
                   retained_target_counts=pread(zdt.Command.READ_TARGET_POSITION),
                   realtime_target_counts=pread(zdt.Command.READ_REALTIME_TARGET_POSITION),
                   position_error_deg=zdt.error_degrees(pread(zdt.Command.READ_POSITION_ERROR),'emm'),
                   speed_rpm=zdt.speed_rpm(pread(zdt.Command.READ_SPEED),'emm'))
            state=pread(zdt.Command.READ_STATE)
            p['position_counts']=pread(zdt.Command.READ_POSITION)
            if (t['speed_rpm'] != 0 or t['stalled'] or ts not in (0,1) or
                    abs(t['target_counts']-t['position_counts'])*360/51200 > .1):
                raise ValueError('PD42 must be stationary at its actual retained target')
            # The FD8 profile owns this tolerance; keep capture and runtime
            # consistent without changing the independent PD42 checks above.
            if state['raw'] & ~3 or not vendor.settled(p,p['retained_target_counts']):
                raise ValueError('ZDT must be stationary with matching position and both targets')
            rows.append(dict(tilt=t,pan=p))
            time.sleep(.05)
        for name, factor in [('tilt',360/51200),('pan',360/65536)]:
            counts=[row[name]['position_counts'] for row in rows]
            if (max(counts)-min(counts))*factor>.03:
                raise ValueError(name+' moved during reference capture')
        return dict(version=version, config_data_hex=cfg.data.hex(' '), options_raw_hex=options['raw_bytes_hex'],
                    fd8_stationary_tolerance_deg=vendor.STATIONARY_TOLERANCE_DEG,
                    measurements=rows, captured_at=time.time())
    finally:
        if pan:pan.close()
        if tilt:tilt.close()
        owner.close()


def apply_tilt_direction(servo, commissioning, positive_direction=None):
    """Record an operator-observed sign; a captured count alone proves no sign."""
    if positive_direction not in (None, 'up', 'down'):
        raise ValueError('tilt-positive-direction must be up or down')
    confirmed = positive_direction is not None
    servo['tilt'].update(direction=-1 if positive_direction == 'down' else 1,
                         motion_enabled=confirmed)
    commissioning['tilt_physical_direction_verified'] = confirmed
    commissioning['tilt_direction_confirmation'] = {
        'source': 'operator_observed_positive_motor_motion' if confirmed else 'not_confirmed',
        'positive_motor_direction': positive_direction,
        'argument': '--tilt-positive-direction ' + positive_direction if confirmed else None,
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-zero-confirmed',action='store_true')
    parser.add_argument('--supported-load',action='store_true')
    parser.add_argument('--tilt-positive-direction',choices=('up','down'),
                        help='Observed nozzle direction for positive PD42 motion; omit to keep tilt locked')
    parser.add_argument('--apply',action='store_true')
    args=parser.parse_args()
    if not (args.operator_zero_confirmed and args.supported_load):
        parser.error('Confirm nozzle at the agreed horizontal centre / level zero and supported tilt load first')
    baseline=read_baseline()
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    ref_id='manual-zero-'+uuid.uuid4().hex
    cfg=json.loads((WORKSPACE/'config/actuator_io.mixed.template.json').read_text())
    cfg['dry_run']=False
    servo=cfg['servo']
    servo.update(reference_id=ref_id, reference_confirmed_this_start=True,
                 calibration_confirmed=True, load_supported_verified=True, enable_on_command=True,
                 position_tolerance_deg=.5, timeout_s=.15, feedback_timeout_s=.6, poll_hz=8)
    # The geometric sign is never inferred from a serial count. Only an
    # explicit observation option below can open tilt motion.
    servo['pan'].update(reference_counts=baseline['measurements'][-1]['pan']['position_counts'],
                         reference_angle_deg=0., min_angle_deg=-90., max_angle_deg=90.,
                         direction=1,gear_ratio=1.,motion_enabled=True,zdt_verified_snapshot=None)
    servo['tilt'].update(reference_counts=baseline['measurements'][-1]['tilt']['position_counts'],
                          reference_angle_deg=0., min_angle_deg=0.,max_angle_deg=60.,
                          direction=1,gear_ratio=1.,motion_enabled=False)
    last=baseline['measurements'][-1]['pan']
    servo['pan']['zdt_fd8_snapshot']=dict(profile='zdt_fd8_closed_loop_v1',
        version=baseline['version'],config_data_hex=baseline['config_data_hex'],options_raw_hex=baseline['options_raw_hex'],
        reference_position_counts=last['position_counts'],retained_target_counts=last['retained_target_counts'],
        realtime_target_counts=last['realtime_target_counts'])
    cfg['commissioning']={'reference':'operator placed horizontal-centre and level nozzle',
        'pan_range_deg':[-90,90],'tilt_range_deg':[0,60],'drive_ratio':1,
        'tilt_physical_direction_verified':False,'camera_to_nozzle_calibrated':False,
        'water_landing_calibrated':False,
        'boot_id':boot_id,'evidence':baseline}
    apply_tilt_direction(servo,cfg['commissioning'],args.tilt_positive_direction)
    validate(cfg)
    control=json.loads((WORKSPACE/'config/local_fire_control.template.json').read_text())
    control.update(enabled=True,pump_enabled=False,max_speed_deg_s=6.)
    profile=dict(schema_version=1,boot_id=boot_id,operator_zero_confirmed=True,reference_id=ref_id)
    if args.apply:
        for key,name,doc in [('io_config','actuator_io.mixed.local.json',cfg),
                             ('control_config','local_fire_control.local.json',control)]:
            target=WORKSPACE/'config'/name
            if target.exists():
                target.with_name(target.name+'.before-'+ref_id).write_bytes(target.read_bytes())
            raw=(json.dumps(doc,ensure_ascii=False,indent=2)+'\n').encode('utf-8')
            target.write_bytes(raw)
            profile[key]={'path':str(target),'sha256':hashlib.sha256(raw).hexdigest()}
        output=Path('/run/rescue/gimbal-profile.json');output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(profile,indent=2),encoding='utf-8')
    print(json.dumps(dict(applied=args.apply,reference_id=ref_id,boot_id=boot_id,
                         pan_counts=last['position_counts'],tilt_counts=servo['tilt']['reference_counts'],
                         motion_axes=[axis for axis in ('pan','tilt') if servo[axis]['motion_enabled']],
                         tilt_positive_direction=args.tilt_positive_direction,
                         pump_enabled=False,hardware_motion_written=False)))


if __name__=='__main__':main()
