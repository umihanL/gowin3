"""Audit ten-channel pose telemetry; run verify_navigation_build.py first."""
from pathlib import Path
import hashlib
import json
import re

root = Path(__file__).resolve().parents[1]
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
baseline = json.loads((root/'impl/imu_pose_telemetry_baseline.json').read_text())
build = json.loads((root/'impl/verified_build.json').read_text())
assert digest(root/build['bitstream']) == build['sha256']
assert build['setup_violated_endpoints'] == build['hold_violated_endpoints'] == 0
for rel, expected in build['input_sha256'].items():
    assert digest(root/rel) == expected, 'Changed build input: '+rel
old_sources = {rel for rel in baseline['sha256'] if rel.startswith('src/')}
changed = sorted(rel for rel in old_sources if digest(root/rel) != baseline['sha256'][rel])
added = sorted(p.relative_to(root).as_posix() for p in (root/'src').rglob('*')
               if p.is_file() and p.relative_to(root).as_posix() not in old_sources)
assert changed == ['src/imu/imu_firewater_tx.v', 'src/ov5640_ddr3_tft.v'], changed
assert not added, added
for rel in ('build.tcl', 'ov5640.gprj'):
    assert digest(root/rel) == baseline['sha256'][rel]
cst = (root/'src/ov5640_ddr3_tft.cst').read_text()
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', cst, re.M))
report = (root/'impl/pnr/ov5640.rpt.txt').read_text()
actual = dict(re.findall(r'^([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
assert all(actual.get(k) == v for k,v in pins.items()), 'Routed pin mismatch'
netlist = (root/'impl/gwsynthesis/ov5640.vg').read_text()
instances = {name: re.search(r'\b\w+\s+'+name+r'\s*\(.*?\);', netlist, re.S)[0]
             for name in ('u_imu_uart', 'u_attitude_overlay', 'u_vehicle')}
for axis in 'xyz':
    assert 'imu_rot_'+axis in instances['u_imu_uart']
    assert 'imu_rot_'+axis in instances['u_attitude_overlay']
for axis in ('roll', 'pitch', 'yaw'):
    assert '.imu_'+axis+'(imu_'+axis+'[15:0])' in instances['u_imu_uart']
for axis in 'wxyz':
    assert '.imu_relative_'+axis+'(imu_relative_'+axis+'[17:0])' in instances['u_imu_uart']
assert '.imu_yaw(imu_yaw[15:0])' in instances['u_vehicle']
log = (root/'sim/imu_test_results.txt').read_text()
assert log.count('PASS ') == 15 and log.count('PASS imu_pose_firewater:') == 2
assert not re.search(r'^(?:FAIL|FATAL|ERROR)\b', log, re.I|re.M)
host_log = (root/'sim/imu_host_test_results.txt').read_text()
assert 'PASS browser pose: 1444' in host_log and 'real 1.8-degree offset retained' in host_log
host_meta = json.loads(host_log.splitlines()[-1])
normalized_html = (root/'docs/imu_pose_viewer.html').read_text(encoding='utf-8').encode('utf-8')
assert host_meta['viewer_sha256'] == hashlib.sha256(normalized_html).hexdigest()
audit_path = 'impl/pnr/ov5640_imu_pose_audit.json'
audit = {k: build[k] for k in ('project', 'bitstream', 'sha256', 'generated_time_utc8',
         'bitstream_bytes', 'resources', 'setup_violated_endpoints', 'hold_violated_endpoints', 'hardware_tested')}
audit.update(feature='Ten-channel RotXYZ, Euler RPY and quaternion WXYZ FireWater; quaternion browser viewer',
    previous_production_sha256=baseline['sha256']['impl/pnr/ov5640.fs'],
    existing_source_changes=changed, added_sources=added,
    unchanged_existing_source_files=len(old_sources)-len(changed),
    cst_sha256=digest(root/'src/ov5640_ddr3_tft.cst'),
    sdc_sha256=digest(root/'src/ov5640_ddr3_tft.sdc'),
    pin_constraints_unchanged=True, pins_verified_from_pnr_report=pins,
    output_channels=['RotX','RotY','RotZ','Roll','Pitch','Yaw','QW','QX','QY','QZ'],
    output_decimals=[1,1,1,1,1,1,4,4,4,4],
    uart=dict(pin='M15', baud=115200, data_bits=8, stop_bits=1, parity='none', max_frames_per_second=20),
    previous_three_channel_values_preserved=True,
    navigation_heading='original relative ZYX Euler yaw',
    spi_fusion_calibration_and_screen_unchanged=True,
    physical_drift_reduction_implemented_or_validated=False,
    error_note='Rotation-vector/Euler viewer mismatch corrected. No stationary auto-zero, bias correction, or fabricated drift suppression.',
    regression_scenarios=15, rotation_vector_cases=3264, browser_rotation_cases=1444,
    host_input='Simulated browser serial input, not physical measurements',
    verification_sha256={rel:digest(root/rel) for rel in (
        'sim/imu_firewater_tb.v', 'sim/run_imu_tests.py', 'sim/imu_test_results.txt',
        'sim/run_imu_host_tests.py', 'sim/imu_host_test_results.txt',
        'docs/imu_pose_viewer.html', 'tools/analyze_imu_pose.py')})
(root/audit_path).write_text(json.dumps(audit,indent=2)+'\n')
build.update(current_change='Ten-channel pose telemetry and quaternion viewer', current_change_audit=audit_path)
(root/'impl/verified_build.json').write_text(json.dumps(build,indent=2)+'\n')
p = root/'impl/pnr/ov5640_download.json'
download = json.loads(p.read_text())
download.update({k:build[k] for k in ('sha256', 'generated_time_utc8', 'bitstream_bytes',
                'setup_violated_endpoints', 'hold_violated_endpoints')})
download.update(current_image='Full production project; ten-channel RotXYZ/Euler/WXYZ FireWater',
                build_audit=audit_path, pin_constraints_unchanged=True, hardware_tested=False)
p.write_text(json.dumps(download,indent=2)+'\n')
print(json.dumps(dict(audit=audit_path,sha256=build['sha256'],source_changes=changed,
                     pins_checked=len(pins),resources=build['resources']),indent=2))
