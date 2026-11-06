"""Verify the quaternion-vector build after verify_navigation_build.py."""
from pathlib import Path
import hashlib
import json
import re

root=Path(__file__).resolve().parents[1]
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
baseline=json.loads((root/'impl/imu_continuity_baseline.json').read_text())
build=json.loads((root/'impl/verified_build.json').read_text())
assert digest(root/build['bitstream'])==build['sha256']
assert build['setup_violated_endpoints']==build['hold_violated_endpoints']==0
for rel,h in build['input_sha256'].items():
    assert digest(root/rel)==h,f'Build input changed: {rel}'
old_sources={rel for rel in baseline['sha256'] if rel.startswith('src/')}
changed=sorted(rel for rel in old_sources if digest(root/rel)!=baseline['sha256'][rel])
added=sorted(p.relative_to(root).as_posix() for p in (root/'src').rglob('*')
             if p.is_file() and p.relative_to(root).as_posix() not in old_sources)
assert changed==['src/imu/attitude_overlay.v','src/imu/imu_attitude.v','src/ov5640_ddr3_tft.v'],changed
assert added==['src/imu/imu_rotation_vector.v'],added
assert digest(root/'build.tcl')==baseline['sha256']['build.tcl']
cst=(root/'src/ov5640_ddr3_tft.cst').read_text()
pins=dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',cst,re.M))
report=(root/'impl/pnr/ov5640.rpt.txt').read_text()
actual=dict(re.findall(r'^([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',report,re.M))
assert all(actual.get(k)==v for k,v in pins.items()),'Routed pin mismatch'

# Check the real synthesized interconnect, in addition to unit arithmetic tests.
netlist=(root/'impl/gwsynthesis/ov5640.vg').read_text()
instances={name:re.search(r'\b\w+\s+'+name+r'\s*\(.*?\);',netlist,re.S)[0]
           for name in ('u_imu_rotation','u_imu_uart','u_attitude_overlay','u_vehicle')}
for name in ('u_imu_uart','u_attitude_overlay'):
    for axis in ('x','y','z'):
        assert 'imu_rot_'+axis in instances[name],f'{name} does not consume vector {axis}'
assert re.search(r'\.imu_yaw\(imu_yaw\[15:0\]\)',instances['u_vehicle']), 'Navigation heading changed'

log=(root/'sim/imu_test_results.txt').read_text()
assert log.count('PASS ')==13 and 'PASS imu_rotation: 3264' in log
assert not re.search(r'^(?:FATAL|FAIL|ERROR)\b',log,re.I|re.M)
audit_path='impl/pnr/ov5640_imu_rotation_audit.json'
audit={k:build[k] for k in ('project','bitstream','sha256','generated_time_utc8','bitstream_bytes',
        'resources','setup_violated_endpoints','hold_violated_endpoints','hardware_tested')}
audit.update(feature='Quaternion rotation-vector display/FireWater, no +/-90 Euler singularity; original Euler navigation heading',
    previous_production_sha256=baseline['sha256']['impl/pnr/ov5640.fs'],
    existing_source_changes=changed,added_sources=added,unchanged_existing_source_files=len(old_sources)-len(changed),
    cst_sha256=digest(root/'src/ov5640_ddr3_tft.cst'),sdc_sha256=digest(root/'src/ov5640_ddr3_tft.sdc'),
    pin_constraints_unchanged=True,pins_verified_from_pnr_report=pins,
    output_channels=['RotX','RotY','RotZ'],output_units='degrees, one decimal place',
    principal_rotation_limit_degrees=180,accumulated_turn_count=False,
    navigation_heading='original relative ZYX Euler yaw',
    regression_scenarios=13,rotation_vector_cases=3264,math_tolerance_degrees=0.3,
    sensor_drift_corrected_or_hardware_validated=False,
    return_bias_note='Identical starting/returning input verified; genuine input orientation offsets remain visible.',
    regression_log='sim/imu_test_results.txt',
    verification_sha256={rel:digest(root/rel) for rel in ('sim/imu_rotation_tb.v',
        'sim/run_imu_tests.py','sim/imu_rotation_vectors.mem','sim/imu_rotation_trace.csv','sim/imu_test_results.txt')})
(root/audit_path).write_text(json.dumps(audit,indent=2)+'\n')
build.update(current_change='Quaternion rotation-vector display and UART',current_change_audit=audit_path)
(root/'impl/verified_build.json').write_text(json.dumps(build,indent=2)+'\n')
p=root/'impl/pnr/ov5640_download.json';download=json.loads(p.read_text())
download.update({k:build[k] for k in ('sha256','generated_time_utc8','bitstream_bytes',
    'setup_violated_endpoints','hold_violated_endpoints')})
download.update(current_image='Production full-feature build with quaternion RotX/RotY/RotZ screen and FireWater',
    build_audit=audit_path,pin_constraints_unchanged=True,hardware_tested=False)
p.write_text(json.dumps(download,indent=2)+'\n')
print(json.dumps(dict(audit=audit_path,changed=changed,added=added,sha256=build['sha256'],
    pins_checked=len(pins),resources=build['resources']),indent=2))
