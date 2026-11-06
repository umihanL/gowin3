"""Record the routed IMU UART build and verify the scope against its baseline."""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
baseline = json.loads((root/'impl/imu_firewater_baseline.json').read_text())
build = json.loads((root/'impl/verified_build.json').read_text())
assert digest(root/build['bitstream']) == build['sha256'], 'Run verify_navigation_build.py first'
assert build['setup_violated_endpoints'] == build['hold_violated_endpoints'] == 0
for rel, expected in build['input_sha256'].items():
    assert digest(root/rel) == expected, f'Build input changed: {rel}'

old_sources = {rel for rel in baseline['sha256'] if rel.startswith('src/')}
changed = sorted(rel for rel in old_sources
                 if digest(root/rel) != baseline['sha256'][rel])
added = sorted(p.relative_to(root).as_posix() for p in (root/'src').rglob('*')
               if p.is_file() and p.relative_to(root).as_posix() not in old_sources)
assert changed == ['src/ov5640_ddr3_tft.v'], f'Unexpected existing source changes: {changed}'
assert added == ['src/imu/imu_firewater_tx.v'], f'Unexpected added sources: {added}'
for rel in ('src/ov5640_ddr3_tft.cst', 'src/ov5640_ddr3_tft.sdc', 'build.tcl'):
    assert digest(root/rel) == baseline['sha256'][rel], f'Unexpected constraint/build change: {rel}'

cst = (root/'src/ov5640_ddr3_tft.cst').read_text()
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', cst, re.M))
report = (root/'impl/pnr/ov5640.rpt.txt').read_text()
routed_pins = dict(re.findall(r'^([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|',
                              report, re.M))
for name, pin in pins.items():
    assert routed_pins.get(name) == pin, f'Routed pin mismatch: {name}'
assert pins['tx'] == 'M15'

netlist = (root/'impl/gwsynthesis/ov5640.vg').read_text()
assert re.search(r'\bmodule imu_firewater_tx\b', netlist), 'IMU telemetry optimized out'
assert re.search(r'\bimu_firewater_tx\s+u_imu_uart\s*\(', netlist), 'IMU telemetry not instantiated'
assert not re.search(r'\bmodule vofa_firewater_tx\b', netlist), 'Old motor telemetry still synthesized'
uart_resources = ET.parse(root/'impl/gwsynthesis/ov5640_syn_rsc.xml').getroot().find(
    "./SubModule[@name='u_imu_uart']").attrib
assert not any(k.startswith('T_MULT') or k == 'T_Bsram' for k in uart_resources), 'Unexpected UART DSP/BSRAM'

log = (root/'sim/imu_test_results.txt').read_text()
for clk, baud in ((1000000,100000),(33000000,115200)):
    assert f'PASS imu_firewater: CLK={clk} BAUD={baud};' in log
assert log.count('PASS ') == 11, 'Incomplete IMU regression'
assert not re.search(r'^(?:FATAL|ERROR|FAIL)\b',log,re.I|re.M)

audit_path = 'impl/pnr/ov5640_imu_firewater_audit.json'
audit = {key: build[key] for key in ('project','bitstream','sha256','generated_time_utc8',
         'bitstream_bytes','resources','setup_violated_endpoints','hold_violated_endpoints','hardware_tested')}
audit.update(
    feature='Three-axis attitude UART in FireWater ASCII, replacing four-channel motor-speed telemetry',
    previous_production_sha256=baseline['sha256']['impl/pnr/ov5640.fs'],
    existing_source_changes=changed, added_sources=added,
    unchanged_existing_source_files=len(old_sources)-len(changed),
    constraints_unchanged=True, cst_sha256=digest(root/'src/ov5640_ddr3_tft.cst'),
    pins_verified_from_pnr_report=pins,
    imu_uart_synthesis_resources=uart_resources,
    uart=dict(tx_pin='M15',baud=115200,data_bits=8,parity='none',stop_bits=1,
              max_frames_per_second=20,channels=['Roll','Pitch','Yaw'],units='degrees',
              decimals=1,example='samples:12.3,-45.6,179.9\r\n',
              fresh_valid_samples_only=True,offline_finishes_active_frame_then_stops=True),
    regression_tests=11,regression_log='sim/imu_test_results.txt',
    verification_sha256={rel:digest(root/rel) for rel in (
        'sim/imu_firewater_tb.v','sim/run_imu_tests.py','sim/imu_test_results.txt')})
(root/audit_path).write_text(json.dumps(audit,indent=2)+'\n')
download_path = root/'impl/pnr/ov5640_download.json'
download = json.loads(download_path.read_text())
download.update({key:build[key] for key in ('sha256','generated_time_utc8','bitstream_bytes',
                'setup_violated_endpoints','hold_violated_endpoints')})
download.update(current_image='Production full-feature build with corrected pitch and Roll/Pitch/Yaw FireWater UART',
                build_audit=audit_path,pin_constraints_unchanged=True,hardware_tested=False)
download_path.write_text(json.dumps(download,indent=2)+'\n')
build.update(current_change='IMU Roll/Pitch/Yaw FireWater UART',current_change_audit=audit_path)
(root/'impl/verified_build.json').write_text(json.dumps(build,indent=2)+'\n')
print(json.dumps(dict(audit=audit_path,changed=changed,added=added,pins_checked=len(pins),
                      sha256=build['sha256'],resources=build['resources']),indent=2))
