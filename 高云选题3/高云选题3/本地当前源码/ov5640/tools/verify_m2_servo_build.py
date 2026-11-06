"""Audit this change against its saved baseline after verify_navigation_build.py."""
from pathlib import Path
import hashlib
import json
import re


ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(rel):
    return (ROOT / rel).read_text(encoding='utf-8')


baseline = json.loads(read('impl/m2_servo_telemetry_baseline.json'))
backup = Path(baseline['backup'])
build = json.loads(read('impl/verified_build.json'))
assert digest(ROOT / build['bitstream']) == build['sha256'], 'Bitstream hash mismatch'
assert build['setup_violated_endpoints'] == build['hold_violated_endpoints'] == 0
for rel, expected in build['input_sha256'].items():
    assert digest(ROOT / rel) == expected, 'Changed build input: ' + rel

old_sources = {rel for rel in baseline['sha256'] if rel.startswith('src/')}
new_sources = {p.relative_to(ROOT).as_posix() for p in (ROOT / 'src').rglob('*') if p.is_file()}
assert new_sources == old_sources, 'Added/deleted source: ' + str(new_sources ^ old_sources)
changed = sorted(rel for rel in old_sources if digest(ROOT / rel) != baseline['sha256'][rel])
assert changed == [
    'src/imu/imu_firewater_tx.v', 'src/ov5640_ddr3_tft.v',
    'src/vehicle/servo_offset.mem', 'src/vehicle/vehicle_config.vh',
    'src/vehicle/vehicle_subsystem.v',
], changed
for rel in ('build.tcl', 'ov5640.gprj', 'src/ov5640_ddr3_tft.cst', 'src/ov5640_ddr3_tft.sdc'):
    assert digest(ROOT / rel) == baseline['sha256'][rel], 'Unexpected change: ' + rel

# Verify that the shared subsystem changed only at its target exports and M2 bridge.
before = (backup / 'src/vehicle/vehicle_subsystem.v').read_text(encoding='utf-8')
subsystem = read('src/vehicle/vehicle_subsystem.v')
reverted = subsystem.replace(
    '    // Export per-wheel targets after the existing rear differential for UART.\n'
    '    output signed [15:0] target_speed1,target_speed2,\n', '')
reverted = reverted.replace('    assign target_speed1=speed1;\n    assign target_speed2=speed2;\n', '')
reverted = reverted.replace(
    '    // M2 only: swap the two bridge commands in logic, preserving stop 00.\n'
    '    wire [1:0] motor2_dir;\n'
    '    assign dir2={motor2_dir[0],motor2_dir[1]};\n', '')
reverted = reverted.replace('.dir(motor2_dir)', '.dir(dir2)')
assert reverted == before, 'Unexpected motor/control subsystem change'
config = read('src/vehicle/vehicle_config.vh')
reverted_config = config.replace(
    '// M2 bridge polarity is reversed in vehicle_subsystem; reverse feedback too.\n'
    '`define VEHICLE_ENCODER2_INVERT 0', '`define VEHICLE_ENCODER2_INVERT 1')
assert reverted_config == (backup / 'src/vehicle/vehicle_config.vh').read_text(encoding='utf-8')
assert '`define VEHICLE_ENCODER1_INVERT 0' in config
assert '`define VEHICLE_SERVO_CENTER_DEG 90' in config
servo = [int(x, 16) for x in read('src/vehicle/servo_offset.mem').split()]
assert servo == [min(70, (a * 70 + 15) // 30) for a in range(64)]
assert servo[0] == 0 and servo[30] == 70 and 90 - max(servo) == 20 and 90 + max(servo) == 160

# Restrict top-level edits to the UART instance, its four nets, and wheel exports.
def without_uart(text):
    start = text.index('    // Keep RotXYZ') if '    // Keep RotXYZ' in text else text.index('    // Exactly seven channels:')
    end = text.index('    attitude_overlay #', start)
    return text[:start] + text[end:]


top = read('src/ov5640_ddr3_tft.v')
reverted_top = without_uart(top).replace(
    '    wire signed [15:0] motor_target1,motor_target2,motor_actual1,motor_actual2;\n', '')
reverted_top = reverted_top.replace(
    '        .target_speed1(motor_target1),.target_speed2(motor_target2),\n', '')
reverted_top = reverted_top.replace(
    '.actual_speed1(motor_actual1),.actual_speed2(motor_actual2)', '.actual_speed1(),.actual_speed2()')
assert reverted_top == without_uart((backup / 'src/ov5640_ddr3_tft.v').read_text(encoding='utf-8'))
assert '.EXTENDED_POSE(0),.MOTOR_TELEMETRY(1)' in top
assert '.CLK_HZ(33000000),.BAUD(115200),.SEND_HZ(20)' in top

cst = read('src/ov5640_ddr3_tft.cst')
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', cst, re.M))
report = read('impl/pnr/ov5640.rpt.txt')
actual = dict(re.findall(r'^([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
assert len(pins) == 116, len(pins)
assert all(actual.get(k) == v for k, v in pins.items()), 'Routed pin mismatch'
for name, ball in {'encoder2_A': 'F19', 'encoder2_B': 'F18',
                   'dir2[0]': 'C17', 'dir2[1]': 'B16', 'tx': 'M15'}.items():
    assert pins[name] == ball

netlist = read('impl/gwsynthesis/ov5640.vg')
instances = {}
for name in ('u_imu_uart', 'u_vehicle', 'u_attitude_overlay'):
    match = re.search(r'\b\w+\s+' + name + r'\s*\(.*?\);', netlist, re.S)
    assert match, 'Missing synthesized instance: ' + name
    instances[name] = match[0]
for axis in ('roll', 'pitch', 'yaw'):
    assert f'.imu_{axis}(imu_{axis}[15:0])' in instances['u_imu_uart']
for name in ('motor_target1', 'motor_actual1', 'motor_target2', 'motor_actual2'):
    assert f'.{name}({name}[15:0])' in instances['u_imu_uart']
    assert f'.{name}({name}[15:0])' in instances['u_vehicle']
assert '.imu_yaw(imu_yaw[15:0])' in instances['u_vehicle']
assert 'imu_rot_' not in instances['u_imu_uart'] and 'imu_relative_' not in instances['u_imu_uart']
for axis in 'xyz':
    assert 'imu_rot_' + axis in instances['u_attitude_overlay']

imu_log = read('sim/imu_test_results.txt')
vehicle_log = read('sim/vehicle_test_results.txt')
assert imu_log.count('PASS ') == 17
assert imu_log.count('exactly seven TX-wire channels') == 2
for check in ('20/90/160 servo pulses', 'M2 bridge swap and forward/reverse encoder feedback',
              'unchanged rear differential', 'per-wheel UART target exports',
              'PASS route execution', 'PASS Ackermann curve tracking', 'PASS obstacle bypass'):
    assert check in vehicle_log, 'Missing regression: ' + check
for content in (imu_log, vehicle_log):
    assert not re.search(r'^(?:FAIL|FATAL|ERROR)\b', content, re.I | re.M)

audit_path = 'impl/pnr/ov5640_m2_servo_audit.json'
audit = {k: build[k] for k in ('project', 'bitstream', 'sha256', 'generated_time_utc8',
         'bitstream_bytes', 'resources', 'setup_violated_endpoints', 'hold_violated_endpoints')}
audit.update(
    feature='Servo 20..160, M2 bridge/encoder reversal, exactly seven-channel RPY/motor FireWater',
    previous_production_sha256=baseline['sha256']['impl/pnr/ov5640.fs'],
    existing_source_changes=changed, added_sources=[],
    unchanged_existing_source_files=len(old_sources) - len(changed),
    cst_sha256=digest(ROOT / 'src/ov5640_ddr3_tft.cst'),
    sdc_sha256=digest(ROOT / 'src/ov5640_ddr3_tft.sdc'),
    pin_constraints_unchanged=True, pins_verified_from_pnr_report=pins,
    servo_center_deg=90, servo_limits_deg=[20, 160],
    servo_endpoints_verified_by_pwm_simulation=True,
    motor2_bridge_bits_swapped=True, motor2_encoder_invert_before=1, motor2_encoder_invert_after=0,
    motor1_shared_drivers_and_other_control_logic_unchanged=True,
    wheel_forward_reverse_verified_by_ps2_and_encoder_simulation=True,
    output_channels=['Roll', 'Pitch', 'Yaw', 'M1 target', 'M1 actual', 'M2 target', 'M2 actual'],
    output_decimals=[1, 1, 1, 0, 0, 0, 0],
    output_units=['degree'] * 3 + ['mm/s'] * 4,
    uart=dict(pin='M15', baud=115200, data_bits=8, stop_bits=1, parity='none', frames_per_second=20),
    unavailable_imu_angle_fields=[0, 0, 0], motor_telemetry_continues_without_imu=True,
    navigation_heading='unchanged original relative ZYX Euler yaw', screen='unchanged RotXYZ',
    spi_fusion_calibration_unchanged=True, hardware_tested=False,
    verification_sha256={rel: digest(ROOT / rel) for rel in (
        'sim/imu_motor_firewater_tb.v', 'sim/run_imu_tests.py', 'sim/imu_test_results.txt',
        'sim/vehicle_io_tb.sv', 'sim/run_vehicle_tests.py', 'sim/vehicle_test_results.txt',
        'tools/verify_m2_servo_build.py', 'tools/generate_vehicle_assets.py')})
(ROOT / audit_path).write_text(json.dumps(audit, indent=2) + '\n', encoding='utf-8')
build.update(current_change=audit['feature'], current_change_audit=audit_path)
(ROOT / 'impl/verified_build.json').write_text(json.dumps(build, indent=2) + '\n', encoding='utf-8')
download_path = ROOT / 'impl/pnr/ov5640_download.json'
download = json.loads(download_path.read_text(encoding='utf-8'))
download.update({k: build[k] for k in ('sha256', 'generated_time_utc8', 'bitstream_bytes',
                'setup_violated_endpoints', 'hold_violated_endpoints')})
download.update(current_image='Full production project; servo 20..160, reversed M2, seven-channel RPY/motor FireWater',
                build_audit=audit_path, pin_constraints_unchanged=True, hardware_tested=False,
                output_channels=audit['output_channels'])
download_path.write_text(json.dumps(download, indent=2) + '\n', encoding='utf-8')
assert digest(ROOT / build['bitstream']) == audit['sha256'] == download['sha256'] == build['sha256']
print(json.dumps(dict(audit=audit_path, sha256=build['sha256'], source_changes=changed,
                     pins_checked=len(pins), output_channels=audit['output_channels'],
                     resources=build['resources']), indent=2))
