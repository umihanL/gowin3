"""Audit imported MOTOR code and pin assignments without changing the source project."""
from pathlib import Path
import difflib
import hashlib
import json
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]
manifest_path = ROOT/'src/vehicle/import_manifest.json'
manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
source = Path(manifest['source'])/'src'
changes = {
    'motor_closed_loop.v': ['Export existing encoder position; decoder, speed estimator and PI logic retained.'],
    'motor_target_counts.v': ['Map the small synchronous conversion ROM to logic to meet the chip BSRAM limit; values and latency unchanged.'],
    'ps2_test_control.v': ['Map the small synchronous axis ROMs to logic to meet the chip BSRAM limit; values, latency and controller behavior unchanged.'],
}
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
diffs = []
for name, info in manifest['files'].items():
    original, local = source/name, ROOT/'src/vehicle/motor'/name
    assert digest(original) == info['source_sha256'], f'MOTOR source changed: {name}'
    a, b = original.read_text().splitlines(), local.read_text().splitlines()
    if a != b:
        assert name in changes, f'Unexpected imported algorithm modification: {name}'
        diffs.extend(difflib.unified_diff(a, b, fromfile=f'MOTOR/src/{name}', tofile=f'src/vehicle/motor/{name}', lineterm=''))
    info.update(imported_sha256=digest(local), changes=changes.get(name, []))
manifest_path.write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
(ROOT/'docs/motor_import.diff').write_text('\n'.join(diffs)+'\n', encoding='utf-8')
pins = dict(re.findall(r'IO_LOC "([^"]+)" (\w+);', (ROOT/'src/ov5640_ddr3_tft.cst').read_text()))
assert len(set(pins.values())) == len(pins), 'Duplicate FPGA pad'
expected = dict(tx='M15', encoder_A='E18', encoder_B='F18', pwm_out='E17',
                encoder2_A='C20', encoder2_B='D20', pwm_out2='F16',
                ps2_cs_n='B16', ps2_mosi='C17', ps2_clk='B15', ps2_miso='D17',
                servo_pwm_out='C15', tof_sda='C19', tof_scl='C18',
                tof_xs1='B18', tof_xs2='B17', tof_xs3='D16', tof_xs4='E16')
expected.update({'dir[0]':'F19', 'dir[1]':'F20', 'dir2[0]':'E19', 'dir2[1]':'D19'})
assert all(pins.get(k) == v for k,v in expected.items()), 'Vehicle pin mismatch'
with zipfile.ZipFile(manifest['backup']) as archive:
    for item in archive.infolist():
        name = item.filename.replace('\\', '/')
        if name.startswith('src/') and not item.is_dir() and not (
                name.startswith('src/navigation/') or name == 'src/ov5640_ddr3_tft.v' or
                name.endswith(('.cst','.sdc'))):
            assert (ROOT/name).read_bytes() == archive.read(item), f'Unexpected existing module change: {name}'
    old_init = archive.read('src/navigation/nav_map_init.vh').decode('ascii')
    new_init = (ROOT/'src/navigation/nav_map_init.vh').read_text()
    occupancy = lambda s: [line.strip() for line in re.findall(r'^(?:raw_map|plan_map|heading_map)\[.*$', s, re.M)]
    assert occupancy(old_init) == occupancy(new_init), 'Occupancy ROM geometry changed'
print(f'PASS import audit: {len(manifest["files"])} MOTOR modules, 22 vehicle pins without duplicates, unchanged camera/DDR/FIFO/IMU and all occupancy ROM words')
