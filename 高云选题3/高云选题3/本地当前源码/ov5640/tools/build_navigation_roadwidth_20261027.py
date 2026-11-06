"""Build and audit the measured wider-road vision update."""
from pathlib import Path
import datetime, hashlib, json, os, re, shutil, subprocess, time

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'impl/baseline_navigation_visual_20261027'
LABEL = 'navigation_roadwidth_20261027'
EXPECTED_OLD_BITSTREAM = '536e8b1d5c724ed80eeb78a268820be2633a9c452d8d64cd0a9259acfe857dfe'
ALLOWED = {
    'src/portrait/vision_video.v',
    'src/portrait/vision_camera_config.vh',
    'src/portrait/vision_ipm_lut.vh',
}
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

def decode(path):
    data = path.read_bytes()
    encoding = 'utf-16' if data.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig'
    return data.decode(encoding).replace('\r\n', '\n')

baseline = json.loads((BASE/'manifest.json').read_text(encoding='utf-8'))
assert all(sha(BASE/name) == digest for name, digest in baseline.items()), 'Rollback snapshot failed integrity check'
hardware = [p for p in (ROOT/'src').rglob('*') if p.is_file() and
            p.suffix in ('.v', '.vo', '.vh', '.mem', '.cst', '.sdc', '.ipc', '.mod')]
hardware += [ROOT/'ov5640.gprj', ROOT/'build.tcl']
all_sources = {p.relative_to(ROOT).as_posix(): sha(p) for p in hardware}
changed = sorted(name for name, digest in all_sources.items()
                 if baseline.get(name) != digest)
assert set(changed) == ALLOWED, changed
assert sha(ROOT/'impl/pnr/ov5640.fs') == EXPECTED_OLD_BITSTREAM, 'Starting bitstream is not the requested restored release'

road = json.loads((ROOT/'src/navigation/road_dimensions.json').read_text(encoding='utf-8'))
assert road['measured_2026_10_27'] == {
    'outer_width_mm': 373, 'inner_width_mm': 340, 's_bend_width_mm': 340}
manifest = {
    'frozen_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'baseline_release': 'host_navigation_20261027',
    'measurements_mm': road['measured_2026_10_27'],
    'changed_hardware_sources': changed,
    'map_files_unchanged': True,
    'hardware_inputs': all_sources,
}
snapshot = ROOT/f'impl/{LABEL}_inputs.json'
snapshot.write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')

environment = os.environ.copy()
environment.pop('GOWIN_INCREMENTAL_PR', None)
start = time.monotonic()
log_name = f'impl/{LABEL}_build.log'
with (ROOT/log_name).open('w', encoding='utf-8') as output:
    result = subprocess.run(
        ['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe', 'build.tcl'],
        cwd=ROOT, env=environment, stdout=output, stderr=subprocess.STDOUT)
metrics = {
    'exit_code': result.returncode,
    'seconds': round(time.monotonic()-start, 3),
    'mode': 'standard_synthesis_and_place_route',
    'incremental_checkpoint_used': False,
    'reason': 'This project previously measured slower with incremental physical matching.',
    'previous_bitstream_sha256': EXPECTED_OLD_BITSTREAM,
    'changed_hardware_sources': changed,
}
(ROOT/f'impl/{LABEL}_build_metrics.json').write_text(json.dumps(metrics, indent=2)+'\n', encoding='utf-8')
assert result.returncode == 0, f'Gowin build failed; see {log_name}'
assert all(sha(ROOT/name) == digest for name, digest in all_sources.items()), 'Source files changed during build'

build = decode(ROOT/log_name)
assert 'Bitstream generation completed' in build and 'Placement and routing completed' in build
assert not re.search(r'^\s*ERROR\b', build, re.M)
timing = decode(ROOT/'impl/pnr/ov5640.tr')
report = decode(ROOT/'impl/pnr/ov5640.rpt.txt')
violations = {}
slack = {}
for kind in ('Setup', 'Hold'):
    match = re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)
    assert match, f'Missing {kind} timing summary'
    violations[kind] = int(match[1])
for kind in ('Setup', 'Hold', 'Recovery', 'Removal'):
    table = re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)', timing, re.M|re.S)
    assert table, f'Missing {kind} timing table'
    values = [float(v) for v in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s', table[1], re.M)]
    assert values, f'Empty {kind} timing table'
    slack[kind] = min(values)
assert not any(violations.values()) and min(slack.values()) >= 0, (violations, slack)

resources = {}
for name in ('Logic', 'Register', 'CLS', 'BSRAM', 'DSP', 'I/O Port'):
    match = re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|', report, re.M)
    assert match, f'Missing {name} resource summary'
    used, total = map(float, match.groups())
    assert used <= total, (name, used, total)
    resources[name] = {'used': used, 'total': total,
                       'percent': round(100*used/total, 2)}

pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',
                       decode(ROOT/'src/ov5640_ddr3_tft.cst'), re.M))
assert pins.get('host_tx') == 'E14' and pins.get('host_rx') == 'E13'
netlist = decode(ROOT/'impl/gwsynthesis/ov5640.vg')
for module in ('vehicle_motion', 'vehicle_route_profile', 'vehicle_vision_fusion',
               'vehicle_bypass', 'vision_video', 'vl53l1x_array_controller',
               'vehicle_host_link', 'host_uart_rx'):
    assert re.search(r'^module '+module+r'\b', netlist, re.M), module

map_names = [
    'src/navigation/map_raw_1cm.mem', 'src/navigation/map_plan_2cm.mem',
    'src/navigation/map_heading_2cm.mem', 'src/navigation/map_central_2cm.mem',
    'src/navigation/nav_centerline_config.vh', 'src/navigation/nav_centerline_init.vh',
    'src/navigation/nav_centerline_reverse.vh', 'src/navigation/nav_centerline_bitmap.vh',
    'src/navigation/nav_centerline_junctions.vh',
]
assert all(all_sources[name] == baseline[name] for name in map_names), 'Map or planned centreline changed'

bitstream = ROOT/'impl/pnr/ov5640.fs'
assert sha(bitstream) != EXPECTED_OLD_BITSTREAM, 'New bitstream was not generated'
release = ROOT/f'impl/pnr/ov5640_{LABEL}.fs'
shutil.copy2(bitstream, release)
generated = datetime.datetime.fromtimestamp(bitstream.stat().st_mtime, datetime.timezone.utc).isoformat()
summary = {
    'status': 'digital_verification_passed',
    'recorded_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'release': LABEL,
    'measurements_mm': road['measured_2026_10_27'],
    'bitstream': 'impl/pnr/ov5640.fs',
    'release_bitstream': release.relative_to(ROOT).as_posix(),
    'sha256': sha(bitstream),
    'bytes': bitstream.stat().st_size,
    'generated_utc': generated,
    'tool': 'Gowin V1.9.12.03',
    'device': 'GW5AT-LV60PG484AC1/I0 B',
    'hardware_tested': False,
    'flashed': False,
    'baseline_bitstream_sha256': EXPECTED_OLD_BITSTREAM,
    'changed_hardware_sources': changed,
    'map_files_unchanged': True,
    'map_file_sha256': {name: all_sources[name] for name in map_names},
    'build_metrics': metrics,
    'timing_violations': violations,
    'min_slack_ns': slack,
    'timing_passed': True,
    'resources': resources,
    'pins': {'host_tx': pins['host_tx'], 'host_rx': pins['host_rx']},
    'limitations': ['Not flashed; physical car and camera conditions were not tested.',
                    'Map geometry remains the restored release geometry; only the visual road-width model changed.',
                    'Camera field of view and distortion remain provisional.'],
}
for name in (f'docs/{LABEL}.json', 'impl/pnr/ov5640_current_audit.json',
             f'impl/pnr/ov5640_{LABEL}_audit.json'):
    (ROOT/name).write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
download = {key: summary[key] for key in (
    'bitstream', 'release_bitstream', 'sha256', 'bytes', 'generated_utc',
    'tool', 'device', 'timing_passed', 'hardware_tested', 'flashed')}
(ROOT/'impl/pnr/ov5640_download.json').write_text(
    json.dumps(download, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps({key: summary[key] for key in (
    'status', 'release_bitstream', 'sha256', 'changed_hardware_sources',
    'timing_violations', 'min_slack_ns', 'resources', 'build_metrics')},
    ensure_ascii=False, indent=2))
