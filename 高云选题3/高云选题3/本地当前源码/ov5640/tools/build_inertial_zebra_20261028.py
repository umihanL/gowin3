"""Build the navigation obstacle-switch release and audit actual PnR."""
from pathlib import Path
import datetime, hashlib, json, os, re, shutil, stat, subprocess, sys, time

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'inertial_zebra_20261028'
BASE = ROOT/'impl/baseline_inertial_zebra_20261028'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def decode(path):
    data = path.read_bytes()
    return data.decode('utf-16' if data.startswith((b'\xff\xfe', b'\xfe\xff'))
                       else 'utf-8-sig').replace('\r\n', '\n')


def inputs():
    paths = [p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in
             ('.v', '.vo', '.vh', '.mem', '.cst', '.sdc', '.ipc', '.mod')]
    paths += [ROOT/'ov5640.gprj', ROOT/'build.tcl']
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in paths}


baseline = json.loads((BASE/'manifest.json').read_text(encoding='utf-8'))
assert all(sha(BASE/name) == digest for name, digest in baseline.items())
frozen = inputs()
changed = sorted(n for n, digest in frozen.items() if baseline.get(n) != digest)
removed = sorted(n for n in baseline if n.startswith('src/') and n not in frozen)
assert set(removed) == {'src/vehicle/vehicle_vision_fusion.v',
    'src/portrait/vision_camera_config.vh', 'src/portrait/vision_ipm_lut.vh'}, removed
assert set(changed) == {
    'src/navigation/vehicle_settings.v', 'src/navigation/navigation_ui.v',
    'src/navigation/nav_renderer.v', 'src/navigation/nav_font.vh',
    'src/navigation/nav_font_rows.mem', 'src/ov5640_ddr3_tft.v',
    'src/vehicle/vehicle_subsystem.v', 'src/vehicle/vehicle_motion.v',
    'src/portrait/vision_video.v', 'src/imu/attitude_overlay.v', 'ov5640.gprj',
}, changed
map_names = [n for n in frozen if n.startswith('src/navigation/') and
             (Path(n).suffix == '.mem' and 'font' not in n or
              Path(n).name.startswith('nav_centerline'))]
assert all(frozen[n] == baseline[n] for n in map_names)
assert set(sys.argv[1:]) <= {'--audit-existing'}, sys.argv[1:]
input_path = ROOT/f'impl/{LABEL}_inputs.json'
metrics_path = ROOT/f'impl/{LABEL}_build_metrics.json'
log_name = f'impl/{LABEL}_build.log'
if '--audit-existing' in sys.argv[1:]:
    # Resume publication after an interrupted copy without rebuilding RTL.
    previous = json.loads(input_path.read_text(encoding='utf-8'))
    assert previous['hardware_inputs'] == frozen, 'Completed build inputs changed'
    assert previous['changed_hardware_sources'] == changed
    started = datetime.datetime.fromisoformat(previous['started_utc']).timestamp()
    for name in (log_name, 'impl/pnr/ov5640.fs', 'impl/pnr/ov5640.tr',
                 'impl/pnr/ov5640.rpt.txt', 'impl/gwsynthesis/ov5640.vg'):
        assert (ROOT/name).stat().st_mtime >= started, name
    metrics = json.loads(metrics_path.read_text(encoding='utf-8'))
else:
    started_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
    input_path.write_text(json.dumps({
        'started_utc': started_utc, 'hardware_inputs': frozen,
        'changed_hardware_sources': changed,
    }, indent=2)+'\n', encoding='utf-8')
    env = os.environ.copy()
    env.pop('GOWIN_INCREMENTAL_PR', None)
    start = time.monotonic()
    with (ROOT/log_name).open('w', encoding='utf-8') as output:
        result = subprocess.run(
            ['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe', 'build.tcl'],
            cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT)
    metrics = {'exit_code': result.returncode, 'seconds': round(time.monotonic()-start, 3),
               'mode': 'standard_synthesis_and_place_route',
               'incremental_checkpoint_used': False}
    metrics_path.write_text(json.dumps(metrics, indent=2)+'\n')
assert metrics['exit_code'] == 0, f'Gowin failed: {log_name}'
assert inputs() == frozen, 'RTL changed during build'
build = decode(ROOT/log_name)
assert 'Bitstream generation completed' in build and 'Placement and routing completed' in build
assert not re.search(r'^\s*ERROR\b', build, re.M)
timing = decode(ROOT/'impl/pnr/ov5640.tr')
report = decode(ROOT/'impl/pnr/ov5640.rpt.txt')
violations, slack, resources = {}, {}, {}
for kind in ('Setup', 'Hold'):
    m = re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)
    assert m, kind
    violations[kind] = int(m[1])
for kind in ('Setup', 'Hold', 'Recovery', 'Removal'):
    m = re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)', timing, re.M|re.S)
    assert m, kind
    values = [float(v) for v in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s', m[1], re.M)]
    assert values, kind
    slack[kind] = min(values)
assert not any(violations.values()) and min(slack.values()) >= 0, (violations, slack)
for name in ('Logic', 'Register', 'CLS', 'BSRAM', 'DSP', 'I/O Port'):
    m = re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|', report, re.M)
    assert m, name
    used, total = map(float, m.groups())
    assert used <= total
    resources[name] = dict(used=used, total=total, remaining=total-used,
                           percent=round(100*used/total, 2))

pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',
                       decode(ROOT/'src/ov5640_ddr3_tft.cst'), re.M))
actual = dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
assert all(actual.get(n) == p for n, p in pins.items()), 'Routed pin mismatch'
netlist = decode(ROOT/'impl/gwsynthesis/ov5640.vg')
for module in ('vehicle_motion', 'vision_zebra', 'vehicle_bypass', 'vision_video',
               'vehicle_host_link', 'vl53l1x_array_controller'):
    assert re.search(r'^module '+module+r'\b', netlist, re.M), module

assert not re.search(r'^module vehicle_vision_fusion\b', netlist, re.M)
bitstream = ROOT/'impl/pnr/ov5640.fs'
assert sha(bitstream) != sha(BASE/'ov5640.fs')
release = ROOT/f'impl/pnr/ov5640_{LABEL}.fs'
if release.exists():
    # Gowin creates read-only bitstreams; copy2 preserves that attribute.
    release.chmod(release.stat().st_mode | stat.S_IWRITE)
shutil.copy2(bitstream, release)
generated = datetime.datetime.fromtimestamp(bitstream.stat().st_mtime, datetime.timezone.utc).isoformat()
summary = dict(
    release=LABEL, status='place_route_timing_passed',
    bitstream='impl/pnr/ov5640.fs', release_bitstream=release.relative_to(ROOT).as_posix(),
    sha256=sha(bitstream), bytes=bitstream.stat().st_size, generated_utc=generated,
    tool='Gowin V1.9.12.03', device='GW5AT-LV60PG484AC1/I0 B',
    hardware_tested=False, flashed=False, timing_passed=True,
    resources=resources, timing_violations=violations, min_slack_ns=slack,
    build_metrics=metrics, hardware_inputs=frozen, changed_hardware_sources=changed,
    removed_hardware_sources=removed, baseline_bitstream_sha256=sha(BASE/'ov5640.fs'), map_files_unchanged=True,
    map_file_sha256={n: frozen[n] for n in map_names}, pins_verified=len(pins),
    camera_intrinsics_calibrated=False,
)
for name in (f'docs/{LABEL}_validation.json', 'impl/pnr/ov5640_current_audit.json',
             f'impl/pnr/ov5640_{LABEL}_audit.json'):
    (ROOT/name).write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
(ROOT/'impl/pnr/ov5640_download.json').write_text(json.dumps({k: summary[k] for k in (
    'release', 'bitstream', 'release_bitstream', 'sha256', 'bytes', 'generated_utc',
    'tool', 'device', 'hardware_tested', 'flashed', 'timing_passed')}, indent=2)+'\n')
print(json.dumps({k: summary[k] for k in ('release_bitstream', 'sha256', 'resources',
      'timing_violations', 'min_slack_ns', 'build_metrics')}, indent=2), flush=True)
