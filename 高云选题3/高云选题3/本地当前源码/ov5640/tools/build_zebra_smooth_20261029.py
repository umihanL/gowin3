"""Build and audit the crosswalk/S-sampling release from frozen hardware inputs."""
from pathlib import Path
import datetime
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'zebra_smooth_20261029'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def read(path):
    b = path.read_bytes()
    return b.decode('utf-16' if b.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig').replace('\r\n', '\n')


def inputs():
    paths = [p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in
             ('.v', '.vo', '.vh', '.mem', '.cst', '.sdc', '.ipc', '.mod')]
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in paths+[ROOT/'ov5640.gprj', ROOT/'build.tcl']}


previous = json.loads((ROOT/f'impl/{LABEL}_previous_inputs.json').read_text())
frozen = inputs()
changed = sorted(n for n, digest in frozen.items() if previous.get(n) != digest)
assert set(previous) == set(frozen)
assert set(changed) == {
    'src/ov5640_ddr3_tft.v', 'src/navigation/navigation_ui.v', 'src/navigation/nav_renderer.v',
    'src/navigation/nav_centerline_config.vh', 'src/navigation/nav_centerline_init.vh',
    'src/navigation/nav_centerline_reverse.vh', 'src/portrait/vision_video.v', 'src/portrait/vision_zebra.v'}
assert sys.argv[1:] in ([], ['--audit-existing'])
record = ROOT/f'impl/{LABEL}_inputs.json'
metrics_file = ROOT/f'impl/{LABEL}_build_metrics.json'
log = ROOT/f'impl/{LABEL}_build.log'
if '--audit-existing' not in sys.argv:
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    record.write_text(json.dumps(dict(started_utc=started, hardware_inputs=frozen), indent=2)+'\n')
    env = os.environ.copy(); env.pop('GOWIN_INCREMENTAL_PR', None)
    start = time.monotonic()
    with log.open('w', encoding='utf-8') as output:
        result = subprocess.run(['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe', 'build.tcl'],
                                cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT)
    metrics_file.write_text(json.dumps(dict(exit_code=result.returncode, seconds=round(time.monotonic()-start, 3)), indent=2))
else:
    saved = json.loads(record.read_text()); assert saved['hardware_inputs'] == frozen
    started = saved['started_utc']
metrics = json.loads(metrics_file.read_text())
assert metrics['exit_code'] == 0, log
assert frozen == inputs(), 'Hardware inputs changed during build'
text = read(log)
assert 'Bitstream generation completed' in text and 'Placement and routing completed' in text
assert not re.search(r'^\s*ERROR\b', text, re.M)
fs = ROOT/'impl/pnr/ov5640.fs'
assert fs.stat().st_mtime >= datetime.datetime.fromisoformat(started).timestamp()
report = read(ROOT/'impl/pnr/ov5640.rpt.txt'); timing = read(ROOT/'impl/pnr/ov5640.tr')
resources, violations, slack = {}, {}, {}
for name in ('Logic', 'Register', 'CLS', 'BSRAM', 'DSP', 'I/O Port'):
    m = re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|', report, re.M)
    assert m, name
    used, total = map(float, m.groups()); assert used <= total
    resources[name] = dict(used=used, total=total, remaining=total-used, percent=round(100*used/total, 2))
for kind in ('Setup', 'Hold'):
    violations[kind] = int(re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)[1])
for kind in ('Setup', 'Hold', 'Recovery', 'Removal'):
    m = re.search(r'^3\.1\.\d '+kind+r' Paths Table\n(.*?)(?=\n3\.)', timing, re.M | re.S)
    slack[kind] = min(float(v) for v in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s', m[1], re.M))
assert not any(violations.values()) and min(slack.values()) >= 0, (violations, slack)
pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', read(ROOT/'src/ov5640_ddr3_tft.cst'), re.M))
actual = dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
assert all(actual.get(n) == p for n, p in pins.items())
from verify_font_netlist import audit as font_audit
from verify_map_netlist import audit as map_audit
from verify_road_netlist import audit as road_audit
roms = dict(font=font_audit(), map=map_audit(), road=road_audit())
release = ROOT/f'impl/pnr/ov5640_{LABEL}.fs'
if release.exists(): release.chmod(release.stat().st_mode | stat.S_IWRITE)
shutil.copy2(fs, release)
summary = dict(release=LABEL, status='place_route_timing_passed',
    bitstream='impl/pnr/ov5640.fs', release_bitstream=release.relative_to(ROOT).as_posix(),
    sha256=sha(fs), bytes=fs.stat().st_size,
    generated_utc=datetime.datetime.fromtimestamp(fs.stat().st_mtime, datetime.timezone.utc).isoformat(),
    tool='Gowin V1.9.12.03', device='GW5AT-LV60PG484AC1/I0 B',
    hardware_tested=False, flashed=False, timing_passed=True,
    resources=resources, timing_violations=violations, min_slack_ns=slack,
    build_metrics=metrics, hardware_inputs=frozen, changed_hardware_sources=changed,
    pins_verified=len(pins), rom_audits=roms,
    visual_navigation=dict(steering=False, pose_correction=False),
    zebra=dict(bar_directions='transverse and longitudinal', adaptive_luma=True,
               preview_threshold_independent=True, limit='min(normal/2, 60 mm/s)', clear='next complete clear frame'),
    s_bend=dict(spacing_mm=10, geometry_unchanged=True, tracker_unchanged=True))
for name in [f'docs/{LABEL}_validation.json', 'impl/pnr/ov5640_current_audit.json', f'impl/pnr/ov5640_{LABEL}_audit.json']:
    (ROOT/name).write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
(ROOT/'impl/pnr/ov5640_download.json').write_text(json.dumps({k: summary[k] for k in
    ('release', 'bitstream', 'release_bitstream', 'sha256', 'bytes', 'generated_utc', 'tool', 'device',
     'hardware_tested', 'flashed', 'timing_passed')}, indent=2)+'\n')
print(json.dumps(dict(release_bitstream=summary['release_bitstream'], resources=resources,
                     timing_violations=violations, min_slack_ns=slack), indent=2), flush=True)
