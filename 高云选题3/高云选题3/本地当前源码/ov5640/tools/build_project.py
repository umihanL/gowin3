"""Build Gowin in an ASCII workspace, validate, then publish locally. Never flash."""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET


PROJECT_FILES = ('ov5640.gprj', 'build.tcl')
REQUIRED = ('pnr/ov5640.fs', 'pnr/ov5640.rpt.txt', 'pnr/ov5640.tr', 'gwsynthesis/ov5640.vg')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    data = path.read_bytes()
    return data.decode('utf-16' if data.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig', errors='replace').replace('\r\n', '\n')


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def source_hashes(root):
    files = [root / name for name in PROJECT_FILES]
    files += [p for p in (root / 'src').rglob('*') if p.is_file()]
    if any(p.is_symlink() or not p.resolve().is_relative_to(root) for p in files):
        raise RuntimeError('Source symlink outside the project or symbolic file is unsupported')
    return {p.relative_to(root).as_posix(): digest(p) for p in sorted(files)}


def validate_project(root):
    # This launcher intentionally supports this self-contained project only.
    for item in ET.parse(root / 'ov5640.gprj').findall('.//File'):
        if item.get('enable', '1') != '1':
            continue
        path = item.get('path', '')
        target = (root / path).resolve()
        if not path or not path.isascii() or not target.is_relative_to(root / 'src') or not target.is_file():
            raise RuntimeError('Project dependency is not a local src file: ' + path)


def run_gowin(tool, stage, log, environment):
    with log.open('w', encoding='utf-8') as output:
        return subprocess.run([str(tool), 'build.tcl'], cwd=stage, env=environment,
                              stdout=output, stderr=subprocess.STDOUT).returncode


def validate_outputs(root, stage, started, hashes):
    if source_hashes(root) != hashes or source_hashes(stage) != hashes:
        raise RuntimeError('Sources changed during the build; outputs were not published')
    for name in REQUIRED:
        path = stage / 'impl' / name
        if not path.is_file() or path.stat().st_size == 0 or path.stat().st_mtime < started:
            raise RuntimeError('Missing, empty or stale build artifact: ' + name)
    log = read(stage / 'build.log')
    if ('Bitstream generation completed' not in log or 'Placement and routing completed' not in log
            or re.search(r'^\s*ERROR\b', log, re.M)):
        raise RuntimeError('Build log does not confirm successful implementation')
    timing = read(stage / 'impl/pnr/ov5640.tr')
    violated, slack = {}, {}
    for kind in ('Setup', 'Hold'):
        match = re.search(fr'<Numbers of {kind} Violated Endpoints>:(\d+)', timing)
        if not match:
            raise RuntimeError('Missing timing summary: ' + kind)
        violated[kind] = int(match[1])
    for kind in ('Setup', 'Hold', 'Recovery', 'Removal'):
        section = re.search(r'^3\.1\.\d+ ' + kind + r' Paths Table\n(.*?)(?=\n3\.|\Z)', timing, re.M | re.S)
        values = re.findall(r'^\s*\d+\s+(-?[\d.]+)\s', section[1], re.M) if section else []
        if not values:
            raise RuntimeError('Missing timing paths: ' + kind)
        slack[kind] = min(float(v) for v in values)
    if any(violated.values()) or min(slack.values()) < 0:
        raise RuntimeError('Timing checks failed: ' + str((violated, slack)))
    report = read(stage / 'impl/pnr/ov5640.rpt.txt')
    resources = {}
    for kind in ('Logic', 'Register', 'CLS', 'BSRAM', 'DSP', 'I/O Port'):
        match = re.search(r'^\s*' + re.escape(kind) + r'\s*\|\s*([\d.]+)/([\d.]+)\s*\|', report, re.M)
        if not match:
            raise RuntimeError('Missing resource entry: ' + kind)
        used, total = map(float, match.groups())
        if total <= 0 or used > total:
            raise RuntimeError('Resource overflow: ' + kind)
        resources[kind] = dict(used=used, total=total, percent=round(100 * used / total, 2))
    pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', read(root / 'src/ov5640_ddr3_tft.cst'), re.M))
    actual = dict(re.findall(r'^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|', report, re.M))
    if not pins or len(pins) != len(set(pins.values())) or any(actual.get(n) != p for n, p in pins.items()):
        raise RuntimeError('Routed pins differ from CST or pin assignments are ambiguous')
    return dict(timing_passed=True, timing_violations=violated, min_slack_ns=slack,
                resources=resources, pins_verified=pins,
                timing_scope='Existing SDC only; constraints and asynchronous/external interfaces still need hardware validation.')


def publish(root, stage, audit):
    """Back up replacement targets first; replace the downloadable bitstream last."""
    impl = root / 'impl'
    backup = impl / 'build_backups' / stage.name
    backup.mkdir(parents=True, exist_ok=False)
    files = {p.relative_to(stage / 'impl').as_posix(): p for p in (stage / 'impl').rglob('*') if p.is_file()}
    files['build.log'] = stage / 'build.log'
    audit_file = stage / 'build_audit.json'
    write_json(audit_file, audit)
    files['pnr/ov5640_current_audit.json'] = audit_file
    files['pnr/ov5640_download.json'] = audit_file
    files['build_inputs.json'] = stage / 'build_inputs.json'
    files['build_metrics.json'] = stage / 'build_metrics.json'
    order = sorted(files, key=lambda name: (name == 'pnr/ov5640.fs', name))
    replaced, added = [], []
    for name in order:
        target = impl / name
        if target.exists():
            saved = backup / name
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target, saved)
            replaced.append(name)
        else:
            added.append(name)
    write_json(backup / 'changes.json', dict(replaced=replaced, added=added, source=str(stage)))
    completed = []
    try:
        for name in order:
            target = impl / name
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(target.name + '.build-new')
            shutil.copyfile(files[name], temporary)
            if digest(temporary) != digest(files[name]):
                raise RuntimeError('Artifact copy verification failed: ' + name)
            shutil.copystat(files[name], temporary)
            if target.exists():
                target.chmod(target.stat().st_mode | 0o200)
            os.replace(temporary, target)
            completed.append(name)
    except Exception:
        for name in reversed(completed):
            target = impl / name
            saved = backup / name
            if saved.is_file():
                shutil.copy2(saved, target)
            else:
                target.unlink()
        raise
    return backup


def build(root, tool, work_root, runner=run_gowin):
    root, tool, work_root = root.resolve(), tool.resolve(), work_root.resolve()
    if not str(work_root).isascii() or not str(tool).isascii():
        raise RuntimeError('Gowin executable and working directory must use ASCII-only paths')
    if not tool.is_file():
        raise RuntimeError('Gowin executable not found: ' + str(tool))
    validate_project(root)
    hashes = source_hashes(root)
    work_root.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='ov5640_', dir=work_root))
    shutil.copytree(root / 'src', stage / 'src')
    for name in PROJECT_FILES:
        shutil.copy2(root / name, stage / name)
    if source_hashes(stage) != hashes:
        raise RuntimeError('Source changed during snapshot; rerun after saving all files')
    started = time.time()
    write_json(stage / 'build_inputs.json', dict(hardware_inputs=hashes, tool=str(tool),
        original_project=str(root), build_directory=str(stage), incremental=False,
        started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
    environment = dict(os.environ)
    environment.pop('GOWIN_INCREMENTAL_PR', None)
    print('Build workspace: ' + str(stage), flush=True)
    print('Live log: ' + str(stage / 'build.log'), flush=True)
    code = runner(tool, stage, stage / 'build.log', environment)
    metrics = dict(exit_code=code, seconds=round(time.time() - started, 3),
        source_unchanged=source_hashes(root) == hashes, hardware_tested=False, flashed=False)
    write_json(stage / 'build_metrics.json', metrics)
    if code != 0:
        raise RuntimeError(f'Gowin exited {code}; prior project outputs preserved. See {stage / "build.log"}')
    audit = validate_outputs(root, stage, started, hashes)
    fs = stage / 'impl/pnr/ov5640.fs'
    audit.update(release='local_' + stage.name, status='place_route_timing_passed',
        bitstream='impl/pnr/ov5640.fs', sha256=digest(fs), bytes=fs.stat().st_size,
        generated_utc=datetime.datetime.fromtimestamp(fs.stat().st_mtime, datetime.timezone.utc).isoformat(),
        tool=str(tool), device=ET.parse(root / 'ov5640.gprj').find('.//Device').get('pn'),
        build_workspace=str(stage), hardware_inputs=hashes, build_metrics=metrics,
        hardware_tested=False, flashed=False)
    backup = publish(root, stage, audit)
    print(json.dumps(dict(bitstream=str(root / audit['bitstream']), sha256=audit['sha256'],
        backup=str(backup), audit=str(root / 'impl/pnr/ov5640_current_audit.json'), flashed=False)), flush=True)
    return audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--gowin', type=Path, required=True)
    parser.add_argument('--work-root', type=Path, default=Path('D:/FPGA/codex_builds'))
    args = parser.parse_args()
    build(args.project, args.gowin, args.work_root)


if __name__ == '__main__':
    main()
