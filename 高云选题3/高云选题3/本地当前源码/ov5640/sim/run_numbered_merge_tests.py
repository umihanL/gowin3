"""Merged numbered-destination + extension regression, pin UART and real planner.

Runs in WSL on Windows. Excludes obsolete *_1 alternatives explicitly and
records source hashes; no board access or hardware motion occurs.
"""
from pathlib import Path
import argparse
import concurrent.futures
import csv
import hashlib
import json
import os
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'diagnostics/numbered_merge_20261031'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def invoke(args, timeout=1200):
    command = ['wsl.exe', '--exec', *args] if os.name == 'nt' else args
    result = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, timeout=timeout)
    text = result.stdout.decode('utf-8', errors='replace').replace('\0', '')
    if result.returncode or 'FATAL:' in text:
        raise RuntimeError(text)
    return text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--only', nargs='+', help='case names; points_1 ... points_9')
    parser.add_argument('--workers', type=int, default=3)
    options = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    source_paths = [p for p in (ROOT / 'src').rglob('*')
                    if p.is_file() and not p.stem.endswith('_1')]
    inputs = {p.relative_to(ROOT).as_posix(): digest(p) for p in source_paths}
    sources = [p.relative_to(ROOT).as_posix()
               for folder in ('src/navigation', 'src/vehicle')
               for p in (ROOT / folder).rglob('*.v') if not p.stem.endswith('_1')]
    sources += ['src/imu/imu_math.v', 'src/portrait/vision_zebra.v',
                'src/portrait/vision_video.v', 'sim/ps2_receiver_model.sv',
                'sim/vl53l1x_sensor_model.v']
    cases = [
        ('point_faults', 'host_points_fault_tb', []),
        ('point_ui_cancel', 'host_points_ui_cancel_tb', []),
        ('zebra_switch', 'zebra_switch_tb', []),
        ('detector', 'vision_zebra_tb', []),
        ('navigation_no_limit', 'navigation_no_zebra_limit_tb', []),
        ('controller', 'vehicle_extension_tb', []),
        ('extension_uart', 'extension_uart_tb', []),
        ('legacy_uart', 'vehicle_host_link_tb', []),
        ('legacy_integration', 'host_navigation_integration_tb', []),
        ('settings', 'vehicle_settings_tb', []),
        ('vehicle_io', 'vehicle_io_tb', []),
    ] + [(f'points_{n}', 'host_points_tb', [f'+point={n}']) for n in range(1, 10)]
    if options.only:
        unknown = set(options.only) - {case[0] for case in cases}
        if unknown:
            parser.error('unknown cases: ' + ', '.join(sorted(unknown)))
        cases = [case for case in cases if case[0] in options.only]
    binaries = {}
    for _, top, _ in cases:
        if top in binaries:
            continue
        binary = (OUT / (top + '.vvp')).relative_to(ROOT).as_posix()
        text = invoke(['iverilog', '-g2012', '-s', top, '-I', 'src/navigation',
                       '-I', 'src/vga', '-o', binary, f'sim/{top}.sv', *sources])
        (OUT / (top + '_compile.log')).write_text(text, encoding='utf-8')
        binaries[top] = binary

    def run(case):
        name, top, extra = case
        start = time.monotonic()
        try:
            text = invoke(['vvp', binaries[top], *extra])
            passed = 'PASS ' in text
        except Exception as error:
            text = str(error)
            passed = False
        (OUT / (name + '.log')).write_text(text, encoding='utf-8')
        record = dict(name=name, passed=passed, seconds=round(time.monotonic()-start, 3),
                      test=f'sim/{top}.sv', test_sha256=digest(ROOT/f'sim/{top}.sv'))
        if name.startswith('points_') and passed:
            point = int(name.split('_')[1])
            route = ROOT / f'sim/host_point_{point}_route.csv'
            record.update(route=route.relative_to(ROOT).as_posix(), route_sha256=digest(route))
            cells = [(17,17),(12,80),(17,142),(88,147),(158,142),
                     (163,80),(158,17),(88,13),(88,73)]
            with route.open() as stream:
                rows = list(csv.DictReader(stream))
            goal = cells[point-1]
            error = sum(abs(int(rows[-1][axis])/256-(2*coordinate+1))
                        for axis, coordinate in zip(('x_q8','y_q8'), goal))*10
            record.update(route_samples=len(rows), goal_manhattan_error_mm=round(error, 3))
            if len(rows) < 2 or error >= 85:
                record['passed'] = False
                print('FAIL route endpoint', point, error, flush=True)
        print(('PASS' if passed else 'FAIL'), name, record['seconds'], flush=True)
        return record

    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, options.workers)) as pool:
        records = list(pool.map(run, cases))
    current = {p.relative_to(ROOT).as_posix(): digest(p) for p in source_paths}
    unchanged = inputs == current
    previous_path = OUT / 'validation.json'
    if options.only and previous_path.exists():
        previous = json.loads(previous_path.read_text(encoding='utf-8'))
        # Never combine passing results from a different RTL revision.
        if previous.get('hardware_inputs') == inputs:
            names = {r['name'] for r in records}
            records = [r for r in previous['cases'] if r['name'] not in names] + records
    report = dict(hardware_inputs=inputs, source_unchanged=unchanged, cases=records,
                  physical_hardware_tested=False)
    previous_path.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    if not unchanged or any(not record['passed'] for record in records):
        raise SystemExit(1)
    print('PASS merged source regression; no hardware testing', flush=True)


if __name__ == '__main__':
    main()
