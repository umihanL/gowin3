"""Run each local Python test module in isolation; never enable board test flags."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--only', nargs='*', help='Test module names; default all test_*.py')
    parser.add_argument('--workers', type=int, choices=range(1, 5), default=3)
    args = parser.parse_args()
    modules = args.only or sorted(p.stem for p in (ROOT/'tests').glob('test_*.py'))
    if any(not re.fullmatch(r'test_[a-z0-9_]+', m) or not (ROOT/'tests'/(m+'.py')).is_file() for m in modules):
        parser.error('Only existing local test modules are accepted')
    args.output.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONUTF8='1',
                       PYTHONPATH=str(ROOT/'src/rescue_vision'), RESCUE_RUN_ROS_TEST='0',
                       RESCUE_RUN_REAL_ASR_TEST='0', ROS_DOMAIN_ID='83', ROS_LOCALHOST_ONLY='1')
    def hashes():
        files = [*ROOT.glob('src/rescue_vision/rescue_vision/*.py'), *ROOT.glob('tools/*.py'),
                 *ROOT.glob('src/rescue_vision/launch/*.py'), *ROOT.glob('config/*.json'),
                 *(p for p in (ROOT/'web').rglob('*') if p.is_file()),
                 *(ROOT/'tests'/(m+'.py') for m in modules)]
        return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    before = hashes()
    started = time.monotonic()
    def run(module):
        begin = time.monotonic()
        command = [sys.executable, '-X', 'utf8', '-m', 'unittest', module, '-v']
        try:
            result = subprocess.run(command, cwd=ROOT/'tests', env=environment,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=240)
            text = result.stdout.decode('utf-8', errors='replace')
            exit_code = result.returncode
        except subprocess.TimeoutExpired as exc:
            text = (exc.stdout or b'').decode('utf-8', errors='replace')+'\nTIMEOUT after 240 seconds\n'
            exit_code = 124
        (args.output/(module+'.log')).write_text(text, encoding='utf-8')
        total = re.search(r'Ran (\d+) tests? in ', text)
        skip_summary = re.search(r'OK \(skipped=(\d+)\)', text)
        skipped = int(skip_summary[1]) if skip_summary else sum('... skipped ' in line for line in text.splitlines())
        # unittest places a test's docstring on its own line before the status.
        # Count that status too; class/module fixture skips are not test cases.
        skipped_cases = sum('... skipped ' in line and
                            not re.match(r'^(setUpClass|setUpModule)\b', line)
                            for line in text.splitlines())
        return dict(module=module, passed=exit_code == 0, exit_code=exit_code,
                    tests=int(total[1]) if total else 0, skipped=skipped,
                    skipped_test_cases=skipped_cases, skipped_fixture_suites=skipped-skipped_cases,
                    executed_test_cases=(int(total[1]) if total else 0)-skipped_cases,
                    seconds=round(time.monotonic()-begin, 3), command=command,
                    log=module+'.log')
    rows = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(run, module) for module in modules]):
            row = future.result(); rows.append(row)
            print(('PASS' if row['passed'] else 'FAIL')+' '+row['module']+
                  ' tests='+str(row['tests'])+' skipped='+str(row['skipped']), flush=True)
    report = dict(generated_utc=datetime.now(timezone.utc).isoformat(),
                  passed=all(row['passed'] for row in rows), source_unchanged=before == hashes(),
                  seconds=round(time.monotonic()-started, 3), modules=len(rows),
                  tests=sum(row['tests'] for row in rows),
                  skipped=sum(row['skipped'] for row in rows),
                  skipped_test_cases=sum(row['skipped_test_cases'] for row in rows),
                  skipped_fixture_suites=sum(row['skipped_fixture_suites'] for row in rows),
                  executed_test_cases=sum(row['executed_test_cases'] for row in rows),
                  failed_modules=[row['module'] for row in rows if not row['passed']],
                  hardware_tested=False, real_asr_test_enabled=False, ros_board_tests_enabled=False,
                  source_sha256=before, cases=sorted(rows, key=lambda row: row['module']))
    (args.output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items() if key not in ('source_sha256', 'cases')}), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
