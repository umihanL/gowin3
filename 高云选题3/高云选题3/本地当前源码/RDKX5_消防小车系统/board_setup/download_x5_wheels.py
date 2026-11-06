"""Download only; never installs into the host interpreter or board."""
from pathlib import Path
import subprocess
import sys
import hashlib
import json
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

ROOT = Path(__file__).resolve().parent
DEST = ROOT / 'wheels'
DEST.mkdir(exist_ok=True)
TARGET = ['--platform', 'manylinux_2_28_aarch64', '--platform', 'manylinux_2_27_aarch64',
          '--platform', 'manylinux2014_aarch64', '--platform', 'manylinux_2_17_aarch64',
          '--platform', 'linux_aarch64', '--python-version', '310', '--implementation', 'cp',
          '--abi', 'cp310', '--only-binary=:all:']

def run(label, args):
    command = [sys.executable, '-m', 'pip', *args]
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (ROOT / (label + '.log')).write_bytes(result.stdout)
    print(label, 'exit', result.returncode, flush=True)
    if result.returncode:
        print(result.stdout.decode('utf-8', errors='replace')[-6000:])
    return result.returncode

if __name__ == '__main__':
    phase = sys.argv[1] if len(sys.argv) > 1 else 'main'
    common = ['download', '--index-url', 'https://pypi.org/simple', '--dest', str(DEST),
              '--constraint', str(ROOT / 'wheel-constraints.txt'), *TARGET]
    if phase == 'main':
        code = run('download_arm64_dependencies', common + [
            'onnxruntime==1.20.1', 'pyclipper==1.3.0.post6', 'Shapely>=1.8,<3',
            'PyYAML>=6,<7', 'six>=1.16', 'Pillow>=9,<12', 'sounddevice>=0.4,<0.6',
            'cffi>=1.0', 'requests', 'tqdm', 'websockets'])
    elif phase == 'models':
        code = run('download_arm64_model_adapters', common + ['--no-deps',
            'rapidocr-onnxruntime==1.4.4', 'vosk==0.3.45'])
    elif phase == 'srt':
        code = run('build_srt_wheel', ['wheel', '--use-pep517', '--index-url', 'https://pypi.org/simple',
                   '--wheel-dir', str(DEST), '--no-deps', 'srt==3.5.3'])
    elif phase == 'manifest':
        files = []
        for file in sorted(DEST.glob('*.whl')):
            if not (file.name.endswith('-any.whl') or 'aarch64' in file.name):
                raise SystemExit('Non-ARM64 wheel: ' + file.name)
            files.append({'file': file.name, 'bytes': file.stat().st_size,
                          'sha256': hashlib.sha256(file.read_bytes()).hexdigest()})
        (ROOT / 'wheels_manifest.json').write_text(json.dumps({
            'target': 'CPython 3.10 Linux aarch64 Ubuntu 22.04',
            'tls_verification': True, 'files': files}, indent=2), encoding='utf-8')
        (ROOT / 'wheels_SHA256SUMS.txt').write_text(''.join(
            f"{item['sha256']}  {item['file']}\n" for item in files), encoding='utf-8')
        print(len(files), 'ARM64/any wheels verified')
        code = 0
    else:
        raise SystemExit('Unknown phase')
    raise SystemExit(code)
