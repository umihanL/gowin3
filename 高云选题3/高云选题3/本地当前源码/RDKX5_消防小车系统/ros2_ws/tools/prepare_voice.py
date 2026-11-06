"""Download the pinned official Vosk Chinese model once; runtime stays offline."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import urllib.request
import zipfile

URL = 'https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip'
NAME = 'vosk-model-small-cn-0.22'
EXPECTED_SHA256 = '3af8b0e7e0f835ae9d414ce5df580237a3cfb08d586c9fbbb0f7ff29ad5b14ba'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    output = workspace / 'models/voice'
    output.mkdir(parents=True, exist_ok=True)
    archive = output / (NAME + '.zip')
    if not archive.is_file():
        temporary = output / (NAME + '.download')
        with urllib.request.urlopen(URL, timeout=90) as response, temporary.open('wb') as target:
            shutil.copyfileobj(response, target)
        temporary.replace(archive)
    if digest(archive) != EXPECTED_SHA256:
        raise RuntimeError('Model archive differs from the reviewed official download; refusing extraction')
    with zipfile.ZipFile(archive) as package:
        for item in package.infolist():
            destination = (output / item.filename).resolve()
            if not destination.is_relative_to(output.resolve()):
                raise RuntimeError('Archive contains unsafe path')
            if not item.filename.startswith(NAME + '/'):
                raise RuntimeError('Unexpected model layout')
        package.extractall(output)
    if not (output / NAME / 'am/final.mdl').is_file():
        raise RuntimeError('Required acoustic model absent')
    manifest = dict(model=NAME, url=URL, license='Apache-2.0',
                    license_source='https://alphacephei.com/vosk/models',
                    downloaded_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    archive_bytes=archive.stat().st_size, sha256=digest(archive),
                    runtime='vosk==0.3.45', platforms=['Linux aarch64', 'Linux x86_64', 'Windows x86_64'],
                    model_files={str(p.relative_to(output / NAME)).replace('\\', '/'): digest(p)
                                 for p in sorted((output / NAME).rglob('*')) if p.is_file()},
                    note='SHA-256 is a local integrity record; official page supplies no published checksum.')
    path = workspace / 'vendor/voice_source'
    path.mkdir(parents=True, exist_ok=True)
    (path / 'model_manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({k: manifest[k] for k in ('model', 'archive_bytes', 'sha256')}, indent=2))


if __name__ == '__main__':
    main()
