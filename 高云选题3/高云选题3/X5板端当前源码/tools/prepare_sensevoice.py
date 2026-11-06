#!/usr/bin/env python3
"""Download reviewed SenseVoice/Silero files. Does not install packages or run audio."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import tempfile
import urllib.request


REVISION = '2365baeacb507f821a0c8120fcee3d484dba7a07'
NAME = 'sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17'
BASE = f'https://huggingface.co/csukuangfj/{NAME}/resolve/{REVISION}/'
ASSETS = (
    (f'models/voice/{NAME}/model.int8.onnx', BASE+'model.int8.onnx', 239233841,
     'c71f0ce00bec95b07744e116345e33d8cbbe08cef896382cf907bf4b51a2cd51'),
    (f'models/voice/{NAME}/tokens.txt', BASE+'tokens.txt', 315894,
     'f449eb28dc567533d7fa59be34e2abca8784f771850c78a47fb731a31429a1dc'),
    (f'models/voice/{NAME}/test_wavs/zh.wav', BASE+'test_wavs/zh.wav', 178988,
     'b77f1794fe374a0ba1ee1dc458bfaf9349496cbbfc32780c50ba3c5a7ad8e373'),
    ('models/voice/silero_vad.onnx',
     'https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.onnx', 643854,
     '9e2449e1087496d8d4caba907f23e0bd3f78d91fa552479bb9c23ac09cbb1fd6'),
    (f'models/voice/{NAME}/LICENSE', BASE+'LICENSE', 71,
     '221c6df10b0931a5629adad671ea48fb7747e034c414b6d2bfa275bc3dd4ea17'),
)
LICENSE_SOURCE = 'https://raw.githubusercontent.com/modelscope/FunASR/main/MODEL_LICENSE'


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def workspace_path(workspace, relative):
    root = Path(workspace).resolve()
    target = root / relative
    if not target.resolve().is_relative_to(root):
        raise ValueError('Destination leaves workspace')
    if target.is_symlink():
        raise ValueError('Refusing symlink destination')
    return target


def verified_download(url, target, size, sha256, *, repair=False, opener=urllib.request.urlopen):
    """An invalid pre-existing file is an error unless repair was explicitly chosen."""
    target = Path(target)
    if target.exists():
        if target.is_file() and target.stat().st_size == size and digest(target) == sha256:
            return 'verified_existing'
        if not repair:
            raise ValueError(f'Existing file failed size/SHA-256 verification: {target}; use --repair to replace after verifying a new download')
        if not target.is_file():
            raise ValueError('Existing destination is not a regular file')
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=target.name+'.', suffix='.download', dir=target.parent)
    temporary = Path(temporary)
    try:
        received = 0
        value = hashlib.sha256()
        request = urllib.request.Request(url, headers={'User-Agent': 'rescue-sensevoice-preparer/1'})
        with os.fdopen(fd, 'wb') as output, opener(request, timeout=90) as response:
            while True:
                block = response.read(1024*1024)
                if not block:
                    break
                received += len(block)
                if received > size:
                    raise ValueError('Download exceeds reviewed file size')
                value.update(block)
                output.write(block)
            output.flush()
            os.fsync(output.fileno())
        if received != size or value.hexdigest() != sha256:
            raise ValueError('Downloaded file failed size/SHA-256 verification; original preserved')
        os.replace(temporary, target)
        return 'downloaded_verified'
    finally:
        temporary.unlink(missing_ok=True)


def snapshot_license(target, *, opener=urllib.request.urlopen):
    """Keep the actual license text and its fetch hash; this moving URL is not pinned."""
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    with opener(LICENSE_SOURCE, timeout=30) as response:
        raw = response.read(128*1024+1)
    if len(raw) > 128*1024 or b'FunASR Model Open Source License Agreement' not in raw:
        raise ValueError('Unexpected model-license response')
    fd, temporary = tempfile.mkstemp(prefix='MODEL_LICENSE.', dir=target.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        Path(temporary).unlink(missing_ok=True)
    return dict(source_url=LICENSE_SOURCE, bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
        pinned=False, note='Fetched license snapshot; model weights follow this separate model license, not the code license.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--repair', action='store_true', help='Replace invalid existing assets only after new bytes verify')
    args = parser.parse_args(argv)
    root = args.workspace.resolve()
    manifest = dict(model='SenseVoice Small INT8', revision=REVISION,
        prepared_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), assets=[],
        runtime_install_command='python -m pip install sherpa-onnx==1.13.8 sherpa-onnx-core==1.13.8',
        runtime_note='Install separately in the X5 environment. This tool installs no packages and opens no microphone.',
        validation_limit='Bundled zh.wav is a public file sample, not acceptance of a user microphone or robot commands.')
    for relative, url, size, sha in ASSETS:
        target = workspace_path(root, relative)
        status = verified_download(url, target, size, sha, repair=args.repair)
        manifest['assets'].append(dict(path=relative, url=url, size=size, sha256=sha, status=status))
    license_path = workspace_path(root, 'vendor/voice_source/sensevoice_MODEL_LICENSE.txt')
    manifest['model_license'] = snapshot_license(license_path)
    manifest['licenses'] = dict(model='FunASR Model Open Source License Agreement', sherpa_code='Apache-2.0', silero='MIT')
    path = workspace_path(root, 'vendor/voice_source/sensevoice_manifest.json')
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
