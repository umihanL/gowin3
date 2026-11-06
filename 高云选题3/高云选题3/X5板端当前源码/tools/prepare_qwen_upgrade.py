"""Fetch pinned Qwen candidates with resumable, verified downloads and provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
REVISION = '91cad51170dc346986eccefdc2dd33a9da36ead9'
BASE = 'https://huggingface.co/Qwen/Qwen2.5-1.5B-Instruct-GGUF/resolve/' + REVISION + '/'
NAME = 'qwen2.5-1.5b-instruct-q4_k_m.gguf'
SIZE = 1117320736
SHA256 = '6a1a2eb6d15622bf3c96857206351ba97e1af16c30d7a74ee38970e434e9407e'


def digest(path):
    sha = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4*1024*1024), b''):
            sha.update(chunk)
    return sha.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=('qwen25-15b', 'qwen3-06b', 'qwen3-17b', 'qwen3-17b-q4_0'), default='qwen3-17b-q4_0')
    choice = parser.parse_args().model
    global REVISION, BASE, NAME, SIZE, SHA256
    if choice == 'qwen3-06b':
        REVISION = '23749fefcc72300e3a2ad315e1317431b06b590a'
        BASE = 'https://huggingface.co/Qwen/Qwen3-0.6B-GGUF/resolve/' + REVISION + '/'
        NAME = 'Qwen3-0.6B-Q8_0.gguf'
        SIZE = 639446688
        SHA256 = '9465e63a22add5354d9bb4b99e90117043c7124007664907259bd16d043bb031'
    elif choice.startswith('qwen3-17b'):
        REVISION = 'd7f544eead698dbd1f15126ef60b45a1e1933222'
        BASE = 'https://huggingface.co/unsloth/Qwen3-1.7B-GGUF/resolve/' + REVISION + '/'
        NAME = 'Qwen3-1.7B-Q4_K_M.gguf'
        SIZE = 1107409472
        SHA256 = 'b139949c5bd74937ad8ed8c8cf3d9ffb1e99c866c823204dc42c0d91fa181897'
        if choice == 'qwen3-17b-q4_0':
            NAME = 'Qwen3-1.7B-Q4_0.gguf'
            SIZE = 1056782912
            SHA256 = 'c876f159707a4e4f70e045106c69db15bfc935a4981706fd4f65c6e7ea1e81c5'
    directory = ROOT / 'models/llm'
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / NAME
    partial = target.with_suffix('.gguf.part')
    if target.exists():
        if target.stat().st_size != SIZE or digest(target) != SHA256:
            raise ValueError('Existing model failed verification; preserved unchanged')
    else:
        for attempt in range(8):
            try:
                offset = partial.stat().st_size if partial.exists() else 0
                if offset == SIZE:
                    break
                if offset > SIZE:
                    raise ValueError('Partial file exceeds reviewed size')
                headers = {'User-Agent': 'Rescue-Qwen-Preparation/1'}
                if offset:
                    headers['Range'] = f'bytes={offset}-'
                req = urllib.request.Request(BASE+NAME+'?download=true', headers=headers)
                with urllib.request.urlopen(req, timeout=45) as response:
                    if offset and (response.status != 206 or not response.headers.get('Content-Range', '').startswith(f'bytes {offset}-')):
                        raise ValueError('Server did not confirm requested resume range')
                    with partial.open('ab' if offset else 'wb') as out:
                        logged = offset
                        while chunk := response.read(1024*1024):
                            offset += len(chunk)
                            if offset > SIZE:
                                raise ValueError('Download exceeds reviewed size')
                            out.write(chunk)
                            if offset-logged >= 32*1024*1024:
                                print(f'{offset}/{SIZE} bytes ({offset/SIZE:.1%})', flush=True)
                                logged = offset
                if offset == SIZE:
                    break
            except (OSError, ValueError) as exc:
                print(f'Attempt {attempt+1}: {type(exc).__name__}: {exc}', flush=True)
                if attempt == 7:
                    raise
                time.sleep(2)
        if partial.stat().st_size != SIZE or digest(partial) != SHA256:
            raise ValueError('Downloaded model failed SHA256 verification')
        partial.replace(target)
    source = ROOT / 'vendor/llm_source' / choice.replace('-', '_')
    source.mkdir(parents=True, exist_ok=True)
    for name in ('README.md', 'LICENSE'):
        url = BASE+name
        if choice == 'qwen3-06b' and name == 'LICENSE':
            url = 'https://huggingface.co/Qwen/Qwen3-0.6B/resolve/c1899de289a04d12100db370d81485cdf75e47ca/LICENSE'
        elif choice.startswith('qwen3-17b') and name == 'LICENSE':
            url = 'https://huggingface.co/Qwen/Qwen3-1.7B-GGUF/resolve/90862c4b9d2787eaed51d12237eafdfe7c5f6077/LICENSE'
        with urllib.request.urlopen(url, timeout=30) as response:
            raw = response.read(100000)
        (source/name).write_bytes(raw)
    repository = {'qwen25-15b': 'Qwen/Qwen2.5-1.5B-Instruct-GGUF',
        'qwen3-06b': 'Qwen/Qwen3-0.6B-GGUF', 'qwen3-17b': 'unsloth/Qwen3-1.7B-GGUF',
        'qwen3-17b-q4_0': 'unsloth/Qwen3-1.7B-GGUF'}[choice]
    manifest = dict(repo=repository, revision=REVISION,
        model_file=NAME, model_bytes=SIZE, sha256=SHA256, source_url=BASE+NAME,
        license='Apache-2.0', quantization='Q8_0' if choice == 'qwen3-06b' else 'Q4_0' if choice.endswith('q4_0') else 'Q4_K_M', role='local text generation; no direct actuator control')
    if choice.startswith('qwen3-17b'):
        manifest.update(base_model='Qwen/Qwen3-1.7B', quantizer='Unsloth (community GGUF conversion)')
    (source/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(json.dumps(manifest), flush=True)


if __name__ == '__main__':
    main()
