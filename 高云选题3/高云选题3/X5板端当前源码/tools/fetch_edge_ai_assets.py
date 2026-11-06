"""Download pinned upstream model/runtime sources; no model is executed here."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
MODEL_REVISION = '9217f5db79a29953eb74d5343926648285ec7e67'
LLAMA_COMMIT = 'aa94f2086147cd4bfbcff818ce998dffce5fcb39'
MODEL_SHA256 = '74a4da8c9fdbcd15bd1f6d01d621410d31c6fc00986f5eb687824e7b93d7a9db'
ASSETS = [
    ('models/llm/qwen2.5-0.5b-instruct-q4_k_m.gguf',
     f'https://huggingface.co/Qwen/Qwen2.5-0.5B-Instruct-GGUF/resolve/{MODEL_REVISION}/qwen2.5-0.5b-instruct-q4_k_m.gguf', MODEL_SHA256),
    ('models/llm/LICENSE',
     f'https://huggingface.co/Qwen/Qwen2.5-0.5B-Instruct-GGUF/raw/{MODEL_REVISION}/LICENSE', None),
    ('models/llm/UPSTREAM_MODEL_CARD.md',
     f'https://huggingface.co/Qwen/Qwen2.5-0.5B-Instruct-GGUF/raw/{MODEL_REVISION}/README.md', None),
    ('vendor/llama_cpp/llama_cpp_source.tar.gz',
     f'https://codeload.github.com/ggml-org/llama.cpp/tar.gz/{LLAMA_COMMIT}', None),
]


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def download(spec):
    relative, url, expected = spec
    path = ROOT/relative
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file() and (expected is None or digest(path) == expected):
        print('Already downloaded:', relative, flush=True)
    else:
        temp = path.with_suffix(path.suffix+'.part')
        for attempt in range(3):
            try:
                request = urllib.request.Request(url+'?download=true&request_time='+str(time.time_ns()),
                    headers={'User-Agent': 'AckermannRescueSim-local-deployment/1.0'})
                received = 0
                mark = 0
                with urllib.request.urlopen(request, timeout=45) as response, temp.open('wb') as output:
                    while True:
                        block = response.read(1024*1024)
                        if not block:
                            break
                        output.write(block)
                        received += len(block)
                        if received//(64*1024*1024) > mark:
                            mark = received//(64*1024*1024)
                            print(path.name, received//(1024*1024), 'MiB', flush=True)
                actual = digest(temp)
                if expected and actual != expected:
                    raise ValueError('SHA256 mismatch: '+relative)
                temp.replace(path)
                print('Verified:', relative, path.stat().st_size, actual, flush=True)
                break
            except Exception as exc:
                print('Download attempt', attempt+1, relative, type(exc).__name__, str(exc), flush=True)
                if attempt == 2:
                    raise
    return {'path': relative, 'url': url, 'size': path.stat().st_size, 'sha256': digest(path)}


if __name__ == '__main__':
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        assets = list(pool.map(download, ASSETS))
    (ROOT/'models/llm/provenance.json').write_text(json.dumps({
        'model': 'Qwen2.5-0.5B-Instruct', 'quantization': 'Q4_K_M',
        'license': 'Apache-2.0', 'model_revision': MODEL_REVISION,
        'runtime_source_commit': LLAMA_COMMIT, 'assets': assets,
        'hardware_backend': 'CPU; not an X5 BPU-converted model',
    }, ensure_ascii=False, indent=2), encoding='utf-8')
