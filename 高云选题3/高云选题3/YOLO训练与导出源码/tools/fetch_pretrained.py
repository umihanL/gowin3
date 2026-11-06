"""Download the official YOLO11n Detect COCO initialization, not trained rescue weights."""
from pathlib import Path
import hashlib
import json
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
URL = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt'


def main():
    target = ROOT / 'weights/yolo11n.pt'
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise SystemExit('Pretrained checkpoint already exists; not overwritten.')
    temporary = target.with_suffix('.pt.download')
    request = urllib.request.Request(URL, headers={'User-Agent': 'RescueDatasetSetup/1.0'})
    with urllib.request.urlopen(request, timeout=60) as source, temporary.open('wb') as out:
        while chunk := source.read(1024 * 1024):
            out.write(chunk)
    # Inspect the archive without executing pickle/checkpoint code.
    if not zipfile.is_zipfile(temporary) or temporary.stat().st_size < 1000000:
        raise ValueError('Download is not a plausible PyTorch checkpoint archive')
    temporary.rename(target)
    record = {'file': 'yolo11n.pt', 'source_url': URL,
              'sha256': hashlib.sha256(target.read_bytes()).hexdigest(), 'bytes': target.stat().st_size,
              'purpose': 'Official COCO pretrained initialization for YOLO11n Detect. NOT trained on the rescue dataset.',
              'verification': 'HTTPS download and ZIP signature; not loaded/executed because PyTorch is not installed.'}
    (target.parent / 'PRETRAINED_SOURCE.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
    print(json.dumps(record))


if __name__ == '__main__':
    main()
