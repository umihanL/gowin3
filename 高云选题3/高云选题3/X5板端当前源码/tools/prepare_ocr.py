"""Fetch the pinned RapidOCR wheel, keep ONNX weights and upstream license.

This downloads pretrained OCR weights only; it never starts training.
Run on the desktop or board. Install rapidocr-onnxruntime==1.4.4 separately.
"""
from pathlib import Path
import hashlib
import json
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
URL = 'https://files.pythonhosted.org/packages/ba/12/1e5497183bdbe782dbb91bad1d0d2297dba4d2831b2652657f7517bfc6df/rapidocr_onnxruntime-1.4.4-py3-none-any.whl'
SHA256 = '971d7d5f223a7a808662229df1ef69893809d8457d834e6373d3854bc1782cbf'
LICENSES = [
    ('RapidOCR-LICENSE', 'https://raw.githubusercontent.com/RapidAI/RapidOCR/86ae3f5079df3422c1829cd84baf19bc8a7a9453/LICENSE',
     '3e0af25fdd06aa9586ae97adb00ea927ebe5a3805ac77d2d3a81ce5f55693333'),
    ('PaddleOCR-LICENSE', 'https://raw.githubusercontent.com/PaddlePaddle/PaddleOCR/8cce9b6fd7ccb50226d0c38f94054d81c29b8184/LICENSE',
     'ff6311c56c1b22c9209c96cc8afc16da91c24ee55a15e14ebc46fe1960dbd856'),
]


def main():
    folder = ROOT / 'models' / 'ocr'
    vendor = ROOT / 'vendor' / 'ocr_source'
    folder.mkdir(parents=True, exist_ok=True)
    vendor.mkdir(parents=True, exist_ok=True)
    wheel = vendor / 'rapidocr_onnxruntime-1.4.4-py3-none-any.whl'
    if not wheel.exists():
        urllib.request.urlretrieve(URL, wheel)
    actual = hashlib.sha256(wheel.read_bytes()).hexdigest()
    if actual != SHA256:
        raise RuntimeError('Pinned OCR wheel SHA256 mismatch')
    models = []
    with zipfile.ZipFile(wheel) as archive:
        (vendor / 'package-METADATA.txt').write_bytes(
            archive.read('rapidocr_onnxruntime-1.4.4.dist-info/METADATA'))
        for member in archive.namelist():
            if member.endswith('.onnx'):
                payload = archive.read(member)
                target = folder / Path(member).name
                target.write_bytes(payload)
                models.append({'file': target.name, 'bytes': len(payload),
                               'sha256': hashlib.sha256(payload).hexdigest()})
            if Path(member).name.upper() in ('LICENSE', 'LICENSE.TXT', 'NOTICE'):
                (vendor / Path(member).name).write_bytes(archive.read(member))
            # Allows desktop verification without modifying the training environment.
            if member.startswith('rapidocr_onnxruntime/') and not member.endswith('/'):
                target = ROOT / 'vendor' / 'ocr_python' / member
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(archive.read(member))
    license_records = []
    for name, url, digest in LICENSES:
        target = vendor / name
        if not target.exists():
            urllib.request.urlretrieve(url, target)
        if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            raise RuntimeError('Pinned OCR license SHA256 mismatch: ' + name)
        license_records.append({'file': name, 'url': url, 'sha256': digest})
    (vendor / 'licenses.json').write_text(json.dumps(license_records, indent=2), encoding='utf-8')
    manifest = {'package': 'rapidocr-onnxruntime', 'version': '1.4.4',
                'source_url': URL, 'wheel_sha256': SHA256,
                'execution': 'ONNX Runtime CPUExecutionProvider (ARM64 CPU on X5); not BPU',
                'license_manifest': 'vendor/ocr_source/licenses.json',
                'models': models}
    (folder / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
