from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import json,zipfile,hashlib,urllib.request,gzip,io
out=Path(r'D:\工程仿真\FPGA赛道_小车仿真\RDKX5_消防小车系统\board_setup\vsix')
out.mkdir(exist_ok=True)
items=[('python','2026.8.0'),('vscode-pylance','2026.4.1'),('debugpy','2026.6.0'),('vscode-python-envs','1.38.0')]
def one(item):
 name,version=item
 url=f'https://marketplace.visualstudio.com/_apis/public/gallery/publishers/ms-python/vsextensions/{name}/{version}/vspackage?targetPlatform=linux-arm64'
 if name=='vscode-pylance':
  # Pylance is a universal JavaScript extension; this version has no platform VSIX.
  url=url.split('?')[0]
 dst=out/f'ms-python.{name}-{version}.vsix'
 if dst.exists():
  payload=dst.read_bytes()
 else:
  request=urllib.request.Request(url,headers={'Accept-Encoding':'identity','User-Agent':'RDKX5-VSCode-Setup/1.0'})
  # urllib's default HTTPS handler verifies the official endpoint certificate.
  with urllib.request.urlopen(request,timeout=60) as response:
   payload=response.read()
 if payload[:2]==b'\x1f\x8b':
  payload=gzip.decompress(payload)
 if not zipfile.is_zipfile(io.BytesIO(payload)):
  raise ValueError(f'{name}: official response is not a ZIP VSIX (magic {payload[:8].hex()})')
 with zipfile.ZipFile(io.BytesIO(payload)) as z:
  assert z.testzip() is None
  meta=json.loads(z.read('extension/package.json'))
  assert meta['name']==name and meta['version']==version
  manifest=z.read('extension.vsixmanifest').decode('utf-8')
  if name in ('debugpy','vscode-python-envs'):assert 'linux-arm64' in manifest
 dst.write_bytes(payload)
 return dict(extension='ms-python.'+name,version=version,bytes=dst.stat().st_size,sha256=hashlib.sha256(dst.read_bytes()).hexdigest(),url=url)
with ThreadPoolExecutor(max_workers=4) as executor:
 results=list(executor.map(one,items))
(out/'manifest.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(results,indent=2))
