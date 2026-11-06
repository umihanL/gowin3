"""Download only the exact APT packages specified by board-authenticated metadata.

All TLS connections use certificate validation. HTTP fallback is restricted to the
original metadata URL; downloaded bytes are accepted only after size and SHA256.
No installation or remote commands are performed by this script.
"""
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parent
DEST = ROOT / 'debs'


def candidates(uri):
    urls = [uri.replace('http://', 'https://', 1), uri]
    if uri.startswith('http://packages.ros.org/ros2/ubuntu/'):
        suffix = uri.split('/ros2/ubuntu/', 1)[1]
        urls += ['https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/' + suffix,
                 'https://mirrors.ustc.edu.cn/ros2/ubuntu/' + suffix]
    if uri.startswith('http://ports.ubuntu.com/pool/'):
        suffix = uri.split('/pool/', 1)[1]
        urls.insert(1, 'https://ports.ubuntu.com/ubuntu-ports/pool/' + suffix)
        urls += ['https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/pool/' + suffix,
                 'https://mirrors.ustc.edu.cn/ubuntu-ports/pool/' + suffix]
    return list(dict.fromkeys(urls))


def valid(data, item):
    return len(data) == item['size'] and hashlib.sha256(data).hexdigest() == item['sha256']


def download(item):
    filename = item['filename']
    if Path(filename).name != filename or '/' in filename or '\\' in filename:
        raise ValueError('Unsafe package filename')
    target = DEST / filename
    result = dict(item, attempts=[], status='failed')
    if target.is_file() and valid(target.read_bytes(), item):
        return dict(result, status='verified_existing')
    for url in candidates(item['uri']):
        attempt = {'url': url}
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'RDKX5-Offline-Deployment/1.0'})
            with urllib.request.urlopen(req, timeout=35) as response:
                attempt['final_url'] = response.url
                attempt['http_status'] = response.status
                data = response.read(item['size'] + 1)
            attempt['size'] = len(data)
            attempt['sha256'] = hashlib.sha256(data).hexdigest()
            if not valid(data, item):
                raise ValueError('Downloaded size or SHA256 differs from authenticated APT metadata')
            temp = target.with_suffix('.deb.partial')
            temp.write_bytes(data)
            temp.replace(target)
            result.update(status='verified_download', downloaded_from=url)
            result['attempts'].append(attempt)
            return result
        except Exception as exc:
            attempt['error'] = '{}: {}'.format(type(exc).__name__, exc)
            result['attempts'].append(attempt)
    return result


def main():
    DEST.mkdir(exist_ok=True)
    entries = json.loads((ROOT / 'apt_offline_manifest.json').read_text(encoding='utf-8-sig'))
    results = []
    with ThreadPoolExecutor(max_workers=6) as pool:
        futures = [pool.submit(download, item) for item in entries]
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            print(result['status'], result['filename'], flush=True)
    results.sort(key=lambda x: x['filename'])
    report = {'generated_utc': datetime.now(timezone.utc).isoformat(),
              'total': len(results), 'verified': sum(r['status'].startswith('verified') for r in results),
              'bytes_verified': sum(r['size'] for r in results if r['status'].startswith('verified')),
              'packages': results}
    (ROOT / 'apt_offline_download_results.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'packages'}), flush=True)
    return 0 if report['verified'] == report['total'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
