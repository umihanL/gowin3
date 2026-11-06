"""Restore the verified release preceding the user's photographed request."""
from pathlib import Path
import datetime
import hashlib
import json
import shutil
import stat
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'impl/baseline_navigation_visual_20261027'
EXPECTED = '536e8b1d5c724ed80eeb78a268820be2633a9c452d8d64cd0a9259acfe857dfe'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
manifest = json.loads((BASE / 'manifest.json').read_text(encoding='utf-8'))
assert all(sha(BASE / n) == h for n, h in manifest.items()), 'Snapshot integrity failure'
assert manifest['impl/pnr/ov5640.fs'] == EXPECTED, 'Wrong requested release'
prior = json.loads((BASE / 'impl/pnr/ov5640_current_audit.json').read_text(encoding='utf-8'))
assert prior['sha256'] == EXPECTED and prior['timing_passed']
assert not any(prior['violated_endpoints'].values())

# Archive everything that will be replaced before making any restoration.
stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
resume = len(sys.argv) == 3 and sys.argv[1] == '--resume'
archive = (ROOT/sys.argv[2]).resolve() if resume else ROOT/f'impl/before_requested_rollback_{stamp}'
assert archive.resolve().is_relative_to((ROOT/'impl').resolve())
assert archive.name.startswith('before_requested_rollback_')
current_src = {p.relative_to(ROOT).as_posix() for p in (ROOT/'src').rglob('*') if p.is_file()}
target_src = {n for n in manifest if n.startswith('src/')}
extra_src = sorted(current_src - target_src)
backup_paths = set(manifest) | current_src | {'README.md', 'docs/resource_usage.md'}
backup_paths.update(p.relative_to(ROOT).as_posix() for folder in ('impl/pnr', 'impl/gwsynthesis')
                    for p in (ROOT/folder).iterdir() if p.is_file())
backup = json.loads((archive/'manifest.json').read_text(encoding='utf-8')) if resume else {}
if not resume:
    assert not archive.exists()
    for n in sorted(backup_paths):
        source = ROOT/n
        if source.is_file():
            destination = archive/n
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
            backup[n] = sha(source)
assert all(sha(archive/n) == h for n, h in backup.items()), 'Current-version archive failed'
(archive/'manifest.json').write_text(json.dumps(backup, indent=2)+'\n', encoding='utf-8')

for n in manifest:
    destination = ROOT/n
    assert destination.resolve().is_relative_to(ROOT.resolve())
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        destination.chmod(destination.stat().st_mode | stat.S_IWRITE)
    shutil.copy2(BASE/n, destination)

# Files introduced by the later navigation implementation must leave src too.
# Their verified copies remain in the archive; historical tests are retained.
for n in extra_src:
    source = (ROOT/n).resolve()
    assert source.is_relative_to((ROOT/'src').resolve())
    assert sha(source) == backup[n] == sha(archive/n)
    source.unlink()

assert all(sha(ROOT/n) == h for n, h in manifest.items()), 'Restored files differ from snapshot'
assert {p.relative_to(ROOT).as_posix() for p in (ROOT/'src').rglob('*') if p.is_file()} == target_src

release = 'impl/pnr/ov5640_restored_host_navigation_20261027.fs'
shutil.copy2(BASE/'impl/pnr/ov5640.fs', ROOT/release)
assert sha(ROOT/release) == sha(ROOT/'impl/pnr/ov5640.fs') == EXPECTED
record = dict(
    action='restore_release_before_photographed_navigation_request',
    restored_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    release='host_navigation_20261027', baseline=BASE.relative_to(ROOT).as_posix(),
    snapshot_files_verified=len(manifest), source_files_verified=len(target_src),
    sha256=EXPECTED, bitstream='impl/pnr/ov5640.fs', release_bitstream=release,
    prior_current_sha256=backup['impl/pnr/ov5640.fs'],
    removed_later_source_files=extra_src,
    archive=archive.relative_to(ROOT).as_posix(),
    rebuilt=False, flashed=False,
    original_generated_utc=prior['generated_utc'],
    original_violated_endpoints=prior['violated_endpoints'],
    original_min_slack_ns=prior['min_slack_ns'],
)
download = json.loads((ROOT/'impl/pnr/ov5640_download.json').read_text(encoding='utf-8'))
download.update(release_bitstream=release, restored_at=record['restored_at'],
                restoration_record='docs/navigation_rollback_20261027.json')
(ROOT/'impl/pnr/ov5640_download.json').write_text(json.dumps(download, indent=2)+'\n', encoding='utf-8')
(ROOT/'docs/navigation_rollback_20261027.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')

notes=f'''# 恢复截图需求之前的版本

已按用户要求恢复 **host_navigation_20261027**，即“地图指定右转却左转、起点前后偏移影响入弯、S 弯提前转向”这条截图需求提出之前的版本。

- 源码、工程配置、综合网表、布局布线检查点和默认位流均从完整原始快照恢复。
- 共核对 {len(manifest)} 个快照文件、{len(target_src)} 个源文件；移出后续新增的 `vehicle_visual_localizer.v`，源码树与目标快照一致。
- 恢复包含 E14 TX / E13 RX 通信框架的版本；该需求之后的导航修改不再生效。
- 默认下载文件：`impl/pnr/ov5640.fs`。
- 独立同内容文件：`{release}`。
- SHA-256：`{EXPECTED}`。
- 原始生成时间：2026-10-27 01:39:26（UTC+8）；直接恢复原始位流，没有重新编译，没有烧录。
- 原构建 Setup/Hold 违规端点：0/0。当前恢复是文件一致性核验，不表示重新进行了实车测试。
- 恢复前的源码与构建输出已保存至 `{record['archive']}/`。

原发布验证记录：`docs/host_navigation_20261027_validation.json`。
恢复核验记录：`docs/navigation_rollback_20261027.json`。
'''
(ROOT/'docs/navigation_rollback_20261027.md').write_text(notes, encoding='utf-8')

resources=prior['resources']
rows=['# 当前构建资源：已恢复的旧版本','',
      '当前版本为截图需求之前的 `host_navigation_20261027`。',
      f'默认位流 `impl/pnr/ov5640.fs`；独立副本 `{release}`。',
      f'SHA-256：`{EXPECTED}`。','',
      '| 资源 | 使用 / 总量 | 使用率 |','| --- | ---: | ---: |']
for name, value in resources.items():
    rows.append(f"| {name} | {value['used']:g} / {value['total']:g} | {value['percent']:.2f}% |")
rows += ['', '原构建 Setup/Hold 违规端点 0/0；本次直接恢复，没有重新构建或烧录。',
         '说明见 [恢复记录](navigation_rollback_20261027.md)。此前其他版本文档仅为历史记录。']
(ROOT/'docs/resource_usage.md').write_text('\n'.join(rows)+'\n', encoding='utf-8')

readme=ROOT/'README.md'
text=readme.read_text(encoding='utf-8')
at=text.index('\n\n')+2
notice=f'''**当前已恢复旧版：** 按用户截图，恢复到该条导航需求提出之前的 `host_navigation_20261027`。源码与原始位流已同步恢复，SHA-256 前缀 `{EXPECTED[:16]}`。下载默认 `impl/pnr/ov5640.fs` 或 `{release}`；未重新编译、未烧录。详见 [恢复记录](docs/navigation_rollback_20261027.md)。以下新版说明均为历史记录，不代表当前运行版本。

'''
readme.write_text(text[:at]+notice+text[at:], encoding='utf-8')

for n, h in manifest.items():
    if n != 'impl/pnr/ov5640_download.json':
        assert sha(ROOT/n) == h, n
assert sha(ROOT/'impl/pnr/ov5640.fs') == sha(ROOT/release) == EXPECTED
print(json.dumps(record, indent=2))
