"""Inspect wheel contents, target ABI and dependency closure without installing."""
from pathlib import Path
from email.parser import BytesParser
import hashlib
import json
import zipfile
from packaging.utils import canonicalize_name, parse_wheel_filename
from packaging.tags import cpython_tags, compatible_tags
from packaging.markers import default_environment
from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet

ROOT = Path(__file__).resolve().parent
PLATFORMS = ['manylinux_2_28_aarch64', 'manylinux_2_27_aarch64',
             'manylinux_2_17_aarch64', 'manylinux2014_aarch64', 'linux_aarch64']
TAGS = set(cpython_tags((3, 10), abis=['cp310'], platforms=PLATFORMS))
TAGS.update(compatible_tags((3, 10), interpreter='cp310', platforms=PLATFORMS))
ENV = default_environment()
ENV.update(python_version='3.10', python_full_version='3.10.12', sys_platform='linux',
           os_name='posix', platform_machine='aarch64', platform_system='Linux',
           implementation_name='cpython', platform_python_implementation='CPython', extra='')

files, errors, available, needs = [], [], {}, []
for path in sorted((ROOT / 'wheels').glob('*.whl')):
    name, version, _, tags = parse_wheel_filename(path.name)
    with zipfile.ZipFile(path) as archive:
        corrupt = archive.testzip()
        if corrupt:
            errors.append(f'{path.name}: CRC failure at {corrupt}')
        metadata = BytesParser().parsebytes(archive.read(next(
            name for name in archive.namelist() if name.endswith('.dist-info/METADATA'))))
    if not TAGS.intersection(tags):
        errors.append(f'{path.name}: incompatible target ABI/platform')
    python_req = metadata.get('Requires-Python')
    if python_req and '3.10.12' not in SpecifierSet(python_req):
        errors.append(f'{path.name}: Python 3.10 unsupported: {python_req}')
    available[canonicalize_name(name)] = str(version)
    needs.extend((str(name), Requirement(req)) for req in metadata.get_all('Requires-Dist', []))
    files.append({'filename': path.name, 'name': str(name), 'version': str(version),
                  'bytes': path.stat().st_size,
                  'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})

# The board's existing vendor OpenCV is intentionally reused. Its import was
# verified on the target; this does not fabricate pip distribution metadata.
preinstalled = {'opencv-python': '4.11.0'}
effective = {**preinstalled, **available}
for parent, requirement in needs:
    if requirement.marker and not requirement.marker.evaluate(ENV):
        continue
    name = canonicalize_name(requirement.name)
    version = effective.get(name)
    if version is None or version not in requirement.specifier:
        errors.append(f'{parent}: unsatisfied {requirement}; available={version}')

report = {'target': ENV, 'preinstalled_imports': preinstalled, 'files': files,
          'errors': errors, 'passed': not errors,
          'note': 'Static archive/ABI/dependency audit, not an ARM64 import or hardware test. '
                  'Install RapidOCR with --no-deps to reuse board OpenCV.'}
(ROOT / 'wheels_audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({'wheel_count': len(files), 'passed': not errors, 'errors': errors}, indent=2))
raise SystemExit(bool(errors))
