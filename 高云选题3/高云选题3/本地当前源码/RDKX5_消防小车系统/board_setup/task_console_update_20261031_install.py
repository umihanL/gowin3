"""Verify/apply a source-only update on an existing X5 workspace; never start services."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tarfile
import time

STAGE = Path(__file__).resolve().parent
REPLACE_CONFIGS = {'config/fpga_map_catalog.json', 'config/numbered_points.json',
                   'config/extension_map.json', 'config/actuator_io.mixed.template.json',
                   'config/fixed_camera_aim.template.json', 'config/fixed_camera_landing.template.json', 'config/semantic_regions.json',
                   'config/region_traversability.json', 'config/fire_commissioning.assumed.draft.json',
                   'config/fixed_camera_aim.installation.json', 'config/fire_commissioning.user_measurements.draft.json'}
CREATE_CONFIGS = {'config/extension_uart.json', 'config/traffic.fpga_navigation.json',
                  'config/region_approach.json', 'config/checkpoint_alignment.json'}
CONFIGS = REPLACE_CONFIGS | CREATE_CONFIGS
SERVICES = ['rescue-task-interaction.service', 'rescue-task-modes.service', 'rescue-presentation.service',
            'rescue-mipi-vision.service', 'rescue-yolo-preview.service']
REQUIRED = ['.venv/bin/python', 'config/voice.local.json', 'config/extension_uart.json',
            'config/extension_map.json', 'config/numbered_points.json', 'config/buildings.json',
            'config/traffic.fpga_navigation.json', 'config/actuator_io.pd42.template.json',
            'config/local_fire_control.template.json', 'config/mount.template.json',
            'config/servo.template.json', 'config/landing.corrected.template.json',
            'config/stereo.template.json', 'config/stereo_runtime.template.json',
            'config/nozzle.template.json', 'config/gs130w_intrinsics_20261010.json',
            'config/road_vision.template.json', 'config/edge_dialogue.template.json']


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verified_payload(workspace: Path):
    manifest = json.loads((STAGE / 'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('release') != 'task_console_update_20261031' or manifest.get('schema_version') != 1:
        raise RuntimeError('Unsupported update manifest')
    files = []
    seen = set()
    for entry in manifest['files']:
        rel = entry['path']
        part = PurePosixPath(rel)
        if part.is_absolute() or '..' in part.parts or '\\' in rel or rel in seen:
            raise RuntimeError(f'Invalid/duplicate path: {rel}')
        if not (rel == 'env.sh' or rel.startswith(('src/', 'web/', 'tools/', 'docs/')) or rel in CONFIGS):
            raise RuntimeError(f'Path outside allowed update areas: {rel}')
        if '.local.' in part.name or part.suffix in ('.so', '.bin', '.pt', '.onnx'):
            raise RuntimeError(f'Protected file in package: {rel}')
        expected_policy = 'create_if_missing' if rel in CREATE_CONFIGS else 'replace'
        if entry.get('policy') != expected_policy:
            raise RuntimeError(f'Invalid installation policy for {rel}')
        source = STAGE / 'payload' / rel
        destination = workspace / rel
        if source.is_symlink() or not source.is_file() or digest(source) != entry['sha256'] or source.stat().st_size != entry['bytes']:
            raise RuntimeError(f'Payload checksum failed: {rel}')
        if not destination.resolve().is_relative_to(workspace):
            raise RuntimeError(f'Destination escapes workspace: {rel}')
        if part.suffix == '.py':
            compile(source.read_bytes(), rel, 'exec')
        seen.add(rel)
        files.append((rel, source, destination))
    if not CONFIGS.issubset(seen):
        raise RuntimeError('Map catalog/mixed actuator template missing')
    uart = json.loads((STAGE/'payload/config/extension_uart.json').read_text(encoding='utf-8-sig'))
    if uart.get('enabled') is not False or uart.get('port') != '':
        raise RuntimeError('Bundled UART default must stay disabled with an empty port')
    return manifest, files



def select_operations(workspace: Path, files):
    """Preserve active settings; recover installed settings before using defaults."""
    operations, preserved = [], []
    for rel, source, destination in files:
        reason = 'replace'
        if rel in CREATE_CONFIGS:
            if destination.exists() or destination.is_symlink():
                preserved.append(rel)
                continue
            candidates = []
            for folder in ('install/rescue_vision/share/rescue_vision/config',
                           'install/share/rescue_vision/config'):
                candidate = workspace / folder / Path(rel).name
                if candidate.is_file():
                    if not candidate.resolve().is_relative_to(workspace):
                        raise RuntimeError(f'Installed setting escapes workspace: {candidate}')
                    candidates.append(candidate)
            if candidates:
                if len({digest(p) for p in candidates}) != 1:
                    raise RuntimeError(f'Conflicting installed settings; source is missing: {rel}')
                source, reason = candidates[0], 'recover_installed_setting'
            else:
                reason = 'create_disabled_default' if rel.endswith('extension_uart.json') else 'create_default'
        operations.append((rel, source, destination, digest(source), reason))
    return operations, preserved


def require_dependencies(workspace: Path, operations) -> None:
    supplied = {entry[0] for entry in operations}
    for rel in REQUIRED:
        if not (workspace / rel).is_file() and rel not in supplied:
            raise RuntimeError(f'Existing board configuration/dependency missing: {rel}; no files changed')


def require_stopped() -> None:
    for service in SERVICES:
        result = subprocess.run(['systemctl', 'is-active', service], capture_output=True, text=True)
        if result.stdout.strip() in ('active', 'activating', 'reloading', 'deactivating'):
            raise RuntimeError(f'Stop {service} deliberately before updating; no files changed.')
    listing = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True)
    # Also catch manually launched nodes, not only systemd owners.
    pattern = re.compile(r'(/rescue_vision/[A-Za-z_][A-Za-z_0-9]*(?:\.py)?(?: |$)|-m rescue_vision\.|tools/(?:presentation_server|detection_preview)\.py(?: |$))')
    for line in listing.splitlines():
        if pattern.search(line):
            raise RuntimeError('A related ROS/dashboard process is still running: ' + line.strip())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path('/root/rescue/ros2_ws'))
    parser.add_argument('--apply', action='store_true', help='Apply and build only after processes have stopped')
    parser.add_argument('--verify-payload', action='store_true', help='Offline payload validation only; no board checks')
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    if not workspace.is_absolute() or workspace == Path(workspace.anchor):
        raise RuntimeError('An existing dedicated workspace is required')
    manifest, files = verified_payload(workspace)
    if args.verify_payload:
        print(json.dumps({'payload_valid': True, 'files': len(files), 'board_checked': False}))
        return
    if sys.platform != 'linux' or os.geteuid() != 0:
        raise RuntimeError('Board preflight/apply requires Linux root; use --verify-payload for local checks')
    if not Path('/opt/ros/humble/setup.bash').is_file():
        raise RuntimeError('Existing ROS 2 Humble installation missing')
    operations, preserved = select_operations(workspace, files)
    require_dependencies(workspace, operations)
    require_stopped()
    if not args.apply:
        print(json.dumps({'preflight_ok': True, 'files': len(operations), 'preserved': preserved, 'changed': False}))
        return
    backup = workspace.parent / 'update_backups' / ('task_console_' + time.strftime('%Y%m%dT%H%M%S') + '_' + str(os.getpid()))
    backup.mkdir(parents=True, exist_ok=False)
    replaced, added = [], []
    with tarfile.open(backup / 'before_update.tar.gz', 'w:gz') as tar:
        for rel, source, destination, expected, reason in operations:
            if destination.exists():
                tar.add(destination, arcname='ros2_ws/' + rel, recursive=False)
                replaced.append(rel)
            else:
                added.append(rel)
        # Board build may replace native outputs and installed copies. Preserve these too.
        for name in ('install', 'build/rescue_vision', 'build/rescue_interfaces'):
            target = workspace / name
            if target.exists():
                tar.add(target, arcname='ros2_ws/' + name)
        for target in (workspace / 'src/rescue_vision/rescue_vision').glob('*.so'):
            tar.add(target, arcname='ros2_ws/' + target.relative_to(workspace).as_posix())
    (backup / 'changes.json').write_text(json.dumps({'replaced': replaced, 'added': added, 'preserved': preserved,
        'applied': [{'path': item[0], 'sha256': item[3], 'reason': item[4]} for item in operations],
        'services_restarted': False, 'manifest': manifest}, ensure_ascii=False, indent=2), encoding='utf-8')
    # setup.py copies config/*.json during colcon. Preserve configurations tuned
    # only in an installed package too, rather than replacing them from source.
    installed_settings = []
    for relative in ('install/rescue_vision/share/rescue_vision/config',
                     'install/share/rescue_vision/config'):
        directory = workspace / relative
        if not directory.exists():
            continue
        if not directory.resolve().is_relative_to(workspace):
            raise RuntimeError('Installed configuration directory escapes workspace')
        for setting in directory.glob('*.json'):
            if 'config/' + setting.name in REPLACE_CONFIGS:
                continue
            if not setting.resolve().is_relative_to(workspace):
                raise RuntimeError('Installed configuration symlink escapes workspace')
            saved = backup / 'installed_config' / setting.relative_to(workspace)
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(setting, saved)
            installed_settings.append((setting, saved))
    for rel, source, destination, expected, reason in operations:
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(destination.name + '.task-update-' + str(os.getpid()))
        shutil.copyfile(source, temporary)
        temporary.chmod(0o755 if rel.endswith('.sh') else 0o644)
        os.replace(temporary, destination)
    try:
        with (backup / 'build.log').open('w', encoding='utf-8') as log:
            result = subprocess.run(['bash', 'tools/build_board.sh'], cwd=workspace,
                                    stdout=log, stderr=subprocess.STDOUT)
    finally:
        for setting, saved in installed_settings:
            if not setting.resolve().is_relative_to(workspace):
                raise RuntimeError(f'Installed configuration path escaped workspace after build: {setting}')
            setting.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(saved, setting)
            if digest(saved) != digest(setting):
                raise RuntimeError(f'Cannot restore installed configuration: {setting}')
    if result.returncode:
        raise RuntimeError(f'Board build failed; services remain stopped. Backup and build log: {backup}')
    for rel, _, destination, expected, reason in operations:
        if digest(destination) != expected:
            raise RuntimeError(f'Post-install source hash mismatch: {rel}; services remain stopped')
    print(json.dumps({'installed': True, 'files': len(operations), 'preserved': preserved, 'backup': str(backup),
                      'services_restarted': False, 'hardware_enabled': False}))

if __name__ == '__main__':
    main()
