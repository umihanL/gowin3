"""Local update package checks. No network, hardware, or Linux service calls."""
from __future__ import annotations
import hashlib
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile

BASE = Path(__file__).resolve().parent
PREFIX = 'task_console_update_20261031'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, default=BASE / (PREFIX + '.tar.gz'))
    archive = parser.parse_args().archive
    spec = importlib.util.spec_from_file_location('task_console_installer', BASE / (PREFIX + '_install.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    checks = []
    with tempfile.TemporaryDirectory(prefix='rescue-update-check-') as temporary:
        stage = Path(temporary) / 'stage'
        stage.mkdir()
        with tarfile.open(archive, 'r:gz') as tar:
            members = tar.getmembers()
            assert all(m.isfile() and not Path(m.name).is_absolute() and '..' not in Path(m.name).parts for m in members)
            tar.extractall(stage)
        module.STAGE = stage
        manifest, files = module.verified_payload((Path(temporary) / 'workspace').resolve())
        checks.append('All payload sizes/SHA256/Python syntax verified')
        configs = {rel for rel, _, _ in files if rel.startswith('config/')}
        assert configs == module.CONFIGS
        assert not any(any(part in rel.split('/') for part in ('.venv', 'models', '__pycache__')) or '.local.' in rel for rel, _, _ in files)
        checks.append('Only allowlisted geometry/template settings and create-only defaults are included')
        mixed = json.loads((stage/'payload/config/actuator_io.mixed.template.json').read_text(encoding='utf-8-sig'))
        assert mixed['dry_run'] is True and mixed['pump']['enabled'] is False
        assert mixed['servo']['enable_on_command'] is False and mixed['servo']['reference_confirmed_this_start'] is False
        assert mixed['servo']['pan']['reference_counts'] is None and mixed['servo']['tilt']['reference_counts'] is None
        impact = json.loads((stage/'payload/config/fixed_camera_landing.template.json').read_text(encoding='utf-8-sig'))
        assert impact.get('calibrated') is False
        checks.append('Mixed template remains dry-run, pump disabled, no enable-on-command or assumed reference')
        workspace = (Path(temporary) / 'policy-workspace').resolve()
        workspace.mkdir()
        _, policy_files = module.verified_payload(workspace)
        operations, preserved = module.select_operations(workspace, policy_files)
        pending = {item[0]: item for item in operations}
        assert pending['config/extension_uart.json'][4] == 'create_disabled_default'
        uart = json.loads(pending['config/extension_uart.json'][1].read_text(encoding='utf-8-sig'))
        assert uart['enabled'] is False and uart['port'] == ''
        assert all(name in pending for name in module.CONFIGS)
        checks.append('Missing numbered-map and default configs are supplied without enabling UART')
        existing = workspace/'config/extension_uart.json'
        existing.parent.mkdir()
        existing.write_text('{"enabled": true, "port": "/dev/ttyS2", "custom": 77}', encoding='utf-8')
        old_bytes = existing.read_bytes()
        operations, preserved = module.select_operations(workspace, policy_files)
        assert 'config/extension_uart.json' in preserved
        assert 'config/extension_uart.json' not in {item[0] for item in operations}
        assert existing.read_bytes() == old_bytes
        checks.append('Existing active source UART is skipped unchanged and excluded from applied hashes')
        installed = workspace/'install/rescue_vision/share/rescue_vision/config/traffic.fpga_navigation.json'
        installed.parent.mkdir(parents=True)
        installed.write_text('{"custom_traffic": 123}', encoding='utf-8')
        operations, preserved = module.select_operations(workspace, policy_files)
        recovered = next(item for item in operations if item[0]=='config/traffic.fpga_navigation.json')
        assert recovered[4] == 'recover_installed_setting' and recovered[1] == installed
        assert recovered[3] == module.digest(installed)
        checks.append('Missing source config recovers existing installed values rather than defaults')
        supplied = {item[0] for item in operations}
        for rel in module.REQUIRED:
            if rel in supplied or (workspace/rel).exists(): continue
            dependency=workspace/rel
            dependency.parent.mkdir(parents=True,exist_ok=True)
            dependency.write_text('{}',encoding='utf-8')
        module.require_dependencies(workspace,operations)
        (workspace/'config/nozzle.template.json').unlink()
        try:
            module.require_dependencies(workspace,operations)
        except RuntimeError:
            pass
        else:
            raise AssertionError('Missing calibration dependency accepted')
        assert 'rescue-task-modes.service' in module.SERVICES
        checks.append('Supplied configs satisfy preflight while absent calibration is rejected; historical task service guarded')
        original = (stage/'manifest.json').read_bytes()
        for illegal in ('config/voice.local.json', '../outside.txt', 'models/model.bin'):
            malformed = json.loads(original)
            malformed['files'][0]['path'] = illegal
            (stage/'manifest.json').write_text(json.dumps(malformed), encoding='utf-8')
            try:
                module.verified_payload((Path(temporary) / 'workspace').resolve())
            except RuntimeError:
                pass
            else:
                raise AssertionError('Accepted prohibited path: ' + illegal)
        (stage/'manifest.json').write_bytes(original)
        checks.append('Protected settings, parent traversal, and model paths rejected')
        rel, source, _ = files[0]
        raw = source.read_bytes()
        source.write_bytes(raw+b'\n')
        try:
            module.verified_payload((Path(temporary) / 'workspace').resolve())
        except RuntimeError:
            pass
        else:
            raise AssertionError('Accepted corrupted payload')
        source.write_bytes(raw)
        checks.append('Corrupted payload rejected before applying')
        result = subprocess.run([sys.executable, str(stage/'install_update.py'), '--verify-payload', '--workspace', str(Path(temporary)/'workspace')],capture_output=True,text=True)
        assert result.returncode == 0, result.stderr
        assert not (Path(temporary)/'workspace').exists()
        checks.append('Offline CLI verification makes no workspace writes')
    report = {'passed': len(checks), 'checks': checks, 'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
              'files': len(manifest['files']), 'board_connected': False, 'deployment_executed': False,
              'limits': ['Board build/service restart was not run', 'Update installer has not been tested on X5 this turn']}
    (BASE/(PREFIX+'_validation.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False))

if __name__ == '__main__':
    main()
