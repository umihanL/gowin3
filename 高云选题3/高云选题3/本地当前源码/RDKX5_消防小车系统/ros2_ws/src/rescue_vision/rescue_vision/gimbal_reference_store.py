"""Persist an operator-confirmed current-boot reference; never access motors."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import re
import uuid


def _encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf-8')


def _atomic(path, raw):
    staged = path.with_name(path.name + '.tmp-' + uuid.uuid4().hex)
    try:
        with staged.open('xb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staged, path)
    finally:
        if staged.exists():
            staged.unlink()


def persist_reference(result, control_config, *, runtime_dir=None, boot_id=None):
    """Only configured local paths are accepted; API callers supply no paths.

    The profile is replaced last, so an interrupted write cannot match hashes
    of partially updated files. Existing source configs are retained as a
    recoverable backup. Optional paths/boot ID support isolated unit tests.
    """
    cfg = deepcopy(result['config'])
    reference = result['reference_id']
    if not isinstance(reference, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,100}', reference):
        raise ValueError('invalid_reference_id')
    servo = cfg.get('servo', {})
    if (cfg.get('dry_run') is not False or servo.get('reference_confirmed_this_start') is not True or
            servo.get('reference_id') != reference):
        raise ValueError('unconfirmed_reference_result')
    current_boot = (Path('/proc/sys/kernel/random/boot_id').read_text().strip()
                    if boot_id is None else boot_id)
    if not current_boot or result.get('boot_id') != current_boot:
        raise ValueError('reference_boot_changed_before_save')
    settings = cfg.get('live_reference') or {}
    workspace = Path(settings.get('workspace', ''))
    if not workspace.is_absolute():
        raise ValueError('reference_workspace_must_be_absolute')
    workspace = workspace.resolve(strict=True)
    directory = workspace / 'config'
    if not directory.is_dir() or directory.is_symlink():
        raise ValueError('reference_config_directory_invalid')
    runtime = Path('/run/rescue') if runtime_dir is None else Path(runtime_dir)
    runtime.mkdir(parents=True, exist_ok=True)
    runtime = runtime.resolve(strict=True)
    profile = runtime / 'gimbal-profile.json'
    configured_profile = Path(settings.get('profile_path', ''))
    if not configured_profile.is_absolute() or configured_profile.resolve() != profile:
        raise ValueError('reference_profile_path_invalid')
    if profile.is_symlink():
        raise ValueError('reference_profile_symlink_not_allowed')
    controls = deepcopy(control_config)
    controls['enabled'] = True
    # Record exactly the active pump policy; a reference action never enables it.
    anchor = servo.get('absolute_reference') or {}
    saved_absolute = anchor.get('schema_version') == 1 and anchor.get('reference_id') == reference
    saved_profile = directory / 'gimbal-absolute-profile.local.json'
    if saved_absolute and saved_profile.is_symlink():
        raise ValueError('absolute_reference_profile_symlink_not_allowed')
    manifest = dict(schema_version=1, boot_id=current_boot,
                    operator_zero_confirmed=True, reference_id=reference,
                    absolute_encoder_reference_saved=saved_absolute)
    updates = []
    for name, filename, document in (
            ('io_config', 'actuator_io.mixed.local.json', cfg),
            ('control_config', 'local_fire_control.local.json', controls)):
        target = directory / filename
        if target.is_symlink() or target.resolve().parent != directory.resolve():
            raise ValueError('reference_config_path_invalid')
        raw = _encoded(document)
        manifest[name] = dict(path=str(target), sha256=hashlib.sha256(raw).hexdigest())
        updates.append((target, raw))
    for target, raw in updates:
        if target.is_file():
            backup = target.with_name(target.name + '.before-' + reference)
            if backup.exists():
                backup = target.with_name(target.name + '.before-' + reference + '-' + uuid.uuid4().hex)
            with backup.open('xb') as stream:
                stream.write(target.read_bytes())
        _atomic(target, raw)
    if saved_absolute:
        _atomic(saved_profile, _encoded(manifest))
    _atomic(profile, _encoded(manifest))
    return manifest
