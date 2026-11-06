"""Build the reviewed, source-only task console update without connecting to a board."""
from __future__ import annotations
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parent.parent
WS = ROOT / 'ros2_ws'
PREFIX = 'task_console_update_20261031'
TOOLS = '''presentation_server.py run_presentation.sh install_presentation_service.sh build_board.sh
start_interaction_services.sh start_perception_services.sh interaction_start_status.py
build_native_preprocess.sh build_native_remap.sh build_native_runtime.sh
run_task_modes.sh run_local_fire_control.sh run_standalone_tasks.sh gimbal_start_profile.py gimbal_task_profiles.py reanchor_pan_reference.py fixed_camera_start_config.py capture_mixed_reference.py stereo_start_profile.py import_factory_stereo.py
run_mipi_camera.sh run_mipi_vision.sh run_detection_preview.sh detection_preview.py
image_transport_env.sh export_fpga_map.py export_region_traversability.py verify_mixed_gimbal_closed_loop.py check_fpga_uart.py check_semantic_navigation.py migrate_pump_to_fpga.py check_neural_tts.py self_check.py preview_fixed_camera_calibration.py'''.split()
CONFIGS = ['fpga_map_catalog.json', 'numbered_points.json', 'extension_map.json', 'actuator_io.mixed.template.json',
           'fixed_camera_aim.template.json', 'fixed_camera_landing.template.json', 'semantic_regions.json', 'region_traversability.json',
           'fire_commissioning.assumed.draft.json', 'fixed_camera_aim.installation.json',
           'fire_commissioning.user_measurements.draft.json', 'stereo.factory.json',
           'stereo_runtime.factory.json', 'gs130w_intrinsics.factory-mapped.json']
CREATE_ONLY = ['extension_uart.json', 'traffic.fpga_navigation.json', 'region_approach.json', 'checkpoint_alignment.json']
DOCS = ['上位机任务驾驶台使用.md', '完整学习与验收手册.md', '拓展任务与水枪接线_20261031.md',
        '当前任务流程与固定双目瞄准.md', '沿途灭火导航口令.md', '语义区域搜索与地图导航.md',
        '云台接管与导航播报修复_20261102.md', '本地全功能验收与标定说明.md', '固定相机实测落点接口.md',
        '火源统计与一键灭火使用.md', '固定相机云台标定操作.md']
EXCLUDE_PARTS = {'__pycache__', '.git', '.venv', 'build', 'install', 'log', 'logs', 'reports'}
SUFFIXES = {'.py', '.cpp', '.hpp', '.h', '.xml', '.cfg', '.msg', '.srv', '.action', '.txt', '.json', '.html', '.css', '.js', '.svg'}

def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def collect() -> dict[str, bytes]:
    chosen = {WS / 'env.sh'}
    for folder in ('src/rescue_vision', 'src/rescue_interfaces', 'web'):
        for path in (WS / folder).rglob('*'):
            rel = path.relative_to(WS)
            if not path.is_file() or any(p in EXCLUDE_PARTS for p in rel.parts):
                continue
            if path.is_symlink():
                raise ValueError(f'Symlink not permitted in source bundle: {rel}')
            if path.suffix in SUFFIXES or '/resource/' in rel.as_posix() or path.name in ('CMakeLists.txt',):
                chosen.add(path)
    chosen.update(WS / 'tools' / name for name in TOOLS)
    chosen.update(WS / 'config' / name for name in CONFIGS + CREATE_ONLY)
    chosen.update(WS / 'docs' / name for name in DOCS)
    result = {}
    for path in sorted(chosen):
        rel = path.relative_to(WS).as_posix()
        if not path.is_file():
            raise FileNotFoundError(path)
        if '.local.' in path.name or path.suffix in ('.so', '.bin', '.onnx', '.pt'):
            raise ValueError(f'Private/device/generated asset prohibited: {rel}')
        result[rel] = path.read_bytes()
    uart = json.loads(result['config/extension_uart.json'].decode('utf-8-sig'))
    if uart.get('enabled') is not False or uart.get('port') != '':
        raise ValueError('Bundled UART default must be disabled with empty port')
    return result

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'board_setup')
    args = parser.parse_args()
    data = collect()
    manifest = {
        'schema_version': 1, 'release': PREFIX, 'workspace': '/root/rescue/ros2_ws',
        'revision': 'field_centering_batch_identity_verified_corridor_20261103',
        'search_navigation_recovery_revision': 'visual_dropout_same_route_bounded_low_readback_20261103',
        'extension2_state_revision': 'atomic_pose_fair_poll_first_failure_measured_stop_20261104',
        'centering_motion_revision': 'shared_odometry_health_stable_stop_anchor_window_target_level_limit_20261104',
        'centering_tolerance_norm': 0.05,
        'fire_batch_revision': 'stationary_gimbal_each_confirmed_target_high_low_before_resume_20261104',
        'manual_pump_prepare_revision': 'new_request_explicit_off_matched_low_ready_before_single_on_20261104',
        'selected_straight_ray_revision': 'admission_matches_explicit_unmeasured_water_policy_20261104',
        'runtime_performance_revision': 'unused_qos_events_removed_immutable_route_http_bytes_bounded_uart_burst_20261104',
        'gimbal_revision': 'bounded_reconnect_explicit_pan_reanchor_shared_mechanical_profiles_20261103',
        'gimbal_fault_status_revision': 'new_fault_invalidates_completed_recovery_20261103',
        'manual_pump_receipt_revision': 'matched_off_confirmation_independent_of_actuator_fault_20261103',
        'manual_pump_recovery_revision': 'page_denial_is_not_uart_disconnect_20261103',
        'gimbal_water_policy_revision': 'runtime_overlay_preserves_saved_reference_hash_20261103',
        'fixed_nozzle_manual_revision': 'explicit_per_request_stationary_nozzle_readonly_timeout_exception_20261104',
        'fire_stop_trigger_revision': 'post_start_complete_box_four_exposures_450ms_both_sides_20261103',
        'preview_layout_revision': 'native_default_stable_fit_layout_20261103',
        'stereo_factory_revision': 'eeprom_eye_mapping_native_roi_exact_pair_observation_20261103',
        'manual_pump_test': {'standalone_enabled': True, 'automatic_start': False, 'duration_ms': [100,1000], 'calibration_required': False},
        'fire_workflows': ['identity_bound_selected_round', 'regional_all_target_statistics', 'calibration_draft_workbench'],
        'region_modes': ['search_only', 'search_and_extinguish'],
        'region_fire_pause': {'enabled_by_default': True,
                                        'consecutive_frames': 4, 'stable_duration_s': 0.45,
                                        'post_stop_confirm_frames': 3, 'min_confidence': 0.90, 'verified_centered_stop_dwell_s': 1.0,
                                        'trigger': 'confirmed_fire_during_continuous_navigation',
                                        'centering_search_only': 'all_visible_flames_signed_local_jogs',
                                        'centering_search_and_extinguish': 'stationary_gimbal_batch_no_wheel_approach',
                                        'resume': 'restore_original_pause_anchor_then_same_route',
                                        'same_frame_overlay': 'extension2_only_exact_capture_sequence_and_frame_id',
                                        'retrigger_cooldown_s': 3.0,
                                        'same_view_alias_window_s': 4.0,
                                        'post_stop_confirmation': 'seeded_candidates_fresh_stationary_evidence',
                                        'local_corridor': 'actual_pose_inflated_grid_retained_straight_span',
                                        'transition_trace': 'command_written_acknowledged_measured_stop'},
        'region_route': {'planning': 'once_for_all_waypoints_and_exit', 'pause_resume': 'same_task_same_route',
                         'waypoint_policy': 'pass_through', 'waypoint_stop': False,
                         'waypoint_speed_override': False, 'terminal_exit_stop': True},
        'semantic_regions_requires_fpga_capability': 'map_caps:0x20|0x40|0x80; opcodes 0x21,0x25,0x26,0x37,0x3A,0x3B',
        'centering_requires_fpga_capability': 'extension_version:2; extension_caps:0x40; opcodes 0x3C,0x3D',
        'region_approach_requires_measured_calibration': True,
        'offline_neural_tts': ['sherpa_matcha', 'sherpa_vits'],
        'audio_revision': 'complete_scene_answer_20261102',
        'voice_switch_revision': 'persistent_runtime_audio_toggle_keeps_typed_commands_20261104',
        'motor_revision': 'mixed_full_target_pulse_position_20261103',
        'fixed_camera_impact_revision': 'explicit_straight_ray_or_measured_impact_20261103',
        'fixed_camera_geometry_revision': 'user_installation_offsets_r2_20261103_calibration_pending',
        'physical_calibration_generated': False,
        'fpga_pump_capability': 'extension_caps:0x20',
        'pump_config_migration_required': True,
        'pump_startup_revision': 'preserve_explicit_fpga_output_region_off_prepare_20261103',
        'numbered_navigation_requires_fpga_heading_capability': '0x10',
        'board_tested': False, 'automatically_start_services': False,
        'source_only': True,
        'preserves': ['models', '.venv', 'config/*.local.json', 'config/extension_uart.json',
                      'active actuator/voice/traffic/calibration/nozzle/reference settings', 'existing native .so until board rebuild'],
        'config_allowlist': ['config/' + name for name in CONFIGS + CREATE_ONLY],
        'create_if_missing': ['config/' + name for name in CREATE_ONLY],
        'files': [{'path': rel, 'bytes': len(blob), 'sha256': sha(blob),
                   'policy': 'create_if_missing' if rel in {'config/' + name for name in CREATE_ONLY} else 'replace'} for rel, blob in sorted(data.items())],
    }
    manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + '\n').encode('utf-8')
    installer = Path(__file__).with_name(PREFIX + '_install.py').read_bytes()
    readme = Path(__file__).with_name(PREFIX + '_说明.md').read_bytes()
    members = {'manifest.json': manifest_bytes, 'install_update.py': installer, 'README.md': readme}
    members.update({'payload/' + name: blob for name, blob in data.items()})
    args.output_dir.mkdir(parents=True, exist_ok=True)
    archive = args.output_dir / (PREFIX + '.tar.gz')
    with archive.open('wb') as sink, gzip.GzipFile(filename='', mode='wb', fileobj=sink, mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode='w', format=tarfile.PAX_FORMAT) as tar:
            for name, blob in sorted(members.items()):
                info = tarfile.TarInfo(name)
                info.size, info.mtime = len(blob), 0
                info.mode = 0o755 if name.endswith('.sh') else 0o644
                info.uid = info.gid = 0
                info.uname = info.gname = ''
                tar.addfile(info, io.BytesIO(blob))
    (args.output_dir / (PREFIX + '.manifest.json')).write_bytes(manifest_bytes)
    digest = sha(archive.read_bytes())
    (args.output_dir / (PREFIX + '.sha256')).write_text(digest + '  ' + archive.name + '\n', encoding='ascii')
    print(json.dumps({'archive': str(archive), 'files': len(data), 'sha256': digest,
                      'bytes': archive.stat().st_size, 'board_connected': False}, ensure_ascii=False))

if __name__ == '__main__':
    main()
