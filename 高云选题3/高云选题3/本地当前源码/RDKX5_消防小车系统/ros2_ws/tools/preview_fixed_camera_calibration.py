"""Offline straight-ray calibration sensitivity rehearsal; never a calibration.

No ROS, serial, GPIO, network, driver or pump interface is imported. Example
geometry exists only in memory and cannot be exported as a deployable config.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.fixed_camera_aim import FixedCameraAim
from rescue_vision.zdt_fd8_control import PULSE_DEG


def illustrative_geometry():
    """Explicit assumed rig, isolated from every deployment configuration."""
    doc = json.loads((ROOT/'config/fixed_camera_aim.template.json').read_text('utf-8'))
    doc.update(calibrated=True, calibration_id='OFFLINE_ASSUMPTION_NOT_A_CALIBRATION',
        stereo_calibration_id='OFFLINE_ASSUMPTION_NO_MEASURED_STEREO',
        actuator_reference_id='OFFLINE_ASSUMPTION_NO_HARDWARE_REFERENCE',
        axis_reference_verified=True, image_size_wh=[1280,1088],
        R_camera_to_gimbal=[[0,0,1],[-1,0,0],[0,-1,0]],
        t_camera_origin_in_gimbal_m=[0,.08,.12],
        tilt_origin_in_pan_m=[.02,0,.03], nozzle_origin_in_tilt_m=[.04,0,0],
        nozzle_direction_in_tilt=[1,0,0], valid_range_m=[.2,2],
        axes=dict(pan=dict(zero_angle_deg=0.,positive_direction=1,limits_deg=[-90.,90.]),
                  tilt=dict(zero_angle_deg=0.,positive_direction=1,limits_deg=[0.,60.])))
    return doc


def plane_error(model, angles, true_target):
    """Intersection with x=target.x facade; excludes gravity/jet dispersion."""
    origin, direction = model.forward(angles)
    if direction[0] <= 0:
        raise ValueError('Ray is not toward the illustrative facade')
    length = (true_target[0]-origin[0])/direction[0]
    if length <= 0:
        raise ValueError('Facade is behind nozzle')
    hit = origin+length*direction
    error = hit-true_target
    return dict(straight_ray_plane_point_m=hit.tolist(),
                straight_ray_error_mm=float(np.linalg.norm(error)*1000),
                horizontal_error_mm=float(error[1]*1000), vertical_error_mm=float(error[2]*1000))


def sensitivity_rows():
    true_doc = illustrative_geometry()
    true_model = FixedCameraAim(true_doc)
    rows = []
    for depth in (.4,.7,1.):
        xyz = np.array([0.,-.02,depth])
        target = true_model.R@xyz+true_model.t
        for kind in ('ideal_assumptions','depth_plus_5_percent','camera_y_plus_10_mm',
                     'extrinsic_yaw_plus_1_deg','pan_zero_plus_1_deg','tilt_pivot_z_plus_10_mm',
                     'pan_quantized_pulse_resolution'):
            estimate = deepcopy(true_doc)
            observed = xyz.copy()
            if kind == 'depth_plus_5_percent':
                observed *= 1.05
            elif kind == 'camera_y_plus_10_mm':
                estimate['t_camera_origin_in_gimbal_m'][1] += .01
            elif kind == 'extrinsic_yaw_plus_1_deg':
                c,s = math.cos(math.radians(1)),math.sin(math.radians(1))
                rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
                estimate['R_camera_to_gimbal'] = (rotation@true_model.R).tolist()
            elif kind == 'pan_zero_plus_1_deg':
                estimate['axes']['pan']['zero_angle_deg'] += 1.
            elif kind == 'tilt_pivot_z_plus_10_mm':
                estimate['tilt_origin_in_pan_m'][2] += .01
            estimated = FixedCameraAim(estimate)
            desired = estimated.R@observed+estimated.t
            goal = np.array(estimated.inverse(desired,[0.,0.])['geometry_target_deg'])
            if kind == 'pan_quantized_pulse_resolution':
                goal[0] = round(goal[0]/PULSE_DEG)*PULSE_DEG
            rows.append(dict(assumed_optical_depth_m=depth, scenario=kind,
                             joint_angle_deg=goal.tolist(), **plane_error(true_model,goal,target)))
    return rows


def build_report():
    captured_path = ROOT/'config/gs130w_intrinsics_20261010.json'
    captured = json.loads(captured_path.read_text('utf-8'))
    source = captured['role_to_source']['left']
    intrinsic = captured['sources'][source]
    fx = intrinsic['K'][0]
    config_paths = [ROOT/'config'/name for name in (
        'stereo.template.json','stereo_runtime.template.json','fixed_camera_aim.template.json',
        'nozzle.template.json','landing.template.json','landing.corrected.template.json',
        'fixed_camera_landing.template.json')]
    config_state = {}
    for path in config_paths:
        document = json.loads(path.read_text('utf-8'))
        config_state[path.name] = {k:v for k,v in document.items()
                                  if k in ('calibrated','enabled','nozzle_calibrated','landing_calibrated',
                                           'metric_depth_verified','synchronization_verified')}
    doc = illustrative_geometry()
    assumptions = {k:doc[k] for k in ('t_camera_origin_in_gimbal_m','R_camera_to_gimbal',
        'tilt_origin_in_pan_m','nozzle_origin_in_tilt_m','nozzle_direction_in_tilt','axes')}
    paths = [captured_path,*config_paths,Path(__file__),
        ROOT/'src/rescue_vision/rescue_vision/fixed_camera_aim.py',
        ROOT/'src/rescue_vision/rescue_vision/fixed_camera_landing.py',
        ROOT/'src/rescue_vision/rescue_vision/fire_aim_node.py',
        ROOT/'src/rescue_vision/rescue_vision/stereo.py',
        ROOT/'src/rescue_vision/rescue_vision/stereo_detection_geometry.py',
        ROOT/'src/rescue_vision/rescue_vision/fire_aim_geometry.py',
        ROOT/'src/rescue_vision/rescue_vision/legacy_landing_calibration.py']
    return dict(generated_utc=datetime.now(timezone.utc).isoformat(), simulation_only=True,
        calibrated=False, hardware_written=False, hardware_tested=False, pump_allowed=False,
        physical_hit_verified=False, deployable_config_written=False,
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        captured_intrinsics=dict(path=str(captured_path),source_topic=source,
            image_size_wh=[intrinsic['width'],intrinsic['height']],fx=fx,
            eeprom_eye_assignment_verified=captured['provenance']['eeprom_eye_assignment_verified'],
            stereo_metric_calibration_verified=captured['provenance']['stereo_metric_calibration_verified'],
            use='Only near-centre pinhole pixel/angular sensitivity; not an independently verified eye calibration'),
        current_template_flags=config_state, illustrative_mount_assumptions=assumptions,
        sensitivity=sensitivity_rows(),
        quantization_bound=[dict(distance_m=d,half_step_deg=PULSE_DEG/2,
            lateral_error_mm=1000*d*math.tan(math.radians(PULSE_DEG/2)),
            angle_for_2mm_deg=math.degrees(math.atan(.002/d))) for d in (.4,.7,1.)],
        near_center_pixel_scale=dict(one_pixel_deg=math.degrees(math.atan(1/fx)),
            note='Raw captured K pinhole scale near centre only; does not remove lens distortion'),
        feedback_budget=[dict(assumed_poll_hz=8,assumed_scan_s=scan,assumed_delivery_s=.02,
            conservative_oldest_sample_age_s=max(.125,scan)+scan+.02,
            threshold_s=.2,
            all_sample_phases_within_threshold=(max(.125,scan)+scan+.02 <= .2))
            for scan in (.02,.08,.16,.24)],
        feedback_budget_note='Illustrative upper bound: next completed snapshot gap plus scan and delivery. '
            'Scan age is already counted by captured-before-read timestamp. No live latency was measured.',
        missing_measurements=[
            'Physical per-eye calibration identity, rotation, exposure synchronization and held-out ruler distances',
            'Fixed camera to gimbal R/t; pan/tilt pivot and nozzle exit offsets, nozzle direction',
            'Installation direction and repeatable reference with fresh encoder feedback',
            'Actual water landing samples at tested ranges, elevations, pressures and pump settings'],
        limits=[
            'This predicts an ideal straight nozzle ray intersecting a facade, not water impact.',
            'The legacy co-rotating camera/nozzle landing table is not transferable to a chassis-fixed camera.',
            'Same startup pose is a repeatability constraint, not a metric/extrinsic/impact calibration.',
            'Each capture creates a new reference_id. A fixed-camera calibration bound to an older reference '
            'must be revalidated/rebound; identical physical startup pose alone cannot establish this.',
            'ZDT 0.1125 degree pulse grid is evaluated without widening the 0.3 degree / 2 mm original limits.'])


LABELS = dict(ideal_assumptions='理想假设',depth_plus_5_percent='距离高估5%',
    camera_y_plus_10_mm='相机横向外参误差10mm',extrinsic_yaw_plus_1_deg='外参偏航误差1°',
    pan_zero_plus_1_deg='水平零位误差1°',tilt_pivot_z_plus_10_mm='俯仰轴高度误差10mm',
    pan_quantized_pulse_resolution='水平目标按0.1125°脉冲分辨率取整')


def markdown(report):
    text = ['# 固定相机、双目测距与喷嘴几何本地验收', '',
        '**仅离线预演。未连接硬件，未写入部署标定，未使能水泵。**', '',
        '现有材料不能支持真实自动命中：保存了GS130W的CameraInfo/EEPROM内参，但双目尺度、'
        '每只镜头的标定对应、固定相机到云台外参和真实水流落点尚未验收。全部模板仍未标定。', '',
        '本次修复：实际编码器姿态的喷嘴射线也必须满足原2mm误差；FD8格点若无法满足原0.3°/2mm门槛，'
        '返回分辨率不足并停止产生瞄准命令。落点表增加畸变一致性校验，并隔离加载后源数据的修改。', '',
        '补全固定相机独立实测落点接口：`fixed_camera_landing.template.json` 仍为false。'
        '将来需要完整的“名义水平角×名义俯仰角×喷嘴几何距离”实测角度修正网格，'
        '每格还需独立落点验证；只在网格内插值，并绑定安装、泵、几何、双目和本次编码器参考ID。'
        '固定相机使用明确的geometry_locked分支，image_locked始终false。此接口已经通过本地模拟，'
        '实际水流仍未标定。详细字段见 `ros2_ws/docs/固定相机实测落点接口.md`。', '',
        '## 直线射线误差敏感性（全部安装尺寸是假设）', '',
        '假设相机相对云台横向偏置80mm、高120mm；俯仰轴前移20mm、高30mm；喷嘴伸出40mm；'
        '相机正前方目标比光轴高20mm。这些数值只存在工具内存中，不能用于部署。'
        '下表是射线与目标平面交点偏差，未模拟重力、水压和水束散开。', '',
        '|扰动|0.4m误差|0.7m误差|1.0m误差|','|---|---:|---:|---:|']
    for scenario,label in LABELS.items():
        values=[row['straight_ray_error_mm'] for row in report['sensitivity'] if row['scenario']==scenario]
        text.append('|'+label+'|'+'|'.join(f'{v:.2f}mm' for v in values)+'|')
    text += ['', '## FD8 分辨率限制', '',
        '|目标距离|0.05625°半格点横向误差|2mm对应角误差|','|---|---:|---:|']
    for row in report['quantization_bound']:
        text.append(f"|{row['distance_m']:.1f}m|{row['lateral_error_mm']:.2f}mm|{row['angle_for_2mm_deg']:.3f}°|")
    text += ['', '0.1125°脉冲格点的最近点可能距理论角0.05625°。把容差改成0.5°只会掩盖精度不足。'
        '需要选择更合适的观察位置、经过实测的更细定位能力或机械减速，随后重新验收。'
        '实际命中还取决于水束宽度、距离误差和安装误差。', '',
        '## 编码器新鲜度预算', '',
        '以下是假设8Hz轮询、20ms传输的最坏采样相位估算，不是实机测速。'
        '程序将串口扫描前的时间作为采样时间，不能把扫描结束时间冒充最新反馈。', '',
        '|扫描耗时|保守最旧样本年龄|所有相位满足200ms|','|---:|---:|---|']
    for row in report['feedback_budget']:
        text.append(f"|{row['assumed_scan_s']*1000:.0f}ms|{row['conservative_oldest_sample_age_s']*1000:.0f}ms|"
                    +('是' if row['all_sample_phases_within_threshold'] else '否')+'|')
    text += ['', '周期性过期会清除连续稳定锁定，因此高延迟可能造成一直无法锁定。'
        '应实测扫描耗时并减少串口往返/提高有效反馈频率，不能直接放宽200ms门槛。', '',
        '## 接下来需要的实测资料', '',
        '1. 20组以上真实同步棋盘格双目照片（建议30～50组），实际格长；保留独立验证距离。'
        '现有 `tools/calibrate_stereo.py` 已提供分组标定与留出验证。',
        '2. 固定相机到云台坐标变换、两转轴和喷嘴位置/方向、轴正负方向以及启动参考。',
        '3. 不同距离与俯仰条件下的真实水流落点、泵设置和保持不变的安装条件。', '',
        '每次摆在同一位置有助于复现零位，但不能代替上述测量。旧版相机与喷嘴一起转动的落点像素表，'
        '不能直接用于现在“相机固定在车身、喷嘴独立转动”的结构。', '',
        '每次capture都会生成新的reference_id。已经绑定旧参考ID的固定相机标定，必须重新验证并绑定本次参考；'
        '程序不会自动绕过ID校验。以后可将稳定安装标识与本次启动零位分别管理，但需明确转换并经实测验证，'
        '本次模拟不能证明每次上电都能命中。', '',
        '## 本地验证', '']
    tests=report.get('test_result')
    if tests:
        text.append(f"{tests['tests']}项：通过{tests['tests']-tests['skipped']}项，跳过{tests['skipped']}项ROS板端隔离测试；"
                    f"失败模块{len(tests['failed_modules'])}。日志：`geometry_tests/`。")
    text += ['', '代码、模板和内参文件SHA256、全部数值与假设保存在同目录 `geometry.json`。', '',
        '复现入口（从ros2_ws运行）：', '', '```powershell',
        '& ..\..\YOLO11n_model\.venv\Scripts\python.exe tools\preview_fixed_camera_calibration.py',
        '```', '', '工具只输出验收报告，不导出可启用的标定文件，也没有电机或水泵接口。', '']
    return '\n'.join(text)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT.parent/'reports/system_acceptance_20261102')
    args=parser.parse_args()
    output=args.output.resolve()
    # Refuse paths where reports could be mistaken for active configurations.
    if any(part.lower() in ('config','install','models','deployment') for part in output.parts):
        parser.error('Use a reports directory, never a deployment/config directory')
    report=build_report()
    tests=output/'geometry_tests/result.json'
    if tests.is_file():
        result=json.loads(tests.read_text('utf-8'))
        report['test_result']={k:result[k] for k in ('passed','tests','skipped','failed_modules','hardware_tested')}
    output.mkdir(parents=True,exist_ok=True)
    (output/'geometry.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (output/'geometry.md').write_text(markdown(report),encoding='utf-8')
    print(json.dumps(dict(report=str(output/'geometry.md'),simulation_only=True,calibrated=False,
                         hardware_written=False,pump_allowed=False),ensure_ascii=False))


if __name__=='__main__':
    main()
