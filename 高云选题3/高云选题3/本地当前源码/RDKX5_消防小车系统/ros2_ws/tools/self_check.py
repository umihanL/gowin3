#!/usr/bin/env python3
"""Read-only live health check: no ROS publishers, device opens or service changes.

Run on X5 after exporting ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 and sourcing env.sh.
PASS means the inspected software condition passed; it never certifies physical IO.
WAIT means disabled, inactive, uncommissioned or not observed in this short window.
"""
import argparse
import concurrent.futures
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import time
import urllib.request


SERVICES = ('rescue-mipi-camera.service', 'rescue-mipi-vision.service',
            'rescue-yolo-preview.service', 'rescue-edge-llm.service',
            'rescue-task-interaction.service')
TOPICS = ('/mission/state', '/actuators/state', '/fpga/link',
          '/diagnostics/perception', '/diagnostics/camera',
          '/diagnostics/building_ocr', '/diagnostics/road', '/voice/status',
          '/traffic/state', '/stereo/depth_status', '/navigation/state',
          '/extension/state', '/gimbal/manual/state', '/fire/proposal')
PREVIEW_URL = 'http://127.0.0.1:8086/api/state?images=0'


def number(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def evaluate(report):
    """Pure interpretation of one sampling window; used by offline regression tests."""
    checks = []
    def add(name, status, message):
        checks.append(dict(name=name, status=status, message=message))
    env = report.get('environment', {})
    if env.get('ros_domain_id') != '0':
        add('ROS 通信域', 'WARN', '真实相机和任务使用域 0；先 export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0，再 source env.sh。env.sh 单独使用默认域 71。')
    elif env.get('ros_localhost_only') != '0':
        add('ROS 通信域', 'WARN', '域为 0，但应设置 ROS_LOCALHOST_ONLY=0，与当前真实服务一致。')
    else:
        add('ROS 通信域', 'PASS', '域 0，与真实相机、视觉和任务服务一致。')
    if report.get('ros_error'):
        add('ROS 环境', 'WARN', report['ros_error'])
    else:
        add('ROS 环境', 'PASS', '已订阅状态话题；本工具没有发布控制命令。')
    for unit in SERVICES:
        info = report.get('services', {}).get(unit, {})
        active = info.get('ActiveState')
        add(unit, 'PASS' if active == 'active' else 'WARN',
            '服务运行中。' if active == 'active' else '服务状态：' + str(active or info.get('error', '未取得；请在 X5 运行。')))
    topics = report.get('topics', {})
    values = {}
    for topic in TOPICS:
        entry = topics.get(topic, {})
        seconds = entry.get('last_received_age_s')
        limit = 6.5 if topic == '/voice/status' else 3.0
        if not entry.get('count'):
            add(topic, 'WAIT', '本次短时采样未收到消息；可能尚未发现节点，建议 --seconds 10 重查。')
        elif not number(seconds) or not 0 <= seconds <= limit:
            add(topic, 'WARN', '最后消息到达时间过旧，不能据此判定当前状态。')
        elif not isinstance(entry.get('value'), dict):
            add(topic, 'WARN', '状态不是 JSON 对象。')
        else:
            values[topic] = entry['value']
            add(topic, 'PASS', '本次收到 {} 条状态，末条到达距采样结束 {:.2f} 秒。'.format(entry['count'], seconds))
        if entry.get('invalid_count', 0):
            add(topic + ' JSON', 'WARN', '本次有 {} 条消息格式无效。'.format(entry['invalid_count']))
    mission = values.get('/mission/state', {})
    road = values.get('/diagnostics/road')
    mode = mission.get('mode')
    if mission:
        add('任务模式', 'PASS', '当前模式 {}；待机不等于行驶，也不等于正在运行 YOLO。'.format(mode or '未知'))
        if (road or {}).get('fixed_camera_observation') is not True and mission.get('mounting_verified') is not True:
            add('云台安装与低头姿态', 'WAIT', '安装/姿态未验证，斑马线观察和真实云台动作可能被正常阻止；不能只修改 verified 字段跳过实测。')
    if road:
        add('道路/红绿灯观察', 'PASS' if road.get('active_mode') else 'WAIT',
            '当前观察模式 {}，已处理 {} 帧；未激活时检查任务和云台姿态，不能据此声称已经看到斑马线。'.format(road.get('active_mode'), road.get('counts', {}).get('processed', 0)))
    traffic = values.get('/traffic/state')
    if traffic:
        add('交通状态机', 'PASS', '当前 state={}，traffic_active={}；这里只读取当前状态，不模拟红灯/绿灯。'.format(traffic.get('state'), traffic.get('traffic_active')))
    camera = values.get('/diagnostics/camera')
    if camera:
        add('相机输入', 'PASS' if camera.get('ok') is True else 'WARN',
            ('物理左眼输入正常；这条诊断不证明右眼、曝光同步或测距已验收。' if camera.get('ok') is True
             else '相机诊断：' + str(camera.get('reason', '未正常接收'))))
    perf = values.get('/diagnostics/perception')
    if perf:
        if perf.get('stage_errors'):
            add('视觉管线', 'WARN', '阶段错误：' + str(perf['stage_errors']))
        elif perf.get('detector_active') is True:
            age = perf.get('processing_last_age_s')
            add('YOLO', 'PASS' if number(age) and 0 <= age < 1.5 else 'WARN',
                '处理帧率 {} FPS；最后处理距现在 {} 秒。'.format(perf.get('processing_fps', '未知'), age))
        else:
            add('YOLO', 'WAIT', '当前未运行检测；standby/道路任务按模式关闭 YOLO 属于正常节省计算。检查 detector_active 后才把 processing_fps 当作识别帧率。')
        constraints = {'capture_resolution': [1280, 1088], 'model_resolution': [640, 640], 'jpeg_quality': 90}
        mismatches = {key: perf.get(key) for key, expected in constraints.items() if perf.get(key) != expected}
        if set(perf.get('enabled_classes') or []) != {'fire', 'firefighter', 'person'}:
            mismatches['enabled_classes'] = perf.get('enabled_classes')
        add('当前画质配置', 'WARN' if mismatches else 'PASS',
            '与当前标准不同或缺失：' + str(mismatches) if mismatches else '1280×1088、模型 640×640、JPEG 90，三类均启用；这是配置检查，不代表实拍准确率验收。')
    ocr = values.get('/diagnostics/building_ocr')
    if ocr:
        if not ocr.get('enabled') or not ocr.get('active'):
            add('楼牌 OCR', 'WAIT', '当前模式未启用或未激活 OCR；进入观察模式并展示楼牌后再验收识别。')
        elif not ocr.get('reader_available') or ocr.get('reader_error'):
            add('楼牌 OCR', 'WARN', 'OCR 模型不可用：' + str(ocr.get('reader_error')))
        else:
            add('楼牌 OCR', 'PASS', 'OCR 模型可用；历史完成数 {}，仍需用真实楼牌检查本次识别。'.format(ocr.get('counts', {}).get('completed', 0)))
    extension = values.get('/extension/state')
    fpga = values.get('/fpga/link')
    if extension and extension.get('real_uart') is True:
        ready = extension.get('protocol_ready') is True and extension.get('status_fresh') is True
        add('FPGA UART', 'PASS' if ready else 'WARN',
            '拓展任务收到真实协议及新鲜FPGA状态；旧桥接器的dry_run不代表本链路断开。' if ready else
            '拓展UART已配置，但协议或状态未就绪：'+str(extension.get('reason', '等待协议和回包')))
    elif fpga:
        if fpga.get('dry_run') is True:
            add('FPGA UART', 'WAIT', '当前演练模式，没有向串口发送实车导航；未接 FPGA 回执不算程序崩溃。')
        elif fpga.get('status_fresh') is not True or fpga.get('error'):
            add('FPGA UART', 'WARN', '真实 UART 模式但未收到新鲜状态或存在错误：' + str(fpga.get('error', '')))
        else:
            add('FPGA UART', 'PASS', '收到新鲜链路状态；仅证明通信，不证明导航精度。')
    actuator = values.get('/actuators/state')
    if actuator:
        if actuator.get('error'):
            add('云台/水泵', 'WARN', '执行器错误：' + str(actuator['error']))
        elif actuator.get('dry_run') is True or not actuator.get('enabled'):
            add('云台/水泵', 'WAIT', '硬件输出禁用或演练中；不会由本检查启用。软件角度指令不等于实际角度反馈。')
        else:
            missing = []
            if actuator.get('feedback_required') is True:
                if actuator.get('feedback_fresh') is not True: missing.append('双轴编码器回包过期或缺失')
                angles = actuator.get('actual_angles_deg')
                if not isinstance(angles, list) or len(angles) != 2 or not all(number(v) for v in angles):
                    missing.append('缺少两轴实际角度')
                if actuator.get('reference_confirmed_this_start') is not True: missing.append('本次上电机械零位未确认')
            if actuator.get('status_fresh') is not True: missing.append('执行器状态未就绪')
            add('云台/水泵', 'WAIT' if missing else 'PASS',
                '；'.join(missing) if missing else '真实角度反馈和本次参考有效；泵与瞄准条件单独检查。')
        manual = values.get('/gimbal/manual/state')
        if manual:
            axes = manual.get('motion_axes') or []
            add('云台各轴', 'PASS' if set(axes) == {'pan', 'tilt'} else 'WAIT',
                '开放运动轴={}；当前条件={}。方向未验收的轴不能以使能回包代替。'.format(axes, manual.get('ready_reason')))
        if actuator.get('pump_backend') == 'fpga_uart':
            ready = actuator.get('pump_enabled') is True and actuator.get('pump_ready') is True and actuator.get('pump_feedback_fresh') is True
            add('B13水泵输出', 'PASS' if ready else 'WAIT',
                '使能={}，真实回读={}，就绪={}，原因={}；电平回读不证明水流落点。'.format(
                    actuator.get('pump_enabled'), actuator.get('pump_feedback_fresh'),
                    actuator.get('pump_ready'), actuator.get('pump_reason')))
    proposal = values.get('/fire/proposal')
    if proposal:
        add('固定相机瞄准', 'PASS' if proposal.get('valid') is True and proposal.get('calibration_ready') is True else 'WAIT',
            '当前原因={}，标定就绪={}，落点补偿={}；机械参考不等于相机外参或落点标定。'.format(
                proposal.get('reason'), proposal.get('calibration_ready'), proposal.get('landing_compensated')))
    stereo = values.get('/stereo/depth_status')
    if stereo:
        add('双目测距', 'WAIT', '状态 {}，门控 {}。深度有效性需左右眼、同步、标定及已知距离独立验收。'.format(stereo.get('status'), stereo.get('gate')))
    voice = values.get('/voice/status')
    if voice:
        mic, tts = voice.get('microphone_enabled'), voice.get('tts_enabled')
        issue = voice.get('tts_error') or voice.get('tts_device_ready') is False
        status = 'WARN' if issue else 'PASS' if mic is True and tts is True else 'WAIT'
        add('语音输入/播报', status,
            '麦克风启用={}，播报启用={}，输出设备就绪={}，播报错误={}；设备实录和听感需另外检查。'.format(
                mic, tts, voice.get('tts_device_ready'), voice.get('tts_error') or '无'))
    preview = report.get('preview', {})
    if preview.get('error'):
        add('网页预览', 'WARN', '状态接口无法读取：' + str(preview['error']))
    elif isinstance(preview.get('value'), dict):
        state = preview['value']
        enabled = (state.get('stream') or {}).get('enabled')
        active = (state.get('stream') or {}).get('active')
        status = 'PASS' if state.get('state') == 'live' else 'WAIT'
        add('网页预览', status, '接口可访问，state={}，请求传图={}，当前租约活动={}；只读了元数据，未开启或续租传图。'.format(state.get('state'), enabled, active))
    else:
        add('网页预览', 'WAIT', '未获得状态接口结果。')
    if report.get('llm', {}).get('ok') is True:
        add('Qwen 服务', 'PASS', '本地 /health 返回正常；未发送推理请求，回答能力需单独提问。')
    else:
        add('Qwen 服务', 'WARN', '本地模型健康接口未就绪：' + str(report.get('llm', {}).get('error', 'unknown')))
    return checks


def probe_services():
    try:
        process = subprocess.run(['systemctl', 'show', *SERVICES, '--no-pager',
            '--property=Id,LoadState,ActiveState,SubState'], capture_output=True, text=True, timeout=5)
        result = {}
        for block in process.stdout.strip().split('\n\n'):
            values = dict(line.split('=', 1) for line in block.splitlines() if '=' in line)
            if values.get('Id'):
                result[values['Id']] = values
        return {unit: result.get(unit, {'error': process.stderr.strip() or 'unit not returned'}) for unit in SERVICES}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {unit: {'error': str(exc)} for unit in SERVICES}


def probe_json(url):
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(url, timeout=3) as response:
            data = response.read(1024 * 1024 + 1)
        if len(data) > 1024 * 1024:
            raise ValueError('状态响应超过 1 MiB')
        value = json.loads(data)
        if not isinstance(value, dict):
            raise ValueError('状态响应不是 JSON 对象')
        return {'value': value, 'ok': True}
    except (OSError, ValueError) as exc:
        return {'error': str(exc), 'ok': False}


def collect_ros(report, seconds):
    if report['environment']['ros_domain_id'] != '0' or report['environment']['ros_localhost_only'] != '0':
        report['ros_error'] = '未在错误通信域订阅；请按上面的域 0 命令重新执行。'
        return
    node = None
    started = False
    try:
        import rclpy
        from std_msgs.msg import String
        rclpy.init(args=[])
        started = True
        node = rclpy.create_node('rescue_readonly_self_check_' + str(os.getpid()))
        def receive(topic, message):
            entry = report['topics'].setdefault(topic, {'count': 0, 'invalid_count': 0})
            try:
                value = json.loads(message.data)
                if not isinstance(value, dict):
                    raise ValueError('not an object')
                entry.update(value=value, last_received_monotonic=time.monotonic())
                entry['count'] += 1
            except (ValueError, TypeError):
                entry['invalid_count'] += 1
        for topic in TOPICS:
            node.create_subscription(String, topic, lambda message, name=topic: receive(name, message), 10)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline-time.monotonic())))
        end = time.monotonic()
        for entry in report['topics'].values():
            if 'last_received_monotonic' in entry:
                entry['last_received_age_s'] = round(end-entry.pop('last_received_monotonic'), 4)
    except Exception as exc:
        report['ros_error'] = '{}: {}。请在 X5 终端 cd /root/rescue/ros2_ws，设置域 0 后 source env.sh；Windows 不能直接运行板端 ROS。'.format(type(exc).__name__, exc)
    finally:
        if node is not None:
            node.destroy_node()
        if started:
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5., help='ROS 状态采样秒数，1–60；默认 5')
    parser.add_argument('--output', type=Path, default=Path('reports/self_check/latest.json'))
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 60:
        parser.error('--seconds must be between 1 and 60')
    report = dict(schema_version=1, readonly=True, physical_acceptance=False,
        generated_utc=datetime.now(timezone.utc).isoformat(), requested_seconds=args.seconds,
        environment=dict(ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
                         ros_localhost_only=os.environ.get('ROS_LOCALHOST_ONLY'),
                         ros_distro=os.environ.get('ROS_DISTRO')),
        topics={})
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        services = pool.submit(probe_services)
        preview = pool.submit(probe_json, PREVIEW_URL)
        llm = pool.submit(probe_json, 'http://127.0.0.1:8088/health')
        collect_ros(report, args.seconds)
        report.update(services=services.result(), preview=preview.result(), llm=llm.result())
    report['checks'] = evaluate(report)
    report['counts'] = {status: sum(c['status'] == status for c in report['checks']) for status in ('PASS', 'WARN', 'WAIT')}
    report['overall'] = 'WARN' if report['counts']['WARN'] else 'WAIT' if report['counts']['WAIT'] else 'PASS'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('只读检查：PASS=本项软件检查通过；WAIT=待启用/待观测/待实测；WARN=需检查。')
    for check in report['checks']:
        print('[{status}] {name}：{message}'.format(**check))
    print('汇总：{}；报告：{}'.format(report['counts'], args.output.resolve()))
    print('本检查未启动/停止服务，未发布任何任务、运动或喷水请求；不等于真实硬件验收。')
    return 1 if report['counts']['WARN'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
