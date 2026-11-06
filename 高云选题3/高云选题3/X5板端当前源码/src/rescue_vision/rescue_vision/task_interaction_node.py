"""The sole voice/text task router; numbered navigation uses explicit rules."""
import json
import re
import time

from .node_common import Node
from std_msgs.msg import String
from .node_common import param, read_json, publish_json, spin
from .task_interaction import TaskRouter, parse_task_text


def region_rejection_speech(reason):
    explanations = {
        'region_search_requires_explicit_region_and_fire': '请说清区域和任务，例如：去C区域搜索火焰；需要自动灭火时，请加上并执行灭火。',
        'region_search_requires_explicit_text': '请使用完整区域口令，例如：去C区域搜索火焰。',
        'text_region_mismatch': '区域或灭火要求与原始口令不一致，请重新输入。',
        'text_action_mismatch': '请求动作与原始口令不一致，请重新输入。',
        'region_search_independent_asr_required': '区域和搜索任务尚未通过独立语音核对，请核对转写或改用文字。',
        'real_region_search_not_enabled': '区域任务执行入口尚未启用。',
        'region_search_requires_extension2': '请先开启拓展任务二。',
        'region_requires_extension2': '请先开启拓展任务二。',
        'select_observation_task_for_voice_and_dialogue': '请先开启拓展任务二，再下达区域语音指令。',
        'real_extension_uart_required': '尚未收到新鲜的真实FPGA串口状态，请检查连接。',
        'real_extension_uart_and_fresh_feedback_required': '尚未收到新鲜的真实FPGA串口状态，请检查连接。',
        'semantic_region_catalog_unavailable': '区域地图目录尚未加载。',
        'region_start_reference_query_required': '正在等待FPGA起点确认序号回读，请稍后重试。',
        'region_start_not_in_clear_map_corridor': '起点不在已验证的可通行道路内，请核对实际摆车位置。',
        'semantic_region_fpga_firmware_required': '请先更新支持区域导航的FPGA固件。',
        'semantic_region_map_identity_mismatch': '板端地图与区域地图不一致，请更新配套地图和固件。',
        'select_fpga_communication_navigation_page': '请在FPGA屏幕选择通信导航页。',
        'mark_start_and_heading_on_fpga_or_hmi': '请先在地图上确认实际起点和车头方向。',
        'region_start_pose_and_heading_required': '请先在地图上确认实际起点和车头方向。',
        'region_start_requires_measured_stop': '尚未确认车辆实际停稳。',
        'region_or_navigation_busy': '已有区域任务或导航正在进行，请先结束当前任务。',
        'extension_movement_in_progress': '已有移动任务正在进行，请先结束当前任务。',
        'region_pump_off_and_disarmed_required': '尚未确认水泵已关闭并解除喷水授权。',
        'region_real_gimbal_reference_required': '双轴实际角度或本次安装参考尚未确认。',
        'region_pump_b13_not_ready': 'FPGA水泵控制输出尚未就绪，请检查B13反馈。',
        'region_pump_b13_firmware_required': '当前FPGA固件尚未回报B13水泵控制能力。',
        'region_pump_output_disabled': 'B13水泵通道已配置，但自动喷水输出尚未启用。',
        'region_pump_feedback_stale': '正在等待新鲜的B13水泵状态回读，请检查FPGA通信。',
        'region_pump_vehicle_moving': 'FPGA尚未确认车辆停稳，不能开始喷水。',
        'region_pump_scope_required': '请在FPGA屏幕选择允许通信控制的任务页面。',
        'region_pump_link_required': 'FPGA水泵通信心跳尚未恢复。',
        'region_pump_off_reset_required': '水泵已停水锁定，请先点击停止喷水，等待关闭状态确认。',
        'region_mount_or_priority_state_not_ready': '安装标定或任务状态尚未就绪，请查看上位机具体原因。',
        'region_priority_state_not_ready': '当前有急停或其他优先任务，请先处理后再搜索。',
        'region_stop_or_pump_off_not_confirmed': '停车或关水反馈尚未确认，请检查上位机状态。',
        'region_auto_controller_not_idle': '请等待上一轮自动灭火结束。',
        'region_live_yolo_required': '尚未收到新鲜的火源视觉检测结果。',
        'region_approach_calibration_required': '请先完成双目、相机安装和喷水距离标定。',
        'region_approach_calibration_verified_required': '双目与相机安装标定尚未验收。',
        'region_approach_map_registration_verified_required': '相机到地图的实测对应关系尚未验收。',
        'region_approach_spray_range_verified_required': '实测喷水距离尚未验收。',
        'region_approach_body_envelope_verified_required': '车身尺寸与道路净空尚未验收。',
        'region_approach_time_sync_verified_required': '相机曝光与惯导时间对应关系尚未验收。',
        'expired_or_future_command': '指令已过期或发送时间不正确，请重新下达。',
        'retired_patrol_fire_use_region_search': '旧的沿途自动灭火口令已停用，请改用：在A区域搜寻火源并执行灭火。',
    }
    detail = explanations.get(reason)
    if detail is None and reason.startswith('region_approach_'):
        detail = '自动靠近的地图或实测几何配置尚未就绪，请查看上位机具体原因。'
    return (detail or '区域任务条件尚未就绪，请查看上位机具体原因。') + ' 本次未下发区域任务。'


class TaskInteraction(Node):
    def __init__(self):
        super().__init__('task_interaction')
        config = param(self, 'buildings_config', '')
        self.router = TaskRouter(read_json(config) if config else None,
                                 param(self, 'navigation_execute_enabled', False),
                                 param(self, 'pump_lease_s', 5.),
                                 param(self, 'manual_motion_enabled', False),
                                 param(self, 'extension_enabled', False))
        self.cache = {}
        self.output_pubs = {key: self.create_publisher(String, topic, 10) for key, topic in (
            ('mission', '/mission/request'), ('navigation', '/navigation/request'),
            ('fire_select', '/fire/select'), ('actuator', '/actuators/request'),
            ('question', '/scene/question'), ('speech', '/speech/say'), ('auto', '/fire/auto/request'),
            ('extension', '/extension/request'), ('pump_manual', '/pump/manual/request'))}
        self.result = self.create_publisher(String, '/interaction/result', 10)
        self.create_subscription(String, '/interaction/command', self.receive, 10)
        for key, topic in (('mission', '/mission/state'), ('proposal', '/fire/proposal'),
                           ('selection', '/fire/selection'), ('actuator', '/actuators/state'),
                           ('telemetry', '/fpga/telemetry'), ('link', '/fpga/link'),
                           ('navigation', '/navigation/state'), ('extension', '/extension/state'),
                           ('pump_manual', '/pump/manual/state')):
            self.create_subscription(String, topic, lambda msg, k=key: self.update(k, msg), 10)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def live_state(self):
        return {k: v for k, (v, received) in self.cache.items()
                if 0 <= time.monotonic()-received <= .5}

    def update(self, key, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('expected_object')
            self.cache[key] = (data, time.monotonic())
        except (ValueError, TypeError):
            self.cache.pop(key, None)

    def receive(self, msg):
        now = self.now()
        request = None
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValueError('command_must_be_object')
            # Compatibility for queued commands from the former UI: the original
            # words are reparsed locally. No model request, worker or model wait.
            if request.get('action') in ('semantic_navigate', 'dialogue') and request.get('text'):
                parsed = parse_task_text(request['text'], self.router.buildings)
                if request.get('action') == 'semantic_navigate' or parsed['action'] in ('navigate_point', 'search_region'):
                    request = dict(request, **parsed)
            if request.get('action') == 'semantic_navigate':
                raise ValueError('numbered_navigation_needs_clear_points')
            self.dispatch(request, now)
        except (ValueError, TypeError, KeyError) as exc:
            result = dict(accepted=False, reason=str(exc), stamp=now)
            cid = request.get('command_id') if isinstance(request, dict) else None
            if isinstance(cid, str) and re.fullmatch(r'[A-Za-z0-9_:.-]{1,100}', cid):
                result['command_id'] = cid
            if isinstance(request, dict) and (request.get('action') == 'search_region'
                    or str(exc).startswith(('region_', 'semantic_region_'))
                    or str(exc) == 'retired_patrol_fire_use_region_search'):
                result.update(action='search_region', navigation_parser='rules',
                    rule_navigation=dict(original_text=request.get('text', ''),
                        region_id=request.get('region_id'), execute_fire=request.get('execute_fire'),
                        model_called=False))
                result['speech'] = region_rejection_speech(str(exc))
            elif isinstance(request, dict) and (request.get('action') in ('navigate_point', 'semantic_navigate')
                    or str(exc).startswith('numbered_navigation_')):
                result.update(action='navigate_point', navigation_parser='rules',
                    rule_navigation=dict(original_text=request.get('text', ''),
                        start_point=request.get('start_point'), target_point=request.get('target_point'),
                        start_heading_deg=request.get('start_heading_deg'), model_called=False))
                explanations = {
                    'numbered_start_heading_required': '请补充车头的地图方向，例如：你现在在8号点位，车头朝左，请前往4号点位。',
                    'numbered_navigation_needs_clear_points': '请说清起点、目标点和车头方向，例如：你现在在8号点位，车头朝左，请前往4号点位。',
                    'numbered_navigation_separate_actions': '请先单独下达导航指令，瞄准和喷水另行下达。',
                    'numbered_navigation_independent_asr_required': '导航点号或方向尚未通过独立语音核对，请核对转写或改用文字。',
                }
                result['speech'] = explanations.get(str(exc), '导航未执行：' + str(exc)) + ' 本次未下发导航。'
            publish_json(self.result, result)
            self.output_pubs['speech'].publish(String(data=result.get('speech', '请求未执行：' + str(exc))))

    def dispatch(self, request, now):
        result = self.router.route(request, now, **self.live_state())
        if result['accepted']:
            order = ('auto', 'actuator', 'extension', 'mission', 'navigation', 'fire_select', 'pump_manual', 'question', 'speech')
            if (result.get('auto') or {}).get('action') == 'start':
                order = ('actuator', 'extension', 'mission', 'navigation', 'auto', 'fire_select', 'pump_manual', 'question', 'speech')
            for key in order:
                value = result.get(key)
                if value is not None:
                    if key == 'speech':
                        self.output_pubs[key].publish(String(data=value))
                    else:
                        publish_json(self.output_pubs[key], value)
        publish_json(self.result, result)
        return result


def main():
    spin(TaskInteraction)
