"""Read-only question routing and grounded status summaries; no control side effects."""
import math
import re


MAX_REPLY_CHARS = 180
KNOWLEDGE_WIRE_CHARS = 90
MAX_CONFIGURED_REPLY_CHARS = 320
MODEL_ERROR_CODES = frozenset((
    'redirect_forbidden', 'response_too_large', 'generation_truncated_or_incomplete',
    'invalid_model_content', 'duplicate_json_key', 'invalid_knowledge_reply_schema',
    'reply_limit_without_complete_sentence', 'invalid_model_intent', 'invalid_generation_schema',
    'current_view_requires_empty_reply', 'model_cannot_override_known_question_intent',
    'observation_hint_not_relevant_to_question', 'knowledge_requires_fixed_intent_and_hint',
    'ungrounded_generated_claim',
    'ungrounded_scene_analysis', 'scene_snapshot_unavailable',
))


def safe_model_error(exc):
    """Expose known diagnostic codes, never arbitrary network/response exception text."""
    message = str(exc)
    return message if isinstance(exc, ValueError) and message in MODEL_ERROR_CODES else type(exc).__name__
CAPABILITIES = (
    '可以问：当前有几个火焰、几名普通人物、有没有消防员、楼牌是什么、水泵状态；'
    '也能问麦克风、FPGA和导航状态，或解释YOLO、双目、ROS2及普通知识、日常聊天。'
    '现场事实来自有时效的检测和节点反馈，知识聊天由本地Qwen回答；'
    '问答本身不执行导航或喷水，也没有联网搜索。')


def normalized_question(text):
    return re.sub(r'\s+', '', text) if isinstance(text, str) else ''


def capability_question(text):
    text = normalized_question(text)
    return any(term in text for term in ('可以问什么', '能问什么', '你能回答', '你会什么', '怎么提问', '能聊什么'))


def model_question(text):
    text = normalized_question(text)
    return any(term in text for term in ('你是谁', '介绍一下自己', '自我介绍', '你是什么模型', '你用的什么模型', '你用什么模型', '模型型号', '你是大模型吗'))


def knowledge_question(text):
    """Generic explanations containing fire/person/building keywords are not a live view.

    Concrete view references win over explanation wording. Bare '现在' may occur
    in ordinary requests such as '现在给我解释YOLO', so it is not a view by itself.
    """
    text = normalized_question(text)
    if any(term in text for term in ('当前', '目前')):
        return False
    if re.sub(r'[?？。！!，,]', '', text) in ('楼牌是什么', '楼号是什么', '楼栋名称是什么', '这是什么楼'):
        return False
    if any(term in text for term in ('当前场景', '当前火场', '当前画面', '现在有', '现在是几',
            '这幅', '画面里', '画面中', '我看到', '你看到', '看到了', '现场', '眼前',
            '周围', '这里', '这儿', '附近', '此处', '这栋', '那栋', '受困', '受难',
            '灭完', '灭火完成', '扑灭了吗', '喷水了吗', '几个人', '多少人', '几处火')):
        return False
    if any(term in text for term in ('为什么喷水停止', '停止不等于扑灭')):
        return False  # Answer from the verified pump-state/verification contract.
    return any(term in text for term in ('是什么', '什么是', '为什么', '原理', '如何', '怎么',
        '介绍', '解释', '讲解', '科普', '区别', '职责', '建议', '步骤', '方案', '聊聊', '讲个'))


def system_question(text):
    text = normalized_question(text)
    if model_question(text):
        return True
    subject = any(term.lower() in text.lower() for term in (
        '系统', '硬件', '麦克风', '语音', '音箱', '扬声器', 'FPGA', '导航', '任务', '红绿灯',
        '斑马线', '相机', '摄像头', 'YOLO', '云台', '舵机'))
    status = any(term in text for term in ('状态', '接好', '接上', '连接', '在线', '离线',
        '可用', '能用', '工作吗', '工作了', '运行吗', '启用', '正在', '当前模式', '什么模式', '到达了吗', '到哪里'))
    return subject and status and not knowledge_question(text)


def latest_information_question(text):
    text = normalized_question(text)
    return any(term in text for term in ('今天天气', '明天天气', '天气预报', '最新新闻',
        '最新消息', '现在几点', '查一下网络', '上网查', '联网搜索', '实时股价'))


def project_reference(text):
    """Audited project explanations do not depend on a 0.5B model's factual recall."""
    clean = normalized_question(text)
    if not knowledge_question(clean) or system_question(clean):
        return None
    lower = clean.lower()
    if any(word in clean for word in ('漏检', '误检', '识别不准')) and any(word in lower for word in ('火焰', 'yolo', '检测')):
        return ('火焰检测漏检可能来自目标像素太少、角度或光照变化、遮挡、训练样本不匹配，以及阈值过高。'
            '本项目应先检查目标大小和原始画面，再按距离、角度、光照分别统计漏检，补相应实拍样本并复验；不能仅靠降低阈值判断效果。')
    if 'ros2' in lower or 'ros节点' in lower:
        return ('ROS2节点是执行一项功能的计算单元，可通过话题发布/订阅数据，也可用服务或动作通信。'
            '一个进程可以包含一个或多个节点。本项目相机、识别、语音和任务节点彼此传消息；发布命令只表示提出请求，实际完成仍需执行节点回执。')
    if 'yolo' in lower:
        return ('YOLO是目标检测模型，将图像中的目标输出为类别、边界框和置信度。'
            '本项目YOLO11n检测火焰、普通人物和消防员壁纸图案，在X5 BPU推理。'
            '检测框不等于双目距离，也不能直接证明人员受困或实际火灾已扑灭；楼牌文字由OCR处理。')
    if 'fpga' in lower:
        return ('本项目分工是FPGA负责已存地图上的导航、运动控制和车辆状态反馈，X5负责视觉识别、语音、场景问答与任务协调，二者通过UART通信。'
            '拓展一提供视觉观察与语音场景问答，拓展二提供编号点语义导航、到达停稳后YOLO识别和指定火源控制；纯识别与纯对话不需要UART，车辆运动需要。'
            '你选定火焰后才请求双轴瞄准，喷水需明确开始和停止指令；任务按钮本身不会启动车辆或喷水。'
            'FPGA斑马线识别有独立开关，只在专用交通任务中触发停车；自主导航不再因斑马线限速。'
            '惯导会累积误差，需要结合编码器和实测验收；执行成功须查真实回执。')
    return None


def status_evidence(value, now, ttl):
    """Require a recent node reception and, if present, a recent source timestamp."""
    if not isinstance(value, dict):
        return {}
    def recent(stamp):
        return type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= now-stamp <= ttl
    if not recent(value.get('_received_stamp')):
        return {}
    if 'stamp' in value and not recent(value['stamp']):
        return {}
    return value


def system_answer(question, report, now, model='Qwen2.5-0.5B-Instruct'):
    question = normalized_question(question)
    if model_question(question):
        return ('我是消防小车的本地语音助手，知识问答使用' + model +
            '。它是量化文本模型，运行在X5 CPU；相机目标由YOLO和OCR识别，再生成结构化事实。没有联网搜索，回答不会自动控制小车。')
    system = report.get('system_status') or {}
    voice = status_evidence(system.get('voice'), now, 12.)
    fpga = status_evidence(system.get('fpga_link'), now, 1.5)
    nav = status_evidence(system.get('navigation'), now, 1.5)
    traffic = status_evidence(system.get('traffic'), now, 1.5)
    mission = status_evidence(system.get('mission'), now, 1.5)
    actuator = status_evidence(system.get('actuator'), now, .5)
    extension = status_evidence(system.get('extension'), now, .6)
    extension_profile = 'extension' in system
    lowered = question.lower()
    all_status = any(term in question for term in ('系统', '硬件', '整体', '全部'))
    lines = []
    if all_status or any(term in question for term in ('麦克风', '语音', '音箱', '扬声器')):
        if not voice:
            lines.append('语音节点状态未知，尚无新鲜反馈。')
        else:
            microphone = '已启用' if voice.get('microphone_enabled') is True else '未启用'
            capture = '正在采集声音' if voice.get('capture_active') is True else '当前未采集声音'
            tts = '已启用' if voice.get('tts_enabled') is True else '未启用'
            lines.append(f'麦克风配置{microphone}，{capture}；语音播报{tts}。配置启用不等于已验证无线配对或听懂说话。')
    if all_status or 'fpga' in lowered:
        if extension_profile:
            lines.append('FPGA拓展协议暂无新鲜状态反馈。' if not extension else
                'FPGA真实UART未启用；视觉观察和纯对话仍可独立运行。' if extension.get('real_uart') is not True else
                'FPGA已收到新鲜的ov5640拓展协议反馈；命令接收与实际完成分别确认。' if
                    extension.get('protocol_ready') is True and extension.get('status_fresh') is True else
                'FPGA拓展协议未确认有效通信，请检查串口和固件。')
        else:
            lines.append('FPGA暂无新鲜状态反馈。' if not fpga else
            'FPGA通信为模拟模式，未确认实车连接。' if fpga.get('dry_run') is True else
            'FPGA已收到新鲜通信状态。' if fpga.get('dry_run') is False and fpga.get('status_fresh') is True and not fpga.get('error') else
            'FPGA暂未确认有效实车通信，请检查UART和板端程序。')
    if all_status or any(term in question for term in ('导航', '任务', '模式')):
        modes = {'patrol': 'FPGA巡逻', 'inspect': '目标识别', 'fire': '火焰跟踪', 'idle': '待机',
                 'standby': '待机', 'traffic': '独立交通识别', 'semantic': '语义运动',
                 'navigate': '导航', 'scene': '场景理解'}
        extension_modes = {'vision': '拓展一：视觉观察与问答', 'dialogue': '拓展二：对话与控制',
                           'semantic': '拓展二：语义运动', 'traffic': '独立交通任务'}
        mode_name = (extension_modes.get(extension.get('task'), '未知') if extension_profile
                     else modes.get(mission.get('mode'), '未知'))
        lines.append('任务模式：' + mode_name + '。')
        numbered = extension.get('numbered_navigation') or {}
        if (('导航' in question or '到达' in question or '任务' in question) and extension_profile
                and numbered.get('command_id')):
            point = numbered.get('target_point')
            goal = '{}号点'.format(point) if type(point) is int and 1 <= point <= 9 else '未确认点位'
            stamp = numbered.get('stamp')
            current = (type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= now-stamp <= .8
                       and numbered.get('status_fresh') is True)
            if (current and numbered.get('valid') is True and numbered.get('arrived') is True
                    and numbered.get('phase') == 'arrived' and numbered.get('vehicle_stopped') is True):
                lines.append('已收到FPGA到达{}及停稳反馈。'.format(goal))
            else:
                phases = {'setting_start': '正在设置已确认起点', 'selecting_point': '正在提交目标点',
                          'planning': 'FPGA正在规划路径', 'starting': '正在等待启动回执',
                          'running': 'FPGA正在执行导航', 'confirming_arrival': '已收到完成事件，正在核对停稳',
                          'stopping': '正在等待停车反馈', 'cancelled': '导航已取消', 'failed': '导航已中断'}
                description = phases.get(numbered.get('phase'), '尚未确认到达') if current else '反馈已过期，不能确认当前位置'
                lines.append('导航目标为{}；{}。'.format(goal, description))
        elif ('导航' in question or '到达' in question or '任务' in question) and extension_profile:
            active = extension.get('active') or {}
            result = extension.get('last_result') or {}
            phase = active.get('phase')
            finished = (phase == 'completed' and result.get('completed') is True and
                        result.get('command_id') == active.get('command_id') and
                        extension.get('status_fresh') is True and extension.get('vehicle_stopped') is True)
            labels = {'waiting_mode': '等待FPGA切换任务', 'waiting_ack': '等待FPGA接收指令',
                      'uploading_route': '正在上传路线约束', 'planning': 'FPGA正在规划路径',
                      'moving': '车辆正在执行运动', 'seeking': '正在侧视寻找火焰',
                      'stopping': '已请求停车，等待实测停稳', 'fault': '任务中断，需查看故障原因'}
            lines.append('拓展任务暂无新鲜状态，不能确认运动结果。' if not extension else
                         '拓展任务已收到完成及停稳反馈。' if finished else
                         '当前没有正在执行的拓展运动任务。' if not active else
                         labels.get(phase, '尚无有效的完成且停稳确认')+'。')
        elif '导航' in question or '到达' in question:
            proposal = nav.get('proposal') or {}
            number = nav.get('building_number')
            goal = str(number) + '号楼' if isinstance(number, (int, str)) and re.fullmatch(r'\d{1,3}', str(number)) else '未确认目标'
            lines.append('导航节点暂无新鲜反馈，不能确认是否到达。' if not nav else
                '导航目标是' + goal + '；当前仅为预览，未执行实车导航。' if nav.get('mode') == 'preview' else
                'FPGA有效回执确认已到达' + goal + '。' if proposal.get('valid') is True and proposal.get('status_fresh') is True and proposal.get('arrived') is True and proposal.get('vehicle_stopped') is True else
                '导航目标是' + goal + '；尚无有效的到达且停稳确认。')
    if all_status or any(term in question for term in ('相机', '摄像头')) or 'yolo' in lowered:
        live = report.get('stale') is False and report.get('detector_available') is True
        stamp = report.get('stamp')
        live = live and type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= now-stamp <= 1.5
        lines.append('YOLO有新鲜检测帧；具体目标请问当前场景。' if live else '暂无新鲜YOLO检测帧，不能确认当前相机识别状态。')
    if any(term in question for term in ('红绿灯', '斑马线')):
        lines.append('斑马线由FPGA识别。')
        if extension_profile:
            switch_known = (extension.get('real_uart') is True and extension.get('status_fresh') is True
                            and extension.get('protocol_ready') is True
                            and (extension.get('capabilities') or {}).get('zebra_switch') is True)
            enabled = extension.get('zebra_detection_enabled') if switch_known else None
            lines.append('识别开关已开启。' if enabled is True else
                         '识别开关已关闭。' if enabled is False else '识别开关状态未知，需新鲜FPGA回读。')
            lines.append('自主导航不因斑马线限速。')
        if extension_profile and extension.get('task') != 'traffic':
            lines.append('尚未确认独立交通任务开启；其他任务不使用灯色放行。')
        else:
            lines.append('红绿灯流程暂无新鲜状态。' if not traffic else
                '红绿灯流程有新鲜节点反馈；实际灯色和放行以当前识别反馈及流程判据为准。')
    if any(term in question for term in ('云台', '舵机')):
        lines.append('云台执行器暂无新鲜状态。' if not actuator else
            '云台执行器处于模拟输出，未确认真实驱动。' if actuator.get('dry_run') is True else
            '云台执行器已启用；实际到位仍需驱动反馈。' if actuator.get('enabled') is True and actuator.get('dry_run') is False and not actuator.get('error') else
            '云台真实输出未就绪。')
    return ''.join(lines)[:650] or '尚无与该问题对应的已验证节点状态。'


GENERAL_SYSTEM_PROMPT = (
    '你是本地中文助手。用简体中文直接回答问题，1到2句约40到80字，按JSON格式答。'
    '你是离线文本模型，无相机、联网或控制权限；不能声称看到现场、查询网络、执行动作或灭火成功。'
    '火灾建议以远离危险、报警为先；不知道就说明。写作请求只输出所需文稿。')


def general_system_prompt(limit):
    """A larger model may use a larger budget; small-talk remains inexpensive."""
    return (f'你是本地中文助手。直接回答，先结论再解释，简单问题简答，最多{limit}字。'
        '只输出JSON；没有相机、联网或控制权限，不知道则说明。')


def brief_question(question):
    text = normalized_question(question).strip('。！？!?，,')
    return text in ('你好', '您好', '嗨', '早上好', '晚上好', '谢谢', '谢谢你', '再见', '你好吗', '在吗')


def _analysis_parts(reply):
    """Carry advice wording across a comma, never across an adversative or sentence."""
    pieces = re.split(r'([。！？!?；;，,\n]|但是|然而|不过|并且|而且|同时|但|且|而|并)', reply)
    contexts, advising = {}, False
    for index in range(0, len(pieces), 2):
        if index and re.fullmatch(r'[。！？!?；;\n]|但是|然而|不过|但|而', pieces[index-1]):
            advising = False
        if re.match(r'^(?:建议|应当|应|可以|可考虑|请|需要|需|优先|首先|先|再|随后|进一步)', pieces[index].strip()):
            advising = True
        contexts[index] = advising
    return pieces, contexts


def scene_outcome_claims(reply):
    """Detect unverified outcomes while keeping uncertainty scoped to one clause.

    Return a copy with genuinely hypothetical outcomes masked so older positive
    claim checks do not reject e.g. "尚不能确认已扑灭". An uncertainty about a
    different noun must not excuse a following success claim after "但" or a comma.
    """
    outcome = re.compile(
        r'扑灭|熄灭|灭掉|(?:扑救|灭火|救援)(?:已经|已|全部|成功)?(?:完成|成功|结束)|'
        r'救出|获救|脱困|被困|受困|受难|死亡|伤亡|脱离危险|'
        r'危险(?:已经|已|完全)?解除|(?:消除|解除).{0,4}(?:危险|风险)|'
        r'(?:人员|居民|所有人|大家|受困者|人群|群众)(?:已经|已|都|全部|均)?(?:安全|无危险)')
    pieces, contexts = _analysis_parts(reply)
    subject = r'(?:火焰|火苗|火灾|火|人员|居民|人物|目标|所有人|大家|是否|已经|完全|真实|实际|真正|全部|已|被|的|是|均|仍|还|都|有|存在)*'
    for index in range(0, len(pieces), 2):
        clause = pieces[index]
        matches = list(outcome.finditer(clause))
        for match in reversed(matches):
            before, after = clause[:match.start()], clause[match.end():]
            # "复核楼牌、模拟泵状态及扑灭效果" names a review item;
            # it does not assert extinction. Require both an explicit review
            # context and a nominal suffix, and reject completed-state wording
            # inside that list item (e.g. "及火已扑灭效果很好").
            item_prefix = re.split(r'、|及|和|与', before)[-1]
            nominal_review = bool(contexts[index]
                and re.match(r'(?:效果|结果)', after)
                and re.search(r'(?:核对|核实|核验|复核|检查|评估|判断|确认|验证|观察|排查)', before)
                and not re.search(r'(?:已经|已|证实|成功|完全|全部|彻底)(?:确认)?$', item_prefix))
            uncertainty = bool(
                re.search(r'(?:不能|无法|尚未|未能|不足以|不代表|不等于|难以)'
                          r'(?:确认|判断|证明|认定|说明|断定|保证|排除)?' + subject + r'$', before)
                or re.search(r'(?:是否|能否|有无)' + subject + r'$', before)
                or re.search(r'(?:如果|假如|假设|若|一旦)[^，。；;]{0,12}$', before)
                or re.match(r'(?:与否|是否)?(?:仍|还)?(?:需|需要|有待|尚待)(?:进一步)?'
                            r'(?:核验|核实|确认|验证|复核)', after)
                or re.search(r'(?:建议|需要|应当|应|请|务必)(?:首先|优先)?'
                             r'(?:确保|保障|保证|保护)$', before)
                or (contexts[index] and re.search(r'(?:核对|核实|核验|复核|检查|评估|判断|确认|验证|观察|排查|保障|确保)'
                    + subject + r'$', before) and not re.search(r'(?:已|已经|成功)'
                    r'(?:核对|核实|核验|复核|检查|评估|判断|确认|验证|观察|排查)' + subject + r'$', before))
                or nominal_review)
            if not uncertainty:
                return True, reply
            clause = clause[:match.start()] + '待复核状态' + clause[match.end():]
        pieces[index] = clause
    return False, ''.join(pieces)


def scene_observation_claims(reply):
    """Observation verbs and '火势' may be review topics, not established evidence."""
    pieces, contexts = _analysis_parts(reply)
    for index in range(0, len(pieces), 2):
        clause = pieces[index]
        for match in re.finditer(r'检出|检测到|可见|看见|看到|发现|确认|实测|测得|距离为|距离是|距目标|火势', clause):
            before = clause[:match.start()]
            uncertain = bool(re.search(r'(?:不能|无法|尚未|未能|不应|不要|有待|待|是否|有无|能否)$', before))
            review = contexts[index] and ((match.group() == '确认' and not re.search(r'(?:已|已经|成功)$', before)) or bool(re.search(
                r'(?:核对|核实|核验|复核|检查|评估|判断|确认|验证|观察|排查)(?:是否|有无|能否)?$', before)))
            if not (uncertain or review):
                return True
    return False


def ungrounded_scene_analysis(reply):
    """Reject measurable/live claims and actuation instructions in the advice field.

    This is an additional content check, not an authorization boundary: model
    output only reaches answer/speech topics and can never dispatch an action.
    All counts, signs and execution state are rendered separately by program code.
    """
    outcome_claim, cautious = scene_outcome_claims(reply)
    if outcome_claim:
        return True
    if ungrounded_reply(cautious, scene_advice=True):
        return True
    if re.search(r'(?:\d|[零〇一二两三四五六七八九十百千万])+(?:\.[0-9]+)?\s*'
                 r'(?:个|名|处|团|号楼|栋|米|厘米|毫米|度|秒|人|火)', reply):
        return True
    if scene_observation_claims(cautious):
        return True
    # The compact snapshot contains counts, never layout, flame severity or
    # verified real-fire observations. Such prose would add invented evidence.
    if re.search(r'(?:位于|左侧|右侧|前方|后方|楼内|楼外|着火|起火|失火|坍塌|爆炸)', cautious):
        return True
    if re.search(r'(?:已经|已|成功|正在|将会)(?:扑灭|灭火|出水|喷水|前进|后退|转向|导航|到达|受困)', cautious):
        return True
    if re.search(r'(?:已|已经|正在|确实|属于|处于|有人|人员|居民).{0,4}(?:受困|受难|死亡|伤亡)', cautious):
        return True
    if re.search(r'(?:开启|打开|启动|执行|开始|停止|关闭|调整|转动|移动|控制).{0,6}'
                 r'(?:水泵|喷水|水枪|云台|车辆|小车|电机|舵机)', reply):
        return True
    if re.search(r'(?:请|应|建议|先|立即).{0,4}(?:前进|后退|左转|右转|导航|喷水|灭火)', reply):
        return True
    return False


def ungrounded_reply(reply, *, scene_advice=False):
    live_pattern = r'(当前|现场|眼前|我看到|检测到).{0,24}(人|火|楼|水)'
    live_claim = bool(re.search(live_pattern, reply))
    if live_claim and scene_advice:
        # "优先确认当前画面是否覆盖目标，再核对楼牌" is an inspection
        # request. Do not let a regexp bridge its comma into an invented claim.
        pieces, contexts = _analysis_parts(reply)
        live_claim = False
        for index in range(0, len(pieces), 2):
            clause = pieces[index]
            if not re.search(live_pattern, clause):
                continue
            review = contexts[index] and bool(re.search(
                r'(?:核对|核实|核验|复核|检查|评估|判断|确认|验证|观察|排查).{0,12}(?:当前|现场|眼前)', clause))
            if not review:
                live_claim = True
                break
    return bool(re.search(r'(我|本车|小车)(已经|已|正在|将|会)(执行|开|关|启|停|去|前往|导航|喷)', reply)
        or re.search(r'(已|已经|确认|成功)(扑灭|灭火|出水|受困|启动水泵|开启喷水)', reply)
        or live_claim
        or re.search(r'(已|已经|刚刚)(联网|上网|搜索|查询到)', reply))
