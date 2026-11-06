"""Grounded visible-frame scene facts and optional local language intent parsing."""
import copy
import json
import logging
import math
import re
import urllib.parse
import urllib.request

from .task_interaction import fresh
from .mipi_building_ocr import observation_signature
from .edge_dialogue import (MAX_REPLY_CHARS, KNOWLEDGE_WIRE_CHARS, MAX_CONFIGURED_REPLY_CHARS,
    CAPABILITIES, GENERAL_SYSTEM_PROMPT, general_system_prompt, brief_question, ungrounded_scene_analysis,
    capability_question, system_question, knowledge_question, latest_information_question,
    system_answer, ungrounded_reply, normalized_question, project_reference, safe_model_error)

INTENTS = frozenset(('scene', 'fire', 'people', 'firefighters', 'building', 'spray', 'help', 'unknown'))
REPLY_AUDIT_LOG = logging.getLogger('rescue_vision.edge_dialogue.audit')
OBSERVATION_HINTS = {
    'none': '',
    'verify_building': '建议进一步核对楼牌，确认目标归属。',
    'inspect_fire': '建议保持稳定观察，复查火焰图案。',
    'inspect_people': '建议继续观察人物，不能仅凭图案判断是否受困。',
    'inspect_firefighters': '建议复查消防员与普通人物的区别。',
    'verify_spray': '建议复查喷射后的画面及实际水流，不能把停止喷水当作已经扑灭。',
    'widen_view': '建议在停稳后补充其他视角，当前画面不能代表整个街区。',
}
def generation_schema(current_view, known_intent=None, *, allow_analysis=False, reply_limit=MAX_REPLY_CHARS):
    """Constrain the model before decoding; independently validate after decoding."""
    intent = dict(type='string', enum=sorted(INTENTS)) if current_view else dict(type='string', const='knowledge')
    hint = dict(type='string', enum=list(OBSERVATION_HINTS)) if current_view else dict(type='string', const='none')
    if current_view and known_intent in INTENTS and known_intent != 'unknown':
        intent = dict(type='string', const=known_intent)
        choices = {'spray': 'verify_spray', 'building': 'verify_building', 'fire': 'inspect_fire',
            'people': 'inspect_people', 'firefighters': 'inspect_firefighters'}
        if known_intent in choices:
            hint = dict(type='string', enum=['none', choices[known_intent]])
    if allow_analysis:
        # Rich advice replaces the old canned hint. These labels are fixed in
        # code so the CPU model spends no generation tokens reproducing them.
        intent = dict(type='string', const=known_intent if known_intent in INTENTS-{'unknown', 'help'} else 'scene')
        hint = dict(type='string', const='none')
    return dict(type='object', additionalProperties=False,
        required=['intent', 'reply', 'observation_hint'], properties={
            'intent': intent,
            'reply': dict(type='string', const='') if current_view and not allow_analysis else dict(type='string', minLength=1, maxLength=reply_limit),
            'observation_hint': hint})


def scene_question(text):
    """Ambiguous requests about this robot's live view cannot become free-form claims."""
    text = normalized_question(text)
    if knowledge_question(text):
        return False
    return any(word in text for word in ('场景', '当前', '现在', '这幅', '画面', '眼前', '看到',
        '周围', '这里', '这儿', '附近', '此处', '现场', '目前', '这栋', '那栋',
        '受困', '受难', '灭完', '灭火完成', '扑灭了吗', '喷水了吗', '几个人', '多少人', '几处火', '楼号', '号楼'))


def compact_facts(report):
    """Short natural facts; never make the model interpret private numeric codes."""
    live = report.get('stale') is False and report.get('detector_available') is True
    unavailable = report.get('stale') is True or report.get('detector_available') is False
    def state(key, yes, no, unknown):
        value = report.get(key)
        return yes if value is True else no if value is False else unknown
    def count(key, label, unit):
        value = report.get(key)
        return (f'{label}{value}{unit}' if live and type(value) is int and 0 <= value <= 999
                else f'{label}数量未知')
    view = '提问时有效画面检出：' if live else '提问时无有效画面：' if unavailable else '画面有效性未知：'
    objects = '、'.join((count('visible_fire_count', '火焰图案', '处'),
        count('visible_person_count', '普通人物', '名'), count('visible_firefighter_count', '消防员图案', '名')))
    sign = state('building_sign_confirmed', '楼牌已确认', '楼牌未确认', '楼牌状态未知')
    association = state('building_association_verified', '目标与楼栋归属已验证',
        '目标与楼栋归属未验证', '目标与楼栋归属未知')
    pump = state('pump_on', '开启', '关闭', '未知')
    mode = state('actuator_dry_run', '模拟泵指令', '水泵控制指令', '输出模式未知，泵指令')
    water = state('water_flow_verified', '实际出水已验证', '实际出水未验证', '实际出水验证状态未知')
    extinguished = state('extinguished_verified', '扑灭已验证', '扑灭未验证', '扑灭验证状态未知')
    return f'{view}{objects}。{sign}；{association}；{mode}{pump}；{water}；{extinguished}。'


def freeze_scene_snapshot(report):
    """Keep an explicit historical observation, never extend control freshness."""
    if not isinstance(report, dict) or report.get('stale') is not False or report.get('detector_available') is not True:
        return None
    stamp = report.get('stamp')
    requested = report.get('_question_stamp', report.get('report_generated_stamp', stamp))
    if (type(stamp) not in (int, float) or not math.isfinite(stamp)
            or type(requested) not in (int, float) or not math.isfinite(requested)
            or not 0 <= requested-stamp <= 1.5):
        return None
    for key in ('visible_fire_count', 'visible_person_count', 'visible_firefighter_count'):
        if type(report.get(key)) is not int or not 0 <= report[key] <= 999:
            return None
    facts = copy.deepcopy(report)
    facts.pop('system_status', None)
    facts.pop('_question_stamp', None)
    return dict(capture_stamp=stamp, requested_stamp=requested,
        source='yolo_detection_snapshot', source_sequence=report.get('source_sequence'),
        coordinate_frame=report.get('coordinate_frame'), mission_mode=report.get('mission_mode', 'unknown'),
        control_use_allowed=False, facts=facts)


def snapshot_answer(question, decision, now):
    """Render historical numbers in program code and visibly separate generated advice."""
    snapshot = decision.get('analysis_snapshot')
    if not isinstance(snapshot, dict) or snapshot.get('control_use_allowed') is not False:
        return None
    stamp, requested = snapshot.get('capture_stamp'), snapshot.get('requested_stamp')
    if (type(stamp) not in (int, float) or not math.isfinite(stamp)
            or type(requested) not in (int, float) or not math.isfinite(requested)
            or not 0 <= requested-stamp <= 1.5 or not 0 <= now-stamp <= 120.):
        return '提问时的场景快照已过期或时间异常，请重新观察后提问。'
    facts = snapshot.get('facts') or {}
    frozen = freeze_scene_snapshot(dict(facts, _question_stamp=requested))
    if frozen is None or frozen['capture_stamp'] != stamp:
        return '提问时没有可核验的场景快照，请重新观察后提问。'
    # Evaluate the historical report at request time; the prefix prevents any
    # past pump/sign observation from being presented as a current state.
    intent = decision.get('intent', 'scene')
    if intent in ('unknown', 'help', 'knowledge'):
        intent = 'scene'
    factual = grounded_answer(question, facts, requested, intent)
    factual = factual.replace('当前画面', '该快照画面').replace('当前', '快照中')
    reply = decision.get('reply', '')
    if not isinstance(reply, str) or ungrounded_scene_analysis(reply):
        reply = ''
    elif reply and reply[-1] not in '。！？!?；;':
        reply += '。'
    prefix = f'以下依据提问时快照（采集距今{now-stamp:.1f}秒），不代表此刻画面。'
    return prefix + factual + (('分析建议：' + reply) if reply else '') + '以上仅供分析，未执行控制。'


STEREO_TECHNICAL_REFERENCE = (
    '先标定双目并进行去畸变、极线校正，匹配左右同一点得到视差d。'
    '在校正后主点一致、匹配有效时，光轴深度Z=fB/d：'
    'f是校正后的焦距、d是视差，均以像素计；B是基线、Z是深度，均以米计。'
    '标定或匹配未通过时不能可靠测距；深度也不等于喷水落点已验证。')
STEREO_DISTANCE_UNKNOWN = '当前场景问答尚未接入经过标定、匹配和时效核验的双目距离读数，不能报告可靠的实测距离。'


def stereo_topic(question):
    question = normalized_question(question)
    return '视差' in question or (any(word in question for word in ('双目', '立体视觉'))
        and any(word in question for word in ('测距', '距离', '深度', '原理', '基线', '焦距', '多远', '测到', '读数')))


def live_stereo_distance_question(question):
    question = normalized_question(question)
    return scene_question(question) and stereo_topic(question) and any(word in question for word in
        ('多少', '多远', '读数', '测到', '测得', '测量值', '距离是', '距离有', '视差是', '视差有'))


def fallback_decision(question, error=None):
    intent = question_intent(question)
    scope = ('capabilities' if capability_question(question) else 'system_status' if system_question(question)
        else 'unavailable_live_information' if latest_information_question(question) else None)
    if scope:
        return dict(intent='help' if scope == 'capabilities' else 'unknown', reply='', observation_hint='none',
            backend='structured_rules', model_error=error, general_chat_available=False, response_scope=scope)
    reference = project_reference(question)
    if reference:
        return dict(intent='knowledge', reply=reference, observation_hint='none', backend='domain_reference',
            model_error=error, general_chat_available=False, response_scope='domain_reference')
    if live_stereo_distance_question(question):
        return dict(intent='unknown', reply='', observation_hint='none', backend='structured_rules',
            model_error=error, general_chat_available=False)
    if stereo_topic(question) and not scene_question(question):
        return dict(intent='knowledge', reply=STEREO_TECHNICAL_REFERENCE, observation_hint='none',
            backend='domain_reference', model_error=error, general_chat_available=False)
    knowledge = knowledge_question(question) or (intent == 'unknown' and not scene_question(question))
    return dict(intent='unknown' if knowledge else intent, reply='', observation_hint='none',
        backend='structured_rules', model_error=error, general_chat_available=False,
        response_scope='general_knowledge' if knowledge else 'current_facts')


def render_dialogue(question, decision, report, now):
    """Only ordinary knowledge can use generated prose; live claims stay deterministic."""
    if decision.get('response_scope') == 'historical_snapshot_analysis':
        text = snapshot_answer(question, decision, now)
        if text is not None:
            return text
    scope = decision.get('response_scope')
    if scope == 'capabilities':
        return CAPABILITIES
    if scope == 'system_status':
        return system_answer(question, report, now, decision.get('model') or 'Qwen2.5-0.5B-Instruct')
    if scope == 'unavailable_live_information':
        return '当前没有接入联网搜索、天气或新闻服务，不能查询实时信息。可以询问本地机器人状态、已识别场景或普通知识。'
    if live_stereo_distance_question(question):
        return STEREO_DISTANCE_UNKNOWN
    if decision.get('backend') == 'domain_reference':
        return '本地技术参考（不代表当前检测或执行结果）：' + decision.get('reply', STEREO_TECHNICAL_REFERENCE)
    intent = decision['intent']
    if intent == 'knowledge':
        if scene_question(question):
            intent = question_intent(question)
            if intent in ('unknown', 'help'):
                intent = 'scene'
        else:
            return '通用知识解释（不代表当前检测或动作结果）：' + decision['reply']
    if intent == 'unknown' and not scene_question(question):
        return '本地知识问答暂未生成有效回答，请稍后重试或换一种问法。'+CAPABILITIES
    text = grounded_answer(question, report, now, intent)
    hint = OBSERVATION_HINTS.get(decision.get('observation_hint'), '')
    if hint:
        text += hint + '以上仅为观察建议，未执行控制。'
    return text[:350]


class SprayHistory:
    """Only consecutive fresh pump command transitions count; this is not a flow sensor."""
    def __init__(self, ttl_s=.5):
        self.ttl_s = ttl_s
        self.previous = None
        self.last_spray = None

    def update(self, actuator, now):
        if not fresh(actuator, now, self.ttl_s) or type(actuator.get('pump_on')) is not bool:
            self.previous = None
            if self.last_spray and self.last_spray['ended_stamp'] is None:
                self.last_spray['observation_interrupted'] = True
            return self.snapshot()
        stamp = actuator['stamp']
        previous = self.previous
        if previous is not None and stamp <= previous['stamp']:
            return self.snapshot()
        # A stream gap cannot prove when the physical/control transition occurred.
        if previous is not None and stamp-previous['stamp'] > self.ttl_s:
            previous = None
            if self.last_spray and self.last_spray['ended_stamp'] is None:
                self.last_spray['observation_interrupted'] = True
        if previous is not None and not previous['pump_on'] and actuator['pump_on']:
            identity = actuator.get('armed_target')
            if isinstance(identity, (list, tuple)) and len(identity) == 3:
                identity = dict(zip(('target_id', 'selection_generation', 'registry_session'), identity))
            self.last_spray = dict(started_stamp=stamp, ended_stamp=None,
                target_identity=copy.deepcopy(identity) if isinstance(identity, dict) else None,
                dry_run=actuator.get('dry_run'), termination_reason=None,
                command_transition_observed=True, observation_interrupted=False,
                water_flow_verified=False, extinguished_verified=False)
        elif (previous is not None and previous['pump_on'] and not actuator['pump_on']
              and self.last_spray is not None and self.last_spray['ended_stamp'] is None
              and not self.last_spray.get('observation_interrupted')):
            self.last_spray['ended_stamp'] = stamp
            self.last_spray['termination_reason'] = str((actuator.get('guard') or {}).get('reason')
                or actuator.get('last_request') or 'pump_command_off')[:160]
        self.previous = dict(stamp=stamp, pump_on=actuator['pump_on'])
        return self.snapshot()

    def snapshot(self):
        return copy.deepcopy(self.last_spray)


def recent_building_observation(ocr, mission, now):
    """Keep a delayed sign reading tied to its original capture, never today's counts."""
    if not isinstance(ocr, dict) or not isinstance(mission, dict):
        return None
    building, matched = ocr.get('building'), ocr.get('matched_detection')
    if (ocr.get('valid') is not True or ocr.get('same_capture_verified') is not True
            or not fresh(ocr, now, 3.) or not isinstance(building, dict)
            or building.get('confirmed') is not True or not isinstance(matched, dict)
            or matched.get('detector_available') is not True):
        return None
    stamp_ns, sequence, frame = ocr.get('capture_stamp_ns'), ocr.get('sequence'), ocr.get('coordinate_frame')
    stamp = ocr.get('stamp')
    if (type(stamp_ns) is not int or stamp_ns <= 0 or type(sequence) is not int
            or not isinstance(frame, str) or not frame
            or matched.get('capture_stamp_ns') != stamp_ns or matched.get('sequence') != sequence
            or matched.get('coordinate_frame') != frame or matched.get('capture_stamp') != stamp
            or ocr.get('capture_stamp') != stamp or stamp_ns/1e9 != stamp
            or building.get('stamp') != stamp or building.get('frame_id') != str(stamp_ns)):
        return None
    preview = ocr.get('unverified_pose') is True
    expected = observation_signature(mission, now, 0., require_settled_pose=not preview)
    if expected is None or ocr.get('observation_signature') != expected:
        return None
    number, name, bid = building.get('number'), building.get('name'), building.get('building_id')
    if (not isinstance(number, (str, int)) or not re.fullmatch(r'\d{1,3}', str(number))
            or not isinstance(name, str) or not 0 < len(name) <= 100
            or not isinstance(bid, str) or not 0 < len(bid) <= 100):
        return None
    return dict(confirmed=True, capture_stamp=stamp, capture_stamp_ns=stamp_ns,
        age_s=round(now-stamp, 3), source_sequence=sequence, coordinate_frame=frame,
        building_id=bid, number=str(number), name=name, unverified_pose=preview,
        fixed_camera_preview=ocr.get('fixed_camera_preview') is True,
        same_capture_verified=True, object_building_association_verified=False,
        observation_scope='recent_visible_sign_only')


def building_sentence(report, now):
    recent = report.get('recent_building_observation')
    if isinstance(recent, dict) and recent.get('confirmed') is True and fresh(recent, now, 3., 'capture_stamp'):
        text = '最近确认看到{}号楼{}的楼牌（约{:.1f}秒前）。'.format(
            recent['number'], recent['name'], now-recent['capture_stamp'])
        if recent.get('unverified_pose'):
            text += '这是固定相机联调观测，云台姿态尚未验证。'
        return text + '楼牌与当前画面目标分别报告，尚未确认目标都属于这栋楼。'
    return '当前楼号尚未确认，不能把整幅画面目标归属到某栋楼。'


def build_scene(detection, now, *, ocr=None, mission=None, actuator=None, targets=None, last_spray=None, ttl_s=1.):
    detection, ocr, mission, actuator = detection or {}, ocr or {}, mission or {}, actuator or {}
    live = bool(fresh(detection, now, ttl_s, 'capture_stamp') and detection.get('detector_available') is True)
    names = []
    if live:
        for d in detection.get('detections', []):
            confidence = d.get('confidence')
            if isinstance(confidence, (float, int)) and not isinstance(confidence, bool) and math.isfinite(confidence) and .4 <= confidence <= 1:
                names.append(d.get('class_name'))
    counts = {name: names.count(name) if live else None for name in ('fire', 'person', 'firefighter')}
    # Even same-frame sign text does not prove ownership of full-view objects.
    # The OCR node has no verified per-facade region; report the sign independently.
    recent_building = recent_building_observation(ocr, mission, now)
    confirmed = False
    act_live, mission_live = fresh(actuator, now, .5), fresh(mission, now, .5)
    report = dict(stamp=detection.get('capture_stamp') if live else now, report_generated_stamp=now, valid=live and confirmed,
        stale=not live, reason='fresh_visible_frame' if live else 'detections_stale_or_disabled',
        detector_available=live, observation_scope='current_view',
        visible_fire_count=counts['fire'], visible_person_count=counts['person'],
        visible_firefighter_count=counts['firefighter'], casualty_count=None, extinguished_verified=False,
        fire_status='unknown' if not live else ('observed' if counts['fire'] else 'not_observed'),
        building_association_verified=confirmed,
        building_number=None, building_name=None,
        building_sign_confirmed=recent_building is not None,
        recent_building_observation=recent_building,
        mission_mode=mission.get('mode') if mission_live else 'unknown',
        pump_on=actuator.get('pump_on') if act_live else None,
        actuator_dry_run=actuator.get('dry_run') if act_live else None,
        actuator_stamp=actuator.get('stamp') if act_live else None,
        water_flow_verified=False,
        unassigned_current_view=dict(fire_observed=bool(counts['fire']) if live else None,
            visible_person_count=counts['person'], visible_firefighter_count=counts['firefighter'],
            casualty_count=None, building_association_verified=confirmed),
        source_sequence=detection.get('sequence') if live else None,
        coordinate_frame=detection.get('coordinate_frame') if live else None)
    report['last_spray'] = copy.deepcopy(last_spray)
    report['task_state'] = ('spraying' if act_live and actuator.get('pump_on') is True else
        'unknown' if not mission_live else
        'awaiting_observation' if mission.get('mode') not in ('inspect', 'fire') else
        'post_spray_review' if last_spray and last_spray.get('ended_stamp') is not None else
        'aiming' if mission.get('mode') == 'fire' else
        'observing' if live else 'awaiting_observation')
    if isinstance(targets, dict) and fresh(targets, now, ttl_s, 'capture_stamp'):
        report['visible_fire_target_ids'] = [t['target_id'] for t in targets.get('targets', []) if type(t.get('target_id')) is int]
    else:
        report['visible_fire_target_ids'] = []
    return report


def question_intent(text):
    if not isinstance(text, str) or not text.strip() or len(text) > 200:
        raise ValueError('invalid_question')
    text = normalized_question(text)
    # A polite prefix such as "分析一下场景里" does not add categories to an
    # otherwise precise count question. Multiple requested classes or genuine
    # advice still need a scene-level answer.
    count_request = bool(re.search(r'(?:几|多少)(?:个|名|处|团|簇)?'
        r'(?:普通人物|普通人|人物|消防员|居民|人员|人|火焰|火苗|火)', text)) or '人数' in text
    reasoning = any(term in text for term in ('建议', '应该', '优先', '方案', '风险', '为什么', '如何', '怎么', '原因'))
    if count_request:
        categories = set()
        if any(term in text for term in ('火焰', '火苗')) or re.search(r'(?:几|多少)(?:个|团|处|簇)?火', text):
            categories.add('fire')
        if any(term in text for term in ('普通人', '人物', '居民', '人员', '人数', '几个人',
                                          '多少人', '有人', '受困', '受难')) or re.search(r'(?:几|多少)(?:个|名)?人', text):
            categories.add('people')
        if '消防员' in text:
            categories.add('firefighters')
        if len(categories) > 1 or reasoning:
            return 'scene'
        if len(categories) == 1:
            return next(iter(categories))
    people = any(x in text for x in ('几个人', '多少人', '人数', '居民', '人物', '受困', '受难'))
    if '消防员' in text and people:
        return 'scene'
    if any(x in text for x in ('灭完', '扑灭', '灭掉', '灭火完成', '喷水', '水泵', '灭火成功')):
        return 'spray'
    if '消防员' in text and not any(x in text for x in ('居民', '人物', '场景')):
        return 'firefighters'
    if any(x in text for x in ('几个人', '多少人', '人数', '居民', '受困', '受难', '救了')):
        return 'people'
    if any(x in text for x in ('楼号', '号楼', '哪里', '楼栋', '楼牌')):
        return 'building'
    if any(x in text for x in ('火焰', '火灾', '火苗')) and '场景' not in text:
        return 'fire'
    if any(x in text for x in ('帮助', '怎么使用', '能做什么', '有哪些模式')):
        return 'help'
    if any(x in text for x in ('场景', '看到', '情况', '周围', '观察', '报告', '对话')):
        return 'scene'
    return 'unknown'


def grounded_answer(question, report, now, intent=None):
    intent = intent or question_intent(question)
    if intent not in INTENTS:
        raise ValueError('invalid_intent')
    if intent == 'help':
        return ('可以说开启任务1进行YOLO识别，再说分析当前火场、寻找火焰，或问楼牌、人数和消防员。'
                '开启任务2后可以说你现在在1号位置，请前往3号位置后停下；到达停稳后自动YOLO识别。'
                '输入或说瞄准2号火焰后才请求双轴瞄准，开启喷水与停止喷水是两条独立指令；说任务结束会返回待机状态。'
                '语音导航目标交给FPGA执行；场景对话回答本身不会驱动车辆。')
    if intent == 'unknown':
        return '我目前支持小车任务和可见场景问答。可以问有几个火焰、看到几个人、有没有消防员、当前楼号和喷水状态。'
    report = report or {}
    if intent == 'spray':
        current = fresh(report, now, .5, 'actuator_stamp')
        prefix = ('当前水泵状态未知。' if not current else
                  '当前处于模拟输出，未据此确认实际出水。' if report.get('actuator_dry_run') is True else
                  '当前泵控制指令为开启，实际水流尚未验证。' if report.get('pump_on') is True else
                  '当前泵控制指令为关闭。' if report.get('pump_on') is False else '当前水泵状态未知。')
        history = report.get('last_spray') or {}
        finished = ('上次模拟喷射指令已结束，等待复查。' if history.get('dry_run') is True else
                    '上次喷射控制指令已结束，等待复查。') if history.get('ended_stamp') is not None else ''
        return prefix + finished + '喷水停止或本帧没检测到火焰，都不能证明已经扑灭，需要再次观察和实际灭火验证。'
    building = building_sentence(report, now)
    if intent == 'building':
        return building
    if not fresh(report, now, 1.5) or report.get('stale') or report.get('detector_available') is not True:
        return ((building if intent == 'scene' and report.get('recent_building_observation') else '')
                + '当前没有新鲜有效的识别画面，请先停车并开启识别，再询问场景。')
    fire = ('当前画面检出{}处火焰图案。'.format(report['visible_fire_count']) if report.get('visible_fire_count')
            else '当前画面未检出火焰，这不能排除视野外或漏检的火焰。')
    people = '当前画面可见{}名普通人物，是否受困以及总人数无法仅凭本帧确认。'.format(report.get('visible_person_count'))
    firefighters = '当前画面可见{}名消防员图案。'.format(report.get('visible_firefighter_count'))
    return {'scene': building + fire + people + firefighters, 'fire': fire, 'people': people,
            'firefighters': firefighters, 'building': building}[intent]


class LocalIntentModel:
    """Optional loopback model; no model output is ever a robot command."""
    def __init__(self, config):
        self.enabled = config.get('enabled') is True
        self.endpoint = config.get('endpoint', 'http://127.0.0.1:8088/v1/chat/completions')
        parsed = urllib.parse.urlparse(self.endpoint)
        if (parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost', '::1')
                or parsed.username or parsed.password or parsed.query or parsed.fragment
                or parsed.path != '/v1/chat/completions'):
            raise ValueError('edge_model_endpoint_must_be_loopback_chat_completions')
        self.model = str(config.get('model', 'local-instruct'))[:100]
        self.mode = config.get('mode', 'intent')
        if self.mode not in ('intent', 'generate'):
            raise ValueError('invalid_edge_model_mode')
        self.timeout = float(config.get('timeout_s', 2.))
        if not math.isfinite(self.timeout) or not .1 <= self.timeout <= 90.:
            raise ValueError('invalid_edge_model_timeout')
        self.answer_timeout = float(config.get('answer_timeout_s', min(90., self.timeout+5.)))
        if not math.isfinite(self.answer_timeout) or not .1 <= self.answer_timeout <= 90.:
            raise ValueError('invalid_answer_timeout')
        self.max_tokens = config.get('max_tokens', 256)
        if type(self.max_tokens) is not int or not 16 <= self.max_tokens <= 256:
            raise ValueError('invalid_max_tokens')
        self.temperature = float(config.get('temperature', .2))
        if not math.isfinite(self.temperature) or not 0 <= self.temperature <= 1.:
            raise ValueError('invalid_temperature')
        self.scene_analysis_enabled = config.get('scene_analysis_enabled', False)
        if type(self.scene_analysis_enabled) is not bool:
            raise ValueError('invalid_scene_analysis_enabled')
        self.reply_max_chars = config.get('reply_max_chars', KNOWLEDGE_WIRE_CHARS)
        self.brief_reply_max_chars = config.get('brief_reply_max_chars', self.reply_max_chars)
        self.analysis_reply_max_chars = config.get('analysis_reply_max_chars', 120)
        for key in ('reply_max_chars', 'brief_reply_max_chars', 'analysis_reply_max_chars'):
            value = getattr(self, key)
            if type(value) is not int or not 24 <= value <= MAX_CONFIGURED_REPLY_CHARS:
                raise ValueError('invalid_' + key)
        self.custom_reply_budget = any(key in config for key in
            ('reply_max_chars', 'brief_reply_max_chars', 'analysis_reply_max_chars'))

    def requires_worker(self, question):
        question = normalized_question(question)
        if (capability_question(question) or system_question(question) or latest_information_question(question)
                or project_reference(question) or live_stereo_distance_question(question)
                or (stereo_topic(question) and not scene_question(question))):
            return False
        if not self.enabled:
            return False
        # "分析一下场景里有几团火焰" is still an exact count request.
        # A polite analysis preamble must not turn a cheap fact into a long LLM job.
        count_request = bool(re.search(r'(?:几|多少)(?:个|名|处|团)?(?:人|居民|消防员|火焰|火苗)', question))
        reasoning_request = any(term in question for term in ('为什么', '如何', '建议', '原因', '优先', '方案', '风险', '应该'))
        if count_request and not reasoning_request and not knowledge_question(question):
            return False
        if self.mode == 'intent':
            return True
        # Exact factual questions must not wait behind a language model.
        return knowledge_question(question) or question_intent(question) == 'unknown' or any(word in question for word in
            ('分析', '解释', '建议', '聊', '为什么', '原理', '如何', '步骤', '方案', '应该'))

    def _request(self, system, user, *, generation=False, schema=None):
        payload = dict(model=self.model, messages=[dict(role='system', content=system),
            dict(role='user', content=user)], max_tokens=self.max_tokens if generation else 32,
            temperature=self.temperature if generation else 0)
        if generation and self.model.startswith('qwen3-'):
            # The launcher separately disables reasoning. These are the Qwen3
            # non-thinking sampling parameters, not inherited server defaults.
            payload.update(top_p=.8, top_k=20, min_p=0., presence_penalty=1.5)
        if generation:
            # Known scope labels are restored locally so CPU generation spends
            # its token budget on prose rather than repeated fixed JSON values.
            fixed_intent = schema['properties']['intent'].get('const')
            reply_only = fixed_intent == 'knowledge' or (self.scene_analysis_enabled
                and 'const' not in schema['properties']['reply'] and fixed_intent in INTENTS)
            configured_limit = self.reply_max_chars if fixed_intent == 'knowledge' else self.analysis_reply_max_chars
            wire_limit = min(configured_limit, schema['properties']['reply'].get('maxLength', configured_limit))
            wire_schema = (dict(type='object', additionalProperties=False, required=['reply'],
                properties={'reply': dict(schema['properties']['reply'], maxLength=wire_limit)}) if reply_only else schema)
            # llama.cpp constrains decoding; still validate independently below.
            payload['response_format'] = dict(type='json_schema', json_schema=dict(
                name='read_only_robot_answer', strict=True, schema=wire_schema))
        request = urllib.request.Request(self.endpoint,
            data=json.dumps(payload, ensure_ascii=False).encode('utf-8'), headers={'Content-Type': 'application/json'})
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *args, **kwargs):
                raise ValueError('redirect_forbidden')
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
        with opener.open(request, timeout=self.timeout) as response:
            body = response.read(16385)
        if len(body) > 16384:
            raise ValueError('response_too_large')
        envelope = json.loads(body)
        choice = envelope['choices'][0]
        if choice.get('finish_reason') not in (None, 'stop'):
            raise ValueError('generation_truncated_or_incomplete')
        content = choice['message']['content']
        if not isinstance(content, str):
            raise ValueError('invalid_model_content')
        # Duplicate keys are not an acceptable interpretation of a strict schema.
        def unique_keys(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    raise ValueError('duplicate_json_key')
                result[key] = value
            return result
        value = json.loads(content, object_pairs_hook=unique_keys)
        if generation and reply_only:
            if not isinstance(value, dict) or set(value) != {'reply'}:
                raise ValueError('invalid_knowledge_reply_schema')
            reply = value['reply']
            if isinstance(reply, str) and len(reply) >= wire_limit:
                endings = list(re.finditer(r'[。！？!?]|\.(?=\s|$)', reply))
                if not endings:
                    raise ValueError('reply_limit_without_complete_sentence')
                reply = reply[:endings[-1].end()].strip()
            value = dict(intent=fixed_intent, reply=reply, observation_hint='none')
        return value

    def classify(self, question):
        fallback = question_intent(question)
        if not self.enabled:
            return fallback, 'structured_rules', None
        system = ('Classify this Chinese robot question. Return only JSON {"intent":"..."}. '
                  'Allowed intents: scene,fire,people,firefighters,building,spray,help,unknown. '
                  'Never output actions, counts, facts or instructions. spray includes asking if a fire was extinguished.')
        try:
            value = self._request(system, question)
            if not isinstance(value, dict) or set(value) != {'intent'} or value['intent'] not in INTENTS:
                raise ValueError('invalid_model_intent')
            return (fallback if fallback in ('spray', 'people') else value['intent']), 'edge_llm', None
        except Exception as exc:
            return fallback, 'structured_rules', safe_model_error(exc)

    def respond(self, question, report):
        fallback = fallback_decision(question)
        if fallback['backend'] == 'domain_reference' or live_stereo_distance_question(question):
            return fallback
        if not self.enabled or not self.requires_worker(question):
            return fallback
        if self.mode == 'intent':
            intent, backend, error = self.classify(question)
            return dict(fallback, intent=intent, backend=backend, model_error=error)
        known_intent = question_intent(question)
        current_view = not knowledge_question(question) and (scene_question(question) or known_intent not in ('unknown', 'help'))
        allow_analysis = current_view and self.scene_analysis_enabled
        snapshot = freeze_scene_snapshot(report) if allow_analysis else None
        if allow_analysis and snapshot is None:
            return fallback_decision(question, 'scene_snapshot_unavailable')
        reply_limit = (self.analysis_reply_max_chars if allow_analysis else
            min(self.brief_reply_max_chars, self.reply_max_chars) if brief_question(question) else self.reply_max_chars)
        # Keep legacy mocked/runtime validation compatible unless a larger budget
        # was explicitly selected. The HTTP grammar still uses the wire budget.
        validation_limit = reply_limit if allow_analysis or self.custom_reply_budget else MAX_REPLY_CHARS
        schema = generation_schema(current_view, known_intent,
            allow_analysis=allow_analysis, reply_limit=validation_limit)
        if allow_analysis:
            system = (f'根据提问时检测快照，说明优先复核什么及原因，通常40～70字，最多{reply_limit}字。'
                '只输出JSON。未检出不等于不存在，未验证不等于成功；'
                '不要编造方位、受困、距离或灭火结论，不下控制指令。')
            user = question + '\n提问时快照：' + compact_facts(snapshot['facts'])
        elif current_view:
            system = ('只选问句类别及观察建议，reply留空。快照可能过时，'
                '不生成现场结论，不执行控制，输出JSON。')
            user = question + '\n快照：' + compact_facts(report)
        else:
            system = general_system_prompt(reply_limit) if self.custom_reply_budget else GENERAL_SYSTEM_PROMPT
            user = question
        value = None
        try:
            value = self._request(system, user, generation=True, schema=schema)
            if (not isinstance(value, dict) or set(value) != {'intent', 'reply', 'observation_hint'}
                    or not isinstance(value['intent'], str) or value['intent'] not in INTENTS | {'knowledge'}
                    or not isinstance(value['reply'], str) or len(value['reply']) > validation_limit
                    or not isinstance(value['observation_hint'], str) or value['observation_hint'] not in OBSERVATION_HINTS):
                raise ValueError('invalid_generation_schema')
            if current_view:
                if value['intent'] not in INTENTS or (not allow_analysis and value['reply'] != ''):
                    raise ValueError('current_view_requires_empty_reply')
                if known_intent not in ('unknown', 'help') and value['intent'] != known_intent:
                    raise ValueError('model_cannot_override_known_question_intent')
                if allow_analysis and value['intent'] != schema['properties']['intent']['const']:
                    raise ValueError('model_cannot_override_known_question_intent')
                if value['observation_hint'] not in schema['properties']['observation_hint'].get('enum', OBSERVATION_HINTS):
                    raise ValueError('observation_hint_not_relevant_to_question')
                if allow_analysis and value['observation_hint'] != 'none':
                    raise ValueError('observation_hint_not_relevant_to_question')
                if allow_analysis:
                    reply = value['reply'].strip()
                    if (not reply or any(ord(ch) < 32 and ch not in '\n\t' for ch in reply)
                            or ungrounded_scene_analysis(reply)):
                        raise ValueError('ungrounded_scene_analysis')
                    if len(reply) >= reply_limit:
                        endings = list(re.finditer(r'[。！？!?]|\.(?=\s|$)', reply))
                        if not endings:
                            raise ValueError('reply_limit_without_complete_sentence')
                        reply = reply[:endings[-1].end()].strip()
                    value['reply'] = reply
            elif value['intent'] != 'knowledge' or value['observation_hint'] != 'none':
                raise ValueError('knowledge_requires_fixed_intent_and_hint')
            if value['intent'] == 'knowledge' and not scene_question(question):
                # Small quantized models sometimes append a fence after a complete
                # sentence. Remove only a trailing run; do not rewrite its claims.
                reply = re.sub(r'`{3,}\s*$', '', value['reply']).strip()
                if len(value['reply']) >= validation_limit:
                    # Grammar can force a valid JSON ending at the character cap
                    # while cutting the natural-language sentence in the middle.
                    endings = list(re.finditer(r'[。！？!?]|\.(?=\s|$)', reply))
                    if not endings:
                        raise ValueError('reply_limit_without_complete_sentence')
                    reply = reply[:endings[-1].end()].strip()
                if (not reply or any(ord(ch) < 32 and ch not in '\n\t' for ch in reply)
                        or ungrounded_reply(reply)):
                    raise ValueError('ungrounded_generated_claim')
                value['reply'] = reply
            result = dict(value, backend='edge_llm_generate', model_error=None, general_chat_available=True,
                model=self.model, response_scope='historical_snapshot_analysis' if allow_analysis else
                    'current_facts' if current_view else 'general_knowledge')
            if allow_analysis:
                result['analysis_snapshot'] = snapshot
            return result
        except Exception as exc:
            error = safe_model_error(exc)
            if isinstance(value, dict) and isinstance(value.get('reply'), str):
                # Local service journal only, never answer API/UI. JSON escapes
                # control characters and hard bounds prevent log injection/bloat.
                REPLY_AUDIT_LOG.warning('model_reply_rejected %s', json.dumps(dict(
                    model=self.model[:100], question=question[:200], reason=error,
                    reply=value['reply'][:MAX_CONFIGURED_REPLY_CHARS],
                    reply_truncated=len(value['reply']) > MAX_CONFIGURED_REPLY_CHARS),
                    ensure_ascii=False, separators=(',', ':')))
            return fallback_decision(question, error)
