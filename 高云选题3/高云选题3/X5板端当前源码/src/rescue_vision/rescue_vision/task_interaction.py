"""Explicit task grammar and command gates. No ROS, network or hardware effects."""
from collections import OrderedDict
import math
import re

from .voice import normalize_voice_text, CHINESE_NUMBERS
from .manual_motion import parameters as manual_parameters, validate as validate_manual, start_guard as manual_start_guard, navigation_busy
from .extension_control import start_guard as extension_start_guard, zebra_switch_guard, dialogue_allowed
from .selected_fire_round import selected_fire_round_reason


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def fresh(value, now, ttl, key='stamp'):
    stamp = value.get(key) if isinstance(value, dict) else None
    return finite(stamp) and finite(now) and 0 <= now - stamp <= ttl


def spoken_number(value):
    if value.isascii() and value.isdigit():
        return int(value)
    if value in CHINESE_NUMBERS:
        return CHINESE_NUMBERS[value]
    if value in ('一百', '两百', '二百', '三百', '四百', '五百', '六百', '七百', '八百', '九百', '一千'):
        return {'一':1,'两':2,'二':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9}[value[0]]*(1000 if value[-1]=='千' else 100)
    return None


def extension_distance_parameters(direction, distance_m):
    from .ov5640_protocol import move_payload
    move_payload(direction, distance_m)
    return dict(direction=direction, distance_m=float(distance_m), speed_m_s=.10,
                max_duration_s=round(distance_m/.10+2., 3))


def point_request_guard(extension, now, confirm=False):
    """Use the live numbered-protocol owner, never the retired CRC bridge."""
    if not fresh(extension, now, .6) or extension.get('real_uart') is not True:
        return 'real_extension_uart_required'
    if extension.get('status_fresh') is not True:
        return 'extension_status_unavailable'
    key = 'can_confirm_point_start' if confirm else 'can_navigate_point'
    if extension.get(key) is not True:
        return extension.get('point_navigation_reason') or 'numbered_navigation_not_ready'
    return ''


def manual_pump_request_guard(state, now):
    if not fresh(state, now, 1.) or state.get('synthetic') is True:
        return 'pump_manual_status_stale'
    return '' if state.get('can_start') is True else state.get('ready_reason') or 'pump_manual_not_ready'


def validate_point_request(request):
    action = request.get('action')
    heading = request.get('start_heading_deg')
    if heading is not None and (not finite(heading) or not -180 <= heading <= 180):
        raise ValueError('start_heading_requires_minus180_to180')
    if action == 'confirm_point_start':
        point = request.get('point_id')
        if type(point) is not int or not 1 <= point <= 9:
            raise ValueError('point_requires_1_to_9')
        if request.get('source') != 'web' or request.get('confirmed') is not True:
            raise ValueError('physical_point_confirmation_requires_console')
        if heading is None:
            raise ValueError('numbered_start_heading_required')
    elif action == 'navigate_point':
        target, start = request.get('target_point'), request.get('start_point')
        if type(target) is not int or not 1 <= target <= 9:
            raise ValueError('point_requires_1_to_9')
        if start is not None and (type(start) is not int or not 1 <= start <= 9):
            raise ValueError('start_point_requires_1_to_9')
        if start == target:
            raise ValueError('start_and_target_point_are_equal')
        if request.get('start_declared') is not None:
            if request.get('start_declared') is not True or start is None:
                raise ValueError('explicit_start_declaration_required')
        if request.get('arrival_policy') not in (None, 'manual', 'scan'):
            raise ValueError('numbered_arrival_requires_observe_or_scan')
        if 'patrol_fire' in request or 'fire_ignore_s' in request:
            raise ValueError('retired_patrol_fire_use_region_search')
    else:
        raise ValueError('unsupported_point_action')


def resolve_building(destination, buildings):
    """Resolve an exact configured name or number, never ask an LLM to invent a goal."""
    entries = buildings.get('buildings', []) if isinstance(buildings, dict) else (buildings or [])
    number_match = re.fullmatch(r'([零一二两三四五六七八九十\d]+)(?:号楼|栋|号栋)', destination)
    number = spoken_number(number_match.group(1)) if number_match else None
    found = [b for b in entries if
             (number is not None and int(b['number']) == number)
             or destination in [normalize_voice_text(str(x)) for x in
                                [b.get('name', ''), *b.get('aliases', [])] if x]]
    if len(found) != 1:
        raise ValueError('unknown_or_ambiguous_building')
    return found[0]


def spoken_heading(value):
    """Read an explicitly spoken angle; never infer orientation from a route."""
    if re.fullmatch(r'[+-]?\d+(?:\.\d+)?', value):
        return float(value)
    sign = -1 if value.startswith('负') else 1
    value = value.removeprefix('负').removeprefix('正')
    pieces = value.split('点')
    if len(pieces) > 2:
        raise ValueError('invalid_spoken_heading')
    whole = pieces[0]
    if '百' in whole:
        lead, rest = whole.split('百', 1)
        hundreds = spoken_number(lead)
        rest = rest.removeprefix('零')
        tail = spoken_number(rest) if rest else 0
        if hundreds is None or not 1 <= hundreds <= 9 or tail is None or not 0 <= tail <= 99:
            raise ValueError('invalid_spoken_heading')
        number = hundreds * 100 + tail
    else:
        number = 0 if whole in ('零', '〇') else spoken_number(whole)
    if number is None:
        raise ValueError('invalid_spoken_heading')
    if len(pieces) == 2:
        digits = {'零':'0','〇':'0','一':'1','二':'2','两':'2','三':'3','四':'4','五':'5','六':'6','七':'7','八':'8','九':'9'}
        if not pieces[1] or any(ch not in digits for ch in pieces[1]):
            raise ValueError('invalid_spoken_heading')
        number = float(str(number) + '.' + ''.join(digits[ch] for ch in pieces[1]))
    return sign * float(number)


def parse_numbered_navigation(clean, original):
    """Parse complete road-point requests; never ask the LLM to fill missing IDs.

    A start declaration is an operator assertion, not automatic localization.
    Wording may vary, but point numbers and declaration semantics remain exact
    so the independent ASR decoder can compare the same structured command.
    """
    if not re.search(r'号(?:点|位置|导航点)|点位|导航点', clean):
        return None
    # The general voice normalizer removes punctuation including dots. Preserve
    # a decimal inside an explicit heading rather than silently turning 9.5°
    # into 95°. Sentence-final punctuation is still ignored.
    nav_text = normalize_voice_text(re.sub(r'(?<=\d)\.(?=\d)', '·', original)).replace('·', '.')
    nav_text = re.sub(r'^(?:请帮我|帮我)', '', nav_text)
    # Do not turn a question, quotation, negative instruction or discussion of
    # navigation into vehicle motion after punctuation normalization.
    if (re.search(r'[?？“”「」『』"\'‘’]', original)
            or any(word in nav_text for word in ('不要', '不能', '别', '禁止', '如果', '假如',
                                                '能否', '能不能', '是否', '吗', '么', '怎么',
                                                '如何', '为什么', '多远', '多久', '是什么意思'))
            or nav_text.startswith(('不', '我想知道', '告诉我', '解释', '假设'))):
        return {'action': 'dialogue'}
    if '沿途' in nav_text and ('灭火' in nav_text or '喷水' in nav_text):
        raise ValueError('retired_patrol_fire_use_region_search')
    number = r'[零一二两三四五六七八九十\d]+'
    def point(name):
        return r'第?(?P<' + name + r'>' + number + r')(?:号(?:点位|导航点|点|位置)|个(?:点位|导航点)|点位|导航点)(?:上|处)?'
    # SenseVoice often produces redundant but unambiguous “导航前往至”.
    # Match those whole phrases; never replace a misheard point number.
    verb = r'(?:导航前往至|导航前往|导航至|导航到|导航去|行驶至|行驶到|移动到|前往至|前往|开往|驶向|去|到)'
    connector = r'(?:然后|接着|接下来)?(?:请帮我|请你|请|帮我|我要|我需要你)?'
    stop = (r'(?:进行前进)?(?:(?:到达后|到达以后|到了以后|到达目标后|到达目标点后|后|并且|并|然后|接着)?'
            r'(?:请)?(?:停下车|停下来|停下|停车|停止行驶|停止))?(?:谢谢)?')
    declaration = (r'(?:(?:你|我|小车)?(?:现在|当前|目前)(?:位于|在)|'
                   r'(?:现在|当前|目前)(?:你|我|小车)(?:位于|在)|'
                   r'(?:你|我|小车)(?:现在|当前|目前)?(?:的位置是|所在的位置是|位于|在)|'
                   r'(?:我的|你的|小车的|我|你|小车)?(?:现在|当前|目前)?(?:实际)?(?:所在)?位置(?:是|为|在))')
    heading = (r'(?:(?:(?:车头|车身)(?:方向)?)?(?:朝向|面向|朝)(?P<heading>'
               r'(?:地图)?(?:右|左|上|下)(?:边|侧|方)?|(?:[+-]?\d+(?:\.\d+)?|[负正]?[零〇一二两三四五六七八九十百点]+)度))?')
    match = re.fullmatch(declaration + point('start') + heading + connector + verb + point('target') + stop, nav_text)
    if match:
        result = dict(action='navigate_point', start_point=spoken_number(match.group('start')),
                      target_point=spoken_number(match.group('target')), start_declared=True)
    else:
        match = re.fullmatch(r'(?:导航)?从' + point('start') + r'(?:出发)?' + heading + connector + verb + point('target') + stop, nav_text)
        if match:
            result = dict(action='navigate_point', start_point=spoken_number(match.group('start')),
                          target_point=spoken_number(match.group('target')))
        else:
            context_start = r'(?:从(?:当前位置|这里|当前点位|当前点|刚到的点)(?:出发)?)?'
            match = re.fullmatch(context_start + heading + connector + verb + point('target') + stop, nav_text)
            if match:
                result = dict(action='navigate_point', target_point=spoken_number(match.group('target')))
            else:
                # Only a recognisable imperative is a failed command. Questions
                # about points and ordinary descriptions remain read-only text.
                # A point/action sentence that does not match the complete
                # grammar is a clarification, never a free-form Qwen question.
                # Keep explanatory questions (为什么/怎么/距离等) read-only.
                intended = (re.match(r'(?:导航|从|前往|去|到|移动到|行驶到|带我|你先|请带)', nav_text)
                            or re.match(declaration, nav_text)
                            or (re.search(r'号(?:点|位置|导航点)|点位', nav_text)
                                and re.search(r'(?:导航|前往|到|去|开到|开往|驶向|带我|请你|你先)', nav_text)))
                if intended:
                    if any(word in nav_text for word in ('喷水', '灭火', '瞄准', '后退', '倒退')):
                        raise ValueError('numbered_navigation_separate_actions')
                    raise ValueError('numbered_navigation_needs_clear_points')
                return None
    heading_text = match.groupdict().get('heading')
    if heading_text is not None:
        if heading_text.endswith('度'):
            result['start_heading_deg'] = spoken_heading(heading_text[:-1])
        else:
            result['start_heading_deg'] = {'右':0., '上':90., '左':180., '下':-90.}[
                re.search('[右上左下]', heading_text).group()]
    validate_point_request(result)
    return result


def parse_region_search(clean, original):
    """Closed semantic-region grammar; region and water authority stay explicit."""
    if not re.search(r'(?:区域|[ABCDabcd诶比西迪]区)', clean):
        return None
    if (re.search(r'[?？“”「」『』"\'‘’]', original)
            or clean.startswith(('不', '别', '禁止', '解释', '告诉我', '我想知道'))
            or any(word in clean for word in ('不要', '不能', '如果', '假如', '能否', '能不能', '是否', '吗', '么', '如何', '为什么'))):
        return dict(action='dialogue')
    # Chinese aliases correspond to spoken letter names, not guessed locations.
    region = r'(?P<region>[ABCDabcd]|诶|比|西|迪)(?:区域|区)'
    prefix = r'(?:请|请帮我|帮我)?(?:在|前往|去|进入|到|导航到)?'
    search = r'(?:搜寻|搜索|寻找|查找)(?:火源|火焰)'
    fire = r'(?:并且|并|然后)?(?:执行|进行)?(?:自动)?灭火'
    match = re.fullmatch(prefix + region + search + r'(?P<fire>' + fire + r')?(?:谢谢)?', clean)
    if not match:
        if any(word in clean for word in ('搜寻', '搜索', '寻找', '灭火', '喷水')):
            raise ValueError('region_search_requires_explicit_region_and_fire')
        return None
    letter = match.group('region')
    region_id = {'诶':'A', '比':'B', '西':'C', '迪':'D'}.get(letter, letter.upper())
    return dict(action='search_region', region_id=region_id, execute_fire=match.group('fire') is not None)


def parse_task_text(text, buildings=None):
    """Unknown text becomes a read-only question, never an inferred robot action."""
    clean = normalize_voice_text(text)
    clean = re.sub('yolo', 'YOLO', clean, flags=re.IGNORECASE)
    if not clean:
        raise ValueError('empty_text')
    out = {'text': text}
    if '试喷' in clean:
        # Retain decimal punctuation while normalizing explicit wake prefixes.
        # Questions, quotations and compound wording never become a pulse.
        if re.search(r'[?？“”「」『』"\'‘’]', text):
            return dict(out, action='dialogue')
        pulse_text = normalize_voice_text(re.sub(r'(?<=\d)\.(?=\d)', '·', re.sub(r'\s+', '', text)))
        if pulse_text == '停止试喷':
            return dict(out, action='stop_spray')
        match = re.fullmatch(r'手动试喷(?:([0-9零〇一二两三四五六七八九十百千]+)毫秒)?', pulse_text)
        if match:
            duration = 400 if match.group(1) is None else spoken_heading(match.group(1))
            if not finite(duration) or not 100 <= duration <= 1000 or int(duration) != duration:
                raise ValueError('pump_manual_duration_ms_must_be_integer_100_to_1000')
            return dict(out, action='manual_pump_test', duration_ms=int(duration))
        return dict(out, action='dialogue')
    # Stops are recognized only as explicit commands, not questions or quoted plans.
    if clean in ('停止喷水', '关闭喷水', '停止灭火', '关闭水泵', '停水', '停止喷射'):
        return dict(out, action='stop_spray')
    if clean in ('紧急停止', '紧急停车', '立即停车', '停车', '停止', '停下',
                 '停止导航', '取消导航', '停止瞄准', '停止自动瞄准',
                 '停止沿途灭火', '取消沿途灭火', '停止沿途灭火导航',
                 '停止区域搜索', '取消区域搜索', '停止区域灭火', '取消区域任务'):
        return dict(out, action='stop')
    if clean in ('任务结束', '结束任务', '任务完成', '结束本次任务', '返回待机', '进入待机', '待机', '结束交通任务'):
        return dict(out, action='finish_task')
    if clean in ('开启拓展任务一', '开启拓展任务1', '开始拓展任务一', '开始拓展任务1',
                 '开启任务一', '开启任务1', '开始任务一', '开始任务1', '纯视觉识别'):
        return dict(out, action='extension_vision')
    if clean in ('开启拓展任务二','开启拓展任务2','开始拓展任务二','开始拓展任务2',
                 '开启任务二','开启任务2','开始任务二','开始任务2','视觉与对话','开启语音对话任务'):
        return dict(out,action='extension_dialogue')
    if clean in ('开启FPGA斑马线识别','开启斑马线检测','开启斑马线识别'):
        return dict(out,action='set_zebra_detection',enabled=True)
    if clean in ('关闭FPGA斑马线识别','关闭斑马线检测','关闭斑马线识别'):
        return dict(out,action='set_zebra_detection',enabled=False)
    if clean in ('开启交通任务', '开始交通任务'):
        return dict(out, action='traffic_task')
    if clean in ('开始交通行驶','开始红绿灯通行测试'):
        return dict(out,action='traffic_drive')
    # Keep decimal points: voice normalization deliberately strips punctuation.
    reference_text = re.sub(r'\s+', '', text)
    match = re.fullmatch(r'确认起点X(\d+(?:\.\d+)?)厘米Y(\d+(?:\.\d+)?)厘米朝向(-?\d+(?:\.\d+)?)度', reference_text, re.IGNORECASE)
    if match:
        x, y, heading = map(float, match.groups())
        if not (0 <= x < 352 and 0 <= y < 320 and -180 <= heading <= 360):
            raise ValueError('reference_outside_map')
        return dict(out, action='set_reference', x_cm=x, y_cm=y, heading_deg=heading, confirmed=True)
    # These two complete compound grammars are recognized before rejecting
    # arbitrary multi-action sentences. An LLM cannot synthesize an action.
    match = re.fullmatch(r'(?:一直)?(?:前进|直走)(?:然后)?在第([一二两三四五六七八九十\d]+)个路口(左转|右转|左拐|右拐)', clean)
    if match:
        count = spoken_number(match.group(1))
        if count is None or not 1 <= count <= 8:
            raise ValueError('junction_count_requires_1_to_8')
        return dict(out, action='route_turn', junction_count=count,
                    turn='left' if match.group(2).startswith('左') else 'right')
    match = re.fullmatch(r'(?:一直)?(?:前进|直走)(?:直到|直至)(?:摄像头识别到)?火焰(?:在(?:视觉)?(?:屏幕|画面)中心(?:范围)?(?:后停车|后停下车)?|居中)(然后自动灭火)?', clean)
    if match:
        return dict(out, action='seek_fire', after_fire='auto' if match.group(1) else 'manual')
    region = parse_region_search(clean, text)
    if region is not None:
        return dict(out, **region)
    numbered = parse_numbered_navigation(clean, text)
    if numbered is not None:
        return dict(out, **numbered)
    # Negation is a phrase/prefix, not an arbitrary character: “识别” contains
    # 别 but is an affirmative command. The full-match allowlist below ensures
    # unrecognized sentences (including other negative wording) cannot actuate.
    if (clean.startswith(('不', '别', '禁止'))
            or any(word in clean for word in ('如果', '假如', '能否', '能不能', '是否', '吗', '么', '或者', '还是', '然后'))):
        return dict(out, action='dialogue')
    # Full phrases only: distances are integer centimetres, never a velocity
    # chosen by an LLM. Unsupported/compound requests remain read-only dialogue.
    match = re.fullmatch(r'(前进|向前走|后退|回退|倒退|向后退|往后退)(?:([零一二两三四五六七八九十百千\d]+)(?:厘米|公分|cm))?', clean)
    if match:
        centimetres = 20 if match.group(2) is None else spoken_number(match.group(2))
        if centimetres is None:
            raise ValueError('distance_requires_5_to_1000_cm')
        return dict(out, action='manual_move', **extension_distance_parameters(
            'forward' if match.group(1) in ('前进', '向前走') else 'reverse', centimetres/100.))
    if clean in ('自动找火灭火', '开始自动找火灭火', '开启自动灭火', '开始自动灭火'):
        return dict(out, action='auto_fire')
    if clean in ('对选定火焰自动瞄准喷水并关水', '对选定火焰自动瞄准喷水关水',
                 '一键瞄准喷水关水', '一键瞄准喷水并关水', '对选定火焰执行一轮灭火'):
        return dict(out, action='extinguish_selected')
    if clean in ('切换自动模式', '到达后自动灭火', '到达后自动找火灭火', '到达火场后自动灭火', '设置自动灭火模式'):
        return dict(out, action='set_arrival_policy', arrival_policy='auto')
    if clean in ('切换手动模式', '手动模式', '语音控制模式', '等待语音指令', '到达后等待语音指令',
                 '到达火场后等待语音指令', '设置语音控制模式', '停止自动灭火', '关闭自动灭火'):
        return dict(out, action='set_arrival_policy', arrival_policy='manual')
    if clean in ('识别红绿灯', '开始识别红绿灯', '开启红绿灯识别', '观察红绿灯', '寻找红绿灯'):
        return dict(out, action='traffic_observe')
    if clean in ('识别斑马线', '开始识别斑马线', '寻找斑马线'):
        raise ValueError('crosswalk_recognition_removed')
    if clean in ('开始巡逻', '开始巡航', '开启巡逻', '巡逻', '开始行驶', '继续行驶', '继续巡逻'):
        return dict(out, action='patrol')
    if clean in ('观察', '开始观察', '开启识别', '开始识别', '开启目标识别', '开启YOLO识别', '开始YOLO识别', 'YOLO识别',
                 '开启识别火焰消防员以及居民', '开启识别火焰消防员居民', '识别火焰消防员居民',
                 '开启火焰消防员居民识别', '开启火焰识别', '开启识别火焰', '观察当前场景',
                 '寻找火焰', '开始寻找火焰', '搜索火焰', '查找火焰', '识别火焰', '查看火场'):
        return dict(out, action='inspect')
    if clean in ('开始喷水', '开启喷水', '开启喷水灭火', '开始喷水灭火', '启动水泵', '开启水泵'):
        return dict(out, action='start_spray')
    if clean in ('报告场景', '报告当前场景', '播报场景', '汇报当前场景'):
        return dict(out, action='report')
    if clean in ('开启场景理解', '开始场景理解', '场景理解', '开启AI对话', '开启ai对话', '开始对话',
                 '分析当前火场', '对当前火场进行分析', '分析火场', '分析当前场景', '请分析当前场景'):
        return dict(out, action='scene')
    match = re.fullmatch(r'(?:自动)?(?:锁定|瞄准|选择)(左边|最左边|右边|最右边|最大|这个|唯一)(?:的)?(?:火焰|火源)', clean)
    if match:
        choice = {'左边': 'leftmost', '最左边': 'leftmost', '右边': 'rightmost', '最右边': 'rightmost',
                  '最大': 'largest', '这个': 'only', '唯一': 'only'}[match.group(1)]
        return dict(out, action='select_fire', selector=choice)
    match = re.fullmatch(r'(?:自动)?(?:锁定|瞄准|选择)([一二两三四五六七八九十\d]+)号(?:的)?(?:火焰|火源)', clean)
    if match:
        value = match.group(1)
        number = spoken_number(value)
        if number is None or not 1 <= number <= 100:
            raise ValueError('invalid_fire_ordinal')
        return dict(out, action='select_fire', target_id=number)
    match = re.fullmatch(r'(?:自动)?(?:锁定|瞄准|选择)(?:从左到右)?第?([一二两三四五六七八九十\d]+)(?:个|团)(?:的)?(?:火焰|火源)', clean)
    if match:
        number = spoken_number(match.group(1))
        if number is None or not 1 <= number <= 32:
            raise ValueError('invalid_fire_ordinal')
        return dict(out, action='select_fire', ordinal=number)
    match = re.fullmatch(r'(?:锁定|瞄准|选择)(?:火焰)([一二两三四五六七八九十\d]+)', clean)
    if match:
        raw = match.group(1)
        number = int(raw) if raw.isascii() and raw.isdigit() else CHINESE_NUMBERS.get(raw)
        if number is None or number <= 0:
            raise ValueError('invalid_fire_id')
        return dict(out, action='select_fire', target_id=number)
    match = re.fullmatch(r'(?:去|前往|到|导航到)(.+?)(观察|巡检|检查|自动找火灭火|自动灭火|等待语音指令|手动灭火)?', clean)
    if match:
        building = resolve_building(match.group(1), buildings)
        result = dict(out, action='navigate', building_id=str(building['building_id']),
                      building_number=str(building['number']), purpose='inspect')
        if match.group(2) in ('自动找火灭火', '自动灭火'):
            result['arrival_policy'] = 'auto'
        elif match.group(2) in ('等待语音指令', '手动灭火'):
            result['arrival_policy'] = 'manual'
        return result
    return dict(out, action='dialogue')


class TaskRouter:
    def __init__(self, buildings=None, navigation_execute_enabled=False, pump_lease_s=5., manual_motion_enabled=False, extension_enabled=False):
        self.buildings = buildings or {'buildings': []}
        self.building_ids = {str(b['building_id']) for b in self.buildings['buildings']}
        self.navigation_execute = bool(navigation_execute_enabled)
        self.manual_motion_enabled = bool(manual_motion_enabled)
        self.extension_enabled = bool(extension_enabled)
        if not finite(pump_lease_s) or not 0 < pump_lease_s <= 10:
            raise ValueError('invalid_pump_lease')
        self.pump_lease_s = float(pump_lease_s)
        self.seen = OrderedDict()
        self.last_arm = None

    def route(self, request, now, mission=None, proposal=None, selection=None, actuator=None,
              telemetry=None, link=None, navigation=None, extension=None, pump_manual=None):
        if not isinstance(request, dict):
            raise ValueError('command_must_be_object')
        cid, source = request.get('command_id'), request.get('source')
        if not isinstance(cid, str) or not re.fullmatch(r'[A-Za-z0-9_:.-]{1,100}', cid):
            raise ValueError('invalid_command_id')
        if source not in ('voice', 'text', 'web', 'test'):
            raise ValueError('unknown_source')
        if source == 'web' and request.get('action') == 'navigate':
            request = dict(request)
            request.setdefault('arrival_policy', 'manual')
        ttl, issued = request.get('ttl_s', 5.), request.get('issued_at')
        action = request.get('action')
        if action not in ('stop', 'stop_spray') and ('patrol_fire' in request or 'fire_ignore_s' in request):
            raise ValueError('retired_patrol_fire_use_region_search')
        if action not in ('stop_spray', 'stop'):
            if not finite(ttl) or not 0 < ttl <= 10 or not finite(issued) or not finite(now) or not 0 <= now-issued <= ttl:
                raise ValueError('expired_or_future_command')
            if action == 'manual_pump_test' and now - issued > 2.:
                raise ValueError('expired_or_future_command')
        # A stop still cancels an active burst even when the sending interface has stale time.
        if cid in self.seen and action not in ('stop_spray', 'stop'):
            return {'accepted': False, 'reason': 'duplicate', 'command_id': cid, 'stamp': now}
        self.seen[cid] = now
        while len(self.seen) > 512:
            self.seen.popitem(last=False)
        text = request.get('text', '')
        if not isinstance(text, str) or len(text) > 200:
            raise ValueError('invalid_text')
        # An old or mismatched ingress must not downgrade a clear navigation
        # command into a question that the LLM answers without executing it.
        if text and action == 'dialogue':
            parsed_dialogue = parse_task_text(text, self.buildings)
            if parsed_dialogue['action'] != 'dialogue':
                raise ValueError('text_action_mismatch')
        # Reparse text actions at this trust boundary: action labels cannot smuggle a negated sentence.
        if text and action not in ('dialogue', 'report', 'scene'):
            parsed = parse_task_text(text, self.buildings)
            if source == 'web' and parsed['action'] == 'navigate':
                parsed.setdefault('arrival_policy', 'manual')
            if parsed['action'] != action:
                raise ValueError('text_action_mismatch')
            if action == 'select_fire' and any(parsed.get(k) != request.get(k) for k in ('target_id', 'selector', 'ordinal')):
                raise ValueError('text_target_mismatch')
            if action == 'navigate' and parsed.get('building_id') != request.get('building_id'):
                raise ValueError('text_building_mismatch')
            if action == 'navigate_point' and any(parsed.get(k) != request.get(k) for k in ('start_point', 'target_point', 'start_declared', 'start_heading_deg')):
                raise ValueError('text_point_mismatch')
            if action == 'search_region' and any(parsed.get(k) != request.get(k) for k in ('region_id', 'execute_fire')):
                raise ValueError('text_region_mismatch')
            if action == 'manual_pump_test' and (type(request.get('duration_ms')) is not int
                    or parsed.get('duration_ms') != request.get('duration_ms')):
                raise ValueError('text_manual_pump_duration_mismatch')
            if action in ('navigate', 'set_arrival_policy') and parsed.get('arrival_policy') != request.get('arrival_policy'):
                raise ValueError('text_arrival_policy_mismatch')
            if action == 'manual_move' and any(parsed.get(k) != request.get(k) for k in
                    ('direction', 'distance_m', 'speed_m_s', 'max_duration_s')):
                raise ValueError('text_manual_motion_mismatch')
            if action == 'route_turn' and any(parsed.get(k) != request.get(k) for k in ('junction_count', 'turn')):
                raise ValueError('text_route_turn_mismatch')
            if action == 'seek_fire' and parsed.get('after_fire') != request.get('after_fire'):
                raise ValueError('text_seek_fire_mismatch')
            if action == 'set_reference' and any(parsed.get(k) != request.get(k) for k in ('x_cm','y_cm','heading_deg','confirmed')):
                raise ValueError('text_reference_mismatch')
            if action=='set_zebra_detection' and parsed.get('enabled') is not request.get('enabled'):
                raise ValueError('text_zebra_switch_mismatch')
        out = dict(accepted=True, command_id=cid, stamp=now, action=action,
                   mission=None, fire_select=None, actuator=None, navigation=None, question=None, speech=None, auto=None, extension=None)
        mission, proposal, selection, actuator = mission or {}, proposal or {}, selection or {}, actuator or {}
        if (self.extension_enabled and action not in ('stop','stop_spray','finish_task','extension_vision','extension_dialogue') and
                (action in ('dialogue','report','scene') or source=='voice') and not dialogue_allowed(extension,now)):
            raise ValueError('select_observation_task_for_voice_and_dialogue')
        if self.extension_enabled and (action == 'auto_fire' or
                action == 'set_arrival_policy' and request.get('arrival_policy') == 'auto' or
                action == 'seek_fire' and request.get('after_fire') == 'auto'):
            raise ValueError('manual_spray_command_required_for_extension_tasks')
        if self.extension_enabled and action=='navigate':
            raise ValueError('building_navigation_not_registered_use_confirmed_junction_route')
        # The live console never falls back to destination preview or a pump dry-run.
        # Recheck here after the HTTP queue: configuration may change before ROS dispatch.
        if source == 'web':
            if action == 'navigate' and not self.navigation_execute:
                raise ValueError('real_navigation_not_enabled')
            needs_pump = action in ('start_spray', 'auto_fire', 'extinguish_selected') or (
                action in ('navigate', 'set_arrival_policy') and request.get('arrival_policy') == 'auto')
            if needs_pump:
                if not fresh(actuator, now, .5):
                    raise ValueError('real_actuator_feedback_required')
                if actuator.get('dry_run') is not False or actuator.get('enabled') is not True:
                    raise ValueError('real_actuator_not_enabled')
                if actuator.get('pump_enabled') is not True:
                    raise ValueError('real_pump_not_enabled')
                if actuator.get('estop') or actuator.get('error'):
                    raise ValueError('actuator_fault_or_estop')
        disarm = dict(action='disarm', command_id=cid, stamp=now, issued_at=now, ttl_s=.5)
        cancel_auto = dict(action='cancel', command_id=cid, issued_at=now, ttl_s=2.)
        if action == 'manual_pump_test':
            if not text or source not in ('voice', 'text', 'web'):
                raise ValueError('manual_pump_requires_explicit_text')
            if source == 'voice' and (not isinstance(request.get('voice_verification'), dict)
                    or request['voice_verification'].get('verified') is not True):
                raise ValueError('manual_pump_independent_asr_required')
            reason = manual_pump_request_guard(pump_manual, now)
            if reason:
                raise ValueError(reason)
            out['pump_manual'] = dict(action='start', duration_ms=request['duration_ms'],
                operator_confirmed=True, command_id=cid, issued_at=now, ttl_s=2.)
            out['speech'] = f'已请求手动定点试喷{request["duration_ms"]}毫秒，等待水泵实际反馈。'
            return out
        if action == 'search_region':
            if not self.extension_enabled:
                raise ValueError('real_region_search_not_enabled')
            if not text or source not in ('voice', 'text', 'web'):
                raise ValueError('region_search_requires_explicit_text')
            if request.get('region_id') not in ('A', 'B', 'C', 'D') or type(request.get('execute_fire')) is not bool:
                raise ValueError('region_search_requires_explicit_region_and_fire')
            execute_fire = request['execute_fire']
            if source == 'voice':
                verification = request.get('voice_verification')
                if not isinstance(verification, dict) or verification.get('verified') is not True:
                    raise ValueError('region_search_independent_asr_required')
            if not fresh(extension, now, .6) or extension.get('real_uart') is not True or extension.get('status_fresh') is not True:
                raise ValueError('real_extension_uart_required')
            if extension.get('task') not in ('dialogue', 'semantic'):
                raise ValueError('region_search_requires_extension2')
            ready_key, reason_key = ('region_ready', 'region_reason') if execute_fire else ('region_observe_ready', 'region_observe_reason')
            if extension.get(ready_key) is not True:
                raise ValueError(extension.get(reason_key) or 'region_search_not_ready')
            out['extension'] = dict(request, issued_at=now, ttl_s=3.)
            out['actuator'], out['auto'] = disarm, cancel_auto
            out['navigation_parser'] = 'rules'
            out['rule_navigation'] = dict(original_text=text, region_id=request['region_id'], execute_fire=execute_fire, model_called=False)
            if execute_fire:
                out['speech'] = (f'已解析在{request["region_id"]}区域搜寻火源并执行灭火。等待执行端确认；'
                                 '由FPGA连续导航，稳定识别火焰后停车，再前进或后退逐个对中。'
                                 '停稳居中一秒、定位和云台条件就绪后瞄准短喷；确认关水、处理完本批火焰后返回原路线暂停位置，'
                                 '继续同一条路线，全部检查完成后驶出区域停车。')
            else:
                out['speech'] = (f'已解析前往{request["region_id"]}区域搜索火焰。等待执行端确认；'
                                 '由FPGA连续导航，火焰置信度至少90%、至少四帧且稳定0.45秒才停车。'
                                 '停车后前进或后退，让本批火焰逐个居中并停稳一秒，再返回原路线暂停位置继续；'
                                 '全部检查完成后驶出区域停车，搜索模式不喷水。')
            return out
        if action in ('confirm_point_start', 'navigate_point'):
            validate_point_request(request)
            if action == 'navigate_point' and source == 'voice':
                verification = request.get('voice_verification')
                if not isinstance(verification, dict) or verification.get('verified') is not True:
                    raise ValueError('numbered_navigation_independent_asr_required')
            # navigation_execute controls the older building/CRC navigator.
            # The fixed-frame owner independently verifies enabled real UART.
            if not self.extension_enabled:
                raise ValueError('real_numbered_navigation_not_enabled')
            if request.get('start_declared') and (not text or source not in ('voice', 'text', 'web')):
                raise ValueError('explicit_start_statement_required')
            reason = point_request_guard(extension, now, action == 'confirm_point_start' or request.get('start_declared') is True)
            if reason:
                raise ValueError(reason)
            heading = request.get('start_heading_deg')
            reference = extension.get('point_reference') or {}
            start = request.get('start_point', reference.get('point_id'))
            map_reference = extension.get('point_start_reference') or {}
            arbitrary_start = (action == 'navigate_point' and request.get('start_declared') is not True
                and request.get('start_point') is None and map_reference.get('valid') is True
                and map_reference.get('kind') == 'map')
            if arbitrary_start:
                start = None
            if action == 'navigate_point' and heading is None:
                if arbitrary_start and finite(map_reference.get('heading_deg')):
                    heading = map_reference['heading_deg']
                elif (reference.get('confirmed') is not True or reference.get('point_id') != start
                        or reference.get('heading_confirmed') is not True
                        or not finite(reference.get('current_heading_deg'))):
                    raise ValueError('numbered_start_heading_required')
                else:
                    heading = reference['current_heading_deg']
            out['extension'] = dict(request, issued_at=now, ttl_s=3.)
            if action == 'navigate_point':
                policy = 'manual'
                out['extension']['arrival_policy'] = policy
                out['actuator'], out['auto'] = disarm, cancel_auto
                out['navigation_parser'] = 'rules'
                out['rule_navigation'] = dict(original_text=text, start_point=start,
                    target_point=request['target_point'], start_heading_deg=heading, model_called=False)
                direction = {0:'右',90:'上',180:'左',-180:'左',-90:'下'}.get(heading)
                orientation = ('朝'+direction) if direction else f'朝向{heading:g}度'
                behavior = '到达停稳后自动YOLO识别，瞄准和喷水等待单独指令。'
                origin = '从已确认当前位置前往' if arbitrary_start else f'{start}号点到'
                out['speech'] = (f'规则解析：{origin}{request["target_point"]}号点，车头{orientation}。'
                    '指令已解析，等待执行端确认；是否开始导航以FPGA状态为准。' + behavior)
            else:
                out['speech'] = '已提交{}号点的实际起点确认，请等待状态更新。'.format(request['point_id'])
            return out
        if self.extension_enabled and action in ('inspect', 'auto_fire', 'select_fire'):
            out['extension'] = dict(action='stop', command_id=cid, issued_at=now, ttl_s=3.)
        if self.extension_enabled and action in ('stop', 'finish_task', 'extension_vision', 'extension_dialogue', 'set_zebra_detection', 'traffic_task', 'traffic_drive', 'traffic_observe', 'manual_move', 'route_turn', 'seek_fire', 'set_reference'):
            if action=='set_zebra_detection':
                if type(request.get('enabled')) is not bool:raise ValueError('zebra_enabled_must_be_boolean')
                reason=zebra_switch_guard(extension,now)
                if reason:raise ValueError(reason)
            if action in ('manual_move', 'route_turn', 'seek_fire','traffic_drive'):
                capability = {'manual_move':'distance', 'route_turn':'route_turn', 'seek_fire':'velocity','traffic_drive':'traffic'}[action]
                reason = extension_start_guard(extension, now, capability)
                if reason:
                    raise ValueError(reason)
                if action=='traffic_drive' and extension.get('zebra_detection_enabled') is not True:
                    raise ValueError('enable_fpga_zebra_detection_before_traffic_drive')
                if action == 'manual_move':
                    expected = extension_distance_parameters(request.get('direction'), request.get('distance_m'))
                    if any(expected[k] != request.get(k) for k in expected):
                        raise ValueError('extension_distance_parameters_mismatch')
                if action == 'seek_fire' and request.get('after_fire') == 'auto':
                    if not fresh(actuator, now, .5) or actuator.get('dry_run') is not False or actuator.get('pump_enabled') is not True:
                        raise ValueError('real_pump_required_for_auto_after_seek')
            out['extension'] = dict(request, issued_at=now, ttl_s=3.)
            if action == 'traffic_observe':
                out['extension']['action'] = 'traffic_task'
            if action not in ('stop', 'finish_task'):
                out['actuator'], out['auto'] = disarm, cancel_auto
                out['speech'] = ('已开启拓展任务一，正在进行YOLO现场识别。你可以说分析当前火场、搜索火焰，或输入要瞄准的火焰编号；喷水需要单独开始和停止指令。' if action == 'extension_vision' else
                    '已开启拓展任务二，等待导航指令。请说你现在在几号位置、请前往几号位置后停下；到达停稳后自动YOLO识别，等待选择火焰。' if action=='extension_dialogue' else
                    '已提交 FPGA 斑马线识别开关请求，等待实际状态回读。' if action=='set_zebra_detection' else
                    '已选择独立交通任务；斑马线由 FPGA 处理，红绿灯识别由 X5 处理。' if action in ('traffic_task','traffic_observe') else
                    '请求已提交，等待 FPGA 实际执行和停车反馈。')
                return out
        if action not in ('dialogue', 'report', 'scene'):
            out['auto'] = cancel_auto
        if action in ('stop_spray', 'stop'):
            out['actuator'] = disarm
            if action == 'stop':
                out['mission'] = {'mode': 'idle'}
                out['navigation'] = dict(action='pause', command_id=cid, issued_at=now, ttl_s=5.)
            out['speech'] = '已请求停止喷水。' if action == 'stop_spray' else '已请求停车并停止喷水。'
        elif action == 'finish_task':
            out['actuator'] = disarm
            out['mission'] = {'mode': 'standby'}
            out['navigation'] = dict(action='cancel', command_id=cid, issued_at=now, ttl_s=5.)
            out['speech'] = '已请求结束任务、停止喷水和导航，进入待机状态。'
        elif action == 'set_arrival_policy':
            policy = request.get('arrival_policy')
            if policy not in ('auto', 'manual'):
                raise ValueError('invalid_arrival_policy')
            out['actuator'] = disarm
            out['mission'] = {'action': 'set_arrival_policy', 'arrival_policy': policy}
            out['speech'] = ('已设置到达后自动找火灭火，仍需满足停车、标定和硬件条件。' if policy == 'auto'
                             else '已设置到达后等待语音指令，自动灭火已取消。')
        elif action == 'extinguish_selected':
            if not text or source not in ('text', 'web', 'voice'):
                raise ValueError('selected_fire_round_requires_explicit_text')
            if source == 'voice' and (not isinstance(request.get('voice_verification'), dict) or
                    request['voice_verification'].get('verified') is not True):
                raise ValueError('selected_fire_round_independent_asr_required')
            reason = selected_fire_round_reason(now, mission=mission, selection=selection,
                proposal=proposal, actuator=actuator, extension=extension,
                extension_enabled=self.extension_enabled, request=request)
            if reason:
                raise ValueError(reason)
            out['actuator'] = dict(disarm, preserve_auto_run_id=cid)
            out['auto'] = dict(action='start', command_id=cid, issued_at=now, ttl_s=2.,
                preferred_target_id=selection['target_id'],
                preferred_registry_session=selection['registry_session'], return_to_standby=False)
            out['speech'] = ('已请求对火焰{}自动瞄准，确认锁定后只短喷一轮，再确认关水并保留观察。'
                             '这不代表火焰已经扑灭。').format(selection['target_id'])
        elif action == 'auto_fire':
            if fresh(mission, now, .5) and mission.get('traffic_active'):
                raise ValueError('complete_crossing_before_auto_fire')
            out['actuator'] = dict(disarm, preserve_auto_run_id=cid)
            out['mission'] = {'mode': 'inspect', 'arrival_policy': 'auto'}
            out['navigation'] = dict(action='pause', command_id=cid, issued_at=now, ttl_s=5.)
            out['auto'] = dict(action='start', command_id=cid, issued_at=now, ttl_s=2.)
            if request.get('preferred_target_id') is not None or request.get('preferred_registry_session') is not None:
                preferred=request.get('preferred_target_id');registry=request.get('preferred_registry_session')
                if type(preferred) is not int or preferred<1 or not isinstance(registry,str) or not registry or len(registry)>100:
                    raise ValueError('invalid_preferred_fire_identity')
                active=(extension or {}).get('active') or {}
                if (not self.extension_enabled or not fresh(extension,now,.5) or
                        active.get('action')!='seek_fire' or active.get('phase')!='completed' or
                        active.get('after_fire')!='auto' or cid!='seek-auto:'+str(active.get('command_id')) or
                        extension.get('vehicle_stopped') is not True or
                        extension.get('target_id')!=preferred or extension.get('target_registry_session')!=registry):
                    raise ValueError('preferred_fire_requires_completed_seek_identity')
                out['auto'].update(preferred_target_id=preferred,preferred_registry_session=registry)
            out['speech'] = '已请求自动找火灭火；停车和瞄准条件满足后才允许短时喷水。'
        elif action == 'traffic_observe':
            if not fresh(mission, now, .5):
                raise ValueError('fresh_mission_required_for_traffic')
            if mission.get('emergency_stop'):
                raise ValueError('fpga_estop')
            out['actuator'] = disarm
            out['mission'] = dict(mode='traffic', request_id=cid, issued_at=now, ttl_s=2.)
            out['speech'] = ('已请求停车，等待两秒后寻找红绿灯；红灯或不确定时保持停车。'
                             '只有原有行驶任务满足稳定绿灯与硬件条件才允许继续。')
        elif action in ('patrol', 'inspect'):
            out['actuator'] = disarm
            out['mission'] = {'mode': 'patrol' if action == 'patrol' else 'inspect'}
            out['navigation'] = dict(action='pause', command_id=cid, issued_at=now, ttl_s=5.)
            out['speech'] = '已请求巡逻；路口由FPGA事件或明确的红绿灯识别指令触发。' if action == 'patrol' else '已请求停车抬头观察。'
            traffic_pending = bool(action == 'inspect' and fresh(mission, now, .5) and mission.get('traffic_active'))
            if traffic_pending:
                out['reason'] = 'traffic_pending_observation_deferred'
                out['speech'] = '路口尚未完成，已请求停车；楼面识别暂缓，请继续巡逻完成过街后再开启识别。'
        elif action == 'select_fire':
            if not fresh(mission, now, .5) or mission.get('mode') not in ('inspect', 'fire') or not mission.get('vehicle_stopped') or mission.get('traffic_active'):
                raise ValueError('observe_and_stop_before_selecting_fire')
            if sum(request.get(key) is not None for key in ('target_id', 'ordinal', 'selector')) != 1:
                raise ValueError('exactly_one_fire_selection_required')
            if request.get('ordinal') is not None:
                ordinal = request['ordinal']
                if type(ordinal) is not int or not 1 <= ordinal <= 32:
                    raise ValueError('invalid_fire_ordinal')
                selected = {'ordinal': ordinal}
            elif request.get('target_id') is not None:
                target = request['target_id']
                if type(target) is not int or target <= 0:
                    raise ValueError('invalid_target_id')
                selected = {'target_id': target}
            else:
                selector = request.get('selector')
                if selector not in ('leftmost', 'rightmost', 'largest', 'only'):
                    raise ValueError('explicit_target_required')
                selected = {'selector': selector}
            out['actuator'] = disarm
            out['mission'] = {'mode': 'fire'}
            out['fire_select'] = dict(selected, action='select', command_id=cid, stamp=now, issued_at=now, ttl_s=2.)
            out['speech'] = ('已请求锁定当前画面从左到右第{}个火焰，尚未启动喷水。'.format(selected['ordinal'])
                             if 'ordinal' in selected else '已请求锁定所选火焰，尚未启动喷水。')
        elif action == 'start_spray':
            if not fresh(mission, now, .5) or mission.get('mode') != 'fire' or not mission.get('vehicle_stopped') or not mission.get('gimbal_ready') or mission.get('traffic_active'):
                raise ValueError('fire_mode_stopped_ready_required')
            if not fresh(selection, now, .5, 'capture_stamp') or selection.get('valid') is not True:
                raise ValueError('fresh_explicit_fire_selection_required')
            if not fresh(proposal, now, .4, 'capture_stamp') or proposal.get('valid') is not True or proposal.get('pump_candidate') is not True:
                raise ValueError('fresh_calibrated_fire_lock_required')
            keys = ('target_id', 'selection_generation', 'registry_session')
            if (type(selection.get('target_id')) is not int or selection['target_id'] <= 0
                    or type(selection.get('selection_generation')) is not int or selection['selection_generation'] <= 0
                    or not isinstance(selection.get('registry_session'), str) or not selection['registry_session']):
                raise ValueError('invalid_selection_identity')
            if any(selection.get(k) is None or selection[k] != proposal.get(k) for k in keys):
                raise ValueError('selected_target_does_not_match_lock')
            if not fresh(actuator, now, .5) or actuator.get('estop') or actuator.get('error') or (actuator.get('guard') or {}).get('armed'):
                raise ValueError('actuator_unavailable_or_already_armed')
            guard = actuator.get('guard') or {}
            if guard.get('state') in ('COOLDOWN', 'EXHAUSTED', 'FAULT') or guard.get('requires_disarm'):
                raise ValueError('pump_cooldown_or_fault')
            if self.last_arm is not None and now-self.last_arm < 1.:
                raise ValueError('repeated_arm_cooldown')
            self.last_arm = now
            out['actuator'] = dict(action='arm_pump', command_id=cid, stamp=now, issued_at=now, ttl_s=.5,
                                   lease_s=self.pump_lease_s, **{k: selection[k] for k in keys})
            out['speech'] = '已提交短时喷水请求，执行器仍会检查锁定、标定和硬件条件。'
        elif action == 'manual_move':
            if not text:
                raise ValueError('explicit_manual_phrase_required')
            motion = validate_manual(request)
            if not self.manual_motion_enabled:
                raise ValueError('manual_motion_not_enabled')
            reason = manual_start_guard(now, mission=mission, actuator=actuator,
                telemetry=telemetry, link=link, navigation=navigation)
            if reason:
                raise ValueError(reason)
            out['actuator'], out['mission'] = disarm, {'mode': 'navigate', 'arrival_policy': 'manual'}
            out['navigation'] = dict(action='manual_move', **motion, command_id=cid,
                owner='fpga', issued_at=now, ttl_s=3., confirmed=True, mode='execute', arrival_policy='manual')
            out['speech'] = '已提交{}{}厘米的短程请求，等待FPGA反馈；到距离或超时必须停车。'.format(
                '前进' if motion['direction'] == 'forward' else '后退', round(motion['distance_m']*100))
        elif action == 'navigate':
            if navigation and navigation.get('motion_kind') == 'manual' and navigation_busy(navigation):
                raise ValueError('stop_manual_motion_before_navigation')
            bid = request.get('building_id')
            if bid not in self.building_ids:
                raise ValueError('unknown_building')
            policy = request.get('arrival_policy', mission.get('arrival_policy', 'manual') if fresh(mission, now, .5) else 'manual')
            if policy not in ('auto', 'manual'):
                raise ValueError('invalid_arrival_policy')
            building = next(b for b in self.buildings['buildings'] if str(b['building_id']) == bid)
            out['actuator'], out['mission'] = disarm, {'mode': 'navigate', 'arrival_policy': policy}
            out['navigation'] = dict(action='navigate', command_id=cid, building_id=bid,
                                     building_number=str(building['number']), purpose='inspect', owner='fpga',
                                     issued_at=now, ttl_s=5., confirmed=True, arrival_policy=policy,
                                     mode='execute' if self.navigation_execute else 'preview')
            out['speech'] = ('已向 FPGA 提交前往{}号楼的目的地，请等待导航状态确认。'.format(building['number'])
                             if self.navigation_execute else '已解析楼栋目的地；当前为预览，尚未发送实车导航指令。')
        elif action in ('dialogue', 'report', 'scene'):
            out['question'] = dict(text='请分析当前场景' if action == 'scene' else text or '报告当前场景',
                                   command_id=cid, stamp=now, ttl_s=3.)
        else:
            raise ValueError('unsupported_action')
        return out
