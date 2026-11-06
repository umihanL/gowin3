"""Local Qwen navigation interpretation with independently grounded parameters.

This module performs bounded loopback inference only. It never sends UART,
publishes ROS commands, or executes model-generated code. Call it on a worker;
the caller must recheck current vehicle/link state before routing ``command``.
"""
from __future__ import annotations
import json
import math
import re
import time
import urllib.parse
import urllib.request

from .task_interaction import parse_task_text, spoken_number, validate_point_request


POINT = re.compile(r'第?([零一二两三四五六七八九十\d]+)号(?:点位|导航点|点|位置)')
ANGLE_NUMBER = r'(?:[+-]?\d+(?:\.\d+)?|[负正]?[零〇一二两三四五六七八九十百点]+)'
REASONS = ('', 'need_points', 'ambiguous', 'not_command', 'multiple_actions')
FIELDS = ('intent', 'start_point', 'target_point', 'start_heading_deg', 'start_declared',
          'start_evidence', 'target_evidence', 'heading_evidence', 'reason')
SCHEMA = dict(type='object', additionalProperties=False, required=list(FIELDS), properties={
    'intent': dict(type='string', enum=['navigate_point','clarification','not_navigation']),
    'start_point': dict(type=['integer','null'], minimum=1, maximum=9),
    'target_point': dict(type=['integer','null'], minimum=1, maximum=9),
    'start_heading_deg': dict(type=['number','null'], minimum=-180, maximum=180),
    'start_declared': dict(type='boolean'),
    'start_evidence': dict(type='string',maxLength=80),
    'target_evidence': dict(type='string',maxLength=80),
    'heading_evidence': dict(type='string',maxLength=40),
    'reason': dict(type='string',enum=list(REASONS))})
SYSTEM = '''你是车载导航意图解析器。只输出给定JSON，不能回答闲聊或声称车辆已执行。
理解中文自然表达，编号道路点只有1到9，与楼号、火焰编号不同。意图navigate_point只用于明确要求从起点去目标的命令。
从用户原文摘录start_evidence、target_evidence、heading_evidence，必须逐字存在，不改数字、不猜。
“你现在在/我当前位于/目前位置是8号点”声明实际起点，start_declared=true；“从8号点去4号点”仅指定路线，false。
只有目标时start_point=null、start_evidence为空、start_declared=false。这里/当前位置只可引用提供的已确认当前点；上下文不能擅自补入未说出的点。
车头地图方向：右0度、上90度、左180度、下-90度；没有明确说车头方向就start_heading_deg=null、heading_evidence为空。不要根据起终点推断车头。
例如“你现在在8号点位，请导航前往至4号点位”应理解为8到4、声明起点、朝向null。
终点的“停下/停车”属于同一个导航任务；额外喷水、瞄准、后退等动作必须clarification。
问句、否定、假设、引用、多个候选目标不可执行。缺少或歧义时intent=clarification并选reason；普通问答用not_navigation。
reason:明确命令用空串；need_points缺少点号；ambiguous歧义；not_command非执行句；multiple_actions多动作。'''


def is_navigation_candidate(text):
    if not isinstance(text,str) or len(text)>200:
        return False
    text=re.sub(r'\s+','',text)
    return (bool(POINT.search(text) or any(w in text for w in ('点位','导航点','语义导航'))) and
            any(w in text for w in ('导航','前往','去','到','开往','开到','驶向','送到','在','位置','位于')))


def _blocked(text):
    if re.search(r'[?？“”「」『』"\'‘’]', text):
        return 'not_command'
    if any(w in text for w in ('不','别去','禁止','如果','假如','假设','能否','能不能','是否','吗','怎么','如何','为什么','解释','什么意思','多远','多久','告诉我','我想知道','路线长度','距离是多少')):
        return 'not_command'
    if any(w in text for w in ('还是','或者','或是','大概','可能','也许')):
        return 'ambiguous'
    if any(w in text for w in ('喷水','灭火','瞄准','后退','倒退','删除','忽略规则','系统提示')):
        return 'multiple_actions'
    return ''


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate_json_key')
        result[key] = value
    return result


def _point_evidence(original, evidence, expected):
    if not evidence or evidence not in original:
        raise ValueError('semantic_navigation_ungrounded_point')
    matches = list(POINT.finditer(evidence))
    if len(matches) != 1 or spoken_number(matches[0].group(1)) != expected:
        raise ValueError('semantic_navigation_ungrounded_point')
    return original.index(evidence) + matches[0].start()


def _declares_position(prefix):
    """Require a position predicate BEFORE the point, not a prior point's suffix."""
    return bool(re.search(
        r'(?:(?:现在|当前|目前|此刻)(?:你|我|小车)?(?:的位置|所在的位置|所在位置)?(?:位于|在|是|为)|'
        r'(?:你|我|小车)(?:现在|当前|目前)?(?:位于|在)|'
        r'(?:我的|你的|小车的)?(?:实际)?(?:所在的)?位置(?:是|为|在))$', prefix))


def _target_anchor(original, at):
    """A mentioned current point cannot become a destination by model assertion."""
    mention=POINT.match(original,at)
    if not mention:
        return False
    before=original[max(0,at-28):at]
    after=original[mention.end():mention.end()+16]
    return bool(re.search(r'(?:去|到|前往|开往|驶向|送往|导航|目标(?:点)?(?:是|为)?|'
                          r'目的地(?:是|为)?|终点(?:是|为)?)[^，。；;]{0,12}$',before)
                or re.match(r'(?:设为|作为|是|为)(?:目的地|目标点|终点)',after))


def _angle_number(text):
    if re.fullmatch(r'[+-]?\d+(?:\.\d+)?',text):
        return float(text)
    sign=-1 if text.startswith('负') else 1
    if text.startswith(('负','正')):text=text[1:]
    parts=text.split('点')
    if len(parts)>2:raise ValueError('invalid_spoken_heading')
    integer=parts[0]
    if '百' in integer:
        hundreds,tail=integer.split('百',1)
        lead=spoken_number(hundreds)
        tail=tail.removeprefix('零')
        remainder=spoken_number(tail) if tail else 0
        if lead is None or not 1<=lead<=9 or remainder is None or not 0<=remainder<=99:
            raise ValueError('invalid_spoken_heading')
        whole=lead*100+remainder
    else:
        whole=0 if integer in ('零','〇') else spoken_number(integer)
    if whole is None:raise ValueError('invalid_spoken_heading')
    digits={'零':'0','〇':'0','一':'1','二':'2','两':'2','三':'3','四':'4','五':'5','六':'6','七':'7','八':'8','九':'9'}
    if len(parts)==2:
        if not parts[1] or any(x not in digits for x in parts[1]):raise ValueError('invalid_spoken_heading')
        return sign*float(str(whole)+'.'+''.join(digits[x] for x in parts[1]))
    return sign*float(whole)


def _heading_evidence(original, evidence, expected):
    if not evidence or evidence not in original:
        raise ValueError('semantic_navigation_ungrounded_heading')
    match = re.fullmatch(r'(?:车头|车身)?(?:朝向|面向|朝|向)(?:地图)?'
                         r'(右|上|左|下)(?:边|侧|方)?', evidence)
    if match:
        value = {'右':0.,'上':90.,'左':180.,'下':-90.}[match.group(1)]
    else:
        match = re.fullmatch(r'(?:车头|车身)?(?:朝向|面向|朝|向)('+ANGLE_NUMBER+r')度',evidence)
        if not match:
            raise ValueError('semantic_navigation_ungrounded_heading')
        value = _angle_number(match.group(1))
    if value != expected:
        raise ValueError('semantic_navigation_ungrounded_heading')


def _context(context, now):
    if not isinstance(context,dict):
        return {}
    stamp=context.get('stamp')
    if (type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0 <= now-stamp <= 1.5
            or context.get('current_point_confirmed') is not True
            or type(context.get('current_point')) is not int or not 1<=context['current_point']<=9):
        return {}
    result={key:context[key] for key in ('current_point','current_point_confirmed','stamp')}
    heading=context.get('current_heading_deg')
    if (context.get('current_heading_confirmed') is True and type(heading) in (int,float)
            and math.isfinite(heading) and -180<=heading<=180):
        result.update(current_heading_confirmed=True,current_heading_deg=heading)
    return result


def verify_semantic_voice(primary_text, secondary_text):
    """Compare independent ASR facts before Qwen, without fixed sentence syntax.

    The caller must separately verify the secondary decoder's confidence. This
    check cannot grant permission: Qwen interpretation and source grounding must
    still pass, then the normal hardware/state guards run.
    """
    result=dict(required=True,verified=False,reason='voice_command_disagreement')
    if not all(isinstance(t,str) and 0<len(t)<=200 for t in (primary_text,secondary_text)):
        return result
    # Vosk emits word-separated Chinese while SenseVoice usually does not.
    # Whitespace is not a semantic difference or an excuse to change a digit.
    primary_text,secondary_text=(re.sub(r'\s+','',t) for t in (primary_text,secondary_text))
    if not all(is_navigation_candidate(t) and not _blocked(t) for t in (primary_text,secondary_text)):
        return result
    def facts(text):
        mentions=list(POINT.finditer(text))
        points=tuple(spoken_number(m.group(1)) for m in mentions)
        if not 1<=len(points)<=2 or any(p is None or not 1<=p<=9 for p in points):
            raise ValueError('point_facts_missing')
        headings=[]
        for m in re.finditer(r'(?:车头|车身)?(?:朝向|面向|朝|向)(?:地图)?'
                             r'([右上左下](?:边|侧|方)?|'+ANGLE_NUMBER+r'度)',text):
            v=m.group(1)
            headings.append(_angle_number(v[:-1]) if v.endswith('度') else {'右':0.,'上':90.,'左':180.,'下':-90.}[v[0]])
        # Use the first mentioned point's prefix. “8号位置” is just a point
        # noun, not a declaration of the following destination's location.
        declarations=tuple(_declares_position(text[:m.start()]) for m in mentions)
        targets=tuple(_target_anchor(text,m.start()) for m in mentions)
        context_ref=any(word in text for word in ('这里','当前位置','刚到的点'))
        return (points,tuple(headings),declarations,targets,context_ref)
    try:
        if facts(primary_text)==facts(secondary_text):result.update(verified=True,reason='')
    except ValueError:
        pass
    return result


def validate_interpretation(original, value, context=None, now=None):
    """Validate schema and independent source evidence; return a canonical command.

    The model chooses semantic roles. The verifier checks exact evidence, point
    role anchors/order, explicit declarations and heading, not an LLM score.
    """
    if not isinstance(value,dict) or set(value)!=set(FIELDS):
        raise ValueError('semantic_navigation_invalid_schema')
    if value['intent'] not in ('navigate_point','clarification','not_navigation') or value['reason'] not in REASONS:
        raise ValueError('semantic_navigation_invalid_schema')
    if type(value['start_declared']) is not bool:
        raise ValueError('semantic_navigation_invalid_schema')
    for key,limit in (('start_evidence',80),('target_evidence',80),('heading_evidence',40)):
        if not isinstance(value[key],str) or len(value[key])>limit:
            raise ValueError('semantic_navigation_invalid_schema')
    for key in ('start_point','target_point'):
        if value[key] is not None and (type(value[key]) is not int or not 1<=value[key]<=9):
            raise ValueError('semantic_navigation_invalid_schema')
    heading=value['start_heading_deg']
    if heading is not None and (type(heading) not in (int,float) or not math.isfinite(heading) or not -180<=heading<=180):
        raise ValueError('semantic_navigation_invalid_schema')
    if value['intent']!='navigate_point':
        return None
    if _blocked(original) or value['reason'] or value['target_point'] is None:
        raise ValueError('semantic_navigation_not_explicit_command')
    evidence_points=[spoken_number(m.group(1)) for m in POINT.finditer(original)]
    mentioned=set(evidence_points)
    if len(evidence_points)>2 or any(p is None or not 1<=p<=9 for p in mentioned):
        raise ValueError('semantic_navigation_ambiguous_points')
    target_at=_point_evidence(original,value['target_evidence'],value['target_point'])
    if not _target_anchor(original,target_at):
        raise ValueError('semantic_navigation_target_action_missing')
    start=value['start_point']
    context=_context(context,time.time() if now is None else now)
    if start is None:
        if value['start_evidence'] or value['start_declared'] or len(evidence_points)!=1:
            raise ValueError('semantic_navigation_ungrounded_point')
    else:
        if value['start_evidence'] in ('这里','当前位置','刚到的点'):
            if (not context or start!=context['current_point'] or value['start_evidence'] not in original
                    or value['start_declared'] or len(evidence_points)!=1):
                raise ValueError('semantic_navigation_context_unavailable')
        else:
            start_at=_point_evidence(original,value['start_evidence'],start)
            if len(evidence_points)!=2 or mentioned!={start,value['target_point']}:
                raise ValueError('semantic_navigation_ambiguous_points')
            if start_at>target_at:
                # “请开到4号点，我现在在8号点” has reversed mention order.
                # Require an explicit actual-position anchor for the latter.
                if not _declares_position(original[:start_at]):
                    raise ValueError('semantic_navigation_point_roles_not_grounded')
        explicit_declared=bool(start is not None and value['start_evidence'] not in ('这里','当前位置','刚到的点') and
            _declares_position(original[:start_at]))
        if value['start_declared'] != explicit_declared:
            raise ValueError('semantic_navigation_declaration_mismatch')
    if heading is None:
        if value['heading_evidence'] or re.search(r'(?:车头|车身|朝向|面向)',original):
            raise ValueError('semantic_navigation_heading_unresolved')
    else:
        if start is None:
            raise ValueError('semantic_navigation_heading_requires_start')
        _heading_evidence(original,value['heading_evidence'],heading)
    command={'action':'navigate_point','target_point':value['target_point']}
    if start is not None:command['start_point']=start
    if value['start_declared']:command['start_declared']=True
    if heading is not None:command['start_heading_deg']=heading
    validate_point_request(command)
    heading_text='' if heading is None else f'车头朝向{heading:g}度'
    canonical=(f'我现在在{start}号点{heading_text}请前往{value["target_point"]}号点'
        if value['start_declared'] else f'从{start}号点{heading_text}前往{value["target_point"]}号点'
        if start is not None else f'前往{value["target_point"]}号点')
    parsed=parse_task_text(canonical)
    if {k:v for k,v in parsed.items() if k!='text'} != command:
        raise ValueError('semantic_navigation_canonical_mismatch')
    return dict(command,text=canonical)


class SemanticNavigationModel:
    def __init__(self, config):
        self.enabled=config.get('enabled') is True
        self.endpoint=config.get('endpoint','http://127.0.0.1:8088/v1/chat/completions')
        address=urllib.parse.urlparse(self.endpoint)
        if (address.scheme!='http' or address.hostname not in ('127.0.0.1','localhost','::1')
                or address.username or address.password or address.query or address.fragment
                or address.path!='/v1/chat/completions'):
            raise ValueError('edge_model_endpoint_must_be_loopback_chat_completions')
        self.model=str(config.get('model','qwen3-1.7b'))[:100]
        timeout=config.get('navigation_timeout_s',90.)
        if (isinstance(timeout,bool) or not isinstance(timeout,(int,float))
                or not 1.<=timeout<=120. or not math.isfinite(timeout)):
            raise ValueError('invalid_navigation_timeout')
        self.timeout=float(timeout)

    def _request(self, text, context):
        payload=dict(model=self.model,messages=[dict(role='system',content=SYSTEM),
            dict(role='user',content=json.dumps({'原话':text,'已核验上下文':context},ensure_ascii=False))],
            max_tokens=256,temperature=0,stream=False,chat_template_kwargs={'enable_thinking':False},
            response_format=dict(type='json_schema',json_schema=dict(name='numbered_navigation',strict=True,schema=SCHEMA)))
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self,*args,**kwargs):raise ValueError('redirect_forbidden')
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect())
        request=urllib.request.Request(self.endpoint,data=json.dumps(payload,ensure_ascii=False).encode(),headers={'Content-Type':'application/json'})
        with opener.open(request,timeout=self.timeout) as response:body=response.read(16385)
        if len(body)>16384:raise ValueError('response_too_large')
        envelope=json.loads(body,object_pairs_hook=_unique)
        choice=envelope['choices'][0]
        if choice.get('finish_reason')!='stop':raise ValueError('generation_truncated_or_incomplete')
        content=choice['message']['content']
        if not isinstance(content,str):raise ValueError('invalid_model_content')
        return json.loads(content,object_pairs_hook=_unique)

    def interpret(self, text, context=None, now=None):
        if not isinstance(text,str) or not text.strip() or len(text)>200:raise ValueError('invalid_text')
        started=time.monotonic()
        result=dict(status='clarification',original_text=text,model=self.model,backend='local_qwen_navigation',
            model_called=False,structured=None,command=None,clarification='',reason='',evidence={})
        reason=_blocked(text)
        if reason:
            result.update(reason='semantic_navigation_'+reason,clarification='这句话包含疑问、否定、歧义或多个动作，未执行导航。请单独明确实际起点、车头方向和目标点。')
            return result
        if not self.enabled:
            result.update(reason='semantic_navigation_model_disabled',clarification='本地导航理解模型未启用，未执行导航。')
            return result
        verified_context=_context(context,time.time() if now is None else now)
        try:
            result['model_called']=True
            value=self._request(text,verified_context)
            result['structured']=value
            command=validate_interpretation(text,value,verified_context,now)
            if command is None:
                result.update(status='not_navigation' if value['intent']=='not_navigation' else 'clarification',
                    reason='semantic_navigation_'+(value['reason'] or 'needs_clarification'),
                    clarification='请说明1至9号中的实际起点、车头地图方向和目标点；本次未下发导航。')
            elif (command.get('start_heading_deg') is None and not
                  (verified_context.get('current_heading_confirmed') is True and
                   command.get('start_point',verified_context.get('current_point'))==verified_context.get('current_point'))):
                result.update(reason='numbered_start_heading_required',
                    clarification='已理解导航点位，但还没有可靠的车头地图朝向。请补充，例如：你现在在8号点位，车头朝右，请前往4号点位。本次未发车。')
            else:
                result.update(status='ready',command=command,evidence=dict(source_grounded=True,
                    context_used=value['start_evidence'] in ('这里','当前位置','刚到的点'),
                    heading_basis='explicit_user_heading' if command.get('start_heading_deg') is not None else 'verified_current_context',
                    original_text=text,model=self.model,structured=value))
        except Exception as exc:
            error=str(exc)[:120] if isinstance(exc,ValueError) else type(exc).__name__
            result.update(reason=error,clarification='导航语义结果未通过原话核验或模型暂不可用，未执行。请核对起点、朝向和目标后重新说一遍。')
        result['elapsed_s']=round(time.monotonic()-started,3)
        return result
