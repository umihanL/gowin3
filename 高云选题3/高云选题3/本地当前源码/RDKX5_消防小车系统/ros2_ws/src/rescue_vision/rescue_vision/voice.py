"""Offline voice intent parsing and evidence-limited scene narration.

This module has no ROS, microphone, network or actuator side effects.
"""
import hashlib
import json
import math
import re
import time
import uuid


CHINESE_NUMBERS = {'一': 1, '二': 2, '两': 2, '三': 3, '四': 4, '五': 5,
                   '六': 6, '七': 7, '八': 8, '九': 9, '十': 10,
                   '十一': 11, '十二': 12, '十三': 13}
for _number in range(14, 100):
    _digits = '零一二三四五六七八九'
    CHINESE_NUMBERS[(_digits[_number // 10] if _number >= 20 else '') + '十'
                    + (_digits[_number % 10] if _number % 10 else '')] = _number

WAKE_WORDS = ('你好消防小车', '消防小车', '你好小车', '小车', '机器人')
DIRECT_STOP_PHRASES = ('停止喷水', '关闭喷水', '停止灭火', '关闭水泵', '停水', '停止喷射', '停止试喷',
                       '紧急停止', '紧急停车', '立即停车', '停车', '停止', '停下')


def normalize_voice_text(text):
    """Only whitespace/punctuation and explicit wake words are normalized."""
    if not isinstance(text, str) or len(text) > 200:
        raise ValueError('invalid_text')
    text = re.sub(r'[\s，。！？、,.!?：:；;]', '', text).strip()
    for prefix in WAKE_WORDS:
        if text.startswith(prefix):
            text = text[len(prefix):]
            break
    for prefix in ('请你', '请'):
        if text.startswith(prefix):
            text = text[len(prefix):]
            break
    return text


def prepare_spoken_command(text, listening_mode='push_to_talk', require_wake_word=True):
    """Continuous microphone input needs an explicit wake prefix; exact stops bypass it.

    This is a command gate, not a neural wake-word detector. Quoted, negated and
    uncertain sentences still pass the shared task parser without gaining actions.
    """
    clean = normalize_voice_text(text)
    if listening_mode not in ('push_to_talk', 'continuous'):
        raise ValueError('invalid_listening_mode')
    if listening_mode == 'continuous' and require_wake_word and clean not in DIRECT_STOP_PHRASES:
        raw = re.sub(r'[\s，。！？、,.!?：:；;]', '', text).strip()
        if not any(raw.startswith(prefix) for prefix in WAKE_WORDS):
            raise ValueError('wake_word_required')
    return text


class IntentParser:
    """A small explicit command grammar, not unrestricted speech understanding."""
    def __init__(self, buildings, ttl_s=5.0):
        self.by_number = {int(b['number']): str(b['building_id']) for b in buildings}
        self.ttl_s = float(ttl_s)
        if not 0 < self.ttl_s <= 30:
            raise ValueError('ttl_out_of_range')

    def parse(self, text, stamp, source='voice'):
        clean = normalize_voice_text(text)
        if not math.isfinite(float(stamp)):
            raise ValueError('invalid_timestamp')
        if not clean:
            raise ValueError('empty_text')
        # A negative, hypothetical, conditional or question must never move the car.
        if any(word in clean for word in ('不', '别', '取消', '如果', '假如', '能否', '能不能',
                                          '是不是', '是否', '可能', '或者', '还是', '吗', '么')):
            raise ValueError('negated_or_ambiguous_command')
        action = None
        building_id = None
        # Explicit emergency stop overrides another affirmative command in one utterance.
        if any(word in clean for word in ('紧急停车', '立即停车', '紧急停止', '立刻停止')):
            action = 'stop'
        elif clean in ('停车', '停止', '停下', '停止行驶', '停止导航', '立即停止', '停'):
            action = 'stop'
        elif clean in ('报告场景', '报告当前场景', '播报场景', '播报当前场景', '汇报当前场景'):
            action = 'report'
        elif clean in ('观察', '开始观察', '观察当前场景', '检查当前场景'):
            action = 'inspect'
        elif clean in ('继续', '继续行驶', '恢复行驶', '继续导航'):
            action = 'resume'
        else:
            match = re.fullmatch(r'(?:去|前往|到|导航到)([零一二两三四五六七八九十\d]+)(?:号楼|栋|号栋)(?:观察|检查|巡检)?', clean)
            if match:
                raw = match.group(1)
                number = int(raw) if raw.isascii() and raw.isdigit() else CHINESE_NUMBERS.get(raw)
                if number is None or number not in self.by_number:
                    raise ValueError('unknown_building')
                action, building_id = 'navigate', self.by_number[number]
        if action is None:
            raise ValueError('unsupported_or_multiple_commands')
        command = {'command_id': str(uuid.uuid4()), 'source': source, 'action': action,
                   'text': text, 'issued_at': float(stamp), 'capture_stamp': float(stamp), 'ttl_s': self.ttl_s}
        if building_id is not None:
            command['building_id'] = building_id
            command['purpose'] = 'inspect'
        return command


def asr_text(result, minimum_confidence=0.65):
    """Use the actual decoder's word confidences; empty/uncertain audio is rejected."""
    if isinstance(result, str):
        result = json.loads(result)
    words = result.get('result') or []
    if not words or not result.get('text', '').strip():
        raise ValueError('no_speech')
    confidences = [float(w.get('conf', 0)) for w in words]
    if not all(math.isfinite(c) and 0 <= c <= 1 for c in confidences):
        raise ValueError('invalid_asr_confidence')
    confidence = sum(confidences) / len(confidences)
    if confidence < minimum_confidence:
        raise ValueError('low_asr_confidence')
    return result['text'], confidence


def _count(value):
    return value if type(value) is int and 0 <= value <= 100 else None


def scene_to_speech(report, now, ttl_s=3.0):
    """Never convert visible persons into confirmed casualties or stale evidence."""
    if not isinstance(report, dict):
        return '当前场景信息不可用，请停车重新观察。'
    stamp = report.get('stamp')
    if (not isinstance(stamp, (int, float)) or not math.isfinite(stamp)
            or not 0 <= now - stamp <= ttl_s or report.get('stale')
            or report.get('reason') == 'expired_observation'):
        return '当前画面信息已过期或尚未获得，请重新观察。'
    if report.get('detector_available') is False:
        return '当前目标识别不可用，无法确认火焰和人物。'
    verified = report.get('valid') is True and report.get('observation_scope') == 'current_view'
    if verified:
        number = report.get('building_number')
        name = str(report.get('building_name') or '')[:20]
        prefix = '当前观察'
        if number is not None:
            prefix += str(number) + '号楼'
        prefix += name + '。'
        fire = report.get('fire_status')
        persons = _count(report.get('visible_person_count'))
        firefighters = _count(report.get('visible_firefighter_count'))
    else:
        view = report.get('unassigned_current_view')
        if not isinstance(view, dict):
            return '尚未获得可靠的楼栋与场景信息，请继续观察。'
        prefix = '当前楼栋归属尚未确认。'
        fire = 'observed' if view.get('fire_observed') is True else ('not_observed' if view.get('fire_observed') is False else 'unknown')
        persons = _count(view.get('visible_person_count'))
        firefighters = _count(view.get('visible_firefighter_count'))
    parts = [prefix]
    parts.append({'observed': '画面中发现火焰图案。',
                  'not_observed': '本次画面未检出火焰。'}.get(fire, '火焰状态未知。'))
    if persons is not None:
        parts.append('当前看到{}名普通人物。'.format(persons))
    if firefighters is not None:
        parts.append('当前看到{}名消防员。'.format(firefighters))
    parts.append('受困人数无法仅凭图像确认。')
    return ''.join(parts)


class SpeechDeduplicator:
    def __init__(self, cooldown_s=12.0):
        self.cooldown_s = max(0.0, float(cooldown_s))
        self.last = {}

    def accept(self, text, now=None):
        now = time.monotonic() if now is None else float(now)
        key = hashlib.sha256(text.encode('utf-8')).hexdigest()
        if key in self.last and now - self.last[key] < self.cooldown_s:
            return False
        self.last = {k: t for k, t in self.last.items() if now - t < self.cooldown_s}
        self.last[key] = now
        return True
