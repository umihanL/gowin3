"""ASR evidence and independent command agreement; no hardware or model I/O."""
from .voice import asr_text, prepare_spoken_command
from .task_interaction import parse_task_text


RISKY_ACTIONS = {'manual_move', 'navigate', 'patrol', 'traffic_observe', 'auto_fire', 'start_spray', 'select_fire',
                 'route_turn', 'seek_firefighter', 'traffic_task', 'traffic_drive', 'set_reference','set_zebra_detection',
                 'navigate_point', 'confirm_point_start', 'search_region', 'extinguish_selected', 'manual_pump_test'}
VERIFIED_NAVIGATION_ACTIONS = frozenset(('navigate_point', 'search_region', 'extinguish_selected', 'manual_pump_test'))


def recognized_text(result, minimum_confidence=.65):
    if result.get('confidence_available') is not False:
        return asr_text(result, minimum_confidence)
    # No guessed score or fabricated word probabilities. Offline text requires
    # explicit speech evidence from the mandatory Silero VAD backend.
    if (result.get('asr_backend') != 'sensevoice' or result.get('speech_guard') != 'silero_vad'
            or result.get('speech_verified') is not True or result.get('vad_speech_detected') is not True):
        raise ValueError('voice_speech_not_verified')
    text = result.get('text')
    if not isinstance(text, str) or not text.strip():
        raise ValueError('no_speech')
    return text.strip(), None


def canonical_command(text, buildings, listening_mode='push_to_talk', require_wake_word=True):
    text = prepare_spoken_command(text, listening_mode, require_wake_word)
    parsed = parse_task_text(text, buildings)
    return {key: value for key, value in parsed.items() if key != 'text'}


def needs_independent_verification(text, result, buildings, listening_mode='push_to_talk', require_wake_word=True):
    # Region water authority and numbered routes require same-PCM agreement
    # even when the primary decoder supplied a high confidence score.
    if result.get('confidence_available') is not False:
        try:
            command = canonical_command(text, buildings, listening_mode, require_wake_word)
        except (ValueError, TypeError):
            # Classification must not discard an entire scored utterance;
            # accept_text owns its normal rule rejection and transcript receipt.
            return False
        return command.get('action') in VERIFIED_NAVIGATION_ACTIONS
    command = canonical_command(text, buildings, listening_mode, require_wake_word)
    return (command.get('action') in RISKY_ACTIONS or
            command.get('action') == 'set_arrival_policy' and command.get('arrival_policy') == 'auto')


def verify_command(primary_text, secondary_result, buildings, minimum_confidence=.65,
                   listening_mode='push_to_talk', require_wake_word=True):
    evidence = dict(required=True, verified=False, engine='vosk-small-cn-0.22',
                    text=str(secondary_result.get('text', ''))[:200], confidence=None)
    try:
        if secondary_result.get('confidence_available') is False:
            raise ValueError('independent_confidence_unavailable')
        text, confidence = asr_text(secondary_result, max(.65, minimum_confidence))
        evidence.update(text=text, confidence=confidence)
        primary = canonical_command(primary_text, buildings, listening_mode, require_wake_word)
        secondary = canonical_command(text, buildings, listening_mode, require_wake_word)
        if primary != secondary:
            raise ValueError('canonical_command_mismatch')
        evidence.update(verified=True, reason='')
    except (TypeError, ValueError) as exc:
        evidence.update(reason='voice_command_disagreement', detail=str(exc))
    return evidence
