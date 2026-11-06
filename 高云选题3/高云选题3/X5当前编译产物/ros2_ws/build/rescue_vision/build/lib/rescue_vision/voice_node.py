"""Offline ASR (push-to-talk or wake-prefixed continuous) and queued TTS."""
import json
from pathlib import Path
import queue
import shutil
import subprocess
import tempfile
import threading
import time
import uuid

from .node_common import Node
from std_msgs.msg import Bool, String
from .node_common import param, read_json, publish_json, spin
from .voice import IntentParser, scene_to_speech, SpeechDeduplicator, prepare_spoken_command
from .audio_pcm import PCM16To16k
from .audio_test import AudioTestRequestGate, PCMLevelMeter, diagnostic_transcript
from .voice_capture import StreamingASR, capture_duration
from .voice_asr_process import load_isolated_asr_backend as load_asr_backend
from .voice_verification import recognized_text, needs_independent_verification, verify_command, VERIFIED_NAVIGATION_ACTIONS
from .extension_control import dialogue_allowed
from .voice_tts_process import MAX_SPOKEN_TEXT_CHARS, speech_chunks
from .voice_runtime import load_voice_runtime, save_voice_runtime


class Voice(Node):
    def __init__(self):
        super().__init__('voice')
        self.workspace = Path(param(self, 'workspace_dir', '.')).resolve()
        path = param(self, 'voice_config', str(self.workspace / 'config/voice.json'))
        self.config = read_json(path)
        self.voice_runtime_path = self.workspace / 'config/voice_runtime.local.json'
        self.voice_enabled, self.voice_runtime_error = load_voice_runtime(self.voice_runtime_path)
        self.voice_requested = self.voice_enabled
        self.voice_generation = 0
        self.voice_control_lock = threading.RLock()
        self.voice_control_thread = None
        self.voice_control_state = dict(request_id='', enabled=self.voice_enabled,
            status='rejected' if self.voice_runtime_error else 'applied')
        if self.voice_runtime_error:
            self.voice_control_state['reason'] = self.voice_runtime_error
        self.audio_stop = threading.Event()
        if not self.voice_enabled:
            self.audio_stop.set()
        self.listening_mode = self.config.get('listening_mode', 'push_to_talk')
        if self.listening_mode not in ('push_to_talk', 'continuous'):
            raise ValueError('listening_mode must be push_to_talk or continuous')
        self.listen_duration_s = capture_duration(self.config, self.listening_mode)
        buildings = read_json(str(self.workspace / 'config/buildings.json'))
        self.task_commands = bool(param(self, 'task_commands', False))
        self.extension_enabled=bool(param(self,'extension_enabled',False))
        self.extension={}
        self.buildings = buildings
        self.parser = IntentParser(buildings['buildings'], self.config.get('command_ttl_s', 5))
        self.command_pub = self.create_publisher(String, '/interaction/command', 10)
        self.status_pub = self.create_publisher(String, '/voice/status', 10)
        self.transcript_pub = self.create_publisher(String, '/voice/transcript', 10)
        self.audio_test_pub = self.create_publisher(String, '/audio/test/status', 10)
        self.create_subscription(String, '/voice/text', self.text_input, 10)
        self.create_subscription(String, '/voice/control', self.voice_control, 10)
        self.create_subscription(Bool, '/audio/listen', self.listen, 10)
        self.create_subscription(String, '/audio/test/request', self.audio_test_request, 10)
        self.create_subscription(String, '/speech/say', self.say, 10)
        self.create_subscription(String, '/scene/report', self.scene, 10)
        if self.extension_enabled:
            self.create_subscription(String,'/extension/state',self.extension_state,10)
        self.stop_event = threading.Event()
        self.end_listen = threading.Event()
        self.abort_listen = threading.Event()
        self.speaking = threading.Event()
        self.listen_lock = threading.Lock()
        self.listening = False
        self.capture_active = False
        self.stop_requested_at = None
        self.stop_request_seq = 0
        self.stop_request_accepted = None
        self.stop_request_reason = None
        self.capture_stop_confirmed = True  # No microphone has been opened.
        self.recognition_phase = 'idle'
        self.last_recognition = None
        self.capture_metrics = {}
        self.capture_started_monotonic = None
        self.voice_status_published_at = 0.
        self.capture_mode = 'idle'
        self.audio_test_session = None
        self.audio_test_stop_request_id = None
        self.audio_test_last = None
        self.audio_test_published_at = 0.
        self.audio_test_gate = AudioTestRequestGate()
        self.continuous_paused = False
        self.next_listen_time = 0.
        self.mic_thread = None
        self.asr_backend = None
        self.verification_backend = None
        self.asr_load_error = ''
        # Keep a timed-out native decoder owned until it really returns;
        # cancelling a Python event cannot interrupt a native ASR call.
        self.asr_decoder = None
        self.latest_scene = None
        self.dedup = SpeechDeduplicator(self.config.get('speech_cooldown_s', 12))
        self.speech_queue = queue.Queue(maxsize=max(1, min(10, int(self.config.get('speech_queue_limit', 3)))))
        self.tts_process = None
        self.neural_tts = None
        self.speech_diagnostics = dict(tts_state='idle', tts_error='',
            tts_audio_device=self.config.get('tts_audio_device'), tts_device_ready=None,
            tts_last_result=None)
        self.tts_thread = None
        if self.voice_enabled and self.config.get('tts_enabled'):
            self.start_speech_worker()
        self.create_timer(5.0, self.heartbeat)
        self.create_timer(.25, self.continuous_tick)
        self.status('ready', microphone_enabled=bool(self.config.get('microphone_enabled')),
                    tts_enabled=bool(self.config.get('tts_enabled')), listening_mode=self.listening_mode,
                    capture_active=False,
                    note=('点击开始收音后等待“正在收音”再说话；默认收音15秒，可随时点击结束。待机不打开麦克风。'
                          if self.listening_mode == 'push_to_talk' and self.config.get('microphone_enabled')
                          else '音频开关与收音模式由voice_config决定。'))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def audio_stopped(self):
        return self.stop_event.is_set() or self.audio_stop.is_set() or not self.voice_enabled

    def microphone_enabled(self):
        return self.voice_enabled and bool(self.config.get('microphone_enabled'))

    def tts_enabled(self):
        return self.voice_enabled and bool(self.config.get('tts_enabled'))

    def start_speech_worker(self):
        self.tts_thread = threading.Thread(target=self.speech_worker, name='voice-speech', daemon=True)
        self.tts_thread.start()

    def voice_control(self, msg):
        request = None
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict) or type(request.get('enabled')) is not bool:
                raise ValueError('voice_control_enabled_must_be_boolean')
            rid = request.get('request_id')
            if not isinstance(rid, str) or not 1 <= len(rid) <= 128 or not rid.strip():
                raise ValueError('voice_control_request_id_required')
            with self.voice_control_lock:
                if self.stop_event.is_set():
                    return
                if rid == self.voice_control_state.get('request_id'):
                    if request['enabled'] != self.voice_control_state.get('enabled'):
                        raise ValueError('voice_control_request_id_conflict')
                    self.status('voice_control')
                    return
                self.voice_requested = request['enabled']
                self.voice_control_state = dict(request_id=rid, enabled=request['enabled'], status='pending')
                if not request['enabled']:
                    # Fence publication immediately; model/process cleanup is asynchronous.
                    self.voice_enabled = False
                    self.voice_generation += 1
                    self.audio_stop.set()
                    self.abort_listen.set()
                    self.end_listen.set()
                    self.continuous_paused = True
                if self.voice_control_thread is None or not self.voice_control_thread.is_alive():
                    self.voice_control_thread = threading.Thread(target=self.apply_voice_control,
                        name='voice-control', daemon=True)
                    self.voice_control_thread.start()
            self.status('voice_control')
        except (ValueError, TypeError) as exc:
            rejected = dict(request_id=request.get('request_id', '') if isinstance(request, dict) else '',
                enabled=request.get('enabled') if isinstance(request, dict) else None,
                status='rejected', reason=str(exc))
            self.status('voice_control_rejected', voice_control=rejected)

    def close_audio_resources(self):
        """Stop a single generation off the ROS executor; never reopen hardware."""
        self.audio_stop.set()
        self.abort_listen.set()
        self.end_listen.set()
        process = self.tts_process
        if process is not None and process.poll() is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass
        decoder = self.asr_decoder
        if decoder is not None:
            decoder.cancel()
        for backend in (self.asr_backend, self.verification_backend, self.neural_tts):
            close = getattr(backend, 'close', None)
            if close is not None:
                close()
        # Workers own stream closure and final status. Do not clear their events
        # or enable a new generation while an old worker can still publish.
        for worker in (self.mic_thread, self.tts_thread):
            while worker is not None and worker.is_alive():
                worker.join(timeout=.1)
                if self.stop_event.is_set():
                    return False
        # Model loading can complete just after the first close pass.
        for backend in (self.asr_backend, self.verification_backend, self.neural_tts):
            close = getattr(backend, 'close', None)
            if close is not None:
                close()
        self.asr_backend = self.verification_backend = self.neural_tts = self.asr_decoder = None
        self.mic_thread = self.tts_thread = self.tts_process = None
        self.speaking.clear()
        self.capture_stop_confirmed = True
        while True:
            try:
                self.speech_queue.get_nowait()
            except queue.Empty:
                break
        return True

    def apply_voice_control(self):
        while not self.stop_event.is_set():
            with self.voice_control_lock:
                request = dict(self.voice_control_state)
                requested = self.voice_requested
            try:
                persistence_error = None
                try:
                    save_voice_runtime(self.voice_runtime_path, requested)
                except Exception as exc:
                    persistence_error = exc
                if self.audio_stop.is_set() or not requested:
                    if not self.close_audio_resources():
                        return
                if persistence_error is not None:
                    raise persistence_error
                with self.voice_control_lock:
                    if self.stop_event.is_set():
                        return
                    if request['request_id'] != self.voice_control_state['request_id']:
                        continue
                    self.voice_runtime_error = ''
                    if requested and not self.voice_enabled:
                        self.audio_stop.clear()
                        self.abort_listen.clear()
                        self.end_listen.clear()
                        self.voice_enabled = True
                        # Restoring the switch is not a new recording request,
                        # including for the optional continuous profile.
                        self.continuous_paused = True
                        self.speech_diagnostics.update(tts_state='idle', tts_error='', tts_device_ready=None)
                        if self.config.get('tts_enabled'):
                            self.start_speech_worker()
                    elif not requested:
                        self.speech_diagnostics.update(tts_state='disabled', tts_error='', tts_device_ready=None)
                    self.voice_control_state = dict(request, status='applied')
                    # Clear while locked, so a new request always starts a worker.
                    self.voice_control_thread = None
                self.status('ready' if self.voice_enabled else 'voice_disabled')
                return
            except Exception as exc:
                with self.voice_control_lock:
                    if request['request_id'] != self.voice_control_state['request_id']:
                        continue
                    self.voice_runtime_error = 'voice_control_failed: ' + str(exc)
                    self.voice_control_state = dict(request, status='rejected', reason=self.voice_runtime_error)
                    self.voice_control_thread = None
                self.status('voice_control_rejected')
                return

    def dialogue_enabled(self):
        return not self.extension_enabled or dialogue_allowed(self.extension,self.now())

    def command_capture_allowed(self):
        # The push-to-talk button must be usable to say "start task 1/2".
        # Parsed commands are still checked individually before publication.
        return self.dialogue_enabled() or (self.task_commands and self.listening_mode=='push_to_talk')

    def extension_state(self,msg):
        previous_task=self.extension.get('task')
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('expected_object')
            self.extension=value
        except (TypeError,ValueError):self.extension={}
        changed_to_closed=(previous_task!=self.extension.get('task') and not self.dialogue_enabled())
        if (not self.command_capture_allowed() or changed_to_closed) and self.capture_mode=='command':
            self.abort_listen.set();self.end_listen.set()

    def backend_status(self):
        backend = self.asr_backend
        return dict(asr_backend=getattr(backend, 'asr_backend', self.config.get('asr_backend', 'vosk')),
            asr_engine=getattr(backend, 'engine', None), asr_ready=backend is not None,
            confidence_available=getattr(backend, 'confidence_available', None),
            asr_process_isolated=getattr(backend, 'process_isolated', False),
            asr_worker_ready=getattr(backend, 'ready', backend is not None),
            asr_load_error=self.asr_load_error,
            asr_fallback_from=getattr(backend, 'fallback_from', None),
            asr_fallback_reason=getattr(backend, 'fallback_reason', None))

    def preparation_cancelled(self, deadline=None, allow_end=True):
        if deadline is not None and time.monotonic() > deadline:
            raise RuntimeError('transcription_timeout')
        return (self.audio_stopped() or self.abort_listen.is_set() or
                (allow_end and self.end_listen.is_set() and
                 self.recognition_phase in ('preparing', 'loading_asr')))

    def set_preparation_budget(self, backend, deadline=None, allow_end=True):
        if getattr(backend, 'process_isolated', False):
            # Cached models must receive this capture's deadline, never retain
            # an expired deadline from a previous successful utterance.
            backend.preparation_cancelled = lambda: self.preparation_cancelled(deadline, allow_end)

    def load_backend(self):
        if self.audio_stopped():
            raise RuntimeError('voice_disabled')
        if self.asr_backend is None:
            self.status('loading_asr')
            try:
                self.asr_backend = load_asr_backend(self.config, self.workspace,
                    preparation_cancelled=self.preparation_cancelled)
                self.asr_load_error = ''
            except Exception as exc:
                self.asr_load_error = str(exc)
                raise
        self.set_preparation_budget(self.asr_backend)
        return self.asr_backend

    def wait_decoder(self, decoder, deadline):
        while not decoder.done.wait(.05):
            if self.abort_listen.is_set() or self.stop_event.is_set():
                raise RuntimeError('playback_or_shutdown_during_transcription')
            if time.monotonic() > deadline:
                raise RuntimeError('transcription_timeout')
        if self.abort_listen.is_set() or self.stop_event.is_set():
            raise RuntimeError('playback_or_shutdown_during_transcription')
        if time.monotonic() > deadline:
            raise RuntimeError('transcription_timeout')
        if decoder.error is not None:
            raise decoder.error

    def verify_voice_action(self, text, result, captured, deadline):
        if result.get('confidence_available') is False and not self.task_commands:
            # The legacy parser has broader action aliases (e.g. “继续” means
            # resume there, but dialogue in the task grammar). Never decide a
            # safety gate with a different parser from the eventual publisher.
            legacy = self.parser.parse(text, self.now(), 'voice')
            from .task_interaction import parse_task_text
            precise = parse_task_text(text, self.buildings)
            if legacy.get('action') not in ('stop','report','inspect') or legacy.get('action') != precise.get('action'):
                raise ValueError('unscored_asr_requires_task_commands')
        args = (self.buildings, self.listening_mode, self.config.get('require_wake_word', True))
        if not needs_independent_verification(text, result, *args):
            return dict(required=False, verified=False)
        self.status('transcribing', verification_phase='independent_command_check')
        if self.verification_backend is None:
            self.verification_backend = load_asr_backend(dict(self.config, asr_backend='vosk'), self.workspace,
                preparation_cancelled=lambda: self.preparation_cancelled(deadline, allow_end=False))
        self.set_preparation_budget(self.verification_backend, deadline, allow_end=False)
        if time.monotonic() > deadline:
            raise RuntimeError('transcription_timeout')
        recognizer = self.verification_backend.create_recognizer(16000)
        recognizer.SetWords(True)
        # Whole-utterance mode: a stop embedded in a second interpretation must
        # not shortcut the final result or hide a conflicting trailing clause.
        decoder = StreamingASR(recognizer, 'verification', .65)
        self.asr_decoder = decoder
        for pcm in captured:
            decoder.submit(pcm)
        decoder.finish(); decoder.start()
        try:
            self.wait_decoder(decoder, deadline)
            return verify_command(text, decoder.combined(), self.buildings,
                self.config.get('min_asr_confidence', .65), self.listening_mode,
                self.config.get('require_wake_word', True))
        finally:
            decoder.cancel()
            if decoder.done.is_set() and self.asr_decoder is decoder:
                self.asr_decoder = None

    def status(self, state, **details):
        if hasattr(self, 'stop_event') and self.stop_event.is_set():
            return
        # A delayed worker/heartbeat must not turn a received stop request back
        # into "recording". Only the capture owner can confirm device closure.
        if (state == 'listening' and self.capture_mode == 'command'
                and self.capture_active and self.end_listen.is_set()):
            state = 'stopping'
        if state in ('preparing', 'loading_asr', 'listening', 'stopping', 'transcribing'):
            self.recognition_phase = state
        metrics = dict(self.capture_metrics)
        if self.capture_mode == 'command' and self.capture_active and self.capture_started_monotonic is not None:
            metrics['elapsed_s'] = min(metrics['duration_s'], max(0., time.monotonic() - self.capture_started_monotonic))
            metrics['remaining_s'] = max(0., metrics['duration_s'] - metrics['elapsed_s'])
        payload = dict(stamp=self.now(), state=state,
                       dialogue_enabled=self.dialogue_enabled(),
                       microphone_enabled=bool(self.config.get('microphone_enabled')),
                       tts_enabled=bool(self.config.get('tts_enabled')),
                       listening=bool(self.listening), capture_active=bool(self.capture_active),
                       stop_requested=self.stop_request_accepted is True,
                       stop_requested_at=self.stop_requested_at,
                       stop_request_seq=self.stop_request_seq,
                       stop_request_accepted=self.stop_request_accepted,
                       stop_request_reason=self.stop_request_reason,
                       capture_stop_confirmed=self.capture_stop_confirmed,
                       listening_mode=self.listening_mode,
                       recognition_phase=self.recognition_phase,
                       capture_mode=self.capture_mode,
                       audio_test_active=bool(self.listening and self.capture_mode == 'audio_test'),
                       last_recognition=(dict(self.last_recognition) if self.last_recognition else None))
        payload.update(metrics)
        payload.update(self.backend_status())
        payload.update(getattr(self, 'speech_diagnostics', {}))
        payload.update(details)
        # Call-site details/configuration never override effective switch state.
        payload.update(voice_enabled=self.voice_enabled,
            voice_control=dict(details.get('voice_control', self.voice_control_state)),
            voice_runtime_error=self.voice_runtime_error,
            microphone_enabled=self.microphone_enabled(), tts_enabled=self.tts_enabled())
        if not self.voice_enabled and state in ('idle', 'ready', 'listening', 'preparing',
                'loading_asr', 'transcribing', 'speaking', 'synthesizing', 'tts_ready', 'tts_preparing'):
            payload['state'] = 'voice_disabled'
        self.voice_status_published_at = time.monotonic()
        publish_json(self.status_pub, payload)

    def heartbeat(self):
        speech_state = getattr(self, 'speech_diagnostics', {}).get('tts_state')
        state = self.recognition_phase if self.listening else (
            'synthesizing' if self.speaking.is_set() and speech_state == 'synthesizing' else
            'speaking' if self.speaking.is_set() else
            'tts_unavailable' if speech_state == 'unavailable' else 'idle')
        self.status(state, capture_active=self.capture_active,
                    microphone_enabled=bool(self.config.get('microphone_enabled')),
                    tts_enabled=bool(self.config.get('tts_enabled')),
                    listening_mode=self.listening_mode, continuous_paused=self.continuous_paused)
        self.audio_test_heartbeat()

    def accept_text(self, text, source, stamp, verification=None, voice_generation=None):
        try:
            submitted_at = self.now()
            if source == 'voice':
                if self.audio_stopped() or voice_generation != self.voice_generation:
                    raise ValueError('voice_disabled_or_capture_cancelled')
                if not 0 <= submitted_at - stamp <= 30.:
                    raise ValueError('transcription_expired')
                text = prepare_spoken_command(text, self.listening_mode,
                                              self.config.get('require_wake_word', True))
            if self.task_commands:
                from .task_interaction import parse_task_text
                command = parse_task_text(text, self.buildings)
                if command.get('action') in VERIFIED_NAVIGATION_ACTIONS:
                    if source == 'voice' and (not isinstance(verification, dict) or verification.get('verified') is not True):
                        raise ValueError({'search_region':'region_search_independent_asr_required',
                            'manual_pump_test':'manual_pump_independent_asr_required',
                            'extinguish_selected':'selected_fire_round_independent_asr_required'}.get(
                            command['action'], 'numbered_navigation_independent_asr_required'))
                    if source == 'voice':
                        command['voice_verification'] = verification
                command.update(command_id=str(uuid.uuid4()), source=source, issued_at=submitted_at,
                               capture_stamp=stamp, ttl_s=self.config.get('command_ttl_s',5.))
                if command['action'] == 'manual_pump_test':
                    command['ttl_s'] = min(command['ttl_s'], 2.)
            else:
                command = self.parser.parse(text, submitted_at, source)
                command['capture_stamp'] = stamp
            if not self.dialogue_enabled() and command.get('action') not in (
                    'stop','stop_spray','finish_task','extension_vision','extension_dialogue'):
                raise ValueError('select_observation_task_for_voice_and_dialogue')
            # Disable and voice-command publication share one short critical
            # section. A late ASR result cannot escape after OFF is received.
            with self.voice_control_lock:
                if source == 'voice' and (self.audio_stopped() or self.abort_listen.is_set()
                        or voice_generation != self.voice_generation):
                    raise ValueError('voice_disabled_or_capture_cancelled')
                publish_json(self.command_pub, command)
            # A submitted command is not a completed navigation action.
            outcome = dict(state='command_submitted', command_id=command['command_id'], action=command['action'])
            self.status(**outcome)
            return outcome
        except (ValueError, TypeError) as exc:
            outcome = dict(state='command_rejected', reason=str(exc), text=str(text)[:200])
            self.status(**outcome)
            return outcome

    def text_input(self, msg):
        if self.voice_enabled and self.capture_mode == 'audio_test' and self.listening:
            self.status('command_rejected', reason='audio_test_active')
            return
        self.accept_text(msg.data, 'text', self.now())

    def continuous_tick(self):
        self.audio_test_heartbeat()
        if not self.command_capture_allowed() and self.capture_mode=='command' and self.listening:
            self.abort_listen.set();self.end_listen.set()
        if (self.capture_mode == 'command' and self.listening and not self.stop_event.is_set()
                and time.monotonic() - self.voice_status_published_at >= (.25 if self.capture_active else 1.)):
            self.status(self.recognition_phase)
        if (self.listening_mode == 'continuous' and not self.continuous_paused
                and self.dialogue_enabled()
                and self.microphone_enabled() and not self.audio_stopped()
                and not self.listening and not self.speaking.is_set()
                and time.monotonic() >= self.next_listen_time):
            self.listen(Bool(data=True))

    def listen(self, msg):
        if not msg.data:
            with self.listen_lock:
                # A monotonically increasing receipt also acknowledges repeated
                # stops while ASR finishes, without cancelling the transcript.
                self.stop_request_seq += 1
                self.stop_requested_at = self.now()
                if self.capture_mode == 'audio_test':
                    self.stop_request_accepted = False
                    self.stop_request_reason = 'audio_test_requires_own_stop'
                    self.status(self.recognition_phase)
                    return
                self.stop_request_accepted = True
                self.stop_request_reason = None
                self.continuous_paused = True
                self.end_listen.set()
                phase = self.recognition_phase if self.listening else 'idle'
                self.status(phase)
            return
        if not self.command_capture_allowed():
            self.status('listen_refused_task_mode',reason='select_observation_task_for_voice_and_dialogue')
            return
        if not self.microphone_enabled():
            self.status('microphone_disabled')
            return
        if self.stop_event.is_set():
            return
        with self.voice_control_lock, self.listen_lock:
            if self.audio_stopped():
                self.status('voice_disabled')
                return
            if self.speaking.is_set():
                self.status('listen_refused_during_speech')
                return
            if self.listening:
                return
            if self.asr_decoder is not None and not self.asr_decoder.done.is_set():
                self.status('asr_unavailable_or_rejected', reason='asr_decoder_still_stopping')
                return
            self.asr_decoder = None
            self.continuous_paused = False
            self.listening = True
            self.capture_mode = 'command'
            self.stop_requested_at = None
            self.stop_request_accepted = None
            self.stop_request_reason = None
            self.capture_stop_confirmed = False
            duration = self.listen_duration_s
            self.capture_started_monotonic = None
            self.capture_metrics = dict(elapsed_s=0., duration_s=duration, remaining_s=duration,
                capture_started_at=None, capture_ended_at=None, sampled_s=0.,
                level_dbfs=None, peak_dbfs=None, clipped_fraction=None, level_stamp=None,
                input_device=self.config.get('audio_input_device'),
                capture_sample_rate_hz=self.config.get('capture_sample_rate_hz', 16000), asr_sample_rate_hz=16000)
            self.end_listen.clear()
            self.abort_listen.clear()
            self.status('preparing', capture_active=False,
                        note='正在准备识别模型；正在收音后可说完整问题，默认15秒，也可点击结束。')
            self.mic_thread = threading.Thread(target=self.recognition_worker,
                args=(self.voice_generation,), daemon=True)
            self.mic_thread.start()

    def recognition_worker(self, voice_generation):
        retry_delay = .2
        outcome = dict(state='listening_cancelled', reason='capture_cancelled_during_preparation')
        decoder = None
        sampled = 0
        capture_closed_monotonic = None
        level = PCMLevelMeter()
        total_level = PCMLevelMeter()
        captured = []
        level_stamp = None
        stream_open_attempted = False
        try:
            import sounddevice as sd
            # A stop received before this worker ran must not even begin a
            # potentially expensive model load. Stops during loading still get
            # an immediate acknowledgement from listen(False).
            if self.stop_event.is_set() or self.end_listen.is_set() or self.abort_listen.is_set():
                return
            backend = self.load_backend()
            if self.stop_event.is_set() or self.end_listen.is_set() or self.abort_listen.is_set():
                return
            sample_rate = int(self.config.get('sample_rate_hz', 16000))
            if sample_rate != 16000:
                raise ValueError('This Chinese model requires 16000 Hz; resample externally if necessary')
            capture_rate = self.config.get('capture_sample_rate_hz', sample_rate)
            duration = self.capture_metrics['duration_s']
            sample_limit = round(sample_rate * duration)
            converter = PCM16To16k(capture_rate)
            recognizer = backend.create_recognizer(sample_rate)
            recognizer.SetWords(True)
            decoder = StreamingASR(recognizer, self.listening_mode, self.config.get('min_asr_confidence', .65))
            chunks = queue.Queue(maxsize=64)
            overrun = threading.Event()
            def audio_callback(indata, frames, timing, status):
                if status:
                    overrun.set()
                try:
                    chunks.put_nowait((bytes(indata), self.now()))
                except queue.Full:
                    overrun.set()
            def consume(block):
                nonlocal sampled, level_stamp
                raw, stamp = block
                pcm = converter.convert(raw)[:max(0, sample_limit - sampled) * 2]
                sampled += len(pcm) // 2
                if pcm:
                    level.add(pcm)
                    total_level.add(pcm)
                    captured.append(pcm)
                    level_stamp = stamp
                    try:
                        decoder.submit(pcm)
                    except queue.Full:
                        overrun.set()
            def update_metrics(reset=False):
                details = level.snapshot(reset=reset)
                if details['level_samples']:
                    details['level_stamp'] = level_stamp
                else:
                    details = {}
                clock = capture_closed_monotonic if capture_closed_monotonic is not None else time.monotonic()
                elapsed = (min(duration, max(0., clock - self.capture_started_monotonic))
                           if self.capture_started_monotonic is not None else 0.)
                self.capture_metrics = dict(self.capture_metrics, elapsed_s=elapsed,
                    remaining_s=max(0., duration - elapsed), sampled_s=sampled / sample_rate,
                    recording_levels=total_level.snapshot(), **details)
            # Model/filter/recognizer preparation must not open a cancelled capture.
            if self.stop_event.is_set() or self.end_listen.is_set() or self.abort_listen.is_set():
                return
            stream_open_attempted = True
            with sd.RawInputStream(samplerate=capture_rate, blocksize=capture_rate // 10, dtype='int16',
                    channels=1, device=self.config.get('audio_input_device'), callback=audio_callback):
                self.asr_decoder = decoder
                decoder.start()
                self.capture_started_monotonic = time.monotonic()
                self.capture_metrics = dict(self.capture_metrics, capture_started_at=self.now())
                last_level = self.capture_started_monotonic
                self.capture_active = True
                self.status('listening', capture_active=True)
                # The capture owner never runs native ASR. Slow decoding cannot extend this
                # monotonic microphone deadline or delay the stop button.
                while (not self.stop_event.is_set() and not self.end_listen.is_set()
                       and not self.abort_listen.is_set() and not overrun.is_set()
                       and not decoder.endpoint.is_set() and not decoder.done.is_set()
                       and time.monotonic() - self.capture_started_monotonic < duration and sampled < sample_limit):
                    try:
                        consume(chunks.get(timeout=.05))
                    except queue.Empty:
                        pass
                    now = time.monotonic()
                    if now - last_level >= .25:
                        last_level = now
                        update_metrics(reset=True)
                        self.status('listening', capture_active=True)
            self.capture_active = False
            self.capture_stop_confirmed = True
            capture_closed_monotonic = time.monotonic()
            capture_stamp = self.now()
            update_metrics()
            self.capture_metrics = dict(self.capture_metrics, capture_ended_at=capture_stamp,
                ended_by=('emergency_stop' if decoder.emergency_result else 'endpoint' if decoder.endpoint.is_set()
                          else 'manual_stop' if self.end_listen.is_set() else 'duration'))
            if self.abort_listen.is_set() or self.stop_event.is_set() or overrun.is_set():
                outcome = dict(state='audio_discarded', reason='playback_shutdown_or_audio_overrun')
                return
            # Retain every actual captured block, including trailing speech
            # received just before a user stop or the fixed deadline.
            while not chunks.empty():
                consume(chunks.get_nowait())
            update_metrics()
            if overrun.is_set():
                outcome = dict(state='audio_discarded', reason='playback_shutdown_or_audio_overrun')
                return
            decoder.finish()
            self.status('transcribing', capture_active=False)
            transcription_deadline = capture_closed_monotonic + 30.
            while not decoder.done.wait(.05):
                if self.abort_listen.is_set() or self.stop_event.is_set():
                    outcome = dict(state='audio_discarded', reason='playback_or_shutdown_during_transcription')
                    return
                if time.monotonic() > transcription_deadline:
                    raise RuntimeError('transcription_timeout')
            if self.abort_listen.is_set() or self.stop_event.is_set():
                outcome = dict(state='audio_discarded', reason='playback_or_shutdown_during_transcription')
                return
            if time.monotonic() > transcription_deadline:
                raise RuntimeError('transcription_timeout')
            if decoder.error is not None:
                raise decoder.error
            combined = decoder.emergency_result or decoder.combined()
            text, confidence = recognized_text(combined, self.config.get('min_asr_confidence', .65))
            metadata = {key: combined[key] for key in ('decode_s','speech_s','audio_s','vad_s','decode_audio_s','speech_verified',
                'vad_speech_detected','speech_guard','segments') if key in combined}
            self.capture_metrics.update(metadata)
            if not 0 <= self.now() - capture_stamp <= 30.:
                raise RuntimeError('transcription_expired')
            verification = dict(required=False, verified=False)
            try:
                verification = self.verify_voice_action(text, combined, captured, transcription_deadline)
                if verification.get('required') and not verification.get('verified'):
                    outcome = dict(state='command_rejected',reason='voice_command_disagreement',text=text)
                else:
                    if self.abort_listen.is_set() or self.stop_event.is_set():
                        outcome = dict(state='audio_discarded',reason='playback_or_shutdown_during_transcription')
                        return
                    if time.monotonic() > transcription_deadline or not 0<=self.now()-capture_stamp<=30.:
                        raise RuntimeError('transcription_timeout')
                    outcome = self.accept_text(text, 'voice', capture_stamp,
                        verification=verification, voice_generation=voice_generation)
            except Exception as exc:
                outcome = dict(state='command_rejected', reason=str(exc), text=text)
                verification.update(required=True,verified=False,error=str(exc))
            if self.abort_listen.is_set() or self.stop_event.is_set():
                outcome = dict(state='audio_discarded',reason='playback_or_shutdown_during_transcription')
                return
            # A transcript proves recognition, while submitted only means the
            # command was published. Router acceptance remains a separate event.
            publish_json(self.transcript_pub, dict(text=text, confidence=confidence, capture_stamp=capture_stamp,
                capture_started_at=self.capture_metrics['capture_started_at'], result_stamp=self.now(),
                engine=backend.engine, confidence_available=backend.confidence_available,
                verification=verification, audio_uploaded=False, **metadata,
                command_id=outcome.get('command_id'), action=outcome.get('action'),
                submitted=outcome.get('state') == 'command_submitted', route_error=outcome.get('reason')))
        except Exception as exc:
            outcome = dict(state='listening_cancelled' if str(exc)=='asr_preparation_cancelled'
                           else 'asr_unavailable_or_rejected', reason=str(exc))
            retry_delay = 2.
        finally:
            if decoder is not None:
                decoder.cancel()
                if decoder.done.is_set() and self.asr_decoder is decoder:
                    self.asr_decoder = None
            with self.listen_lock:
                self.listening = False
                self.capture_active = False
                if not stream_open_attempted:
                    self.capture_stop_confirmed = True
                self.recognition_phase = 'idle'
                self.capture_mode = 'idle'
                self.next_listen_time = time.monotonic() + retry_delay
                if not self.stop_event.is_set():
                    self.last_recognition = dict(outcome, stamp=self.now())
                    self.status(**outcome)

    def audio_test_status(self, state, *, session_id=None, remember=True, **details):
        if self.stop_event.is_set():
            return
        session_id = session_id if session_id is not None else self.audio_test_session
        previous = (self.audio_test_last or {}) if remember and (
            self.audio_test_last or {}).get('session_id') == session_id else {}
        stamp = self.now()
        payload = dict(stamp=stamp, session_id=session_id, command_id=session_id, state=state,
            request_id=self.audio_test_stop_request_id or session_id,
            request_action='stop' if self.audio_test_stop_request_id else 'start', request_accepted=True,
            mode='audio_test', command_published=False, audio_uploaded=False,
            active=bool(self.listening and self.capture_mode == 'audio_test'),
            active_session_id=self.audio_test_session if self.listening and self.capture_mode == 'audio_test' else None,
            capture_active=bool(self.capture_active and self.capture_mode == 'audio_test'),
            elapsed_s=previous.get('elapsed_s', 0.), duration_s=10., sampled_s=previous.get('sampled_s', 0.),
            level_dbfs=previous.get('level_dbfs'), peak_dbfs=previous.get('peak_dbfs'),
            clipped_fraction=previous.get('clipped_fraction'), level_stamp=previous.get('level_stamp'),
            input_device=self.config.get('audio_input_device'),
            capture_sample_rate_hz=self.config.get('capture_sample_rate_hz', 16000), asr_sample_rate_hz=16000,
            result_stamp=stamp if state in ('completed', 'cancelled', 'error') else None,
            text='', confidence=None, confidence_gate_passed=False, reason='')
        payload.update(details)
        payload.update(self.backend_status())
        if remember:
            self.audio_test_last = payload
            self.audio_test_published_at = time.monotonic()
        publish_json(self.audio_test_pub, payload)

    def audio_test_heartbeat(self):
        # Model loading and final ASR run on the worker and may take seconds.
        # Keep liveness separate from immutable sample/result timestamps.
        if (self.audio_test_last is not None and not self.stop_event.is_set()
                and time.monotonic() - self.audio_test_published_at >= 1.):
            self.audio_test_published_at = time.monotonic()
            publish_json(self.audio_test_pub, dict(self.audio_test_last, stamp=self.now()))

    def audio_test_request(self, msg):
        """A fresh, deduplicated request reserves the same single microphone worker."""
        request = None
        try:
            request = json.loads(msg.data)
            with self.voice_control_lock, self.listen_lock:
                if self.stop_event.is_set():
                    return
                enabled, identifier = self.audio_test_gate.accept(request, self.now())
                if not enabled:
                    if self.listening and self.capture_mode == 'audio_test':
                        self.audio_test_stop_request_id = identifier
                        self.end_listen.set()
                        # Acknowledge this stop immediately without inventing a
                        # new recording state or changing the startup session ID.
                        current = dict(self.audio_test_last or {})
                        current.update(stamp=self.now(), request_id=identifier,
                            request_action='stop', request_accepted=True)
                        self.audio_test_last = current
                        self.audio_test_published_at = time.monotonic()
                        publish_json(self.audio_test_pub, current)
                    else:
                        raise ValueError('no_audio_test_active')
                    return
                if not self.microphone_enabled() or self.audio_stopped():
                    raise ValueError('microphone_disabled')
                if self.speaking.is_set():
                    raise ValueError('listen_refused_during_speech')
                if self.listening:
                    raise ValueError('microphone_busy')
                if self.asr_decoder is not None and not self.asr_decoder.done.is_set():
                    raise ValueError('asr_decoder_still_stopping')
                self.asr_decoder = None
                self.continuous_paused = True
                self.listening = True
                self.capture_mode = 'audio_test'
                self.stop_requested_at = None
                self.stop_request_accepted = None
                self.stop_request_reason = None
                self.capture_stop_confirmed = False
                self.audio_test_session = identifier
                self.audio_test_stop_request_id = None
                self.end_listen.clear()
                self.abort_listen.clear()
                self.status('preparing', capture_active=False)
                self.audio_test_status('preparing', session_id=identifier)
                self.mic_thread = threading.Thread(target=self.audio_test_worker, daemon=True)
                self.mic_thread.start()
        except (ValueError, TypeError) as exc:
            identifier = request.get('command_id') if isinstance(request, dict) else None
            if self.audio_test_last is not None and not self.stop_event.is_set():
                # A stale/duplicate stop must not replace the valid session's
                # progress or completed transcript with an unrelated error.
                publish_json(self.audio_test_pub, dict(self.audio_test_last, stamp=self.now(),
                    request_id=identifier, request_rejected=True, request_accepted=False,
                    request_reason=str(exc)))
            else:
                self.audio_test_status('error', session_id=identifier, remember=False, request_id=identifier,
                    reason=str(exc), request_reason=str(exc), request_rejected=True, request_accepted=False)

    def audio_test_worker(self):
        """Capture evidence only. This path never parses or publishes robot commands."""
        session_id = self.audio_test_session
        terminal, reason = 'cancelled', 'capture_cancelled_during_preparation'
        transcript = {}
        captured = []
        sampled = 0
        elapsed = 0.
        level = PCMLevelMeter()
        total_level = PCMLevelMeter()
        level_stamp = None
        decoder = None
        try:
            import sounddevice as sd
            backend = self.load_backend()
            sample_rate = int(self.config.get('sample_rate_hz', 16000))
            if sample_rate != 16000:
                raise ValueError('This Chinese model requires 16000 Hz')
            capture_rate = self.config.get('capture_sample_rate_hz', sample_rate)
            converter = PCM16To16k(capture_rate)
            if self.stop_event.is_set() or self.end_listen.is_set() or self.abort_listen.is_set():
                return
            chunks = queue.Queue(maxsize=64)
            overrun = threading.Event()
            def callback(indata, frames, timing, status):
                if status:
                    overrun.set()
                try:
                    # Timestamp belongs to an actual received PCM block;
                    # status heartbeats never manufacture a newer level.
                    chunks.put_nowait((bytes(indata), self.now()))
                except queue.Full:
                    overrun.set()
            def consume(block):
                nonlocal sampled, level_stamp
                raw, sample_stamp = block
                # Bound retained PCM to ten seconds even if the device delivers
                # one extra callback during stream shutdown.
                pcm = converter.convert(raw)[:max(0, 16000 * 10 - sampled) * 2]
                sampled += len(pcm) // 2
                captured.append(pcm)
                level.add(pcm)
                total_level.add(pcm)
                if pcm:
                    level_stamp = sample_stamp
            with sd.RawInputStream(samplerate=capture_rate, blocksize=capture_rate // 10, dtype='int16',
                    channels=1, device=self.config.get('audio_input_device'), callback=callback):
                started = time.monotonic()
                last_level = started
                self.capture_active = True
                self.status('listening', capture_active=True)
                self.audio_test_status('listening', session_id=session_id,
                    capture_sample_rate_hz=capture_rate, asr_sample_rate_hz=sample_rate)
                while (not self.stop_event.is_set() and not self.end_listen.is_set()
                       and not self.abort_listen.is_set() and not overrun.is_set()
                       and time.monotonic() - started < 10. and sampled < 16000 * 10):
                    try:
                        consume(chunks.get(timeout=.1))
                    except queue.Empty:
                        pass
                    now = time.monotonic()
                    elapsed = min(10., max(0., now - started))
                    if now - last_level >= .25:
                        last_level = now
                        self.audio_test_status('listening', session_id=session_id,
                            elapsed_s=elapsed, sampled_s=sampled / 16000,
                            recording_levels=total_level.snapshot(),
                            level_stamp=level_stamp if level.count else None, **level.snapshot(reset=True))
                elapsed = min(10., max(0., time.monotonic() - started))
            self.capture_active = False
            transcription_deadline = time.monotonic()+30.
            if self.stop_event.is_set() or self.abort_listen.is_set() or overrun.is_set():
                reason = 'audio_overrun' if overrun.is_set() else 'playback_or_shutdown'
                return
            while not chunks.empty():
                consume(chunks.get_nowait())
            self.recognition_phase = 'transcribing'
            self.status('transcribing', capture_active=False)
            details = level.snapshot()
            if not level.count:
                details = {}
            else:
                details['level_stamp'] = level_stamp
            self.audio_test_status('transcribing', session_id=session_id, elapsed_s=elapsed,
                sampled_s=sampled / 16000, recording_levels=total_level.snapshot(), **details)
            self.set_preparation_budget(backend, transcription_deadline, allow_end=False)
            recognizer = backend.create_recognizer(16000)
            recognizer.SetWords(True)
            decoder = StreamingASR(recognizer, 'diagnostic', self.config.get('min_asr_confidence', .65))
            self.asr_decoder = decoder
            for pcm in captured:
                if self.stop_event.is_set() or self.abort_listen.is_set():
                    reason = 'playback_or_shutdown_during_transcription'
                    return
                decoder.submit(pcm)
            decoder.finish(); decoder.start()
            self.wait_decoder(decoder, transcription_deadline)
            if self.stop_event.is_set() or self.abort_listen.is_set():
                reason = 'playback_or_shutdown_during_transcription'
                return
            transcript = diagnostic_transcript(decoder.results, self.config.get('min_asr_confidence', .65))
            terminal, reason = 'completed', transcript.pop('reason')
        except Exception as exc:
            terminal, reason = ('cancelled' if self.abort_listen.is_set() or self.stop_event.is_set() else 'error'), str(exc)
        finally:
            if decoder is not None:
                decoder.cancel()
                if decoder.done.is_set() and self.asr_decoder is decoder:
                    self.asr_decoder = None
            with self.listen_lock:
                self.listening = False
                self.capture_active = False
                self.recognition_phase = 'idle'
                self.capture_mode = 'idle'
                # A microphone test must never turn into automatic command
                # recording. Only an explicit normal Listen start resumes it.
                self.continuous_paused = True
                if not self.stop_event.is_set():
                    self.audio_test_status(terminal, session_id=session_id, capture_active=False,
                        elapsed_s=elapsed, sampled_s=sampled / 16000, reason=reason,
                        recording_levels=total_level.snapshot(), **transcript)
                    self.status('idle')

    def scene(self, msg):
        try:
            report = json.loads(msg.data)
            if not isinstance(report, dict):
                raise ValueError('scene must be object')
            self.latest_scene = report
            stamp = report.get('stamp', -1e9)
            if (self.dialogue_enabled() and self.config.get('auto_scene_speech') and report.get('valid') is True
                    and isinstance(stamp, (int, float))
                    and 0 <= self.now() - stamp <= self.config.get('scene_ttl_s', 3)):
                self.enqueue(scene_to_speech(report, self.now(), self.config.get('scene_ttl_s', 3)),
                             valid_for_s=self.config.get('scene_ttl_s', 3) - (self.now() - stamp))
        except (ValueError, TypeError):
            self.status('invalid_scene_message')

    def say(self, msg):
        # JSON {action: report_scene} requests a report evaluated for freshness now.
        text = msg.data
        valid_for_s = 3.0
        if text.lstrip().startswith('{'):
            try:
                data = json.loads(text)
                text = (scene_to_speech(self.latest_scene, self.now(), self.config.get('scene_ttl_s', 3))
                        if data.get('action') == 'report_scene' else data['text'])
                if data.get('action') == 'report_scene' and isinstance(self.latest_scene, dict):
                    stamp = self.latest_scene.get('stamp')
                    if isinstance(stamp, (int, float)):
                        remaining = self.config.get('scene_ttl_s', 3) - (self.now() - stamp)
                        if 0 < remaining <= self.config.get('scene_ttl_s', 3):
                            valid_for_s = remaining
            except (ValueError, KeyError, TypeError):
                self.status('invalid_speech_message')
                return
        self.enqueue(text, force=True, valid_for_s=valid_for_s)

    def enqueue(self, text, force=False, valid_for_s=3):
        if not isinstance(text, str) or not text.strip() or len(text) > MAX_SPOKEN_TEXT_CHARS:
            self.speech_status('unavailable', reason='speech_text_requires_1_to_900_characters')
            return
        if not force and not self.dedup.accept(text):
            return
        if not self.tts_enabled():
            self.status('speech_preview', text=text, playback=False)
            return
        try:
            with self.voice_control_lock:
                if self.audio_stopped() or not self.tts_enabled():
                    return
                self.speech_queue.put_nowait((text, time.monotonic() + max(0, valid_for_s)))
        except queue.Full:
            self.status('speech_queue_full')

    def speech_status(self, state, **details):
        """Keep synthesis/playback failures visible across voice heartbeats."""
        diagnostics = dict(getattr(self, 'speech_diagnostics', {}))
        diagnostics.update(tts_state=state, tts_error=str(details.get('reason', ''))
                           if state == 'unavailable' else '')
        if state in ('spoken', 'unavailable', 'expired', 'interrupted'):
            diagnostics['tts_last_result'] = dict(state=state, stamp=self.now(), **details)
        self.speech_diagnostics = diagnostics
        label = {'preparing': 'tts_preparing', 'ready': 'tts_ready',
                 'unavailable': 'tts_unavailable', 'expired': 'speech_expired',
                 'playing': 'speaking', 'interrupted': 'speech_interrupted'}.get(state, state)
        self.status(label, **details)

    def check_speech_output(self):
        from .voice_tts_process import alsa_playback_status
        result = alsa_playback_status(self.config.get('tts_audio_device'))
        self.speech_diagnostics = dict(getattr(self, 'speech_diagnostics', {}),
            tts_audio_device=result['tts_audio_device'],
            tts_device_ready=result['tts_device_ready'], tts_device_checked_at=self.now())
        if result['tts_device_ready'] is False:
            raise RuntimeError(result['reason'] + ': ' + str(result['tts_audio_device']))

    def play_speech_chunk(self, chunk, text, backend, index, count):
        """One bounded piece; the worker owns the whole answer and its speaking flag."""
        audio_path = None
        metrics = {}
        evidence = dict(tts_backend=backend, text=text, chunk_index=index, chunk_count=count)
        try:
            if index > 1:
                self.check_speech_output()
            device = self.config.get('tts_audio_device')
            timeout = 45
            if backend in ('sherpa_vits', 'sherpa_matcha'):
                from .voice_tts_process import ProcessNeuralTTS
                with self.voice_control_lock, self.listen_lock:
                    if self.audio_stopped():
                        return None
                    if self.neural_tts is None:
                        self.neural_tts = ProcessNeuralTTS(self.config, self.workspace)
                self.speech_status('synthesizing', **evidence)
                wav, metrics = self.neural_tts.synthesize(chunk)
                if self.audio_stopped():
                    return None
                executable = shutil.which('aplay')
                if not executable:
                    raise RuntimeError('Install ALSA aplay for neural speech output')
                with tempfile.NamedTemporaryFile(prefix='rescue-tts-', suffix='.wav', delete=False) as output:
                    audio_path = Path(output.name)
                    output.write(wav)
                command = [executable, '-q']
                if device is not None:
                    command += ['-D', str(device)]
                command.append(str(audio_path))
                stdin = None
                timeout = max(10., min(125., metrics['duration_s'] + 5.))
            elif backend == 'espeak':
                executable = shutil.which(self.config.get('tts_executable', 'espeak-ng'))
                if not executable:
                    raise RuntimeError('Install espeak-ng to enable offline Chinese speech')
                command = [executable, '-v', self.config.get('tts_voice', 'cmn'),
                           '-s', str(int(self.config.get('tts_words_per_minute', 145))), '--stdin']
                if device is not None:
                    command[1:1] = ['-d', str(device)]
                stdin = chunk.encode('utf-8')
            else:
                raise ValueError('Unknown TTS backend: ' + str(backend))
            if self.audio_stopped():
                return None
            self.check_speech_output()  # USB may have been removed during synthesis.
            self.speech_status('playing', **evidence)
            with self.voice_control_lock:
                if self.audio_stopped():
                    return None
                self.tts_process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            if self.audio_stopped():
                self.tts_process.terminate()
            _, stderr = self.tts_process.communicate(stdin, timeout=timeout)
            if self.audio_stopped():
                return None
            if self.tts_process.returncode:
                raise RuntimeError(stderr.decode('utf-8', 'replace')[-400:])
            return metrics
        finally:
            if self.tts_process is not None and self.tts_process.poll() is None:
                self.tts_process.kill()
                self.tts_process.wait(timeout=2)
            self.tts_process = None
            if audio_path is not None:
                audio_path.unlink(missing_ok=True)

    def speech_worker(self):
        backend = self.config.get('tts_backend', 'espeak')
        if (self.config.get('tts_enabled') and self.config.get('tts_prewarm', False)
                and backend in ('sherpa_vits', 'sherpa_matcha')):
            try:
                from .voice_tts_process import ProcessNeuralTTS
                self.check_speech_output()
                with self.voice_control_lock, self.listen_lock:
                    if self.audio_stopped():
                        return
                    self.neural_tts = ProcessNeuralTTS(self.config, self.workspace)
                self.speech_status('preparing', tts_backend=backend)
                # No speaking flag or microphone cancellation while loading.
                metadata = self.neural_tts.prepare()
                if not self.audio_stopped():
                    self.speech_status('ready', tts_backend=backend, **metadata)
            except Exception as exc:
                if not self.audio_stopped():
                    self.speech_status('unavailable', reason=str(exc), tts_backend=backend)
        while not self.audio_stopped():
            try:
                text, deadline = self.speech_queue.get(timeout=.2)
            except queue.Empty:
                continue
            if time.monotonic() > deadline:
                self.speech_status('expired', reason='speech_expired_before_start', text=text)
                continue
            chunks, completed = [], 0
            try:
                # A missing USB output must fail before expensive synthesis,
                # and must not cancel microphone capture or claim playback.
                self.check_speech_output()
                with self.voice_control_lock, self.listen_lock:
                    if self.audio_stopped():
                        continue
                    self.speaking.set()
                    self.abort_listen.set()
                # The user should finish listening before TTS. Any overlapping capture is discarded.
                backend = self.config.get('tts_backend', 'espeak')
                chunks = [chunk for chunk in speech_chunks(text) if chunk.strip()]
                totals = dict(synthesis_s=0., duration_s=0.)
                for index, chunk in enumerate(chunks, 1):
                    if self.audio_stopped():
                        break
                    metrics = self.play_speech_chunk(chunk, text, backend, index, len(chunks))
                    if metrics is None:
                        break
                    for key in totals:
                        totals[key] += metrics.get(key, 0.)
                    completed += 1
                if completed == len(chunks) and not self.audio_stopped():
                    self.speech_status('spoken', text=text, tts_backend=backend,
                        chunk_count=len(chunks), chunks_completed=completed, **dict(metrics, **totals))
                else:
                    self.speech_status('interrupted', reason='speech_cancelled', text=text,
                        tts_backend=backend, chunk_count=len(chunks), chunks_completed=completed)
            except Exception as exc:
                if self.tts_process is not None and self.tts_process.poll() is None:
                    self.tts_process.kill()
                    self.tts_process.wait(timeout=2)
                self.speech_status('interrupted' if self.audio_stopped() else 'unavailable',
                    reason='speech_cancelled' if self.audio_stopped() else str(exc), text=text,
                    chunk_count=len(chunks), chunks_completed=completed)
            finally:
                self.tts_process = None
                # Short guard after playback avoids tail echo entering a new capture.
                self.audio_stop.wait(.35)
                self.speaking.clear()

    def destroy_node(self):
        with self.voice_control_lock:
            self.voice_enabled = False
            self.stop_event.set()
            self.audio_stop.set()
        self.end_listen.set()
        self.abort_listen.set()
        if self.voice_control_thread is not None:
            self.voice_control_thread.join(timeout=3)
        if self.asr_decoder is not None:
            self.asr_decoder.cancel()
        for backend in (self.asr_backend, self.verification_backend):
            close = getattr(backend, 'close', None)
            if close is not None:
                close()
        if self.tts_process is not None and self.tts_process.poll() is None:
            self.tts_process.terminate()
        with self.listen_lock:
            if self.neural_tts is not None:
                self.neural_tts.close()
        if self.mic_thread is not None:
            self.mic_thread.join(timeout=3)
        if self.tts_thread is not None:
            self.tts_thread.join(timeout=3)
        return super().destroy_node()


def main():
    spin(Voice)
