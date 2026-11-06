"""Cached, reclaimable local ASR worker; no microphones, network or commands.

Native inference cannot be interrupted by a Python thread Event. Keeping the
model in one child process lets a timed-out capture reclaim it before the next
recording. Successful recordings reuse that process and the exact same model.
"""
import multiprocessing
import threading
import time


def _worker(connection, config, workspace):
    # Import native runtimes only in the child, after spawn (never fork a ROS
    # executor or an already initialized ONNX runtime).
    from .asr_backend import load_asr_backend
    try:
        backend = load_asr_backend(config, workspace)
        metadata = {name: getattr(backend, name, None) for name in (
            'asr_backend', 'engine', 'confidence_available', 'fallback_from', 'fallback_reason')}
        connection.send(('ready', metadata))
        recognizer = None
        active_session = None
        while True:
            session, method, args = connection.recv()
            try:
                if method == 'create':
                    recognizer = backend.create_recognizer(*args)
                    active_session = session
                    value = None
                elif session != active_session or recognizer is None:
                    raise RuntimeError('asr_session_not_current')
                elif method in ('SetWords', 'AcceptWaveform', 'Result', 'FinalResult'):
                    value = getattr(recognizer, method)(*args)
                else:
                    raise ValueError('invalid_asr_worker_operation')
                connection.send(('ok', session, value))
            except Exception as exc:
                connection.send(('error', session, type(exc).__name__ + ': ' + str(exc)[:400]))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send(('startup_error', type(exc).__name__ + ': ' + str(exc)[:400]))
        except (EOFError, BrokenPipeError, OSError):
            pass
    finally:
        connection.close()


class ProcessASRBackend:
    """One native model, one active utterance, bounded cancellation/recovery."""
    process_isolated = True

    def __init__(self, config, workspace, *, context=None, worker_target=None,
                 operation_timeout_s=60., startup_timeout_s=90., preparation_cancelled=None):
        self.config = dict(config)
        self.workspace = str(workspace)
        self.context = context or multiprocessing.get_context('spawn')
        self.worker_target = worker_target or _worker
        self.operation_timeout_s = operation_timeout_s
        self.startup_timeout_s = startup_timeout_s
        self.preparation_cancelled = preparation_cancelled
        self._io_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._process = None
        self._connection = None
        self._generation = 0
        self._session = 0
        self._closed = False
        self.asr_backend = config.get('asr_backend', 'vosk')
        self.engine = None
        self.confidence_available = None
        self.fallback_from = self.fallback_reason = None
        with self._io_lock:
            self._ensure_worker()

    @property
    def ready(self):
        with self._state_lock:
            return self._process is not None and self._process.is_alive()

    def _receive(self, connection, process, generation, timeout, preparing=False):
        deadline = time.monotonic() + timeout
        while True:
            if preparing and self.preparation_cancelled is not None and self.preparation_cancelled():
                raise RuntimeError('asr_preparation_cancelled')
            with self._state_lock:
                if generation != self._generation or self._closed:
                    raise RuntimeError('asr_worker_cancelled')
            if connection.poll(.05):
                return connection.recv()
            if not process.is_alive():
                raise RuntimeError('asr_worker_exited')
            if time.monotonic() >= deadline:
                raise RuntimeError('asr_worker_timeout')

    def _ensure_worker(self):
        with self._state_lock:
            if self._closed:
                raise RuntimeError('asr_backend_closed')
            if self._process is not None and self._process.is_alive():
                return self._connection, self._process, self._generation
        self._discard_worker()
        parent, child = self.context.Pipe()
        process = self.context.Process(target=self.worker_target,
            args=(child, self.config, self.workspace), daemon=True)
        with self._state_lock:
            if self._closed:
                parent.close(); child.close()
                raise RuntimeError('asr_backend_closed')
            self._generation += 1
            generation = self._generation
            self._connection, self._process = parent, process
            try:
                process.start()
            except Exception:
                self._connection = self._process = None
                parent.close(); child.close()
                raise
        child.close()
        try:
            message = self._receive(parent, process, generation, self.startup_timeout_s, preparing=True)
            if len(message) != 2 or message[0] != 'ready' or not isinstance(message[1], dict):
                raise RuntimeError('asr_worker_load_failed: ' + str(message)[-400:])
            for key, value in message[1].items():
                if key in ('asr_backend', 'engine', 'confidence_available', 'fallback_from', 'fallback_reason'):
                    setattr(self, key, value)
            return parent, process, generation
        except Exception:
            self._discard_worker(generation)
            raise

    def _discard_worker(self, generation=None, session=None):
        # Do not acquire _io_lock: the native call being cancelled owns it.
        with self._state_lock:
            if generation is not None and generation != self._generation:
                return
            if session is not None and session != self._session:
                return
            process, connection = self._process, self._connection
            self._process = self._connection = None
            self._generation += 1
        if process is not None:
            if process.is_alive():
                process.terminate()
            process.join(timeout=.5)
            if process.is_alive():
                process.kill()
                process.join(timeout=.5)
        if connection is not None:
            connection.close()

    def _request_locked(self, session, method, args, generation):
        with self._state_lock:
            if (generation != self._generation or session != self._session
                    or self._connection is None):
                raise RuntimeError('asr_session_expired')
            connection, process = self._connection, self._process
        try:
            connection.send((session, method, args))
            response = self._receive(connection, process, generation, self.operation_timeout_s,
                                     preparing=method in ('create', 'SetWords'))
            if len(response) != 3 or response[1] != session:
                raise RuntimeError('invalid_asr_worker_reply')
            if response[0] == 'error':
                raise RuntimeError(response[2])
            if response[0] != 'ok':
                raise RuntimeError('invalid_asr_worker_reply')
            return response[2]
        except Exception:
            self._discard_worker(generation)
            raise

    def create_recognizer(self, sample_rate=16000):
        with self._io_lock:
            _, _, generation = self._ensure_worker()
            with self._state_lock:
                if generation != self._generation or self._closed:
                    raise RuntimeError('asr_worker_cancelled')
                self._session += 1
                session = self._session
            self._request_locked(session, 'create', (sample_rate,), generation)
            return ProcessASRRecognizer(self, session, generation)

    def call(self, session, method, args, generation):
        with self._io_lock:
            return self._request_locked(session, method, args, generation)

    def cancel(self, session, generation):
        self._discard_worker(generation, session)

    def close(self):
        with self._state_lock:
            self._closed = True
        self._discard_worker()


class ProcessASRRecognizer:
    def __init__(self, backend, session, generation):
        self.backend, self.session, self.generation = backend, session, generation
        self.finished = False

    def _call(self, method, *args):
        return self.backend.call(self.session, method, args, self.generation)

    def SetWords(self, enabled):
        return self._call('SetWords', bool(enabled))

    def AcceptWaveform(self, pcm):
        return self._call('AcceptWaveform', bytes(pcm))

    def Result(self):
        return self._call('Result')

    def FinalResult(self):
        try:
            return self._call('FinalResult')
        finally:
            self.finished = True

    def Cancel(self):
        if not self.finished:
            self.backend.cancel(self.session, self.generation)


def load_isolated_asr_backend(config, workspace, *, preparation_cancelled=None):
    return ProcessASRBackend(config, workspace, preparation_cancelled=preparation_cancelled)
