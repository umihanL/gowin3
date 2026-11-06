"""Independent, atomic persistence for the microphone/speaker master switch."""
import json
import os
from pathlib import Path
import tempfile


def load_voice_runtime(path):
    try:
        value = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(value, dict) or type(value.get('enabled')) is not bool:
            raise ValueError('enabled must be boolean')
        return value['enabled'], ''
    except FileNotFoundError:
        # This release intentionally starts OFF when no runtime choice exists:
        # the USB microphone/speaker are temporarily unplugged. A subsequently
        # saved enabled:true takes precedence without altering device settings.
        return False, ''
    except (OSError, UnicodeError, ValueError) as exc:
        return False, 'voice_runtime_invalid: ' + str(exc)


def save_voice_runtime(path, enabled):
    if type(enabled) is not bool:
        raise ValueError('enabled must be boolean')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                prefix='.voice-runtime-', suffix='.tmp', delete=False) as output:
            temporary = Path(output.name)
            json.dump({'enabled': enabled}, output, ensure_ascii=False)
            output.write('\n')
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
