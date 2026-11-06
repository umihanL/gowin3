"""Real voice-node dispatch with mocked decoders, no recording or movement."""
import unittest
from unittest.mock import Mock, patch

import test_voice_capture_lifecycle as capture_fixture
from test_selected_fire_round import TEXT, state
from rescue_vision.task_interaction import TaskRouter


class SelectedFireVoiceIngressTests(unittest.TestCase):
    setUpClass = classmethod(capture_fixture.VoiceCaptureLifecycleTests.setUpClass.__func__)
    setUp = capture_fixture.VoiceCaptureLifecycleTests.setUp
    make_voice = capture_fixture.VoiceCaptureLifecycleTests.make_voice

    def verify(self, secondary):
        voice = self.make_voice()
        voice.verification_backend = Mock()
        decoder = Mock(); decoder.done.is_set.return_value = True
        decoder.combined.return_value = dict(text=secondary, result=[dict(conf=.99)])
        with patch.object(self.module, 'StreamingASR', return_value=decoder), patch.object(voice, 'wait_decoder'):
            proof = voice.verify_voice_action(TEXT, dict(confidence_available=True), [b'\0\0' * 1600], 100000000.)
        voice.verification_backend.create_recognizer.assert_called_once_with(16000)
        return voice, proof

    def test_verified_same_pcm_command_reaches_router_with_proof(self):
        voice, proof = self.verify('一键瞄准喷水并关水')
        self.assertTrue(proof['verified'], proof)
        result = voice.accept_text(TEXT, 'voice', 10., verification=proof)
        self.assertEqual(result['state'], 'command_submitted')
        command = voice.command_pub.values[-1]
        self.assertEqual(command['voice_verification'], proof)
        out = TaskRouter(extension_enabled=True).route(command, 10., **state(10.))
        self.assertEqual(out['auto']['preferred_target_id'], 2)
        self.assertEqual(out['actuator']['action'], 'disarm')

    def test_disagreement_or_no_independent_proof_never_publishes(self):
        voice, proof = self.verify('锁定火焰2')
        self.assertFalse(proof['verified'])
        for evidence in (None, {}, proof):
            result = voice.accept_text(TEXT, 'voice', 10., verification=evidence)
            self.assertEqual(result['reason'], 'selected_fire_round_independent_asr_required')
        self.assertEqual(voice.command_pub.values, [])
        self.assertEqual(voice.accept_text('停水', 'voice', 10.)['state'], 'command_submitted')


if __name__ == '__main__':
    unittest.main()
