import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.gimbal_reference_store import persist_reference
from rescue_vision import gimbal_reference_store as store


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root/'config').mkdir()
        self.runtime = self.root/'runtime'
        self.result = dict(reference_id='manual-zero-test', boot_id='boot-test',
            config=dict(dry_run=False, servo=dict(reference_confirmed_this_start=True,
                reference_id='manual-zero-test'), live_reference=dict(workspace=str(self.root),
                profile_path=str(self.runtime/'gimbal-profile.json'))))
        self.control = dict(enabled=True, pump_enabled=False, max_speed_deg_s=6.)

    def save(self):
        return persist_reference(self.result, self.control, runtime_dir=self.runtime, boot_id='boot-test')

    def test_writes_hash_bound_current_boot_profile_last_and_preserves_prior_files(self):
        old = self.root/'config/actuator_io.mixed.local.json'
        old.write_text('old installation')
        profile = self.save()
        self.assertEqual(profile['boot_id'], 'boot-test')
        self.assertTrue(profile['operator_zero_confirmed'])
        self.assertEqual(old.with_name(old.name+'.before-manual-zero-test').read_text(), 'old installation')
        for key in ('io_config', 'control_config'):
            doc = profile[key]
            self.assertEqual(hashlib.sha256(Path(doc['path']).read_bytes()).hexdigest(), doc['sha256'])
        saved = json.loads(Path(profile['control_config']['path']).read_text())
        self.assertFalse(saved['pump_enabled'])
        self.assertEqual(json.loads((self.runtime/'gimbal-profile.json').read_text()), profile)

    def test_rejects_boot_change_unconfirmed_result_and_external_profile(self):
        for mutate in (lambda:self.result.update(boot_id='old-boot'),
                       lambda:self.result['config']['servo'].update(reference_confirmed_this_start=False),
                       lambda:self.result['config']['live_reference'].update(profile_path=str(self.root/'elsewhere.json'))):
            original = json.dumps(self.result)
            mutate()
            with self.assertRaises(ValueError): self.save()
            self.assertFalse((self.root/'config/actuator_io.mixed.local.json').exists())
            self.result = json.loads(original)

    def test_atomic_failure_never_publishes_a_new_profile(self):
        original = store._atomic
        count = 0
        def fail_control(path, raw):
            nonlocal count
            count += 1
            if count == 2: raise OSError('disk unavailable')
            original(path, raw)
        with mock.patch.object(store, '_atomic', side_effect=fail_control):
            with self.assertRaises(OSError): self.save()
        self.assertFalse((self.runtime/'gimbal-profile.json').exists())

    def test_capture_does_not_enable_an_inactive_pump_or_change_speed(self):
        self.save()
        output = json.loads((self.root/'config/local_fire_control.local.json').read_text())
        self.assertEqual(output, self.control)

    def test_repeated_reference_replaces_profile_hash_and_preserves_previous_zero(self):
        first = self.save()
        self.result.update(reference_id='manual-zero-second')
        self.result['config']['servo'].update(reference_id='manual-zero-second',
            pan=dict(reference_counts=123), tilt=dict(reference_counts=456))
        second = self.save()
        self.assertNotEqual(first['io_config']['sha256'], second['io_config']['sha256'])
        saved = self.root/'config/actuator_io.mixed.local.json'
        self.assertEqual(json.loads(saved.read_text())['servo']['pan']['reference_counts'], 123)
        backup = saved.with_name(saved.name+'.before-manual-zero-second')
        self.assertEqual(json.loads(backup.read_text())['servo']['reference_id'], 'manual-zero-test')
        self.assertEqual(json.loads((self.runtime/'gimbal-profile.json').read_text()), second)

    def test_absolute_origin_survives_boots_with_same_identity_and_unique_backups(self):
        self.result['config']['servo']['absolute_reference'] = {
            'schema_version':1,'reference_id':self.result['reference_id']}
        first=self.save()
        persistent=self.root/'config/gimbal-absolute-profile.local.json'
        self.assertEqual(json.loads(persistent.read_text()),first)
        for boot in ('boot-next','boot-third'):
            self.result['boot_id']=boot
            profile=persist_reference(self.result,self.control,runtime_dir=self.runtime,boot_id=boot)
            self.assertEqual(profile['reference_id'],first['reference_id'])
            self.assertTrue(profile['absolute_encoder_reference_saved'])
            self.assertEqual(json.loads(persistent.read_text()),profile)
            self.assertEqual(json.loads((self.runtime/'gimbal-profile.json').read_text()),profile)
        self.assertEqual(len(list((self.root/'config').glob('actuator_io.mixed.local.json.before-*'))),2)

    def test_old_reference_without_absolute_encoder_does_not_create_persistent_anchor(self):
        profile=self.save()
        self.assertFalse(profile['absolute_encoder_reference_saved'])
        self.assertFalse((self.root/'config/gimbal-absolute-profile.local.json').exists())


if __name__ == '__main__': unittest.main()
