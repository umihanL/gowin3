import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

path = Path(__file__).resolve().parents[1] / 'tools/gimbal_start_profile.py'
spec = importlib.util.spec_from_file_location('gimbal_start_profile', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StartProfileTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root/'config').mkdir()
        self.profile = self.root/'profile.json'
        self.doc = {'schema_version': 1, 'boot_id': 'boot-a', 'operator_zero_confirmed': True, 'reference_id': 'zero-a'}
        for name, cfg in (
            ('io_config', {'dry_run': False, 'servo': {'backend': 'mixed_serial', 'reference_id': 'zero-a', 'calibration_confirmed': True, 'load_supported_verified': True}}),
            ('control_config', {'enabled': True, 'pump_enabled': False})):
            file = self.root/'config'/(name+'.local.json')
            raw=json.dumps(cfg).encode(); file.write_bytes(raw)
            self.doc[name]={'path': str(file), 'sha256': hashlib.sha256(raw).hexdigest()}
        self.save()

    def tearDown(self):
        self.tmp.cleanup()

    def save(self):
        self.profile.write_text(json.dumps(self.doc))

    def test_valid_boot_and_measured_files_allow_explicit_launch(self):
        args, reason=module.resolve(self.profile,self.root,'boot-a')
        self.assertEqual(reason,'')
        self.assertIn('actuator_dry_run:=false',args)
        self.assertIn('pd42_reference_confirmed:=true',args)

    def test_new_boot_never_reuses_old_zero(self):
        args, reason=module.resolve(self.profile,self.root,'boot-b')
        self.assertFalse(args); self.assertIn('boot_mismatch',reason)

    def test_changed_config_requires_new_review(self):
        Path(self.doc['io_config']['path']).write_text('{}')
        args, reason=module.resolve(self.profile,self.root,'boot-a')
        self.assertFalse(args); self.assertIn('config_changed',reason)

    def test_missing_confirmation_stays_dry(self):
        self.doc['operator_zero_confirmed']=False; self.save()
        self.assertFalse(module.resolve(self.profile,self.root,'boot-a')[0])

    def test_external_config_stays_dry(self):
        self.doc['io_config']['path']=str(self.root/'external.local.json'); self.save()
        self.assertFalse(module.resolve(self.profile,self.root,'boot-a')[0])

    def test_missing_profile_stays_dry(self):
        self.profile.unlink()
        self.assertFalse(module.resolve(self.profile,self.root,'boot-a')[0])

    def live_installation(self):
        io={'dry_run':False,'servo':{'backend':'mixed_serial',
            'reference_id':'old-zero','reference_confirmed_this_start':True,
            'pan':{'port':'/dev/ttyS3','reference_counts':-7,'min_angle_deg':-90,'max_angle_deg':90},
            'tilt':{'port':'/dev/ttyS1','reference_counts':26408,'direction':1,'min_angle_deg':0,'max_angle_deg':60}},
            'pump':{'backend':'fpga_uart','enabled':True},
            'commissioning':{'boot_id':'old-boot','tilt_physical_direction_verified':True,'tilt_positive_direction':'up'}}
        control={'enabled':False,'pump_enabled':True,'max_speed_deg_s':6}
        (self.root/'config/actuator_io.mixed.local.json').write_text(json.dumps(io))
        (self.root/'config/local_fire_control.local.json').write_text(json.dumps(control))
        return io,control

    def test_new_boot_starts_real_readback_without_reusing_zero_or_enabling_pump(self):
        io,control=self.live_installation()
        args,reason=module.resolve_live_start(self.profile,self.root,'boot-b',self.root/'runtime')
        self.assertEqual(len(args),4)
        self.assertIn('actuator_dry_run:=false',args)
        self.assertIn('pd42_reference_confirmed:=false',args)
        self.assertIn('boot_mismatch',reason)
        live=json.loads((self.root/'runtime/gimbal-live-io.json').read_text())
        self.assertIs(live['dry_run'],False)
        self.assertIs(live['servo']['reference_confirmed_this_start'],False)
        self.assertIs(live['servo']['live_readback_until_reference'],True)
        self.assertEqual(live['servo']['pan'],io['servo']['pan'])
        self.assertEqual(live['servo']['tilt'],io['servo']['tilt'])
        self.assertEqual(live['commissioning'],io['commissioning'])
        self.assertIs(live['pump']['enabled'],False)
        self.assertEqual(json.loads((self.root/'config/actuator_io.mixed.local.json').read_text()),io)
        self.assertEqual(json.loads((self.root/'config/local_fire_control.local.json').read_text()),control)
        live_control=json.loads((self.root/'runtime/gimbal-live-control.json').read_text())
        self.assertIs(live_control['enabled'],True);self.assertIs(live_control['pump_enabled'],False)

    def test_live_start_with_valid_boot_preserves_verified_profile_without_regenerating_reference(self):
        args,reason=module.resolve_live_start(self.profile,self.root,'boot-a',self.root/'runtime')
        self.assertEqual(reason,'');self.assertIn('pd42_reference_confirmed:=true',args)
        self.assertFalse((self.root/'runtime').exists())

    def test_no_saved_installation_only_opens_known_ports_without_inventing_direction_verification(self):
        io,control=self.live_installation()
        io['commissioning']={};io['servo']['tilt']['direction']=None
        local=self.root/'config/actuator_io.mixed.local.json';local.unlink()
        (self.root/'config/actuator_io.mixed.template.json').write_text(json.dumps(io))
        self.profile.unlink()
        args,_=module.resolve_live_start(self.profile,self.root,'new-boot',self.root/'runtime')
        self.assertIn('actuator_dry_run:=false',args)
        live=json.loads((self.root/'runtime/gimbal-live-io.json').read_text())
        self.assertEqual(live['commissioning'],{})
        self.assertIsNone(live['servo']['tilt']['direction'])
        self.assertEqual(live['servo']['pan']['port'],'/dev/ttyS3')

    def test_invalid_existing_installation_fails_instead_of_silently_using_a_template(self):
        self.live_installation();self.profile.unlink()
        (self.root/'config/actuator_io.mixed.local.json').write_text('{bad')
        with self.assertRaises(ValueError):
            module.resolve_live_start(self.profile,self.root,'new-boot',self.root/'runtime')
        self.assertFalse((self.root/'runtime').exists())

    def absolute_installation(self):
        io,control=self.live_installation()
        io['servo'].update(calibration_confirmed=True,load_supported_verified=True,
            absolute_reference={'schema_version':1,'reference_id':'old-zero'})
        control['enabled']=True
        profile=dict(schema_version=1,boot_id='old-boot',operator_zero_confirmed=True,
                     reference_id='old-zero',absolute_encoder_reference_saved=True)
        for name,filename,doc in [('io_config','actuator_io.mixed.local.json',io),
                                  ('control_config','local_fire_control.local.json',control)]:
            path=self.root/'config'/filename;raw=json.dumps(doc).encode();path.write_bytes(raw)
            profile[name]={'path':str(path),'sha256':hashlib.sha256(raw).hexdigest()}
        saved=self.root/'config/gimbal-absolute-profile.local.json'
        saved.write_text(json.dumps(profile))
        return io,profile,saved

    def test_persistent_anchor_only_authorizes_live_readback_not_motion_on_a_new_boot(self):
        io,_,_=self.absolute_installation()
        args,reason=module.resolve_live_start(self.profile,self.root,'boot-new',self.root/'runtime')
        self.assertIn('pd42_reference_confirmed:=false',args)
        self.assertIn('saved_absolute_reference_waiting_for_live_readback',reason)
        live=json.loads((self.root/'runtime/gimbal-live-io.json').read_text())
        self.assertTrue(live['servo']['absolute_reference_restore_allowed'])
        self.assertFalse(live['servo']['reference_confirmed_this_start'])
        self.assertEqual(live['servo']['absolute_reference'],io['servo']['absolute_reference'])
        self.assertFalse(live['pump']['enabled'])

    def test_changed_or_unconfirmed_persistent_files_do_not_restore_absolute_zero(self):
        _,profile,saved=self.absolute_installation()
        for key,value in [('absolute_encoder_reference_saved',False),('operator_zero_confirmed',False)]:
            changed=dict(profile);changed[key]=value;saved.write_text(json.dumps(changed))
            module.resolve_live_start(self.profile,self.root,'boot-new',self.root/'runtime')
            live=json.loads((self.root/'runtime/gimbal-live-io.json').read_text())
            self.assertFalse(live['servo']['absolute_reference_restore_allowed'])
        saved.write_text(json.dumps(profile))
        local=self.root/'config/actuator_io.mixed.local.json'
        local.write_text(local.read_text()+' ')
        module.resolve_live_start(self.profile,self.root,'boot-new',self.root/'runtime')
        self.assertFalse(json.loads((self.root/'runtime/gimbal-live-io.json').read_text())['servo']['absolute_reference_restore_allowed'])

    def test_same_x5_boot_still_checks_absolute_encoder_after_possible_motor_power_cycle(self):
        _,profile,_=self.absolute_installation()
        self.profile.write_text(json.dumps(profile))
        args,reason=module.resolve_live_start(self.profile,self.root,'old-boot',self.root/'runtime')
        self.assertIn('pd42_reference_confirmed:=false',args)
        self.assertIn('waiting_for_live_readback',reason)
        self.assertTrue(json.loads((self.root/'runtime/gimbal-live-io.json').read_text())['servo']['absolute_reference_restore_allowed'])


if __name__ == '__main__':
    unittest.main()
