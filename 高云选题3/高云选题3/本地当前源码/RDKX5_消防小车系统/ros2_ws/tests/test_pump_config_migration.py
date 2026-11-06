"""Configuration migration changes only pump and never enables output."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
TOOL=ROOT/'tools/migrate_pump_to_fpga.py'
spec=importlib.util.spec_from_file_location('pump_migration',TOOL)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class PumpMigrationTests(unittest.TestCase):
    def document(self):
        return dict(dry_run=False,servo=dict(backend='mixed_serial',pan=dict(reference_counts=123)),
            pump=dict(enabled=True,gpio_chip='/dev/gpiochip0',line_offset=23,active_low=False),
            commissioning=dict(calibration_id='measured-keep'))

    def test_only_pump_changes_and_new_backend_stays_disabled(self):
        old=self.document();new=module.migrated(old)
        self.assertFalse(new['pump']['enabled'])
        self.assertEqual(new['pump']['pin'],'B13')
        self.assertNotIn('line_offset',new['pump'])
        self.assertTrue(old['pump']['enabled'])
        for key in ('servo','dry_run','commissioning'):self.assertEqual(old[key],new[key])

    def test_already_migrated_valid_configuration_is_idempotent(self):
        new=module.migrated(self.document())
        self.assertEqual(module.migrated(new),new)
        new['pump']['enabled']=True
        self.assertEqual(module.migrated(new),new)
        new['pump']['line_offset']=23
        with self.assertRaises(ValueError):module.migrated(new)

    def test_preview_apply_backup_and_noop_apply(self):
        with tempfile.TemporaryDirectory(prefix='pump-config-test-') as directory:
            config=Path(directory)/'actuator.local.json'
            original=(json.dumps(self.document())+'\n').encode()
            config.write_bytes(original)
            def run(*extra):
                result=subprocess.run([sys.executable,'-X','utf8',str(TOOL),'--config',str(config),*extra],
                                      capture_output=True,encoding='utf-8',check=True)
                return json.loads(result.stdout)
            preview=run()
            self.assertFalse(preview['applied']);self.assertEqual(config.read_bytes(),original)
            applied=run('--apply')
            self.assertEqual(Path(applied['backup']).read_bytes(),original)
            self.assertFalse(json.loads(config.read_text())['pump']['enabled'])
            again=run('--apply')
            self.assertFalse(again['changed']);self.assertIsNone(again['backup'])


if __name__=='__main__':unittest.main()
