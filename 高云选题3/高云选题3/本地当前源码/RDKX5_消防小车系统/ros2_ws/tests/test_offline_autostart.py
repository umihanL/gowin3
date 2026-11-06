"""Generate and inspect boot units locally; never access systemd or hardware."""
import os
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

WORKSPACE = Path(__file__).resolve().parents[1]
BASH = os.environ.get('RESCUE_TEST_BASH') or (
    r'D:\Git\bin\bash.exe' if Path(r'D:\Git\bin\bash.exe').is_file() else shutil.which('bash'))


@unittest.skipUnless(BASH, 'Bash is required to test generated startup units')
class OfflineAutostartTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.bin = self.root/'bin'
        self.bin.mkdir()
        self.log = self.root/'forbidden-calls.log'
        self.bash_env=self.root/'bash_env.sh'
        self.bash_env.write_text('if command -v cygpath >/dev/null 2>&1; then test_bin="$(cygpath -u "$AUTOSTART_TEST_BIN")"; else test_bin="$AUTOSTART_TEST_BIN"; fi\nexport PATH="$test_bin:$PATH"\n', newline='\n')
        self.env = dict(os.environ, PATH=str(self.bin)+os.pathsep+os.environ.get('PATH', ''),
                        AUTOSTART_TEST_LOG=self.log.as_posix(), AUTOSTART_TEST_BIN=self.bin.as_posix(),
                        BASH_ENV=self.bash_env.as_posix())
        for name in ('systemctl', 'systemd-run', 'id'):
            self.shim(name, '#!/bin/bash\nprintf "%s\\n" "'+name+' $*" >> "$AUTOSTART_TEST_LOG"\nexit 91\n')

    def tearDown(self):
        self.temp.cleanup()

    def shim(self, name, content):
        target=self.bin/name
        target.write_text(content, encoding='utf-8', newline='\n')
        target.chmod(0o755)

    def run_script(self, script, *args, cwd=WORKSPACE, env=None):
        return subprocess.run([str(BASH), str(script), *map(str, args)], cwd=cwd,
                              env=env or self.env, capture_output=True, text=True,
                              encoding='utf-8', errors='replace', timeout=20)

    def test_all_changed_shell_files_parse(self):
        for name in ('install_offline_autostart.sh', 'run_standalone_tasks.sh',
                     'start_perception_services.sh', 'start_interaction_services.sh'):
            result=self.run_script('-n', WORKSPACE/'tools'/name)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_output_mode_is_host_independent_and_emits_supervised_local_units(self):
        output=self.root/'units'
        result=self.run_script('tools/install_offline_autostart.sh', '--output-dir', output.as_posix())
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.log.exists(), 'Output mode must not call id/systemctl/systemd-run')
        services=sorted(output.glob('*.service'))
        self.assertEqual(len(services), 4)
        for service in services:
            value=service.read_text()
            self.assertIn('PartOf=rescue-standalone.target', value)
            self.assertIn('Restart=on-failure', value)
            self.assertIn('WorkingDirectory=/root/rescue/ros2_ws', value)
            self.assertNotIn('network-online', value)
            self.assertNotIn('ExecStartPre=', value)
        target=(output/'rescue-standalone.target').read_text()
        self.assertIn('WantedBy=multi-user.target', target)
        self.assertNotIn('rescue-edge-llm.service', target, 'Optional units are not invented in output mode')
        self.assertIn('run_standalone_tasks.sh', (output/'rescue-task-interaction.service').read_text())
        self.assertIn('EnvironmentFile=-/etc/default/rescue-standalone', (output/'rescue-task-interaction.service').read_text())

    def test_output_replacement_preserves_previous_unit(self):
        output=self.root/'units'
        output.mkdir()
        original=output/'rescue-mipi-camera.service'
        original.write_text('prior customized unit\n')
        result=self.run_script('tools/install_offline_autostart.sh', '--output-dir', output.as_posix())
        self.assertEqual(result.returncode, 0, result.stderr)
        backups=list((output/'backups').glob('*/rescue-mipi-camera.service'))
        self.assertEqual(len(backups), 1)
        self.assertEqual(backups[0].read_text(), 'prior customized unit\n')
        self.assertFalse(self.log.exists())

    def test_invalid_workspace_cannot_inject_unit_directives(self):
        for value in ('relative', '/', '/root/../etc', '/root/a b', '/root/a%u', '/root/a\nExecStart=bad'):
            with self.subTest(value=value):
                output=self.root/'invalid'
                result=self.run_script('tools/install_offline_autostart.sh', '--output-dir', output.as_posix(), '--workspace', value)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())
        self.assertFalse(self.log.exists())

    def test_runner_starts_real_gimbal_readback_and_keeps_explicit_voice_override(self):
        tools=self.root/'workspace'/'tools';tools.mkdir(parents=True)
        shutil.copyfile(WORKSPACE/'tools/run_standalone_tasks.sh', tools/'run_standalone_tasks.sh')
        shutil.copyfile(WORKSPACE/'tools/fixed_camera_start_config.py', tools/'fixed_camera_start_config.py')
        shutil.copyfile(WORKSPACE/'tools/gimbal_start_profile.py', tools/'gimbal_start_profile.py')
        module=tools.parent/'src/rescue_vision/rescue_vision';module.mkdir(parents=True)
        shutil.copyfile(WORKSPACE/'src/rescue_vision/rescue_vision/fixed_camera_aim.py', module/'fixed_camera_aim.py')
        shutil.copyfile(WORKSPACE/'src/rescue_vision/rescue_vision/fixed_camera_landing.py', module/'fixed_camera_landing.py')
        config=tools.parent/'config';config.mkdir()
        shutil.copyfile(WORKSPACE/'config/fixed_camera_aim.template.json', config/'fixed_camera_aim.template.json')
        shutil.copyfile(WORKSPACE/'config/fixed_camera_landing.template.json', config/'fixed_camera_landing.template.json')
        shutil.copyfile(WORKSPACE/'config/actuator_io.mixed.template.json', config/'actuator_io.mixed.template.json')
        shutil.copyfile(WORKSPACE/'config/local_fire_control.template.json', config/'local_fire_control.template.json')
        self.shim('python3', '#!/bin/bash\nexec "'+Path(sys.executable).as_posix()+'" "$@"\n')
        (tools/'run_task_modes.sh').write_text('#!/bin/bash\nprintf "%s\\n" "$@"\n', newline='\n')
        voice=self.root/'custom voice.json';voice.write_text('{}')
        env=dict(self.env, RESCUE_STANDALONE_VOICE_CONFIG=voice.as_posix(),
                 RESCUE_GIMBAL_RUNTIME_DIR=(self.root/'runtime').as_posix(),
                 RESCUE_STANDALONE_IO_CONFIG='/tmp/unreviewed-actuator.json',
                 RESCUE_STANDALONE_ACTUATOR_DRY_RUN='false')
        result=self.run_script(tools/'run_standalone_tasks.sh', env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        for gate in ('uart_dry_run:=true','actuator_dry_run:=false','navigation_execute_enabled:=true',
                     'pd42_reference_confirmed:=false','fixed_camera_observation:=true','task_initial_mode:=inspect'):
            self.assertIn(gate, result.stdout.splitlines())
        self.assertIn('voice_config:='+voice.as_posix(), result.stdout.splitlines())
        io_args=[line for line in result.stdout.splitlines() if line.startswith('io_config:=')]
        self.assertEqual(len(io_args), 1)
        self.assertTrue(io_args[0].endswith('/runtime/gimbal-live-io.json'), io_args)
        self.assertNotIn('unreviewed-actuator', result.stdout)
        self.assertTrue(any(line.startswith('fixed_camera_aim_config:=') and
                            line.endswith('/config/fixed_camera_aim.template.json')
                            for line in result.stdout.splitlines()))
        self.assertTrue(any(line.startswith('fixed_camera_landing_config:=') and
                            line.endswith('/config/fixed_camera_landing.template.json')
                            for line in result.stdout.splitlines()))
        installer=(WORKSPACE/'tools/install_offline_autostart.sh').read_text()
        self.assertIn('config/actuator_io.mixed.template.json', installer)
        self.assertNotIn('config/actuator_io.pd42.template.json', installer)
        self.assertNotIn('continuous', result.stdout)
        for override in ('actuator_dry_run:=false', 'io_config:=/tmp/unreviewed-actuator.json'):
            rejected=self.run_script(tools/'run_standalone_tasks.sh', override, env=env)
            self.assertEqual(rejected.returncode, 2)
        # Actual selection and CLI failure propagation for the independent
        # water table, using synthetic files only inside this temporary tree.
        from test_fixed_camera_landing import landing_document
        local_water = config/'fixed_camera_landing.local.json'
        local_water.write_text(json.dumps(landing_document()))
        selected=self.run_script(tools/'run_standalone_tasks.sh', env=env)
        self.assertEqual(selected.returncode, 0, selected.stderr)
        self.assertIn('fixed_camera_landing_config:='+local_water.resolve().as_posix(), selected.stdout.splitlines())
        explicit_water = config/'custom water.json'
        explicit_water.write_text(json.dumps(landing_document()))
        local_water.write_text('{bad-json')
        water_env = dict(env, RESCUE_STANDALONE_FIXED_CAMERA_LANDING_CONFIG=explicit_water.as_posix())
        selected=self.run_script(tools/'run_standalone_tasks.sh', env=water_env)
        self.assertEqual(selected.returncode, 0, selected.stderr)
        self.assertIn('fixed_camera_landing_config:='+explicit_water.resolve().as_posix(), selected.stdout.splitlines())
        for content in ('{bad-json', (config/'fixed_camera_landing.template.json').read_text()):
            local_water.write_text(content)
            rejected=self.run_script(tools/'run_standalone_tasks.sh', env=env)
            self.assertEqual(rejected.returncode, 2)
            self.assertIn('Fixed-camera startup configuration rejected', rejected.stderr)
            self.assertNotIn('fixed_camera_landing_config:=', rejected.stdout)
        local_water.unlink()
        # A malformed existing local calibration fails before launch. It is
        # never silently replaced by the unmeasured template.
        (config/'fixed_camera_aim.local.json').write_text('{bad-json')
        rejected=self.run_script(tools/'run_standalone_tasks.sh', env=env)
        self.assertEqual(rejected.returncode, 2)
        self.assertIn('Fixed-camera startup configuration rejected', rejected.stderr)
        self.assertNotIn('fixed_camera_aim_config:=', rejected.stdout)

    def test_existing_inactive_units_are_started_without_transient_name_collision(self):
        self.shim('id', '#!/bin/bash\necho 0\n')
        self.shim('systemctl', '#!/bin/bash\nprintf "%s\\n" "$*" >> "$AUTOSTART_TEST_LOG"\ncase "$1" in is-active) exit 3;; cat|start) exit 0;; *) exit 92;; esac\n')
        self.shim('python3', '#!/bin/bash\ncat >/dev/null\n')
        tools=self.root/'workspace'/'tools';tools.mkdir(parents=True)
        for name in ('start_perception_services.sh','start_interaction_services.sh'):
            shutil.copyfile(WORKSPACE/'tools'/name, tools/name)
        result=self.run_script(tools/'start_interaction_services.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls=self.log.read_text()
        for unit in ('rescue-mipi-camera','rescue-mipi-vision','rescue-yolo-preview','rescue-task-interaction'):
            self.assertIn('start '+unit+'.service', calls)
        self.assertNotIn('systemd-run', calls)


if __name__ == '__main__':
    unittest.main()
