"""Mock-only tests. Never request real GPIO, PWM, I2C, or board connections."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision import local_actuator_io as io


def config(real=False):
    cfg = json.loads((ROOT/'config/actuator_io.template.json').read_text(encoding='utf-8'))
    cfg['pump']['enabled'] = True
    if real:
        cfg['dry_run'] = False
        cfg['servo']['calibration_confirmed'] = True
        for name, channel in (('pan', 0), ('tilt', 1)):
            cfg['servo'][name].update(chip='/sys/class/pwm/pwmchip5', channel=channel)
        cfg['pump'].update(gpio_chip='/dev/gpiochip0', line_offset=12, active_low=False)
    return cfg


class HardwareFixture(unittest.TestCase):
    def setUp(self):
        controller = patch.object(io, '_verify_pwm_controller')
        controller.start()
        self.addCleanup(controller.stop)


class ActuatorValidationTests(HardwareFixture):
    def test_dry_run_has_no_io_and_does_not_center(self):
        with patch.object(io.os, 'open', side_effect=AssertionError('IO')), \
             patch.object(io.importlib, 'import_module', side_effect=AssertionError('hardware import')):
            actuator = io.create_actuator_io(config())
            self.assertEqual(len(actuator.commands), 1)
            self.assertFalse(actuator.commands[-1]['pwm_enabled'])
            actuator.set_gimbal(90, 180)
            self.assertEqual(actuator.commands[-1]['pulse_us'], {'pan': 1500, 'tilt': 2000})
            actuator.set_pump(True)
            actuator.disable_gimbal()
            self.assertEqual(actuator.commands[-1]['operation'], 'disable_gimbal')
            actuator.close()
            actuator.close()
            with self.assertRaises(RuntimeError):
                actuator.set_pump(True)

    def test_invalid_real_configuration_rejected_before_any_io(self):
        cases = []
        cfg = config(); cfg['dry_run'] = False; cases.append(cfg)
        cfg = config(True); cfg['pump']['active_low'] = None; cases.append(cfg)
        cfg = config(True); cfg['servo']['tilt']['channel'] = 0; cases.append(cfg)
        cfg = config(True); cfg['servo']['frequency_hz'] = 60; cases.append(cfg)
        cfg = config(True); cfg['servo']['pan']['chip'] = '/sys/class/pwm/../other'; cases.append(cfg)
        cfg = config(True); cfg['servo']['pan']['min_pulse_us'] = 0; cases.append(cfg)
        cfg = config(True); cfg['servo']['pan']['max_angle_deg'] = float('nan'); cases.append(cfg)
        with patch.object(io, '_ExclusiveOwner') as owner:
            for cfg in cases:
                with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                    io.create_actuator_io(cfg)
            owner.assert_not_called()

    def test_both_angles_checked_before_writes_and_inversion(self):
        cfg = config(); cfg['servo']['pan']['invert'] = True
        actuator = io.create_actuator_io(cfg)
        actuator.set_gimbal(0, 90)
        self.assertEqual(actuator.commands[-1]['pulse_us']['pan'], 2000)
        count = len(actuator.commands)
        for pan, tilt in ((90, 181), (-1, 90), (True, 90), (90, float('inf'))):
            with self.assertRaises(ValueError):
                actuator.set_gimbal(pan, tilt)
        self.assertEqual(len(actuator.commands), count)
        with self.assertRaises(ValueError):
            actuator.set_pump(1)


class GPIOTests(unittest.TestCase):
    def test_gpiod_v2_atomic_inactive_and_logical_polarity(self):
        request = Mock()
        module = SimpleNamespace(line=SimpleNamespace(Value=SimpleNamespace(INACTIVE=0, ACTIVE=1),
            Direction=SimpleNamespace(OUTPUT='output')), LineSettings=Mock(), request_lines=Mock(return_value=request))
        cfg = config(True)['pump']; cfg['active_low'] = True
        with patch.object(io.importlib, 'import_module', return_value=module):
            pump = io._PumpGPIO(cfg)
        module.LineSettings.assert_called_once_with(direction='output', active_low=True, output_value=0)
        pump.set(True); request.set_value.assert_called_with(12, 1)
        pump.close(); request.set_value.assert_called_with(12, 0)
        request.release.assert_called_once()

    def test_gpiod_v1_atomic_inactive_and_cleanup(self):
        chip, line = Mock(), Mock()
        chip.get_line.return_value = line
        module = SimpleNamespace(Chip=Mock(return_value=chip), LINE_REQ_DIR_OUT=3, LINE_REQ_FLAG_ACTIVE_LOW=4)
        cfg = config(True)['pump']; cfg['active_low'] = True
        with patch.object(io.importlib, 'import_module', return_value=module):
            pump = io._PumpGPIO(cfg)
        line.request.assert_called_once_with(consumer='rescue-pump', type=3, flags=4, default_vals=[0])
        pump.set(True); line.set_value.assert_called_with(1)
        pump.close(); line.set_value.assert_called_with(0)
        line.release.assert_called_once(); chip.close.assert_called_once()

    def test_missing_dependency_and_request_failure_do_not_succeed(self):
        with patch.object(io.importlib, 'import_module', side_effect=ModuleNotFoundError('gpiod')):
            with self.assertRaises(ModuleNotFoundError):
                io._PumpGPIO(config(True)['pump'])
        chip, line = Mock(), Mock()
        chip.get_line.return_value = line
        line.request.side_effect = OSError('busy')
        module = SimpleNamespace(Chip=Mock(return_value=chip), LINE_REQ_DIR_OUT=3)
        with patch.object(io.importlib, 'import_module', return_value=module):
            with self.assertRaises(OSError):
                io._PumpGPIO(config(True)['pump'])
        chip.close.assert_called_once()


class PWMTests(unittest.TestCase):
    def test_export_initial_off_period_order_no_center_then_enable(self):
        axis = config(True)['servo']['pan']
        state = {'exported': False}
        events = []
        def write(path, value):
            events.append((Path(path).name, value))
            if Path(path).name == 'export': state['exported'] = True
        with patch.object(Path, 'exists', lambda _path: state['exported']), \
             patch.object(Path, 'read_text', return_value='2'), patch.object(io, '_write', side_effect=write):
            pwm = io._HardwarePWM(axis, 50)
            self.assertEqual(events, [('export', 0), ('enable', 0), ('duty_cycle', 0),
                                      ('period', 20000000), ('polarity', 'normal')])
            pwm.set(1500)
            self.assertEqual(events[-2:], [('duty_cycle', 1500000), ('enable', 1)])
            pwm.disable()
            self.assertEqual(events[-2:], [('enable', 0), ('duty_cycle', 0)])
            pwm.set(1600)
            self.assertEqual(events[-2:], [('duty_cycle', 1600000), ('enable', 1)])
            pwm.close()
            self.assertEqual(events[-3:], [('enable', 0), ('duty_cycle', 0), ('unexport', 0)])

    def test_existing_channel_is_never_taken_over(self):
        with patch.object(Path, 'exists', return_value=True), patch.object(io, '_write') as write:
            with self.assertRaisesRegex(RuntimeError, 'already exported'):
                io._HardwarePWM(config(True)['servo']['pan'], 50)
            write.assert_not_called()

    def test_export_error_does_not_unexport_someone_elses_channel(self):
        axis = config(True)['servo']['pan']
        with patch.object(Path, 'exists', return_value=False), patch.object(Path, 'read_text', return_value='2'), \
             patch.object(io, '_write', side_effect=OSError('EBUSY')) as write:
            with self.assertRaises(OSError):
                io._HardwarePWM(axis, 50)
            self.assertEqual(write.call_count, 1)


class LifecycleTests(HardwareFixture):
    def test_pump_created_before_pwm_no_motion_until_command(self):
        events = []
        owner, pump, pan, tilt = Mock(), Mock(), Mock(), Mock()
        def gpio(_config): events.append('gpio-off'); return pump
        def pwm(axis, _frequency): events.append('pwm-off'); return pan if axis['channel'] == 0 else tilt
        with patch.object(io, '_ExclusiveOwner', return_value=owner), patch.object(io, '_PumpGPIO', side_effect=gpio), \
             patch.object(io, '_HardwarePWM', side_effect=pwm):
            actuator = io.create_actuator_io(config(True))
        self.assertEqual(events, ['gpio-off', 'pwm-off', 'pwm-off'])
        pan.set.assert_not_called(); tilt.set.assert_not_called(); pump.set.assert_not_called()
        actuator.set_gimbal(90, 90)
        pan.set.assert_called_once_with(1500); tilt.set.assert_called_once_with(1500)
        actuator.disable_gimbal(); pan.disable.assert_called_once(); tilt.disable.assert_called_once()
        actuator.close(); pump.close.assert_called_once(); pan.close.assert_called_once(); owner.close.assert_called_once()

    def test_disabled_pump_allows_servo_only_real_run_without_gpio_config(self):
        cfg = config(True)
        cfg['pump'] = {'enabled': False, 'gpio_chip': None, 'line_offset': None, 'active_low': None}
        owner, pan, tilt = Mock(), Mock(), Mock()
        with patch.object(io, '_ExclusiveOwner', return_value=owner), patch.object(io, '_PumpGPIO') as gpio, \
             patch.object(io, '_HardwarePWM', side_effect=[pan, tilt]):
            actuator = io.create_actuator_io(cfg)
        gpio.assert_not_called()
        actuator.set_pump(False)
        actuator.set_gimbal(90, 90)
        with self.assertRaisesRegex(RuntimeError, 'disabled'):
            actuator.set_pump(True)
        self.assertFalse(actuator.closed)
        actuator.close()
        pan.close.assert_called_once(); tilt.close.assert_called_once()

    def test_default_template_pump_is_disabled_even_in_dry_run(self):
        cfg = json.loads((ROOT/'config/actuator_io.template.json').read_text(encoding='utf-8'))
        actuator = io.create_actuator_io(cfg)
        self.assertFalse(actuator.config['pump']['enabled'])
        actuator.set_pump(False)
        with self.assertRaisesRegex(RuntimeError, 'disabled'):
            actuator.set_pump(True)
        actuator.close()

    def test_initialization_and_command_failure_close_every_resource(self):
        owner, pump, pan = Mock(), Mock(), Mock()
        with patch.object(io, '_ExclusiveOwner', return_value=owner), patch.object(io, '_PumpGPIO', return_value=pump), \
             patch.object(io, '_HardwarePWM', side_effect=[pan, OSError('second PWM failed')]):
            with self.assertRaises(OSError):
                io.create_actuator_io(config(True))
        pump.close.assert_called_once(); pan.close.assert_called_once(); owner.close.assert_called_once()
        owner, pump, pan, tilt = Mock(), Mock(), Mock(), Mock()
        with patch.object(io, '_ExclusiveOwner', return_value=owner), patch.object(io, '_PumpGPIO', return_value=pump), \
             patch.object(io, '_HardwarePWM', side_effect=[pan, tilt]):
            actuator = io.create_actuator_io(config(True))
        tilt.set.side_effect = OSError('PWM disconnected')
        pump.close.side_effect = OSError('GPIO disconnected')
        with self.assertRaisesRegex(RuntimeError, 'shutdown'):
            actuator.set_gimbal(90, 90)
        self.assertTrue(actuator.closed)
        pan.close.assert_called_once(); tilt.close.assert_called_once(); owner.close.assert_called_once()
        self.assertFalse(actuator.commands[-1]['shutdown_confirmed'])

    def test_exclusive_owner_and_release_on_file_lock_failure(self):
        fake = SimpleNamespace(LOCK_EX=1, LOCK_NB=2, flock=Mock())
        with patch.object(io.importlib, 'import_module', return_value=fake), \
             patch.object(io.os, 'open', return_value=17), patch.object(io.os, 'close') as close:
            owner = io._ExclusiveOwner()
            with self.assertRaises(RuntimeError): io._ExclusiveOwner()
            owner.close(); owner.close(); close.assert_called_once_with(17)
            fake.flock.side_effect = BlockingIOError('owned by another process')
            with self.assertRaises(BlockingIOError): io._ExclusiveOwner()
            fake.flock.side_effect = None
            next_owner = io._ExclusiveOwner(); next_owner.close()


class ControllerTests(unittest.TestCase):
    def test_expected_controller_accepts_changed_pwmchip_number(self):
        axis = config(True)['servo']['pan']
        with patch.object(Path, 'resolve', return_value=Path('/sys/devices/platform/soc/34170000.pwm/pwm/pwmchip5')) as resolve:
            io._verify_pwm_controller(axis)
        resolve.assert_called_once_with(strict=True)

    def test_camera_lpwm_and_substring_spoof_are_rejected_before_hardware(self):
        cfg = config(True)
        for resolved in ('/sys/devices/platform/soc/34110000.lpwm/pwm/pwmchip5',
                         '/sys/devices/platform/soc/not34170000.pwm/pwm/pwmchip5'):
            with patch.object(Path, 'resolve', return_value=Path(resolved)), patch.object(io, '_ExclusiveOwner') as owner:
                with self.assertRaisesRegex(ValueError, 'controller'):
                    io.create_actuator_io(cfg)
                owner.assert_not_called()
        cfg['servo']['pan']['controller'] = '34110000.lpwm'
        with self.assertRaisesRegex(ValueError, 'controller'):
            io.create_actuator_io(cfg)


if __name__ == '__main__':
    unittest.main()
