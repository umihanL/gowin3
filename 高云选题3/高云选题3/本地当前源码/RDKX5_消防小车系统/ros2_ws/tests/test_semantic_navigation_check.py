"""The historical check filename now validates rules without model or hardware I/O."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('check_rule_navigation',Path(__file__).resolve().parents[1]/'tools/check_semantic_navigation.py')
check=importlib.util.module_from_spec(spec);spec.loader.exec_module(check)

class RuleNavigationCheckTests(unittest.TestCase):
    def invoke(self,*args):
        output=io.StringIO()
        with patch('urllib.request.build_opener',side_effect=AssertionError('rule check used network')):
            with contextlib.redirect_stdout(output):code=check.main(list(args))
        return code,json.loads(output.getvalue())

    def test_ready_is_diagnostic_with_no_model_or_command(self):
        code,report=self.invoke()
        self.assertEqual(code,0,report)
        self.assertFalse(report['model_called']);self.assertTrue(report['read_only'])
        self.assertFalse(report['hardware_commands_sent'])
        self.assertEqual(report['backend'],'explicit_navigation_rules')
        self.assertEqual(report['structured']['start_heading_deg'],180.)
        self.assertEqual(report['structured']['start_point'],8)
        self.assertEqual(report['structured']['target_point'],4)
        self.assertNotIn('command',report)

    def test_config_is_compatibility_only_not_read(self):
        code,report=self.invoke('--config','/no/model/config/exists.json')
        self.assertEqual(code,0);self.assertFalse(report['model_called'])

    def test_missing_physical_context_requires_clarification(self):
        for text in ('前往4号点','从8号点前往4号点'):
            code,report=self.invoke('--text',text)
            self.assertEqual(code,1);self.assertEqual(report['status'],'clarification')

    def test_invalid_command_returns_error_not_model_fallback(self):
        code,report=self.invoke('--text','从8号点前往10号点')
        self.assertEqual(code,2);self.assertFalse(report['model_called'])

if __name__=='__main__':unittest.main()
