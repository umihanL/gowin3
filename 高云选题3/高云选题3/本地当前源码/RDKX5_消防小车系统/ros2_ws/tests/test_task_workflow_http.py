"""Real HTTP ingress with local evidence fixtures; no UART or robot output."""
import unittest
import test_numbered_interaction as fixture


class TaskWorkflowHttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.NumberedHttpTests.setUpClass()
        cls.module = fixture.NumberedHttpTests.module

    setUp = fixture.NumberedHttpTests.setUp
    tearDown = fixture.NumberedHttpTests.tearDown
    post = fixture.NumberedHttpTests.post

    def no_reference(self, **changes):
        value = fixture.extension(point_reference={'confirmed': False}, can_navigate_point=False,
                                  point_navigation_reason='point_reference_required')
        value.update(changes)
        self.state.update('extension', value)

    def test_explicit_current_position_statement_can_queue_without_old_reference(self):
        self.no_reference()
        status, value = self.post(dict(request_id='declared1', text='我现在在1号点，请前往3号点后停下'), '/api/command')
        self.assertEqual(status, 202, value)
        self.assertEqual(value['action'], 'navigate_point')
        sent = []
        self.state.drain(lambda kind, data: sent.append((kind, data)), lambda: 100.)
        self.assertEqual(len(sent), 1)
        self.assertTrue(sent[0][1]['start_declared'])
        self.assertEqual((sent[0][1]['start_point'], sent[0][1]['target_point']), (1, 3))
        self.assertNotIn('completed', value)

    def test_ordinary_route_cannot_create_a_location_and_structured_endpoint_cannot_inject_declaration(self):
        self.no_reference()
        status, _ = self.post(dict(request_id='ordinary1', text='从1号点到3号点'), '/api/command')
        self.assertEqual(status, 409)
        status, _ = self.post(dict(request_id='forged1', action='navigate_point', start_point=1,
                                   target_point=3, start_declared=True))
        self.assertEqual(status, 400)

    def test_declaration_rechecks_real_link_and_permission_before_dispatch(self):
        self.no_reference()
        status, _ = self.post(dict(request_id='declared2', text='我现在在1号点，请前往3号点后停下'), '/api/command')
        self.assertEqual(status, 202)
        self.no_reference(can_confirm_point_start=False, point_navigation_reason='measured_stop_required')
        self.state.drain(lambda *_: self.fail('must not dispatch after losing stop evidence'), lambda: 100.)
        self.assertEqual(self.state.commands['declared2']['status'], 'rejected')
        self.no_reference(real_uart=False)
        status, _ = self.post(dict(request_id='declared3', text='我现在在1号点，请前往3号点后停下'), '/api/command')
        self.assertEqual(status, 409)

    def test_idle_can_start_microphone_and_submit_explicit_task_mode(self):
        self.state.update('extension', fixture.extension(task='idle', dialogue_enabled=False))
        self.state.ros['listen_ready'] = True
        self.state.update('voice', dict(stamp=100., microphone_enabled=True))
        status, value = self.post(dict(request_id='listen1', enabled=True), '/api/listen')
        self.assertEqual(status, 202, value)
        status, value = self.post(dict(request_id='begin1', text='开启任务1'), '/api/command')
        self.assertEqual(status, 202, value)
        self.assertEqual(value['action'], 'extension_vision')


if __name__ == '__main__':
    unittest.main()
