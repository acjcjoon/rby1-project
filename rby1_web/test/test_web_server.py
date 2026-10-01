"""Exercise physical-motion boundaries without requiring ROS or hardware."""
import http.client
import json
from pathlib import Path
import re
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rby1_web.web_server import CommandMailbox, create_server


class MailboxTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.box = CommandMailbox(clock=lambda: self.now)
        self.box.update_state({'stream_enabled': True})

    def submit(self, seq=1, session='a', operation='set_velocity', gesture='hold1', **args):
        self.box.submit({'seq': seq, 'session': session,
                         'operation': operation, 'arguments': args, 'gesture': gesture})

    def test_web_watchdog_is_not_refreshed_by_ros_publishing(self):
        self.submit(vx=0.1)
        self.assertEqual(self.box.drain()[0][0], 'set_velocity')
        self.now += 0.2
        self.assertEqual(self.box.drain()[0][0], 'set_velocity')
        self.now += 0.11
        self.assertEqual(self.box.drain(), [('stop', {})])
        self.assertEqual(self.box.drain(), [])

    def test_old_motion_cannot_override_newer_stop(self):
        self.submit(seq=5, vx=0.1)
        self.submit(seq=7, operation='stop')
        with self.assertRaises(ValueError):
            self.submit(seq=6, vx=0.1)
        self.assertEqual(self.box.drain(), [('stop', {})])

    def test_one_browser_drives_but_any_browser_can_stop(self):
        self.submit(vx=0.1)
        with self.assertRaises(ValueError):
            self.submit(session='b', vx=0.2)
        self.submit(session='b', seq=2, operation='stop')
        self.assertEqual(self.box.drain(), [('stop', {})])

    def test_backend_loss_drops_motion_and_rejects_new_commands(self):
        self.submit(vx=0.1)
        self.now += 1.1
        self.assertEqual(self.box.drain(), [('stop', {})])
        with self.assertRaises(ValueError):
            self.submit(seq=2, vx=0.1)
        self.submit(seq=3, operation='stop')

    def test_global_stop_requires_a_new_button_press(self):
        self.submit(vx=0.1)
        self.submit(session='b', operation='stop')
        with self.assertRaises(ValueError):
            self.submit(seq=2, vx=0.1)
        self.assertEqual(self.box.drain(), [('stop', {})])
        self.submit(seq=3, gesture='hold2', vx=0.1)
        self.assertEqual(self.box.drain()[0][0], 'set_velocity')

    def test_speed_limits_nonfinite_and_unsupported_operations(self):
        for args in ({'vx': 0.21}, {'wz': -0.41}, {'vx': float('nan')},
                     {'vx': float('inf')}, {'vx': True}, {'vx': '0.1'}):
            with self.assertRaises(ValueError):
                self.submit(**args)
        with self.assertRaises(ValueError):
            self.submit(operation='move_joint_group')

    def test_stop_cancels_pending_setup_and_stale_actions_expire(self):
        self.submit(operation='stream_on')
        self.submit(seq=2, operation='stop')
        self.assertEqual(self.box.drain(), [('stop', {})])
        self.submit(seq=3, operation='stream_on')
        self.now += 1.1
        self.box.update_state({})
        self.assertEqual(self.box.drain(), [])

    def test_shutdown_commands_stop_before_disabling(self):
        self.submit(vx=0.1)
        self.submit(seq=2, operation='disable')
        self.assertEqual(self.box.drain(), [
            ('stop', {}), ('request_control_manager', {'command': 'disable'})])

    def test_strafe_is_preserved_and_combined_speed_is_bounded(self):
        self.submit(vy=0.1)
        self.assertEqual(self.box.drain()[0][1]['vy'], 0.1)
        with self.assertRaises(ValueError):
            self.submit(seq=2, vx=0.2, vy=0.2)


class OperatorMailboxTests(unittest.TestCase):
    submit = MailboxTests.submit

    def setUp(self):
        self.now = 10.0
        self.box = CommandMailbox(clock=lambda: self.now, operator_enabled=True)
        self.box.update_state({'stream_enabled': True})

    def test_manual_motion_does_not_require_tcp_tracking_or_pose(self):
        self.box.update_operator(bridge={'connected': False, 'tracking_ok': False}, pose=None)
        self.submit(vx=0.1)
        self.assertEqual(self.box.drain()[0], ('set_velocity', {'vx': 0.1, 'vy': 0.0, 'wz': 0.0}))

    def test_navigation_heartbeat_is_owned_by_originating_browser(self):
        self.submit(operation='navigate', name='Point 1')
        self.assertEqual(self.box.drain(), [('stop', {}), ('navigate', {'name': 'Point 1'})])
        self.now += 0.8
        self.box.update_state({})
        self.box.status('a')
        self.now += 0.8
        self.box.update_state({})
        self.box.status('b')
        self.assertEqual(self.box.drain(), [])
        self.now += 0.8
        self.box.update_state({})
        self.assertEqual(self.box.drain(), [('stop', {}), ('nav_cancel', {})])

    def test_manual_takeover_cancels_navigation_before_motion(self):
        self.submit(operation='navigate', name='Point 1')
        self.box.drain()
        self.submit(seq=2, vx=0.1)
        self.assertEqual([operation for operation, _ in self.box.drain()],
                         ['nav_cancel', 'set_velocity'])

    def test_any_browser_can_cancel_even_with_backend_offline(self):
        self.submit(operation='navigate', name='Point 1')
        self.box.drain()
        self.now += 2
        self.submit(session='b', operation='nav_cancel')
        self.assertEqual(self.box.drain(), [('stop', {}), ('nav_cancel', {})])

    def test_manual_takeover_discards_a_not_yet_dispatched_goal(self):
        self.submit(operation='navigate', name='Point 1')
        self.submit(seq=2, vx=0.1)
        commands = [operation for operation, _ in self.box.drain()]
        self.assertNotIn('navigate', commands)
        self.assertEqual(commands, ['stop', 'nav_cancel', 'set_velocity'])


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.box = CommandMailbox()
        self.box.update_state({})
        self.server = create_server(('127.0.0.1', 0), self.box)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = http.client.HTTPConnection('127.0.0.1', self.server.server_port)

    def tearDown(self):
        self.client.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, method, path, body=None, headers=None):
        self.client.request(method, path, body=body, headers=headers or {})
        response = self.client.getresponse()
        return response.status, response.read().decode('utf-8'), dict(response.getheaders())

    def test_real_http_page_state_token_and_motion(self):
        code, page, headers = self.request('GET', '/')
        self.assertEqual(code, 200)
        self.assertNotIn('__API_TOKEN__', page)
        self.assertEqual(headers['X-Frame-Options'], 'DENY')
        token = re.search(r'const token=("[^"]+")', page).group(1)
        body = json.dumps({'session': 'browser', 'seq': 1,
                           'operation': 'set_velocity', 'arguments': {'vx': 0.1},
                           'gesture': 'hold1'})
        self.assertEqual(self.request('POST', '/api/command', body)[0], 403)
        self.assertEqual(self.request('POST', '/api/command', body,
                                     {'X-RBY1-Token': json.loads(token)})[0], 202)
        self.assertEqual(self.box.drain()[0][0], 'set_velocity')
        code, state, _ = self.request('GET', '/api/state')
        self.assertEqual(code, 200)
        self.assertTrue(json.loads(state)['connected'])
        self.assertEqual(self.request('GET', '/not-a-file')[0], 404)


if __name__ == '__main__':
    unittest.main()
