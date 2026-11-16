"""A transform check pauses commands, then resumes the same native goal path."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from r3_navigation_recovery_gate import RecoveryInterlock, NavigationRecoveryGate, ZERO


class RecoveryGateTest(unittest.TestCase):
    def status(self, stamp=100., hold=False, **updates):
        status = dict(published_ros_s=stamp, run_id='this-run', map_file='/map/edited.yaml',
                      navigation_monitor=True, ready=True,
                      navigation_recovery=dict(hold_commands=hold))
        status.update(updates)
        return status

    def test_native_commands_continue_despite_delayed_or_unready_python_report(self):
        gate = RecoveryInterlock()
        status = self.status(ready=False)
        before = copy.deepcopy(status)
        gate.update(status, 100.)
        for i in range(161):
            now = 100.+i*.05
            self.assertEqual(gate.filter((.18, 0., .30), now, now)[0], (.18, 0., .30))
        self.assertEqual(status, before)

    def test_explicit_hold_brakes_then_fresh_release_resumes_without_action_operations(self):
        gate = RecoveryInterlock()
        gate.update(self.status(), 100.)
        self.assertEqual(gate.filter((.12, 0., .2), 100., 100.)[0], (.12, 0., .2))
        gate.update(self.status(100.1, hold=True), 100.1)
        self.assertEqual(gate.filter((.12, 0., .2), 100.1, 100.1)[0], ZERO)
        gate.update(self.status(100.8, hold=False), 100.8)
        self.assertEqual(gate.filter((.12, 0., .2), 100.8, 100.8)[0], (.12, 0., .2))

    def test_hold_cannot_expire_itself_when_validation_reports_stop(self):
        gate = RecoveryInterlock()
        gate.update(self.status(hold=True), 100.)
        self.assertEqual(gate.filter((.1, 0., .2), 130., 130.)[0], ZERO)
        self.assertTrue(gate.hold)

    def test_delayed_out_of_order_future_or_untyped_release_cannot_reopen_gate(self):
        for release, now in (
                (self.status(99.9), 100.1),
                (self.status(100.1), 101.),
                (self.status(102.), 101.),
                (self.status(100.1, navigation_recovery={'hold_commands': 'false'}), 100.1),
                (self.status(100.1, navigation_recovery=None), 100.1)):
            with self.subTest(release=release, now=now):
                gate = RecoveryInterlock()
                gate.update(self.status(hold=True), 100.)
                gate.update(release, now)
                self.assertEqual(gate.filter((.1, 0., .2), now, now)[0], ZERO)
                gate.update(self.status(now+.1), now+.1)
                self.assertEqual(gate.filter((.1, 0., .2), now+.1, now+.1)[0], (.1, 0., .2))

    def test_identity_role_and_real_integrity_faults_remain_closed_after_release(self):
        for change in (dict(run_id='new-run'), dict(map_file='/map/different.yaml'),
                       dict(navigation_monitor=False), dict(localization_fault='map removed')):
            with self.subTest(change=change):
                gate = RecoveryInterlock()
                gate.update(self.status(), 100.)
                gate.update(self.status(100.1, **change), 100.1)
                gate.update(self.status(100.2), 100.2)
                self.assertEqual(gate.filter((.1, 0., .2), 100.2, 100.2)[0], ZERO)
                self.assertIsNotNone(gate.fault)

    def test_stale_future_and_nonfinite_commands_never_pass(self):
        gate = RecoveryInterlock()
        for command, stamp, now in (((.1, 0., .2), 100., 100.501),
                                    ((.1, 0., .2), 100.1, 100.),
                                    ((float('nan'), 0., .2), 100., 100.)):
            self.assertEqual(gate.filter(command, stamp, now)[0], ZERO)

    def test_400ms_controller_gap_retains_native_timeout_unless_recovery_is_active(self):
        gate = RecoveryInterlock()
        gate.update(self.status(), 100.)
        self.assertEqual(gate.filter((.1, 0., .2), 100., 100.4)[0], (.1, 0., .2))
        gate.update(self.status(100.4, hold=True), 100.4)
        self.assertEqual(gate.filter((.1, 0., .2), 100., 100.4)[0], ZERO)

    def test_runtime_publishes_zero_during_hold_then_same_input_after_release(self):
        gate = RecoveryInterlock()
        node = SimpleNamespace(interlock=gate, command=(.1, 0., .2),
            command_received_at=100., now=lambda: 100., output=Mock(),
            last_reason=None, get_logger=Mock(return_value=Mock()))
        gate.update(self.status(hold=True), 100.)
        NavigationRecoveryGate.tick(node)
        message = node.output.publish.call_args.args[0]
        self.assertEqual((message.linear.x, message.linear.y, message.angular.z), ZERO)
        gate.update(self.status(100.1), 100.1)
        node.now = lambda: 100.1
        NavigationRecoveryGate.tick(node)
        message = node.output.publish.call_args.args[0]
        self.assertEqual((message.linear.x, message.linear.y, message.angular.z), (.1, 0., .2))


class RecoveryGateLaunchTest(unittest.TestCase):
    def test_gate_preserves_native_smoother_and_collision_chain_with_one_output_owner(self):
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location('r3_nav_gate_launch',
            root/'camera/detect_astra/launch/r3_navigation.launch.py')
        launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch)
        with patch.object(launch, 'Node', side_effect=lambda **kw: dict(kind='node', **kw)), \
                patch.object(launch, 'ExecuteProcess', side_effect=lambda **kw: dict(kind='process', **kw)):
            actions = launch.navigation_actions(None)
        smoother = next(action for action in actions if action.get('package') == 'nav2_velocity_smoother')
        self.assertIn(('cmd_vel', 'cmd_vel_nav_checked'), smoother['remappings'])
        gate = next(action for action in actions if action['kind'] == 'process')
        self.assertEqual(Path(gate['cmd'][1]), root/'simulation/r3_navigation_recovery_gate.py')
        params = launch.navigation_parameters()
        self.assertEqual(params['collision_monitor']['ros__parameters']['cmd_vel_in_topic'], '/cmd_vel_smoothed')
        self.assertEqual(params['collision_monitor']['ros__parameters']['cmd_vel_out_topic'], '/cmd_vel_safe')
        self.assertEqual(params['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance'], 15.)


if __name__ == '__main__':
    unittest.main()
