"""A rejected map update pauses one native goal and can recover in place.

The injected outlier is synthetic. The November 16 telemetry omitted the exact
TF/AMCL pair that caused cancellation, so these cases test the recovery contract
without claiming to replay an unrecorded localization jump.
"""
import copy
import math
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from r3_amcl_bridge import TrialCorrection
from r3_core import compose, inverse, wrap
from r3_recognition_goal import RecognitionGoal


BASELINE = (2.8813592944, 1.6224353343, -1.7646325614)
CHANGED = (BASELINE[0]+.20, BASELINE[1]+.01, BASELINE[2]+math.radians(6.))


def covariance():
    values = [0.]*36
    values[0] = values[7] = .015**2
    values[35] = math.radians(.5)**2
    return values


def fit(stamp, **updates):
    result = dict(ready=True, inlier_ratio=.9, median_distance_m=.01, scan_stamp_s=stamp)
    result.update(updates)
    return result


def initialized(*, recover=True):
    correction = TrialCorrection(bound_total=False, recover_jumps=recover)
    for stamp in (100., 100.5, 101.):
        correction.accept(BASELINE, covariance(), stamp, stamp, navigation_scan=fit(stamp))
    if correction.baseline is None:
        raise AssertionError('Fixture failed to establish the original accepted baseline')
    return correction


class NavigationCorrectionRecoveryTest(unittest.TestCase):
    def assertPoseClose(self, actual, expected):
        self.assertAlmostEqual(actual[0], expected[0])
        self.assertAlmostEqual(actual[1], expected[1])
        self.assertAlmostEqual(wrap(actual[2]-expected[2]), 0.)

    def begin_recovery(self):
        correction = initialized()
        accepted = correction.accept(CHANGED, covariance(), 101.2, 101.2,
                                     navigation_scan=fit(101.2))
        self.assertFalse(accepted)
        self.assertIsNone(correction.fault)
        self.assertFalse(correction.measurement_ok)
        self.assertTrue(correction.navigation_recovery['hold_commands'])
        self.assertPoseClose(correction.last_transform, BASELINE)
        self.assertPoseClose(correction.baseline, BASELINE)
        return correction

    def test_large_update_is_held_in_navigation_without_reanchoring_or_latching(self):
        correction = self.begin_recovery()
        self.assertEqual(correction.amcl_stamp, 101.)
        self.assertPoseClose(correction.target, (0., 0., 0.))
        self.assertEqual(correction.applied, (0., 0., 0.))
        event = correction.last_jump
        self.assertEqual(event['previous_map_odom_xyyaw'], list(BASELINE))
        self.assertEqual(event['candidate_map_odom_xyyaw'], list(CHANGED))
        self.assertEqual(event['previous_amcl_stamp_s'], 101.)
        self.assertEqual(event['candidate_amcl_stamp_s'], 101.2)
        self.assertAlmostEqual(event['translation_m'], math.hypot(.20, .01))
        self.assertAlmostEqual(event['yaw_deg'], 6.)
        self.assertEqual(event['observed_ros_s'], 101.2)

    def test_three_independent_matching_scans_can_qualify_the_same_paired_posterior(self):
        correction = self.begin_recovery()
        for scan_stamp, accepted in ((101.45, False), (101.7, True)):
            result = correction.accept(CHANGED, covariance(), 101.2, scan_stamp,
                                       navigation_scan=fit(scan_stamp))
            self.assertEqual(result, accepted)
        self.assertIsNone(correction.fault)
        self.assertTrue(correction.measurement_ok)
        self.assertFalse(correction.navigation_recovery['hold_commands'])
        self.assertEqual(correction.amcl_stamp, 101.2)  # Original capture was not renewed.
        self.assertPoseClose(correction.last_transform, CHANGED)
        self.assertPoseClose(correction.baseline, BASELINE)
        self.assertPoseClose(correction.target, compose(inverse(BASELINE), CHANGED))

    def test_duplicate_scan_messages_do_not_complete_the_recovery_window(self):
        correction = self.begin_recovery()
        for now in (101.3, 101.5, 101.7, 101.9):
            self.assertFalse(correction.accept(CHANGED, covariance(), 101.2, now,
                navigation_scan=fit(101.2)))
        self.assertTrue(correction.navigation_recovery['hold_commands'])
        self.assertPoseClose(correction.last_transform, BASELINE)

    def test_three_captures_still_need_half_a_second_of_independent_scan_evidence(self):
        correction = self.begin_recovery()
        for stamp in (101.3, 101.4, 101.69):
            self.assertFalse(correction.accept(CHANGED, covariance(), 101.2, stamp,
                navigation_scan=fit(stamp)))
        self.assertTrue(correction.navigation_recovery['hold_commands'])
        self.assertTrue(correction.accept(CHANGED, covariance(), 101.2, 101.7,
            navigation_scan=fit(101.7)))

    def test_bad_or_stale_scan_cannot_release_a_navigation_hold(self):
        for bad_fit in (fit(101.7, ready=False), fit(101.7, inlier_ratio=.64),
                        fit(101.7, median_distance_m=.051), fit(100.7)):
            with self.subTest(scan=bad_fit):
                correction = self.begin_recovery()
                self.assertFalse(correction.accept(CHANGED, covariance(), 101.2, 101.45,
                    navigation_scan=fit(101.45)))
                self.assertFalse(correction.accept(CHANGED, covariance(), 101.2, 101.7,
                    navigation_scan=bad_fit))
                self.assertTrue(correction.navigation_recovery['hold_commands'])
                self.assertPoseClose(correction.last_transform, BASELINE)

    def test_old_amcl_capture_cannot_be_made_fresh_by_new_matching_scans(self):
        correction = self.begin_recovery()
        for stamp in (103.21, 103.5, 104.):
            self.assertFalse(correction.accept(CHANGED, covariance(), 101.2, stamp,
                navigation_scan=fit(stamp)))
        self.assertTrue(correction.navigation_recovery['hold_commands'])
        self.assertPoseClose(correction.last_transform, BASELINE)
        self.assertEqual(correction.amcl_stamp, 101.)

    def test_strict_trial_correction_still_latches_the_identical_outlier(self):
        correction = initialized(recover=False)
        self.assertFalse(correction.accept(CHANGED, covariance(), 101.2, 101.2,
            navigation_scan=fit(101.2)))
        self.assertIn('jumped', correction.fault)
        for stamp in (101.45, 101.7, 101.9):
            self.assertFalse(correction.accept(CHANGED, covariance(), 101.2, stamp,
                navigation_scan=fit(stamp)))
        self.assertIn('jumped', correction.fault)


class SameNativeGoalRecoveryTest(unittest.TestCase):
    def probe(self):
        return SimpleNamespace(map_signature='unchanged-grid',
            navigation_map_signature='unchanged-grid', navigation_run_id='same-run',
            selected_map_file='/test/map.yaml',
            stationary_status=dict(run_id='same-run', map_file='/test/map.yaml',
                navigation_monitor=True, localization_fault=None),
            stationary_snapshot=dict(amcl_stamp_s=101., map_odom_xyyaw=list(BASELINE)),
            navigation_observation_stamp=101., last_map_odom=BASELINE,
            navigation_action_active=True,
            goal_handle=SimpleNamespace(goal_id='same-A-action', cancel_goal_async=Mock()),
            client=SimpleNamespace(send_goal_async=Mock()),
            current_pose=Mock(side_effect=AssertionError('parked arrival check during transit')))

    def test_held_and_recovered_transform_keeps_the_original_action_and_future(self):
        correction = initialized()
        probe = self.probe()
        probe.monitor_navigation_session = MethodType(RecognitionGoal.monitor_navigation_session, probe)
        callbacks = [0]
        original_handle = probe.goal_handle
        future = SimpleNamespace(done=lambda: callbacks[0] >= 4,
                                 result=lambda: 'original A action completed')
        probe.result_future = future

        def spin():
            callbacks[0] += 1
            if callbacks[0] <= 3:
                now = (101.2, 101.45, 101.7)[callbacks[0]-1]
                accepted = correction.accept(CHANGED, covariance(), 101.2, now,
                                             navigation_scan=fit(now))
                probe.stationary_status['navigation_recovery'] = copy.deepcopy(
                    correction.navigation_recovery)
                if accepted:
                    probe.stationary_snapshot = dict(amcl_stamp_s=correction.amcl_stamp,
                        map_odom_xyyaw=list(correction.last_transform))

        probe.spin_once = spin
        with patch('r3_recognition_goal.rclpy.ok', return_value=True):
            result = RecognitionGoal.await_future(probe, future, time.monotonic()+5., True)
        self.assertEqual(result, 'original A action completed')
        self.assertIs(probe.goal_handle, original_handle)
        self.assertIs(probe.result_future, future)
        self.assertFalse(probe.stationary_status['navigation_recovery']['hold_commands'])
        self.assertEqual(probe.navigation_observation_stamp, 101.2)
        self.assertEqual(probe.last_map_odom, CHANGED)
        probe.goal_handle.cancel_goal_async.assert_not_called()
        probe.client.send_goal_async.assert_not_called()
        probe.current_pose.assert_not_called()

    def test_recovery_state_cannot_hide_a_map_session_role_or_latched_fault_change(self):
        changes = (('run_id', 'different-run'), ('map_file', '/test/different.yaml'),
                   ('navigation_monitor', False), ('localization_fault', 'actual odom reset'))
        for key, value in changes:
            with self.subTest(field=key):
                probe = self.probe()
                probe.stationary_status.update(navigation_recovery=dict(hold_commands=False,
                    reason='candidate correction validated by fresh distinct LiDAR scans'))
                probe.stationary_status[key] = value
                with self.assertRaises(ValueError):
                    RecognitionGoal.monitor_navigation_session(probe)
        probe = self.probe()
        probe.map_signature = 'different-grid'
        with self.assertRaisesRegex(ValueError, 'occupancy map changed'):
            RecognitionGoal.monitor_navigation_session(probe)


if __name__ == '__main__':
    unittest.main()
