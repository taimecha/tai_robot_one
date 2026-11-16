"""Parked snapshot reuse requires continuous, independent live evidence."""
from copy import deepcopy
import math
import unittest

from r3_stationary_localization import StationaryEvidence


FIT = {'ready': True, 'inlier_ratio': .9, 'median_distance_m': .025}
POSE = (1., 2., .3)
STOPPED = (0., 0., 0.)


class StationaryEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.guard = StationaryEvidence()
        self.metrics = {
            'converged': True,
            'covariance_6x6': [.0025 if i in (0, 7) else 0. for i in range(36)],
            'yaw_std_rad': math.radians(2.),
        }
        self.guard.update_odom(POSE, STOPPED, 100., 100.)
        self.assertTrue(self.guard.capture(99.9, POSE, self.metrics))

    def support(self, start=100., count=5, step=.25, pose=POSE):
        for i in range(count):
            now = start + i * step
            self.guard.update_odom(pose, STOPPED, now, now)
            self.guard.observe_scan(now, FIT, now)
        return now

    def test_fresh_distinct_scans_must_span_one_second(self):
        for stamp in (100., 100.25, 100.5, 100.75):
            self.guard.update_odom(POSE, STOPPED, stamp, stamp)
            self.guard.observe_scan(stamp, FIT, stamp)
            self.assertFalse(self.guard.ready(stamp)[0])
        self.guard.update_odom(POSE, STOPPED, 101., 101.)
        self.guard.observe_scan(101., FIT, 101.)
        self.assertTrue(self.guard.ready(101.)[0])

    def test_separately_checked_raw_stream_allows_score_gap_to_rebuild_evidence(self):
        self.support()
        self.guard.update_odom(POSE,STOPPED,101.25,101.25)
        self.guard.update_odom(POSE,STOPPED,101.5,101.5)
        self.guard.update_odom(POSE,STOPPED,101.75,101.75)
        self.guard.observe_scan(101.75,FIT,101.75,check_capture_continuity=False)
        self.assertIsNone(self.guard.fault)
        self.assertFalse(self.guard.ready(101.75)[0])
        for stamp in (102.,102.25,102.5,102.75):
            self.guard.update_odom(POSE,STOPPED,stamp,stamp)
            self.guard.observe_scan(stamp,FIT,stamp,check_capture_continuity=False)
        self.assertTrue(self.guard.ready(102.75)[0])
        self.assertEqual(self.guard.observation_stamp,99.9)

    def test_repeated_callback_for_one_scan_does_not_add_evidence(self):
        for _ in range(20):
            self.guard.observe_scan(100., FIT, 100.1)
        self.assertEqual(list(self.guard.scans), [100.])
        self.assertFalse(self.guard.ready(100.1)[0])

    def test_continuous_support_reuses_old_stamp_without_covariance_change(self):
        original_metrics = deepcopy(self.metrics)
        now = self.support(count=201)
        self.assertTrue(self.guard.ready(now)[0])
        self.assertEqual(self.guard.observation_stamp, 99.9)
        self.assertEqual(self.guard.anchor, POSE)
        self.assertEqual(self.metrics, original_metrics)
        self.assertEqual(self.guard.metrics, original_metrics)
        self.assertLessEqual(len(self.guard.scans), 7)

    def test_velocity_fault_is_latched_after_robot_stops(self):
        self.support()
        self.guard.update_odom(POSE, (.006, 0., 0.), 101.1, 101.1)
        self.assertFalse(self.guard.ready(101.1)[0])
        self.support(start=101.25)
        self.assertFalse(self.guard.ready(102.25)[0])
        self.assertIn('motion', self.guard.fault)
        self.assertFalse(self.guard.capture(99.9, POSE, self.metrics))
        self.assertFalse(self.guard.capture(99.8, POSE, self.metrics))
        self.assertTrue(self.guard.capture(102.25, POSE, self.metrics))
        self.support(start=102.5)
        self.assertTrue(self.guard.ready(103.5)[0])

    def test_displacement_and_turn_each_invalidate_even_if_twist_is_zero(self):
        for pose in ((1.006, 2., .3), (1., 2., .3 + math.radians(.6))):
            with self.subTest(pose=pose):
                guard = StationaryEvidence()
                guard.update_odom(POSE, STOPPED, 100., 100.)
                guard.capture(99.9, POSE)
                guard.update_odom(pose, STOPPED, 100.1, 100.1)
                self.assertIn('displacement', guard.fault)
                guard.update_odom(POSE, STOPPED, 100.2, 100.2)
                self.assertIsNotNone(guard.fault)

    def test_angle_wrap_and_small_odom_noise_do_not_claim_motion(self):
        guard = StationaryEvidence()
        anchor = (1., 2., math.pi - .002)
        pose = (1.001, 1.999, -math.pi + .002)
        guard.update_odom(anchor, STOPPED, 100., 100.)
        guard.capture(99.9, anchor)
        for i in range(5):
            stamp = 100. + i * .25
            guard.update_odom(pose, STOPPED, stamp, stamp)
            guard.observe_scan(stamp, FIT, stamp)
        self.assertIsNone(guard.fault)
        self.assertTrue(guard.ready(101.)[0])

    def test_invalid_or_future_odom_latches_fault(self):
        for stamp, now, pose in ((100.2, 100., POSE),
                                 (math.nan, 100., POSE),
                                 (100., math.inf, POSE),
                                 (100., 100., (math.nan, 2., .3))):
            with self.subTest(stamp=stamp, now=now, pose=pose):
                self.setUp()
                self.guard.update_odom(pose, STOPPED, stamp, now)
                self.assertIsNotNone(self.guard.fault)
                self.support()
                self.assertFalse(self.guard.ready(101.)[0])

    def test_invalid_or_future_scan_latches_fault(self):
        for stamp, now in ((100.2, 100.),
                            (math.nan, 100.), (100., math.inf)):
            with self.subTest(stamp=stamp, now=now):
                self.setUp()
                self.guard.observe_scan(stamp, FIT, now)
                self.assertIsNotNone(self.guard.fault)
                self.support()
                self.assertFalse(self.guard.ready(101.)[0])

    def test_late_queued_odom_pauses_then_recovers_with_new_scan_window(self):
        self.support()
        self.assertTrue(self.guard.ready(101.)[0])
        # Capture stamps remain continuous even though one queued callback is
        # 373 ms late. Its original stamp must be retained, never refreshed.
        self.guard.update_odom(POSE, STOPPED, 101.2, 101.573)
        self.assertIsNone(self.guard.fault)
        self.assertEqual(self.guard.odom_stamp, 101.2)
        self.assertEqual(list(self.guard.scans), [])
        self.assertFalse(self.guard.ready(101.573)[0])
        # LiDAR capture continuity is independently maintained during the
        # callback delay; stale odom still keeps the overall gate closed.
        self.guard.observe_scan(101.4, FIT, 101.573)
        self.assertFalse(self.guard.ready(101.573)[0])
        for stamp in (101.6, 101.85, 102.1, 102.35):
            self.guard.update_odom(POSE, STOPPED, stamp, stamp)
            self.guard.observe_scan(stamp, FIT, stamp)
            self.assertFalse(self.guard.ready(stamp)[0])
        self.guard.update_odom(POSE, STOPPED, 102.6, 102.6)
        self.guard.observe_scan(102.6, FIT, 102.6)
        self.assertTrue(self.guard.ready(102.6)[0])
        self.assertEqual(self.guard.observation_stamp, 99.9)

    def test_late_queued_scan_tracks_capture_time_without_adding_confidence(self):
        self.support()
        self.guard.observe_scan(101.2, FIT, 101.8)
        self.assertIsNone(self.guard.fault)
        self.assertEqual(self.guard.scan_stamp, 101.2)
        self.assertEqual(list(self.guard.scans), [])
        self.assertFalse(self.guard.ready(101.8)[0])
        # Drain queued captures with uninterrupted stamps. None may count as a
        # fresh scan, and none alone proves a physical sensor stream gap.
        self.guard.update_odom(POSE, STOPPED, 101.4, 101.8)
        self.guard.observe_scan(101.4, FIT, 102.)
        self.guard.update_odom(POSE, STOPPED, 101.6, 102.)
        self.guard.observe_scan(101.6, FIT, 102.2)
        self.guard.update_odom(POSE, STOPPED, 102., 102.2)
        self.assertIsNone(self.guard.fault)
        self.assertEqual(list(self.guard.scans), [])
        for stamp in (102., 102.25, 102.5, 102.75):
            self.guard.update_odom(POSE, STOPPED, stamp, stamp)
            self.guard.observe_scan(stamp, FIT, stamp)
            self.assertFalse(self.guard.ready(stamp)[0])
        self.guard.update_odom(POSE, STOPPED, 103., 103.)
        self.guard.observe_scan(103., FIT, 103.)
        self.assertTrue(self.guard.ready(103.)[0])
        self.assertEqual(self.guard.observation_stamp, 99.9)

    def test_motion_in_late_queued_odom_still_latches_fault(self):
        self.support()
        self.guard.update_odom(POSE, (.006, 0., 0.), 101.2, 101.573)
        self.assertIn('motion', self.guard.fault)
        self.support(start=101.25)
        self.assertFalse(self.guard.ready(102.25)[0])

    def test_absence_then_new_capture_with_real_gap_latches_fault(self):
        for stream in ('odom', 'scan'):
            with self.subTest(stream=stream):
                self.setUp()
                self.support()
                self.assertFalse(self.guard.ready(102.)[0])
                if stream == 'odom':
                    self.guard.update_odom(POSE, STOPPED, 102., 102.)
                else:
                    self.guard.observe_scan(102., FIT, 102.)
                self.assertIn('gap', self.guard.fault)
                self.support(start=102.25)
                self.assertFalse(self.guard.ready(103.25)[0])

    def test_late_message_with_actual_capture_gap_still_latches_fault(self):
        for stream in ('odom', 'scan'):
            with self.subTest(stream=stream):
                self.setUp()
                self.support()
                if stream == 'odom':
                    self.guard.update_odom(POSE, STOPPED, 101.75, 102.2)
                else:
                    self.guard.observe_scan(101.75, FIT, 102.4)
                self.assertIn('gap', self.guard.fault)
                self.assertFalse(self.guard.ready(102.4)[0])

    def test_stream_gap_and_clock_reversal_latch_fault(self):
        for stream in ('odom', 'scan'):
            for stamp in (101.75, 100.9):
                with self.subTest(stream=stream, stamp=stamp):
                    self.setUp()
                    self.support()
                    if stream == 'odom':
                        self.guard.update_odom(POSE, STOPPED, stamp, stamp)
                    else:
                        self.guard.observe_scan(stamp, FIT, stamp)
                    self.assertIsNotNone(self.guard.fault)
                    self.assertFalse(self.guard.ready(stamp)[0])

    def test_current_stale_streams_cannot_validate_old_snapshot(self):
        self.support()
        self.assertFalse(self.guard.ready(101.3)[0])  # Odom older than .25 s.
        self.guard.update_odom(POSE, STOPPED, 101.4, 101.4)
        self.assertFalse(self.guard.ready(101.6)[0])  # Scan older than .5 s.

    def test_bad_scan_clears_entire_confidence_window(self):
        for bad_fit in ({'ready': False},
                        {'ready': True, 'inlier_ratio': .64, 'median_distance_m': .025},
                        {'ready': True, 'inlier_ratio': .9, 'median_distance_m': .13},
                        {'ready': True, 'inlier_ratio': math.nan, 'median_distance_m': .025}):
            with self.subTest(fit=bad_fit):
                self.setUp()
                self.support()
                self.guard.update_odom(POSE, STOPPED, 101.25, 101.25)
                self.guard.observe_scan(101.25, bad_fit, 101.25)
                self.assertFalse(self.guard.ready(101.25)[0])
                self.assertEqual(list(self.guard.scans), [])
                self.support(start=101.5, count=4)
                self.assertFalse(self.guard.ready(102.25)[0])
                self.guard.update_odom(POSE, STOPPED, 102.5, 102.5)
                self.guard.observe_scan(102.5, FIT, 102.5)
                self.assertTrue(self.guard.ready(102.5)[0])


if __name__ == '__main__':
    unittest.main()
