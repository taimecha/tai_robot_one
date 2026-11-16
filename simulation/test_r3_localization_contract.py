"""AMCL run identity and rigid-frame invariants without ROS or robot access."""
import math
import unittest

from r3_core import compose, errors
from r3_localization_contract import CorrectionHistory, checked_status, localization_spec
from r3_pallet_obstacles import classify_ranges


class LocalizationContractTest(unittest.TestCase):
    def setUp(self):
        self.spec = dict(mode='amcl', run_id='one-trial', map_file='/maps/selected.yaml',
                         baseline_map_odom=[2., -3., .7],
                         correction_at_capture_xyyaw=[.01, -.02, .03])
        self.status = dict(ready=True, frame_id='r3_trial', run_id='one-trial',
                           map_file='/maps/selected.yaml', published_ros_s=100.,
                           baseline_map_odom=[2., -3., .7],
                           correction_xyyaw=[.02, .01, -.01])

    def test_run_and_baseline_changes_require_a_new_target(self):
        target = dict(frame_id='r3_trial', localization=self.spec)
        self.assertEqual(localization_spec(target), self.spec)
        self.assertEqual(checked_status(self.status, 100.1, self.spec), self.status)
        for update in (dict(run_id='restarted'), dict(map_file='/maps/another.yaml'),
                       dict(baseline_map_odom=[2.01, -3., .7])):
            changed = dict(self.status, **update)
            with self.subTest(update=update), self.assertRaises(ValueError):
                checked_status(changed, 100.1, self.spec)

    def test_localized_targets_cannot_silently_be_treated_as_raw_odom(self):
        self.assertIsNone(localization_spec(dict(frame_id='odom')))
        for target in (dict(frame_id='r3_trial'),
                       dict(frame_id='odom', localization=self.spec),
                       dict(frame_id='r3_trial', localization=dict(self.spec, run_id=''))):
            with self.subTest(target=target), self.assertRaises(ValueError):
                localization_spec(target)

    def test_stale_and_nonfinite_correction_is_rejected(self):
        for update in (dict(published_ros_s=99.4), dict(published_ros_s=100.2),
                       dict(ready=False), dict(correction_xyyaw=[0., math.nan, 0.]),
                       dict(baseline_map_odom=[0., 0.])):
            with self.subTest(update=update), self.assertRaises(ValueError):
                checked_status(dict(self.status, **update), 100., self.spec)

    def test_scan_uses_interpolated_capture_correction_not_latest(self):
        history = CorrectionHistory()
        history.add(dict(self.status, published_ros_s=100., correction_xyyaw=[0., 0., 0.]))
        history.add(dict(self.status, published_ros_s=100.1,
                         correction_xyyaw=[.003, 0., math.radians(.05)]))
        capture = history.at(100.05)
        self.assertAlmostEqual(capture[0], .0015)
        self.assertAlmostEqual(math.degrees(capture[2]), .025)
        raw_base, raw_sensor = (.4, -.1, .2), (.45, -.1, .2)
        # The controller and scan projection share one rigid transform at
        # capture time. The sensor mounting relative to base is preserved.
        from r3_core import inverse
        localized_base = compose(capture, raw_base)
        localized_sensor = compose(capture, raw_sensor)
        original_mount = compose(inverse(raw_base), raw_sensor)
        localized_mount = compose(inverse(localized_base), localized_sensor)
        for a, b in zip(original_mount, localized_mount):
            self.assertAlmostEqual(a, b)

    def test_history_only_holds_endpoints_for_one_bridge_period(self):
        history = CorrectionHistory()
        history.add(self.status)
        self.assertEqual(history.at(99.95), tuple(self.status['correction_xyyaw']))
        self.assertEqual(history.at(100.05), tuple(self.status['correction_xyyaw']))
        for stamp in (99.949, 100.051):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                history.at(stamp)
        history.add(dict(self.status, published_ros_s=100.3))
        with self.assertRaisesRegex(ValueError, 'gap'):
            history.at(100.15)

    def test_history_interpolates_yaw_wrap_and_rejects_reference_changes(self):
        history = CorrectionHistory()
        history.add(dict(self.status, correction_xyyaw=[0.,0.,math.radians(179.)]))
        history.add(dict(self.status, published_ros_s=100.1,
                         correction_xyyaw=[0.,0.,math.radians(-179.)]))
        self.assertAlmostEqual(abs(history.at(100.05)[2]), math.pi)
        with self.assertRaisesRegex(ValueError, 'reference changed'):
            history.add(dict(self.status, published_ros_s=100.2, run_id='new-run'))
        with self.assertRaises(ValueError):
            history.at(100.1)

    def test_history_rejects_clock_reversal_and_keeps_memory_bounded(self):
        history = CorrectionHistory()
        for i in range(1000):
            history.add(dict(self.status, published_ros_s=100.+i*.05))
        self.assertLessEqual(len(history.entries), 42)
        with self.assertRaisesRegex(ValueError, 'clock went backwards'):
            history.add(self.status)
        self.assertEqual(len(history.entries), 0)

    def test_rigid_frame_change_preserves_errors_and_obstacle_classification(self):
        # A point inside the pallet must remain exempt after the *same* frame
        # transform is applied to sensor and pallet. A nearby external object
        # must remain an obstacle; transforming only one violates this result.
        geometry = dict(base_fork=[.3655, 0., 0.], pallet_length=.355, pallet_width=.355)
        base = (.2, -.03, .08)
        sensor = (0., 0., 0.)
        target = (1.8, 0., .05)
        ranges = [1., 1.9, 2.5]
        reference = classify_ranges(ranges, 0., .001, sensor, target, geometry)
        self.assertEqual(reference[1:], (1, 2))
        before = errors(base, target, geometry)
        for correction in ((.04, -.03, .02), (2., -3., .7), (-1., .5, -1.)):
            transformed = classify_ranges(ranges, 0., .001,
                                          compose(correction, sensor),
                                          compose(correction, target), geometry)
            self.assertEqual(transformed, reference)
            after = errors(compose(correction, base), compose(correction, target), geometry)
            for key in before:
                self.assertAlmostEqual(after[key], before[key])


if __name__ == '__main__':
    unittest.main()
