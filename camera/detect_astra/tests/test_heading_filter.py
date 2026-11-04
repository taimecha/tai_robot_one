import math
from pathlib import Path
import statistics
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from pallet_heading_filter import HeadingDisplayFilter, heading_correction_required


class HeadingFilterTest(unittest.TestCase):
    def test_reduces_jitter_but_follows_rotation(self):
        filter_ = HeadingDisplayFilter()
        raw = [1.0 + delta for _ in range(10) for delta in (-.4, .4, -.3, .3)]
        filtered = [filter_.update(angle, 1_000_000_000 + i * 500_000_000)
                    for i, angle in enumerate(raw)]
        self.assertLess(statistics.pstdev(filtered[8:]), statistics.pstdev(raw[8:]) * .5)
        # A clear rotation reaches over 90% of the new angle in one 2-FPS step.
        rotated = filter_.update(8.6, 1_000_000_000 + len(raw) * 500_000_000)
        self.assertLess(abs(rotated - 8.6), .4)

    def test_unique_timestamps_gap_and_reset(self):
        filter_ = HeadingDisplayFilter()
        filter_.update(1.4, 1_000_000_000)
        value = filter_.update(1.8, 1_500_000_000)
        for _ in range(10):
            self.assertEqual(filter_.update(1.8, 1_500_000_000), value)
        self.assertEqual(filter_.update(-1.4, 3_000_000_000), -1.4)
        filter_.reset()
        self.assertEqual(filter_.update(1.4, 3_500_000_000), 1.4)
        self.assertEqual(filter_.update(-1.4, 2_000_000_000), -1.4)
        with self.assertRaises(ValueError):
            filter_.update(math.nan, 4_000_000_000)
        self.assertIsNone(filter_.angle_deg)

    def test_orientation_tolerance_keeps_small_nonzero_angles(self):
        for angle in (-.999, -.4, 0, .4, .999):
            filter_ = HeadingDisplayFilter()
            self.assertEqual(filter_.update(angle, 1), angle)
            self.assertFalse(heading_correction_required(angle))
        for angle in (-8.6, -1, 1, 8.6):
            self.assertTrue(heading_correction_required(angle))
