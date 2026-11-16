"""Whole-map matching and held-out validation on independently raycast scans."""
import math
from types import SimpleNamespace as NS
import unittest

import numpy as np

from r3_global_scan_match import GlobalScanMatcher


def grid_message(data, yaw=0.):
    return NS(info=NS(width=data.shape[1], height=data.shape[0], resolution=.05,
        origin=NS(position=NS(x=-2., y=1.), orientation=NS(x=0., y=0.,
            z=math.sin(yaw/2), w=math.cos(yaw/2)))), data=data.ravel().tolist())


def raycast(data, base, mounting=(0., 0., 0.)):
    """Simulated first wall intersection independent of the matcher scorer."""
    ranges = []
    c, s = math.cos(base[2]), math.sin(base[2])
    sx = base[0]+c*mounting[0]-s*mounting[1]
    sy = base[1]+s*mounting[0]+c*mounting[1]
    for angle in np.arange(360)*math.pi/180-math.pi:
        direction = base[2]+mounting[2]+angle
        distance = .1
        while distance < 8.:
            x, y = sx+distance*math.cos(direction), sy+distance*math.sin(direction)
            ix, iy = math.floor(x/.05), math.floor(y/.05)
            if not (0 <= iy < data.shape[0] and 0 <= ix < data.shape[1]):
                distance = math.inf
                break
            if data[iy, ix] >= 65:
                break
            distance += .01
        ranges.append(distance)
    return NS(angle_min=-math.pi, angle_increment=math.pi/180,
              range_min=.1, range_max=8., ranges=ranges)


class GlobalScanMatchTest(unittest.TestCase):
    def room(self, symmetric=False):
        data = np.zeros((80, 100), dtype=int)
        data[:2, :] = data[-2:, :] = data[:, :2] = data[:, -2:] = 100
        if not symmetric:
            data[45:70, 23:27] = 100
            data[55:59, 27:49] = 100
            data[20:25, 75:92] = 100
            data[8:20, 57:61] = 100
        return data

    def test_global_search_recovers_asymmetric_room_with_rotated_origin_and_mount(self):
        data = self.room()
        yaw = .4
        matcher = GlobalScanMatcher(grid_message(data, yaw))
        base = (2.225, 1.525, math.radians(20.))
        mounting = (-.15, .02, -.035)
        scan = raycast(data, base, mounting)
        report = matcher.match(scan, mounting)
        self.assertTrue(report['accepted'], report)
        truth = matcher._map_pose(base)
        estimate = report['best']['pose_xyyaw']
        self.assertLess(math.dist(estimate[:2], truth[:2]), .075)
        error = math.atan2(math.sin(estimate[2]-truth[2]), math.cos(estimate[2]-truth[2]))
        self.assertLess(abs(error), math.radians(2.))
        self.assertGreaterEqual(report['runner_up_margin'], .10)
        self.assertGreaterEqual(report['held_out']['inlier_ratio'], .75)
        self.assertFalse(report['uniqueness_physically_proven'])
        self.assertFalse(report['physical_accuracy_verified'])
        true_score = matcher.evaluate(scan, mounting, truth)
        wrong_score = matcher.evaluate(scan, mounting, [truth[0]+.4, truth[1], truth[2]+.2])
        self.assertGreater(true_score['score'], wrong_score['score'])

    def test_repeated_room_symmetry_rejects_ambiguous_heading(self):
        data = self.room(symmetric=True)
        matcher = GlobalScanMatcher(grid_message(data))
        report = matcher.match(raycast(data, (2.225, 1.525, .3)), (0., 0., 0.))
        self.assertFalse(report['accepted'])
        self.assertLess(report['runner_up_margin'], .10)
        self.assertIn('ambiguous', report['reason'])

    def test_held_out_corrupt_beams_cannot_be_accepted_from_training_fit(self):
        data = self.room()
        matcher = GlobalScanMatcher(grid_message(data))
        scan = raycast(data, (2.225, 1.525, math.radians(20.)))
        scan.ranges[1::2] = [r+.65 for r in scan.ranges[1::2]]
        report = matcher.match(scan, (0., 0., 0.))
        self.assertFalse(report['accepted'])
        self.assertGreater(report['best']['score'], .65)
        self.assertIn('held-out', report['reason'])

    def test_weak_angular_support_and_invalid_scan_are_rejected_before_search(self):
        matcher = GlobalScanMatcher(grid_message(self.room()))
        scan = NS(angle_min=-.7, angle_increment=.01, range_min=.1, range_max=8., ranges=[1.]*140)
        report = matcher.match(scan, (0., 0., 0.))
        self.assertFalse(report['accepted']); self.assertIsNone(report['best'])
        self.assertIn('angular sectors', report['reason'])
        scan.angle_increment = 0.
        self.assertIn('invalid', matcher.match(scan, (0., 0., 0.))['reason'])

    def test_unknown_and_outside_endpoints_are_penalized(self):
        data = self.room(); data[20:50, 50:80] = -1
        matcher = GlobalScanMatcher(grid_message(data))
        scan = NS(angle_min=-math.pi, angle_increment=math.pi/180,
                  range_min=.1, range_max=100., ranges=[50.]*360)
        score = matcher.evaluate(scan, (0., 0., 0.), matcher._map_pose((2.225, 1.525, 0.)))
        self.assertEqual(score['inlier_ratio'], 0.)
        self.assertEqual(score['median_distance_m'], .5)
        self.assertEqual(score['crossing_wall_ratio'], 1.)

    def test_free_island_missed_by_coarse_grid_returns_rejection(self):
        # There is legal free space, but its sole cell lies between all coarse
        # samples. This must return a diagnostic instead of scoring no poses.
        data = np.full((5, 5), 100, dtype=int)
        data[1, 1] = 0
        matcher = GlobalScanMatcher(grid_message(data))
        scan = NS(angle_min=-math.pi, angle_increment=math.pi/180,
                  range_min=.1, range_max=8., ranges=[1.]*360)
        report = matcher.match(scan, (0., 0., 0.))
        self.assertFalse(report['accepted'])
        self.assertIsNone(report['best'])
        self.assertEqual(report['distinct_candidates'], [])
        self.assertIn('no free-space sample', report['reason'])


if __name__ == '__main__':
    unittest.main()
