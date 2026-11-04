#!/usr/bin/env python3
"""Obstacle-height regression: tilted floor is rejected, low pallet retained."""
import sys
from pathlib import Path
import unittest
import yaml
from collections import deque
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from publish_rgb_calibrated_cloud import floor_relative_cloud, rotation_matrix, CalibratedCloud


class FloorHeightTests(unittest.TestCase):
    def test_floor_learning_needs_fresh_continuous_stationary_odometry(self):
        def node():
            return SimpleNamespace(quiet_odom=deque((99.+i*.05,True) for i in range(21)),
                odom_received=100.,odom_stamp_s=100.,
                get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100000000000)))
        self.assertTrue(CalibratedCloud.stationary_for_floor(node(),100.))
        for change in ('stale_header','stale_receipt','moving','gap','short_window'):
            n=node()
            if change=='stale_header':n.odom_stamp_s=99.
            if change=='stale_receipt':n.odom_received=99.
            if change=='moving':n.quiet_odom[-2]=(99.95,False)
            if change=='gap':n.quiet_odom=deque([(99.,True),(99.5,True),(100.,True)])
            if change=='short_window':n.quiet_odom=deque([(99.9,True),(100.,True)])
            with self.subTest(change=change):
                self.assertFalse(CalibratedCloud.stationary_for_floor(n,100.))

    def test_current_nav_profile_rejects_observed_floor_and_keeps_low_pallet(self):
        root=Path(__file__).resolve().parents[1]
        old=np.array(yaml.safe_load((root/'config/calibration/astra_pro_nav_floor_2026_11_01.yaml').read_text())['/spatial_depth_cloud']['ros__parameters']['floor_plane_coefficients'])
        new=np.array(yaml.safe_load((root/'config/calibration/astra_pro_nav_floor_2026_11_02.yaml').read_text())['/spatial_depth_cloud']['ros__parameters']['floor_plane_coefficients'])
        xy=np.array([[1.2,0.],[2.1,-.8],[2.6,-.9]])
        floor=np.c_[xy,xy@new[:2]+new[2]]
        pallet=floor.copy();pallet[:,2]+=.10*np.sqrt(1+sum(new[:2]**2))
        previous=floor_relative_cloud(floor,np.eye(3),np.zeros(3),old)
        self.assertTrue(np.any(previous[:,2]>=.06))
        corrected=floor_relative_cloud(np.stack((floor,pallet)),np.eye(3),np.zeros(3),new)
        np.testing.assert_allclose(corrected[0,:,2],0.,atol=1e-7)
        self.assertTrue(np.all(corrected[1,:,2]>.06))
        np.testing.assert_allclose(corrected[:,:,:2],np.stack((floor,pallet))[:,:,:2])

    def test_floor_and_pallet_under_residual_pitch(self):
        plane = np.array([-.063, 0., .043])
        xy = np.array([[1., -.2], [2., .2], [3., -.4]])
        floor = np.column_stack((xy, xy @ plane[:2] + plane[2]))
        pallet = floor.copy()
        pallet[:, 2] += .12 * np.sqrt(1 + plane[0]**2)
        points = np.stack((floor, pallet))
        corrected = floor_relative_cloud(points, np.eye(3), np.zeros(3), plane)
        np.testing.assert_allclose(corrected[0, :, 2], 0., atol=1e-7)
        np.testing.assert_allclose(corrected[1, :, 2], .12, atol=1e-7)
        self.assertTrue(np.all(corrected[0, :, 2] < .06))
        self.assertTrue(np.all(corrected[1, :, 2] > .06))
        np.testing.assert_allclose(corrected[..., :2], points[..., :2])

    def test_optical_transform_and_missing_depth(self):
        rotation = rotation_matrix([-.5, .5, -.5, .5])
        points = np.array([[[.2, .3, 2.], [np.nan, np.nan, np.nan]]])
        result = floor_relative_cloud(points, rotation, np.array([.367, 0., .8]), [0., 0., 0.])
        np.testing.assert_allclose(result[0, 0], [2.367, -.2, .5], atol=1e-6)
        self.assertTrue(np.isnan(result[0, 1]).all())

    def test_invalid_transform_rejected(self):
        for q in ([0., 0., 0., 0.], [0., 0., float('nan'), 1.]):
            with self.assertRaises(ValueError):
                rotation_matrix(q)


if __name__ == '__main__':
    unittest.main()
