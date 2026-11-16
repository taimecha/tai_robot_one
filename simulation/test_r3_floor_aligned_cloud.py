"""Floor geometry and frozen alignment tests without ROS nodes or hardware."""
import math
import unittest
from copy import deepcopy

import numpy as np

from r3_floor_aligned_cloud import (FloorAlignmentLock, fit_floor, floor_transform,
                                    normal_angle_deg, quaternion_matrix)


def plane_points(normal,offset,x_bounds=(.6,2.8),y_bounds=(-1.,1.),noise=.0):
    normal=np.asarray(normal,dtype=float);normal/=np.linalg.norm(normal)
    x,y=np.meshgrid(np.linspace(*x_bounds,60),np.linspace(*y_bounds,35))
    z=-(normal[0]*x+normal[1]*y+offset)/normal[2]
    points=np.column_stack((x.ravel(),y.ravel(),z.ravel()))
    if noise:
        points[:,2]+=np.random.default_rng(11).normal(0.,noise,len(points))
    return points


def fit(normal=(0.,0.,1.),offset=-.027):
    return dict(accepted=True,normal=list(normal),offset_m=offset)


def warmed_lock():
    lock=FloorAlignmentLock()
    lock.observe(fit(),0.)
    return lock


def frozen_lock():
    lock=warmed_lock()
    for stamp in (10.,10.5,11.,11.5,12.):
        lock.observe(fit(),stamp)
    return lock


class FloorAlignedCloudMathTest(unittest.TestCase):
    def test_observed_sloped_ground_is_recovered_amid_noise_and_obstacles(self):
        normal=np.array([.0503,-.0486,.99755]);normal/=np.linalg.norm(normal)
        ground=plane_points(normal,-.0267,noise=.002)
        rng=np.random.default_rng(17)
        obstacles=np.column_stack((rng.uniform(.6,2.8,600),rng.uniform(-1.,1.,600),
                                   rng.uniform(.025,.095,600)))
        result=fit_floor(np.vstack((ground,obstacles)))
        self.assertTrue(result['accepted'],result)
        self.assertLess(normal_angle_deg(normal,result['normal']),.15)
        self.assertAlmostEqual(result['offset_m'],-.0267,delta=.002)
        self.assertLess(result['median_residual_m'],.003)
        self.assertGreater(result['inlier_ratio'],.7)
        self.assertLessEqual(result['sampled_count'],2500)

    def test_alignment_removes_floor_tilt_and_offset_while_preserving_lengths(self):
        normal=np.array([.0503,-.0486,.99755]);normal/=np.linalg.norm(normal)
        offset=-.0267
        ground=plane_points(normal,offset)
        rotation,translation=floor_transform(normal,offset)
        aligned=ground@rotation.T+translation
        self.assertLess(np.max(np.abs(aligned[:,2])),1e-12)
        np.testing.assert_allclose(rotation@normal,[0.,0.,1.],atol=1e-12)
        np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12)
        elevated=ground[0]+.2*normal
        self.assertAlmostEqual((rotation@elevated+translation)[2],.2)
        self.assertAlmostEqual(np.linalg.norm(aligned[-1]-aligned[0]),
                               np.linalg.norm(ground[-1]-ground[0]))

    def test_camera_xy_pivot_is_fixed_while_ground_is_leveled(self):
        normal=np.array([.0503,-.0486,.99755]);normal/=np.linalg.norm(normal)
        camera=np.array([.432,.025,.27])
        ground=plane_points(normal,-.0267)
        rotation,translation=floor_transform(normal,-.0267,pivot=camera)
        np.testing.assert_allclose((rotation@camera+translation)[:2],camera[:2],atol=1e-12)
        aligned=ground@rotation.T+translation
        self.assertLess(np.max(np.abs(aligned[:,2])),1e-12)

    def test_implausible_floor_or_insufficient_support_is_rejected(self):
        cases=[np.zeros((100,3)),
               plane_points([0.,0.,1.],-.14),
               plane_points([math.sin(math.radians(9.)),0.,math.cos(math.radians(9.))],0.),
               plane_points([0.,0.,1.],0.,x_bounds=(.6,1.2),y_bounds=(-.2,.2))]
        for points in cases:
            with self.subTest(points=len(points)):
                self.assertFalse(fit_floor(points)['accepted'])
        with self.assertRaises(ValueError):
            fit_floor(np.zeros((10,2)))

    def test_stable_distinct_fits_are_required_for_two_seconds_before_freezing(self):
        lock=warmed_lock()
        for stamp in (10.,10.5,11.,11.5):
            self.assertFalse(lock.observe(fit(),stamp))
            self.assertIsNone(lock.frozen)
        self.assertTrue(lock.observe(fit(),12.))
        frozen=lock.frozen.copy()
        self.assertEqual(frozen['stable_duration_s'],2.)
        self.assertEqual(frozen['fit_count'],5)
        self.assertTrue(lock.observe(fit(offset=-.028),12.5))
        self.assertEqual(lock.frozen,frozen)

    def test_bad_fit_motion_time_gap_and_changing_plane_cannot_create_alignment(self):
        lock=warmed_lock()
        for stamp in (10.,10.5,11.,11.5):
            lock.observe(fit(),stamp)
        self.assertFalse(lock.observe(dict(accepted=False,reason='floor occluded'),12.))
        self.assertEqual(len(lock.samples),0)
        self.assertFalse(lock.observe(fit(),12.5,stationary=False))
        self.assertIsNone(lock.frozen)
        self.assertFalse(lock.observe(fit(),13.))
        self.assertFalse(lock.observe(fit(),15.))
        self.assertEqual(len(lock.samples),1)
        tilted=[math.sin(math.radians(.8)),0.,math.cos(math.radians(.8))]
        self.assertFalse(lock.observe(fit(tilted),15.5))
        self.assertEqual(len(lock.samples),1)
        self.assertIsNone(lock.frozen)

    def test_valid_contradiction_after_freeze_latches_fault_instead_of_adapting(self):
        lock=frozen_lock()
        frozen=deepcopy(lock.frozen)
        self.assertFalse(lock.observe(fit(offset=-.05),12.5))
        self.assertIsNone(lock.fault)
        self.assertFalse(lock.observe(fit(offset=-.05),13.))
        self.assertIsNone(lock.fault)
        self.assertFalse(lock.observe(fit(offset=-.05),13.5))
        self.assertIsNotNone(lock.fault)
        self.assertEqual(lock.frozen,frozen)
        for stamp in (14.,14.5,15.,15.5,16.):
            self.assertFalse(lock.observe(fit(),stamp))
        self.assertEqual(lock.frozen,frozen)

    def test_camera_warmup_does_not_count_toward_stable_fit_window(self):
        lock=FloorAlignmentLock()
        for stamp in np.arange(0.,10.,.5):
            self.assertFalse(lock.observe(fit(),float(stamp)))
            self.assertIsNone(lock.frozen)
            self.assertEqual(len(lock.samples),0)
        for stamp in (10.,10.5,11.,11.5):
            self.assertFalse(lock.observe(fit(),stamp))
        self.assertTrue(lock.observe(fit(),12.))
        self.assertEqual(lock.frozen['warmup_duration_s'],10.)

    def test_freeze_uses_median_plane_instead_of_last_fit(self):
        lock=warmed_lock()
        angles=(-.05,0.,.01,.02,.20)
        offsets=(-.030,-.027,-.026,-.025,-.021)
        for stamp,angle,offset in zip((10.,10.5,11.,11.5,12.),angles,offsets):
            angle=math.radians(angle)
            lock.observe(fit((math.sin(angle),0.,math.cos(angle)),offset),stamp)
        self.assertTrue(lock.ready)
        self.assertAlmostEqual(lock.frozen['offset_m'],-.026)
        self.assertAlmostEqual(math.degrees(math.atan2(lock.frozen['normal'][0],
                                                    lock.frozen['normal'][2])),.01,places=6)

    def test_single_changed_or_rejected_fit_withholds_until_original_matches_two_seconds(self):
        angle=math.radians(.69)
        for transient in (fit((math.sin(angle),0.,math.cos(angle))),
                          dict(accepted=False,reason='floor occluded')):
            with self.subTest(transient=transient):
                lock=frozen_lock()
                frozen=deepcopy(lock.frozen)
                self.assertFalse(lock.observe(transient,12.5))
                self.assertIsNone(lock.fault)
                for stamp in (13.,13.5,14.,14.5):
                    self.assertFalse(lock.observe(fit(),stamp))
                self.assertTrue(lock.observe(fit(),15.))
                self.assertEqual(lock.frozen,frozen)

    def test_duplicate_changed_fit_does_not_satisfy_persistent_change_requirement(self):
        lock=frozen_lock()
        for _ in range(10):
            self.assertFalse(lock.observe(fit(offset=-.05),12.5))
        self.assertIsNone(lock.fault)
        self.assertFalse(lock.observe(fit(offset=-.05),13.))
        self.assertIsNone(lock.fault)
        self.assertFalse(lock.observe(fit(offset=-.05),13.5))
        self.assertIsNotNone(lock.fault)

    def test_motion_and_changed_camera_pivot_immediately_invalidate_frozen_alignment(self):
        for kwargs in ({'stationary':False},{'pivot':(.006,0.,0.)}):
            with self.subTest(kwargs=kwargs):
                lock=frozen_lock()
                frozen=deepcopy(lock.frozen)
                self.assertFalse(lock.observe(fit(),12.5,**kwargs))
                self.assertIsNotNone(lock.fault)
                self.assertFalse(lock.observe(fit(),13.))
                self.assertEqual(lock.frozen,frozen)

    def test_withheld_sensor_output_recovers_only_after_original_plane_is_reverified(self):
        lock=frozen_lock()
        frozen=deepcopy(lock.frozen)
        lock.withhold('camera cloud is stale')
        self.assertFalse(lock.ready)
        for stamp in (12.5,13.,13.5,14.):
            self.assertFalse(lock.observe(fit(),stamp))
        self.assertTrue(lock.observe(fit(),14.5))
        self.assertEqual(lock.frozen,frozen)

    def test_camera_quaternion_rotation_is_normalized_and_invalid_values_rejected(self):
        angle=math.radians(20.)
        rotation=quaternion_matrix([0.,math.sin(angle/2),0.,math.cos(angle/2)])
        np.testing.assert_allclose(rotation@[1.,0.,0.],[math.cos(angle),0.,-math.sin(angle)],atol=1e-12)
        for invalid in ([0.,0.,0.,0.],[0.,0.,0.,2.],[0.,float('nan'),0.,1.]):
            with self.subTest(value=invalid),self.assertRaises(ValueError):
                quaternion_matrix(invalid)


if __name__=='__main__':
    unittest.main()
