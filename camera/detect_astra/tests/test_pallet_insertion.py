"""Insertion checks for physically different bodies, slots and stopping sweeps."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_insertion_core import (InsertionConfig, Opening, make_plan,
    geometry_issue, braking_poses, cloud_collision, terminal_geometry)
from pallet_insertion_scene import RegisteredScene, choose_entry_height, transform_xy, ground_model, ground_points


class InsertionGeometryTest(unittest.TestCase):
    def test_nearest_safe_line_keeps_one_cm_one_degree_and_full_fork_clearance(self):
        c=InsertionConfig(root_stop_m=.02,minimum_root_gap_m=.01,
            longitudinal_tolerance_m=.005,lateral_tolerance_m=.01,heading_tolerance_rad=math.radians(1))
        opening=Opening(1.71,.024,0.,.245)
        start=(0.,0.,math.atan2(opening.y,opening.x))
        nominal=(opening.x-c.fork_root_m-c.root_stop_m,opening.y,0.)
        plan=make_plan(start,nominal,opening,0.,c)
        self.assertLess(abs(plan.goal[1]-opening.y),.01)
        self.assertGreater(abs(plan.goal[1]-opening.y),0.)
        for pose in plan.poses:
            self.assertEqual(geometry_issue(pose,opening,0.,c),'')
            self.assertLess(abs(pose[2]-opening.yaw),math.radians(1))
        far=Opening(1.71,.07,0.,.245)
        with self.assertRaises(ValueError):make_plan((0.,0.,0.),nominal,far,0.,c)

    def test_live_recognition_path_reserves_heading_margin(self):
        c=InsertionConfig(root_stop_m=.02,minimum_root_gap_m=.01,
            longitudinal_tolerance_m=.005,lateral_tolerance_m=.01,
            heading_tolerance_rad=math.radians(1))
        opening=Opening(1.760115865329391,-.00866279843812382,
                        .006013235677026566,.24473153054714203)
        start=(0.,0.,0.)
        nominal=(opening.x-(c.fork_root_m+c.root_stop_m)*math.cos(opening.yaw),
                 opening.y-(c.fork_root_m+c.root_stop_m)*math.sin(opening.yaw),
                 opening.yaw)
        plan=make_plan(start,nominal,opening,0.,c)
        goal_cross=-math.sin(opening.yaw)*(plan.goal[0]-opening.x)+math.cos(opening.yaw)*(plan.goal[1]-opening.y)
        self.assertAlmostEqual(goal_cross,.005,places=5)
        bearing=math.atan2(plan.goal[1]-start[1],plan.goal[0]-start[0])
        self.assertLess(abs(bearing-opening.yaw),math.radians(.6))
        self.assertTrue(all(not geometry_issue(p,opening,0.,c) for p in plan.poses))

    def setUp(self):
        self.c = InsertionConfig()
        self.opening = Opening(1.6555, 0., 0., .245)
        self.plan = make_plan((0.,0.,0.), (1.30,.05,0.), self.opening, 0.)

    def test_corrects_map_registration_without_changing_saved_goal(self):
        self.assertEqual(self.plan.nominal_goal, (1.30,.05,0.))
        self.assertAlmostEqual(self.plan.goal[0], 1.25)
        self.assertAlmostEqual(self.plan.goal[1], 0.)
        self.assertGreater(self.plan.correction_m, .05)
        self.assertTrue(all(not geometry_issue(p,self.opening,0.) for p in self.plan.poses))

    def test_live_small_yaw_offset_can_join_axis_before_forks_enter(self):
        c=InsertionConfig(lateral_tolerance_m=.02,heading_tolerance_rad=math.radians(2))
        opening=Opening(1.64244,-.00586,math.radians(-1),.245)
        goal=(opening.x-(c.fork_root_m+c.root_stop_m)*math.cos(opening.yaw),
              opening.y-(c.fork_root_m+c.root_stop_m)*math.sin(opening.yaw),opening.yaw)
        plan=make_plan((0.,0.,0.),goal,opening,0.,c)
        self.assertTrue(all(not geometry_issue(p,opening,0.,c) for p in plan.poses))
        self.assertEqual(geometry_issue((0.,.05,0.),opening,0.,c),'')
        # Side protection is mandatory once the tips cross the front plane.
        self.assertIn('axis',geometry_issue((1.20,.03,opening.yaw),opening,0.,c))

    def test_cannot_force_an_unrelated_saved_goal_to_the_current_pallet(self):
        with self.assertRaisesRegex(ValueError, 'beyond correction'):
            make_plan((0.,0.,0.),(1.30,.25,0.),self.opening,0.)
        with self.assertRaisesRegex(ValueError, 'heading disagrees'):
            make_plan((0.,0.,0.),(1.30,0.,math.pi/2),self.opening,0.)

    def test_fork_fits_at_home_but_cannot_raise_into_the_beam(self):
        self.assertEqual(geometry_issue(self.plan.goal,self.opening,0.),'')
        self.assertIn('vertical',geometry_issue(self.plan.goal,self.opening,.025))

    def test_tip_width_and_heading_are_checked_after_front_plane_crossing(self):
        narrow = Opening(1.6555,0.,0.,.18)
        pose=(1.25,.014,math.radians(1.4))
        self.assertIn('swept width',geometry_issue(pose,narrow,0.))
        self.assertIn('Heading',geometry_issue((1.25,0.,.1),self.opening,0.))

    def test_previously_verified_goal_does_not_waive_body_plane_protection(self):
        self.assertIn('Body/fork-root',geometry_issue((1.30,0.,0.),self.opening,0.))

    def test_box_above_forks_still_hits_mast_when_body_reaches_it(self):
        self.assertIsNone(cloud_collision(np.array([[.60,.05,.12]]),(0.,0.,0.),0.))
        self.assertEqual(cloud_collision(np.array([[.35,.05,.12]]),(0.,0.,0.),0.)[0], 'carriage')
        self.assertEqual(cloud_collision(np.array([[.32,.11,.4]]),(0.,0.,0.),0.)[0], 'mast')

    def test_a_new_object_in_the_hole_is_not_exempted_as_pallet(self):
        self.assertEqual(cloud_collision(np.array([[.60,.05,.070]]),(0.,0.,0.),0.)[0], 'fork')
        self.assertEqual(cloud_collision(np.array([[.1775,.214,.08]]),(0.,0.,0.),0.)[0], 'wheel')

    def test_wheel_uses_cylindrical_tread_and_retains_radial_margin(self):
        self.assertIsNone(cloud_collision(np.array([[.2707,-.2364,.0203]]),(0.,0.,0.),0.))
        self.assertEqual(cloud_collision(np.array([[.272,-.214,.0875]]),(0.,0.,0.),0.)[0],'wheel')
        self.assertEqual(cloud_collision(np.array([[.1775,-.236,.01]]),(0.,0.,0.),0.)[0],'wheel')

    def test_entire_reaction_and_braking_motion_is_checked(self):
        sweep=list(braking_poses((1.20,0.,0.),.12,0.))
        self.assertAlmostEqual(sweep[-1][0]-1.20,.12*.15+.12**2/(2*.15),places=3)
        self.assertTrue(any(geometry_issue(p,self.opening,0.) for p in sweep))

    def test_terminal_requires_stillness_and_valid_actual_opening_geometry(self):
        self.assertTrue(terminal_geometry(self.plan.goal,0.,0.,self.plan))
        self.assertFalse(terminal_geometry(self.plan.goal,.02,0.,self.plan))
        self.assertFalse(terminal_geometry((1.30,0.,0.),0.,0.,self.plan))

    def test_nonfinite_inputs_and_unmeasured_height_do_not_become_free_space(self):
        with self.assertRaises(ValueError):
            Opening(float('nan'),0.,0.,.245)
        with self.assertRaises(ValueError):
            cloud_collision(np.array([[float('nan'),0.,0.]]),(0.,0.,0.),0.)
        with self.assertRaises(ValueError):
            InsertionConfig(opening_top_m=.06)


class InsertionSceneTest(unittest.TestCase):
    def scene(self):
        mask=np.zeros((100,100),np.uint8)
        mask[10:45,10:90]=255
        mask[45:90,15:30]=255
        mask[45:90,70:85]=255
        cloud=np.full((100,100,3),np.nan)
        v,u=np.nonzero(mask)
        y=(50-u)*.005
        cloud[v,u]=np.column_stack((1.6+.02*y,y,np.where(v<45,.12,.04)))
        cloud[0,0]=(.8,.5,.2)
        v,u=np.nonzero(~mask.astype(bool))
        cloud[v,u]=np.column_stack((.8+v*.015,(50-u)*.02,np.full(len(v),.008)))
        cloud[0,0]=(.8,.5,.2)
        opening=Opening(1.64,.02,0.,.245)
        return RegisteredScene(cloud,mask,(10,10,90,90),opening,(0.,0.,0.))

    def test_registration_corrects_only_positive_pallet_points(self):
        scene=self.scene()
        np.testing.assert_allclose(scene.other,[[.8,.5,.2]])
        points=np.array([scene.raw_pallet[0],[.8,.5,.2],[1.8,0.,.07]])
        corrected,selected=scene.classify(points)
        self.assertEqual(selected.tolist(),[True,False,False])
        np.testing.assert_allclose(corrected[1:],points[1:])
        self.assertGreater(scene.metadata['rotation_rad'],.01)

    def test_observed_floor_contact_is_allowed_but_a_raised_patch_is_retained(self):
        x,y=np.meshgrid(np.linspace(.8,2.,40),np.linspace(-1.,1.,40))
        p=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.008)))
        coef,tolerance=ground_model(p)
        self.assertTrue(ground_points(p,coef,tolerance).all())
        self.assertFalse(ground_points(np.array([[1.,.2,.028]]),coef,tolerance)[0])
        with self.assertRaises(ValueError):
            ground_model(p[:20])

    def test_requires_pixel_correspondence_and_two_observed_legs(self):
        with self.assertRaises(ValueError):
            RegisteredScene(np.zeros((10,10,3)),np.zeros((100,)),(0,0,100,100),
                            Opening(1.6,0.,0.,.245),(0.,0.,0.))
        with self.assertRaises(ValueError):
            RegisteredScene(np.zeros((100,100,3)),np.ones((100,100)),(0,0,100,100),
                            Opening(1.6,0.,0.,.245),(0.,0.,0.))

    def test_height_avoids_observed_bottom_edge_and_stays_below_measured_beam(self):
        c=InsertionConfig()
        points=np.array([[.8,.05,.064],[1.2,.05,.12]])
        h=choose_entry_height(points,(0.,0.,0.),(1.25,0.,0.),c)
        self.assertGreaterEqual(h,.008)
        self.assertLessEqual(.073+h,.100-.008+1e-9)
        moved=transform_xy(points,(3.,2.,math.pi/2))
        self.assertAlmostEqual(h,choose_entry_height(moved,(3.,2.,math.pi/2),
                                (3.,3.25,math.pi/2),c))

    def test_full_height_obstruction_and_empty_cloud_cannot_be_waived(self):
        c=InsertionConfig()
        points=np.array([[.8,.05,z] for z in np.arange(.05,.101,.001)])
        with self.assertRaisesRegex(ValueError,'No collision-free'):
            choose_entry_height(points,(0.,0.,0.),(1.25,0.,0.),c)
        with self.assertRaises(ValueError):
            choose_entry_height(np.empty((0,3)),(0.,0.,0.),(1.25,0.,0.),c)


if __name__ == '__main__':
    unittest.main()
