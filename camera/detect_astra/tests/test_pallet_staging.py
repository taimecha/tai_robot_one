"""Geometry, staging transitions and filled-footprint safety without actuators."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_alignment_core import OdomSample
from pallet_staging_core import StagingConfig, StagingController, staging_candidates, plane_clearance, bounded_nav_command, planning_clearance_m
from pallet_staging_safety import FootprintGrid, route_clear, swept_poses, approach_route_issue, route_heading_bounds, braking_poses


def grid():
    return NS(header=NS(frame_id='odom'),info=NS(resolution=.05,width=160,height=160,
        origin=NS(position=NS(x=-4.,y=-4.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*25600)


class StagingTest(unittest.TestCase):
    def test_operator_accepted_near_station_needs_no_extra_approach_reserve(self):
        cfg=StagingConfig(lateral_tolerance_m=.015,heading_tolerance_deg=2.)
        c=StagingController(cfg)
        heading=math.radians(.9079440883728989);left=-.010575228960059524
        forward=(1.2931118192438904+cfg.fork_root_m*math.cos(heading)
                 -left*math.sin(heading))/math.cos(heading)
        self.seed(c,forward=forward,left=left,heading=heading)
        self.assertTrue(c.start(1.,1.)[0])
        self.assertEqual(c.state,'CHECKING')
        self.assertFalse(c.candidates)
        self.assertEqual((c.v,c.w),(0.,0.))
        self.assertGreaterEqual(plane_clearance(c.target,(0.,0.,0.),cfg.fork_root_m),1.2)

    def seed(self,c,forward=2.,left=.05,heading=0.):
        for t in (.9,.95,1.):
            c.odom.add(OdomSample(t,0.,0.,0.))
            c.observe(t,t,forward,left,heading)

    def test_nav_limits_preserve_curvature_and_reject_reverse_or_invalid(self):
        cfg=StagingConfig()
        for w in (.259,-.259,.8,-.8):
            v,limited=bounded_nav_command(.1,w,cfg)
            self.assertLessEqual(v,cfg.max_linear_mps)
            self.assertLessEqual(abs(limited),cfg.max_angular_rps)
            self.assertAlmostEqual(limited/v,w/.1)
        for v,w in ((-.01,0.),(.2,0.),(.1,1.1),(float('nan'),0.)):
            with self.assertRaises(ValueError):bounded_nav_command(v,w,cfg)

    def test_diagonal_goals_use_clearance_surplus_and_small_turns_at_any_pose(self):
        cfg=StagingConfig()
        for angle in (-2.8,-.2,0.,.2,2.8):
            base=(3.,-2.,angle)
            target=(base[0]+1.955*math.cos(angle)-.05*math.sin(angle),
                    base[1]+1.955*math.sin(angle)+.05*math.cos(angle),angle)
            goals=staging_candidates(target,base,cfg)
            self.assertTrue(goals)
            turns=[]
            for x,y,yaw in goals:
                cross=-(target[0]-x)*math.sin(angle)+(target[1]-y)*math.cos(angle)
                self.assertAlmostEqual(cross,0.)
                self.assertGreaterEqual(plane_clearance(target,(x,y,yaw),cfg.fork_root_m),1.3-1e-9)
                dx,dy=x-base[0],y-base[1]
                travel=math.atan2(dy,dx)
                relative=math.atan2(math.sin(travel-angle),math.cos(travel-angle))
                self.assertLess(abs(relative),math.pi/2)
                self.assertGreater(dx*math.cos(angle)+dy*math.sin(angle),0.)
                turns.append(abs(relative))
            self.assertEqual(turns,sorted(turns))
        goals=staging_candidates((1.955,.05,0.),(0.,0.,0.),cfg)
        self.assertAlmostEqual(goals[0][0],1.955-.3655-planning_clearance_m(cfg))
        self.assertAlmostEqual(goals[0][1],.05)
        # A centered robot does not approach gratuitously.
        self.assertEqual(staging_candidates((2.,0.,0.),(0.,0.,0.),cfg)[0],(0.,0.,0.))

    def test_insufficient_diagonal_space_never_falls_back_to_sideways_pivot(self):
        cfg=StagingConfig()
        self.assertAlmostEqual(planning_clearance_m(cfg),1.3225)
        for target in ((1.6,.1,0.),(5.,0.,0.),(2.,.5,math.pi)):
            self.assertEqual(staging_candidates(target,(0.,0.,0.),cfg),[])
        c=StagingController();self.seed(c,forward=1.65,left=.05)
        self.assertFalse(c.start(1.,1.)[0])
        self.assertEqual(c.state,'NO_SAFE_GOAL')
        self.assertEqual((c.v,c.w),(0.,0.))

    def test_large_diagonal_is_rejected_and_more_distance_allows_a_small_turn(self):
        cfg=StagingConfig();target=(2.,-.25,math.radians(1.2))
        self.assertEqual(staging_candidates(target,(0.,0.,0.),cfg),[])
        base=(-.5,0.,0.)
        goals=staging_candidates(target,base,cfg)
        self.assertTrue(goals)
        g=goals[0];angle=math.atan2(g[1]-base[1],g[0]-base[0])
        self.assertLessEqual(abs(angle),math.radians(30))
        self.assertGreaterEqual(plane_clearance(target,g,cfg.fork_root_m),1.3-1e-9)
        corners=[base,(base[0],base[1],angle),(g[0],g[1],angle),g]
        route=[]
        for a,b in zip(corners,corners[1:]):route.extend(swept_poses(a,b))
        self.assertEqual(approach_route_issue(route,FootprintGrid(grid()),target,cfg),'')
        lo,hi=route_heading_bounds(route,target[2])
        self.assertLessEqual(lo,angle-target[2])
        self.assertGreaterEqual(hi,0.)
        self.assertFalse(lo <= angle-target[2]-.1 <= hi)

    def test_complete_braking_sweep_includes_reaction_and_measured_speed(self):
        cfg=StagingConfig()
        stop=list(braking_poses((0.,0.,0.),.1,0.,cfg))
        self.assertAlmostEqual(stop[-1][0],.1*.2+.1**2/(2*.08))
        self.assertAlmostEqual(stop[-1][1],0.)
        faster=list(braking_poses((0.,0.,0.),.14,0.,cfg))
        self.assertGreater(faster[-1][0],stop[-1][0])
        reverse=list(braking_poses((0.,0.,0.),-.1,0.,cfg))
        self.assertAlmostEqual(reverse[-1][0],-stop[-1][0])
        pivot=list(braking_poses((0.,0.,0.),0.,.24,cfg))
        self.assertAlmostEqual(pivot[-1][2],.24*.2+.24**2/(2*.4))
        msg=grid()
        # Obstacle reached only near the end of the braking sweep must block.
        cell=int((0.+4)/.05)*160+int((.9+4)/.05)
        msg.data[cell]=100;g=FootprintGrid(msg)
        self.assertTrue(g.clear(stop[0]))
        self.assertTrue(any(not g.clear(p) for p in stop))

    def test_lattice_fallback_cannot_restore_a_90_degree_turn(self):
        cfg=StagingConfig();g=FootprintGrid(grid());target=(2.,.05,0.)
        self.assertIn('heading',approach_route_issue(
            [(0.,0.,0.),(0.,0.,math.pi/2),(0.,.05,0.)],g,target,cfg))
        travel=math.atan2(.04,.2)
        diagonal=[(0.,0.,0.),(0.,0.,travel),(.2,.04,travel),(.2,.04,0.)]
        self.assertEqual(approach_route_issue(diagonal,g,target,cfg),'')
        blocked=grid()
        cell=int((.2+4)/.05)*160+int((.9+4)/.05)
        for cost in (100,-1):
            blocked.data[cell]=cost
            obstacle_grid=FootprintGrid(blocked)
            self.assertTrue(obstacle_grid.clear(diagonal[0]))
            self.assertIn('footprint',approach_route_issue(diagonal,obstacle_grid,target,cfg))

    def test_extra_heading_reserve_is_checked_against_obstacles_before_approval(self):
        class HeadingSensitiveGrid:
            last_issue=(.9,.3,100)
            def clear(self,pose):return abs(pose[2])<.06
        poses=[(0.,0.,0.),(.1,0.,0.)];target=(2.,0.,0.);cfg=StagingConfig()
        self.assertTrue(route_clear(poses,HeadingSensitiveGrid(),target,cfg))
        self.assertIn('reserved tracking yaw',approach_route_issue(poses,HeadingSensitiveGrid(),target,cfg))
        self.assertEqual(approach_route_issue(poses,FootprintGrid(grid()),target,cfg),'')
        lo,hi=route_heading_bounds(poses,0.)
        self.assertAlmostEqual(lo,-.08);self.assertAlmostEqual(hi,.08)

    def test_approach_rejects_body_clear_route_inside_actual_laser_stop_zone(self):
        msg=grid()
        msg.data[int((-.175+4)/.05)*160+int((.95+4)/.05)]=100
        g=FootprintGrid(msg);poses=[(0.,0.,0.),(.03,0.,0.)]
        self.assertTrue(route_clear(poses,g,(2.,0.,0.),StagingConfig()))
        self.assertFalse(g.approach_clear((0.,0.,0.)))
        self.assertIn('footprint',approach_route_issue(poses,g,(2.,0.,0.),StagingConfig()))

    def test_distance_is_perpendicular_and_valid_for_every_rotation(self):
        cfg=StagingConfig()
        target=(2.5,.3,.1)
        goal=staging_candidates(target,(0.,0.,0.),cfg)[0]
        for i in range(361):
            self.assertGreaterEqual(plane_clearance(target,(goal[0],goal[1],math.radians(i)),cfg.fork_root_m),1.3-1e-9)
        self.assertNotAlmostEqual(plane_clearance(target,(0.,0.,0.),cfg.fork_root_m),2.5-cfg.fork_root_m)

    def test_reject_too_close_and_outside_camera_range(self):
        c=StagingController(); self.seed(c,forward=1.5)
        self.assertFalse(c.start(1.,1.)[0]); self.assertEqual(c.state,'TOO_CLOSE')
        self.assertEqual(staging_candidates((5.,0.,0.),(0.,0.,0.),StagingConfig()),[])

    def test_snapshot_navigation_does_not_require_visible_pallet(self):
        c=StagingController(); self.seed(c); self.assertTrue(c.start(1.,1.)[0])
        goal=c.goal; c.navigation_started(); c.invalidate_target('Out of view')
        c.odom.add(OdomSample(1.1,0.,0.,0.))
        c.nav_v=.08; c.nav_command_wall=1.1
        c.tick(1.1,1.1)
        self.assertTrue(c.active); self.assertEqual(c.goal,goal)
        self.assertGreater(c.v,0.)
        c.tick(1.5,1.5)
        self.assertFalse(c.active); self.assertEqual((c.v,c.w),(0.,0.))

    def test_arrival_requires_new_measurements_then_both_strict_tolerances(self):
        c=StagingController(); self.seed(c,left=0.)
        c.start(1.,1.); c.navigation_started(); c.navigation_finished(True,1.1,1.1)
        for i in range(23):
            t=1.1+i*.05
            c.odom.add(OdomSample(t,0.,0.,0.))
            c.tick(t,t)
        self.assertEqual(c.state,'CHECKING'); self.assertIsNone(c.target)
        for i in range(45):
            t=2.3+i*.05
            c.odom.add(OdomSample(t,0.,0.,0.))
            c.observe(t,t,2.,0.,0.)
            c.tick(t,t)
            self.assertNotEqual(c.state,'ALIGNED')
        c.post_measurement_stable=True
        for i in range(25):
            t=5.+i*.05
            c.odom.add(OdomSample(t,0.,0.,0.))
            c.observe(t,t,2.,0.,0.); c.tick(t,t)
        self.assertEqual(c.state,'ALIGNED')
        self.assertEqual((c.v,c.w),(0.,0.))

    def test_lost_rgbd_cannot_reset_physical_arrival_or_cause_navigation_timeout(self):
        c=StagingController();self.seed(c)
        self.assertTrue(c.start(1.,1.)[0]);c.navigation_started()
        c.navigation_finished(True,1.1,1.1)
        for i in range(14):
            t=1.15+i*.05
            c.invalidate_target('LOST_RGBD: geometry jumped')
            c.odom.add(OdomSample(t,0.,0.,0.))
            c.tick(t,t)
            self.assertEqual((c.v,c.w),(0.,0.))
        self.assertEqual(c.state,'CHECKING')
        self.assertTrue(c.stationary_arrival_confirmed)
        for i in range(170):
            t=1.85+i*.05
            c.invalidate_target('LOST_RGBD: geometry jumped')
            c.odom.add(OdomSample(t,0.,0.,0.));c.tick(t,t)
        self.assertEqual(c.state,'REACQUIRE_REQUIRED')
        self.assertFalse(c.active)

    def test_motion_resets_physical_arrival_even_when_rgbd_is_lost(self):
        c=StagingController();self.seed(c);c.start(1.,1.)
        c.navigation_started();c.navigation_finished(True,1.1,1.1)
        for i in range(8):
            t=1.15+i*.05
            c.odom.add(OdomSample(t,0.,0.,0.,.03 if i==5 else 0.,0.))
            c.invalidate_target('LOST_RGBD');c.tick(t,t)
        self.assertEqual(c.state,'NAV_STOPPING')
        self.assertFalse(c.stationary_arrival_confirmed)

    def test_axis_handoff_never_moves_or_succeeds_without_cancel_confirmation(self):
        c=StagingController();self.seed(c,forward=1.7,left=0.)
        c.start(1.,1.);c.navigation_started()
        self.assertTrue(c.begin_axis_handoff(1.))
        for t in (1.1,1.5,2.,3.,4.1):
            c.odom.add(OdomSample(t,0.,0.,0.));c.invalidate_target('LOST_RGBD')
            c.tick(t,t)
            self.assertEqual((c.v,c.w),(0.,0.))
            self.assertFalse(c.stationary_arrival_confirmed)
        self.assertEqual(c.state,'BLOCKED')

    def test_angle_trim_never_translates_and_lateral_failure_replans(self):
        for left,state in ((0.,'TURNING'),(.06,'PLANNING')):
            c=StagingController(); self.seed(c,forward=2.5,left=left,heading=math.radians(3))
            c.start(1.,1.)
            # Place base on the measured normal for the angle-only case.
            p=c.goal if left==0. else (0.,0.,0.)
            c.state='CHECKING'; c.arrival_validated=True
            c.odom.add(OdomSample(1.1,p[0],p[1],0.))
            c.target_stamp=c.target_received=1.1
            c.tick(1.1,1.1)
            self.assertEqual(c.state,state)
            self.assertEqual(c.v,0.)

    def test_already_on_axis_needs_only_yaw_not_a_longitudinal_nav_goal(self):
        c=StagingController();heading=math.radians(3)
        self.seed(c,forward=1.67,left=1.67*math.tan(heading),heading=heading)
        self.assertTrue(c.start(1.,1.)[0])
        self.assertEqual(c.state,'CHECKING')
        self.assertEqual(c.goal[:2],(0.,0.))
        self.assertEqual(c.candidates,[])
        self.assertEqual((c.v,c.w),(0.,0.))

    def test_near_pallet_within_requested_tolerances_needs_no_forward_goal(self):
        c=StagingController()
        self.seed(c,forward=1.64244,left=-.00586,heading=math.radians(-.2))
        self.assertTrue(c.start(1.,1.)[0])
        self.assertEqual(c.state,'CHECKING')
        self.assertEqual(c.candidates,[])
        for i in range(145):
            t=1.05+i*.05
            c.odom.add(OdomSample(t,0.,0.,0.))
            c.observe(t,t,1.64244,-.00586,math.radians(-.2))
            c.post_measurement_stable=t>=6.1
            c.tick(t,t)
            self.assertEqual((c.v,c.w),(0.,0.))
        self.assertEqual(c.state,'ALIGNED')
        self.assertGreaterEqual(c.snapshot()['fork_plane_distance_m'],1.20)

    def test_near_pallet_cannot_skip_a_real_lateral_or_yaw_error(self):
        for left,heading in ((.01,0.),(.015,0.),(0.,math.radians(1.))):
            c=StagingController();self.seed(c,forward=1.64244,left=left,heading=heading)
            c.start(1.,1.)
            self.assertNotEqual(c.state,'ALIGNED')
            self.assertEqual((c.v,c.w),(0.,0.))

    def test_centered_opening_with_small_yaw_error_never_plans_diagonal(self):
        c=StagingController()
        self.seed(c,forward=1.78,left=-.003,heading=math.radians(1.3))
        self.assertTrue(c.start(1.,1.)[0])
        self.assertEqual(c.state,'CHECKING')
        self.assertEqual(c.candidates,[])
        for i in range(10):
            t=1.05+i*.05
            c.odom.add(OdomSample(t,0.,0.,0.))
            c.observe(t,t,1.78,-.003,math.radians(1.3))
            c.post_measurement_stable=True
            c.tick(t,t)
            self.assertNotIn(c.state,('PLANNING','NAVIGATING'))
            self.assertEqual(c.v,0.)

    def test_reacquired_pallet_jump_blocks_restart(self):
        c=StagingController();self.seed(c,forward=2.2,left=-.042,
                                     heading=math.radians(1.))
        self.assertTrue(c.start(1.,1.)[0])
        old=c.frozen_target
        c.odom.add(OdomSample(2.,.11,-.047,0.))
        c.state='REACQUIRE_REQUIRED';c.active=False
        c.stationary_arrival_confirmed=True
        c.target=(old[0],old[1]+.26,old[2]+math.radians(10.))
        c.target_stamp=c.target_received=2.;c.confirmed=3
        self.assertFalse(c.start(2.,2.)[0])
        self.assertEqual(c.state,'TARGET_JUMP')

    def test_actual_initial_lateral_pass_latches_yaw_only_even_after_lateral_changes(self):
        c=StagingController(StagingConfig(lateral_tolerance_m=.015,heading_tolerance_deg=2.))
        self.seed(c,forward=1.7999,left=.0122933,heading=math.radians(3.1941))
        self.assertTrue(c.start(1.,1.)[0])
        self.assertEqual(c.state,'CHECKING')
        self.assertTrue(c.heading_only_trim)
        self.assertEqual((c.v,c.w),(0.,0.))
        self.assertFalse(c.candidates)
        target=c.target
        for i in range(35):
            t=1.05+i*.05
            angle=target[2]*min(1.,i/10)
            c.odom.add(OdomSample(t,0.,0.,angle))
            dx,dy=target[:2]
            c.observe(t,t,dx*math.cos(angle)+dy*math.sin(angle),
                      -dx*math.sin(angle)+dy*math.cos(angle),target[2]-angle)
            c.post_measurement_stable=True
            c.tick(t,t)
            self.assertEqual(c.v,0.)
            self.assertNotIn(c.state,('PLANNING','NAVIGATING'))
        self.assertEqual(c.state,'YAW_CORRECTED')
        self.assertFalse(c.active)
        self.assertEqual((c.v,c.w),(0.,0.))
        self.assertGreater(abs(c.snapshot()['errors']['lateral_m']),.015)
        # This is a geometric distance requirement, not a waived obstacle
        # check. A farther observation can yield a shallow candidate.
        goals=staging_candidates(c.target,(-.45,0.,0.),c.config)
        self.assertTrue(goals)
        for g in goals:
            angle=math.atan2(g[1],g[0]+.45)
            self.assertLessEqual(abs(angle),math.radians(30))
            self.assertLessEqual(abs(angle-c.target[2]),math.radians(30))

    def test_yaw_only_mode_is_preserved_across_visual_reacquisition(self):
        c=StagingController(StagingConfig(lateral_tolerance_m=.015,heading_tolerance_deg=2.))
        self.seed(c,forward=1.8,left=.012,heading=math.radians(3.))
        self.assertTrue(c.start(1.,1.)[0]);old=c.target
        c.state='REACQUIRE_REQUIRED';c.active=False;c.stationary_trim_start_confirmed=True
        c.odom.add(OdomSample(2.,0.,0.,math.radians(2.)))
        c.target=old;c.target_stamp=c.target_received=2.;c.confirmed=3
        self.assertTrue(c.start(2.,2.)[0])
        self.assertTrue(c.heading_only_trim)
        self.assertEqual(c.state,'CHECKING')
        self.assertFalse(c.candidates)

    def test_initial_approach_reserves_space_for_new_measurement_lateral_correction(self):
        c=StagingController(StagingConfig(initial_clearance_margin_m=.30))
        self.seed(c,forward=2.4,left=-.042,heading=math.radians(1.))
        self.assertTrue(c.start(1.,1.)[0])
        first=c.goal;old=c.frozen_target
        self.assertGreaterEqual(plane_clearance(old,first,c.config.fork_root_m),1.42-1e-9)
        c.odom.add(OdomSample(2.,first[0],first[1],first[2]))
        c.state='REACQUIRE_REQUIRED';c.active=False;c.stationary_arrival_confirmed=True
        c.target=(old[0],old[1]+.035,old[2]);c.target_stamp=c.target_received=2.;c.confirmed=3
        self.assertTrue(c.start(2.,2.)[0],c.reason)
        self.assertEqual(c.corrections,1)
        self.assertEqual(c.state,'PLANNING')
        self.assertGreaterEqual(plane_clearance(c.frozen_target,c.goal,c.config.fork_root_m),1.3225-1e-9)

    def test_close_session_needing_a_large_diagonal_stops_without_turning(self):
        c=StagingController();self.seed(c,forward=1.777,left=-.15,heading=math.radians(-1.67))
        self.assertFalse(c.start(1.,1.)[0],c.reason)
        self.assertEqual(c.state,'NO_SAFE_GOAL')
        self.assertIn('≤30°',c.reason)
        self.assertEqual((c.v,c.w),(0.,0.))

    def test_first_approach_can_try_checked_shallower_turn_after_roomier_candidates_fail(self):
        c=StagingController();self.seed(c,forward=1.992,left=-.035,heading=0.)
        self.assertTrue(c.start(1.,1.)[0],c.reason)
        self.assertGreaterEqual(plane_clearance(c.frozen_target,c.candidates[0],c.config.fork_root_m),1.42-1e-9)
        fallback=[g for g in c.candidates if plane_clearance(c.frozen_target,g,c.config.fork_root_m)<1.42]
        self.assertTrue(fallback)
        for g in fallback:
            self.assertGreaterEqual(plane_clearance(c.frozen_target,g,c.config.fork_root_m),1.3225-1e-9)
        self.assertLess(abs(math.atan2(fallback[0][1],fallback[0][0])),
                        abs(math.atan2(c.candidates[0][1],c.candidates[0][0])))

    def test_old_two_degree_acceptance_cannot_authorize_one_degree_insertion(self):
        c=StagingController();self.seed(c,forward=1.64244,left=-.00586,heading=math.radians(-1.))
        c.start(1.,1.)
        self.assertNotEqual(c.state,'ALIGNED')
        self.assertEqual((c.v,c.w),(0.,0.))

    def test_tiny_axis_offset_centres_visible_opening_without_more_translation(self):
        c=StagingController();self.seed(c,forward=1.71,left=-.169,heading=math.radians(-5.25))
        self.assertTrue(c.start(1.,1.)[0],c.reason)
        self.assertEqual(c.state,'CHECKING')
        p=[0.,0.,0.];target=c.target
        for i in range(400):
            t=1.05+i*.05
            c.odom.add(OdomSample(t,*p,0.,c.w))
            dx,dy=target[0]-p[0],target[1]-p[1]
            c.observe(t,t,math.cos(p[2])*dx+math.sin(p[2])*dy,
                -math.sin(p[2])*dx+math.cos(p[2])*dy,target[2]-p[2])
            c.post_measurement_stable=t>=6.
            c.tick(t,t);self.assertEqual(c.v,0.)
            p[2]+=c.w*.05
            if c.state=='ALIGNED':break
        self.assertEqual(c.state,'ALIGNED',c.snapshot())
        self.assertLess(abs(c.snapshot()['errors']['lateral_m']),.01)
        self.assertLess(abs(c.snapshot()['errors']['heading_deg']),1.)

    def test_precision_turn_with_motor_delay_and_isolated_bad_frames_settles(self):
        c=StagingController();self.seed(c,forward=1.78,left=.209,heading=math.radians(7.))
        self.assertTrue(c.start(1.,1.)[0],c.reason)
        target=c.target;yaw=0.;speed=0.;queue=[0.,0.]
        for i in range(800):
            t=1.05+i*.05
            requested=queue.pop(0);queue.append(c.w)
            speed += (requested-speed)*(1-math.exp(-.05/.08))
            yaw += speed*.05
            c.odom.add(OdomSample(t,0.,0.,yaw,0.,speed))
            if i>140 and i%23==0:
                c.invalidate_target('single synthetic bad frame')
            else:
                c.observe(t,t,math.cos(yaw)*target[0]+math.sin(yaw)*target[1],
                    -math.sin(yaw)*target[0]+math.cos(yaw)*target[1],target[2]-yaw)
            c.post_measurement_stable=t>=6.
            c.tick(t,t)
            self.assertEqual(c.v,0.)
            if not c.active:break
        self.assertEqual(c.state,'ALIGNED',c.snapshot())
        self.assertLess(abs(c.snapshot()['errors']['lateral_m']),.01)
        self.assertLess(abs(c.snapshot()['errors']['heading_deg']),1.)

    def test_transient_loss_after_verified_arrival_uses_its_own_continuous_deadline(self):
        c=StagingController();self.seed(c,left=0.)
        c.start(1.,1.);c.arrival_validated=True;c.arrival_wall=1.
        c.odom.add(OdomSample(20.,0.,0.,0.));c.invalidate_target('one bad frame')
        c.tick(20.,20.)
        self.assertTrue(c.active)
        self.assertEqual((c.v,c.w),(0.,0.))
        c.odom.add(OdomSample(20.8,0.,0.,0.));c.tick(20.8,20.8)
        self.assertEqual(c.state,'REACQUIRE_REQUIRED')

    def test_filled_footprint_catches_interior_unknown_and_swept_obstacle(self):
        msg=grid(); g=FootprintGrid(msg); self.assertTrue(g.clear((0.,0.,0.)))
        cell=int((.2+4)/.05)*160+int((.2+4)/.05)
        msg.data[cell]=100
        self.assertFalse(g.clear((0.,0.,0.)))
        msg.data[cell]=-1
        self.assertFalse(g.clear((0.,0.,0.)))
        msg.data[cell]=0
        target=(2.5,0.,0.); cfg=StagingConfig()
        self.assertTrue(route_clear([(0.,0.,0.),(.1,0.,0.)],g,target,cfg))
        # A middle footprint collision must not be missed between endpoints.
        msg.data[int((.6+4)/.05)*160+int((.3+4)/.05)]=100
        self.assertFalse(route_clear([(0.,0.,0.),(0.,0.,math.pi)],g,target,cfg))
        self.assertFalse(route_clear([(1.,0.,0.),(1.1,0.,0.)],g,target,cfg))

    def test_inflation_is_not_applied_twice_to_the_filled_footprint(self):
        msg=grid()
        msg.data=[99]*len(msg.data)
        g=FootprintGrid(msg)
        self.assertTrue(g.clear((0.,0.,0.)))
        # Physical obstacles and unknown cells inside the body still block,
        # even when the polygon perimeter itself contains no obstacle.
        center=int((.1+4)/.05)*160+int((.2+4)/.05)
        for cost in (100,-1):
            msg.data[center]=cost
            self.assertFalse(g.clear((0.,0.,0.)))

    def test_short_lateral_pivot_checks_physical_obstacles_and_unknown(self):
        msg=grid()
        cell=int((-.575+4)/.05)*160+int((.625+4)/.05)
        # This inflation cell enters the fork's swept area during a 90°
        # pivot. Its physical obstacle lies beyond the swept envelope.
        msg.data[cell]=99
        msg.data[int((-.575+4)/.05)*160+int((.975+4)/.05)]=100
        corners=[(0.,0.,0.),(0.,0.,-math.pi/2),(0.,-.07,-math.pi/2),(0.,-.07,0.)]
        route=[]
        for a,b in zip(corners,corners[1:]):route.extend(swept_poses(a,b))
        g=FootprintGrid(msg); cfg=StagingConfig(); target=(2.,-.07,0.)
        self.assertTrue(route_clear(route,g,target,cfg))
        for cost in (100,-1):
            msg.data[cell]=cost
            self.assertTrue(g.clear(corners[0]))
            self.assertFalse(route_clear(route,g,target,cfg))

    def test_missing_rgbd_after_arrival_stays_stopped_and_requests_reacquisition(self):
        c=StagingController(); self.seed(c)
        c.start(1.,1.); c.navigation_started(); c.navigation_finished(True,1.1,1.1)
        for i in range(210):
            t=1.1+i*.05
            c.odom.add(OdomSample(t,0.,0.,0.)); c.tick(t,t)
            self.assertEqual((c.v,c.w),(0.,0.))
        self.assertEqual(c.state,'REACQUIRE_REQUIRED'); self.assertFalse(c.active)

    def test_start_requires_stationary_base_and_invalid_vision_keeps_snapshot_command(self):
        c=StagingController(); self.seed(c)
        c.odom.add(OdomSample(1.05,0.,0.,0.,.03,0.))
        self.assertFalse(c.start(1.05,1.05)[0])
        c.odom.add(OdomSample(1.1,0.,0.,0.))
        c.observe(1.1,1.1,2.,.08,0.)
        self.assertTrue(c.start(1.1,1.1)[0]); c.navigation_started()
        c.v,c.w=.04,.10
        c.invalidate_target('Outside camera')
        self.assertEqual((c.v,c.w),(.04,.10))
        self.assertIsNotNone(c.frozen_target)

    def test_nav_stop_request_does_not_keep_translating_during_a_pivot(self):
        c=StagingController(); self.seed(c); c.start(1.,1.); c.navigation_started()
        c.v,c.w=.10,.22
        c.nav_v=c.nav_w=0.; c.nav_command_wall=1.05
        c.odom.add(OdomSample(1.05,0.,0.,0.))
        c.tick(1.05,1.05)
        self.assertEqual((c.v,c.w),(0.,0.))

    def test_interlock_and_navigation_failure_always_disarm(self):
        for fail in ('interlock','action'):
            c=StagingController(); self.seed(c); c.start(1.,1.); c.navigation_started()
            if fail=='interlock': c.tick(1.,1.,False,'Laser lost')
            else: c.navigation_finished(False,1.,1.)
            self.assertFalse(c.active); self.assertEqual((c.v,c.w),(0.,0.))


if __name__=='__main__':unittest.main()
