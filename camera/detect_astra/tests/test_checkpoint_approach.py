"""Actual logged misalignment must reach checkpoint before straight insertion."""
import math
from pathlib import Path
import sys
import unittest
from dataclasses import replace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_insertion_core import Opening,opening_coordinates,fork_vertices,wrap
from pallet_checkpoint import checkpoint_plan,CheckpointApproach,checkpoint_pose_issue,MAX_CHECKPOINT_TURN_RAD
from pallet_visual_pickup import VisualPickupController
from test_visual_pickup import config,lift


class CheckpointTest(unittest.TestCase):
    def test_farther_checkpoint_shortens_approach_without_widening_turn_limit(self):
        o=Opening(1.81,.02,0.,.255)
        old=checkpoint_plan((0.,0.,0.),o,0.,config())
        new=checkpoint_plan((0.,0.,0.),o,0.,replace(config(),checkpoint_root_gap_m=1.20))
        self.assertAlmostEqual(old.goal[0]-new.goal[0],.10)
        self.assertLess(new.travel_m,old.travel_m)
        self.assertTrue(all(not checkpoint_pose_issue(p,o,0.,replace(config(),checkpoint_root_gap_m=1.20)) for p in new.poses))

    def test_logged_153232_route_explains_thirty_degree_limit(self):
        o=Opening(1.7441805248260498,.0241080392152071,math.radians(4.655152994862625),.24391754)
        with self.assertRaisesRegex(ValueError,'33.90° >30°'):
            checkpoint_plan((0.,0.,0.),o,0.,replace(config(),checkpoint_root_gap_m=1.20))
        p=checkpoint_plan((0.,0.,0.),o,0.,replace(config(),checkpoint_root_gap_m=1.15))
        self.assertAlmostEqual(opening_coordinates(o,p.goal)[1],0.,places=10)
        self.assertAlmostEqual(p.travel_m,math.hypot(p.goal[0],p.goal[1]),places=10)
        self.assertLessEqual(max(abs(wrap(math.atan2(p.goal[1],p.goal[0]))),
            abs(wrap(math.atan2(p.goal[1],p.goal[0])-o.yaw))),math.radians(30))

    def opening(self):return Opening(1.357564+.3655,.0446176,math.radians(3.10107),.255)

    def latest_opening(self):
        heading=math.radians(5.041599090244467);lateral=.07231075909337227
        forward=.3655+(1.390476851348454-lateral*math.sin(heading))/math.cos(heading)
        return Opening(forward,lateral,heading,.255)

    def opening_from_204212_log(self):
        # Opening transformed from the 20:42:12 preflight map/odom snapshot.
        return Opening(1.7603509298005904,-.10103790361464139,
            -.025724050347053412,.24483563005924225)

    def simulate(self,opening=None,measure=True,until_final=False,heading_drift=0.,pivot_creep=False,
                 pivot_bias=0.,pivot_linear_gain=.65,pivot_angular_gain=.55,pivot_command_delay_s=0.):
        cfg=config();opening=opening or self.opening();c=VisualPickupController(cfg)
        pose=[0.,0.,0.];v=w=0.;dt=.05;trace=[]
        c.start(0.,pose,opening,(0.,0.,0.),0.,False)
        pivot_starts={};commands=[]
        for i in range(1800):
            t=i*dt
            if measure and c.phase in ('REMEASURE','FINAL_ALIGN'):c.observe(t,t,opening)
            c.tick(t,pose,v,w,lift(),ros_now=t)
            approach=c.checkpoint_approach
            subphase=approach.phase if approach else ''
            offset=(math.hypot(pose[0]-approach.pivot_anchor[0],pose[1]-approach.pivot_anchor[1])
                    if approach else 0.)
            trace.append((t,tuple(pose),v,w,c.v,c.w,c.phase,c.state,subphase,offset))
            if until_final and c.phase=='FINAL_INSERTION':return c,pose,trace
            if not c.active:return c,pose,trace
            target_v=c.v
            target_w=c.w
            commands.append((t,c.v,c.w))
            if pivot_bias and c.phase=='CHECKPOINT_APPROACH' and subphase.startswith('TURN'):
                due=[row for row in commands if row[0]<=t-pivot_command_delay_s+1e-9]
                _,delayed_v,delayed_w=due[-1] if due else (0.,0.,0.)
                target_v=delayed_v*pivot_linear_gain;target_w=delayed_w*pivot_angular_gain
                if abs(c.w)>.005 or abs(w)>.015:target_v+=pivot_bias
            if pivot_creep and c.phase=='CHECKPOINT_APPROACH' and c.checkpoint_approach:
                phase=c.checkpoint_approach.phase
                if phase.startswith('TURN'):
                    pivot_starts.setdefault(phase,t)
                    if t-pivot_starts[phase]<.35:target_v=.005123971
            v+=max(-.15*dt,min(.3*dt,target_v-v));w+=max(-.4*dt,min(.4*dt,target_w-w))
            w+=heading_drift*dt
            pose[0]+=v*math.cos(pose[2]+w*dt/2)*dt
            pose[1]+=v*math.sin(pose[2]+w*dt/2)*dt;pose[2]=wrap(pose[2]+w*dt)
        self.fail('Timeout '+c.state+' '+c.reason)

    def test_logged_geometry_reaches_inserted_then_explicit_lift(self):
        c,pose,trace=self.simulate()
        self.assertEqual(c.state,'INSERTED',c.reason)
        self.assertIsNone(c.lift_target)
        self.assertTrue(any(row[6]=='REMEASURE' for row in trace))
        self.assertTrue(any(row[6]=='FINAL_INSERTION' for row in trace))
        for row in trace:
            if row[6]=='CHECKPOINT_APPROACH':
                self.assertFalse(checkpoint_pose_issue(row[1],self.opening(),0.,config()))
                self.assertLessEqual(abs(row[1][2]),math.radians(12))
        c.begin_lift(trace[-1][0]+.1,pose,0.)
        self.assertEqual(c.state,'LIFTING');self.assertAlmostEqual(c.lift_target,.08)

    def test_101444_small_heading_uses_outside_controller_before_strict_insertion(self):
        start=(-.018396698883757816,-.01259590098038211,.035184791137135456)
        dx=1.7434209859350533-start[0];dy=.06216299385940547-start[1]
        o=Opening(dx*math.cos(start[2])+dy*math.sin(start[2]),
            -dx*math.sin(start[2])+dy*math.cos(start[2]),
            .05146388077589474-start[2],.2421695441007614)
        c=VisualPickupController(config());preview=c.preview((0.,0.,0.),o,0.)
        c.start(0.,(0.,0.,0.),o,(99.,99.,2.),0.,False)
        self.assertIsInstance(c.checkpoint_approach,CheckpointApproach)
        self.assertEqual(preview,c.plan)
        c,pose,trace=self.simulate(o)
        self.assertEqual(c.state,'INSERTED',c.reason)
        self.assertTrue(any(r[6]=='REMEASURE' for r in trace))
        self.assertIsNone(c.lift_target)

    def test_aligned_1p20m_standoff_has_short_checked_checkpoint_route(self):
        o=Opening(1.201+config().insertion.fork_root_m,0.,0.,.245)
        c,pose,trace=self.simulate(o)
        self.assertEqual(c.state,'INSERTED',c.reason)
        self.assertTrue(any(r[6]=='REMEASURE' for r in trace))
        self.assertIsNone(c.lift_target)

    def test_missing_checkpoint_rgbd_cannot_enter_or_lift(self):
        c,pose,trace=self.simulate(measure=False)
        self.assertEqual(c.state,'BLOCKED',c.reason)
        self.assertEqual(c.phase,'REMEASURE')
        self.assertFalse(c.lift_target)
        self.assertLess(max(opening_coordinates(self.opening(),p)[0] for p in fork_vertices(pose,config().insertion)),0.)

    def test_latest_7cm_5degree_log_has_checked_outside_route_and_strict_final_entry(self):
        opening=self.latest_opening();cfg=config()
        plan=checkpoint_plan((0.,0.,0.),opening,0.,cfg)
        bearing=math.atan2(plan.goal[1],plan.goal[0])
        self.assertGreater(abs(wrap(bearing-opening.yaw)),math.radians(12))
        self.assertLess(abs(wrap(bearing-opening.yaw)),MAX_CHECKPOINT_TURN_RAD)
        for pose in plan.poses:
            self.assertLess(max(opening_coordinates(opening,p)[0]
                for p in fork_vertices(pose,cfg.insertion)),0.)
        controller,pose,trace=self.simulate(opening)
        self.assertEqual(controller.state,'INSERTED',controller.reason)
        self.assertIsNone(controller.lift_target)
        self.assertTrue(any(row[6]=='REMEASURE' for row in trace))
        for row in trace:
            if row[6]=='FINAL_INSERTION':
                self.assertLessEqual(abs(wrap(row[1][2]-opening.yaw)),math.radians(.5))
        self.assertLess(abs(opening_coordinates(opening,pose)[1]),.01)

    def test_over_30_degree_checkpoint_route_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'30°'):
            checkpoint_plan((0.,0.,0.),Opening(1.67,-.09,math.radians(2),.255),0.,config())

    def test_sideways_large_angle_close_start_and_invalid_height_are_rejected(self):
        o=self.opening();cfg=config()
        for start,height in (((0.,0.,math.radians(40)),0.),((0.,.25,0.),0.),((.5,0.,0.),0.),((0.,0.,0.),.04)):
            with self.assertRaises(ValueError):checkpoint_plan(start,o,height,cfg)

    def test_braking_and_path_deviation_rejects_before_fork_entry(self):
        cfg=config();o=self.opening();p=checkpoint_plan((0.,0.,0.),o,0.,cfg)
        control=CheckpointApproach(p,cfg,0.)
        control.phase='DRIVE'
        control.tick(.1,(0.,.02,control.bearing),0.,0.,0.)
        self.assertTrue(control.issue)
        control=CheckpointApproach(p,cfg,0.)
        control.tick(.1,(p.goal[0]+.04,p.goal[1],o.yaw),.1,0.,0.)
        self.assertTrue(control.issue)

    def test_final_straight_residual_and_frozen_line_have_separate_limits(self):
        c,pose,trace=self.simulate(until_final=True)
        self.assertEqual(c.phase,'FINAL_INSERTION')
        self.assertLessEqual(c.config.insertion.heading_tolerance_rad,math.radians(1.5))
        self.assertAlmostEqual(c.visual_config.final_heading_tolerance_rad,math.radians(.5))
        self.assertAlmostEqual(c.config.tracking_tolerance_m,.005)
        c.tick(trace[-1][0]+.1,(pose[0],pose[1],pose[2]+math.radians(.6)),0.,0.,lift())
        self.assertEqual(c.state,'BLOCKED')

    def test_logged_speed_transient_with_1mm_drift_does_not_abort_pivot(self):
        cfg=config();p=checkpoint_plan((0.,0.,0.),self.latest_opening(),0.,cfg)
        control=CheckpointApproach(p,cfg,0.)
        control.tick(.6,(.0012,0.,-.012),.005123971,-.026039,0.)
        self.assertFalse(control.issue,control.issue)
        self.assertLess(control.v,0.);self.assertGreaterEqual(control.v,-.012)
        self.assertLess(control.w,0.)

    def test_pivot_accumulated_drift_and_predicted_stopping_drift_are_bounded(self):
        cfg=config();p=checkpoint_plan((0.,0.,0.),self.latest_opening(),0.,cfg)
        for x,v in ((.0051,0.),(0.,.016)):
            control=CheckpointApproach(p,cfg,0.)
            control.tick(.5,(x,0.,0.),v,0.,0.)
            self.assertTrue(control.issue,(x,v))
            self.assertEqual((control.v,control.w),(0.,0.))
        control=CheckpointApproach(p,cfg,0.)
        for i in range(20):
            control.tick(i*.1,(i*.0004,0.,-.012),.004,0.,0.)
            if control.issue:break
        self.assertTrue(control.issue)
        self.assertLessEqual(i*.0004,.0053)

    def test_pivot_must_settle_linear_motion_before_drive(self):
        cfg=config();p=checkpoint_plan((0.,0.,0.),self.latest_opening(),0.,cfg)
        control=CheckpointApproach(p,cfg,0.)
        for t in (0.,.5,1.):control.tick(t,(0.,0.,control.bearing),.004,0.,0.)
        self.assertEqual(control.phase,'TURN_TO_PATH');self.assertFalse(control.issue)
        for t in (1.1,1.5,2.):control.tick(t,(0.,0.,control.bearing),0.,0.,0.)
        self.assertEqual(control.phase,'DRIVE')

    def test_entire_pickup_survives_bounded_startup_creep_at_both_pivots(self):
        control,pose,trace=self.simulate(self.latest_opening(),pivot_creep=True)
        self.assertEqual(control.state,'INSERTED',control.reason)
        self.assertIsNone(control.lift_target)
        self.assertTrue(any(row[2]>.005 for row in trace))
        self.assertTrue(any(row[6]=='FINAL_INSERTION' for row in trace))

    def test_persistent_logged_pivot_bias_is_corrected_without_widening_5mm_limit(self):
        opening=Opening(1.7505331325448037,.10064299064503585,.09557382135231517,.2447548806667328)
        for bias,gain,delay in ((.0045,.5,0.),(-.0045,.5,0.),(.0045,.5,.15),(-.0045,1.2,.15),
                               (.0045,2.,.20)):
            controller,pose,trace=self.simulate(opening,pivot_bias=bias,pivot_linear_gain=gain,
                pivot_command_delay_s=delay)
            self.assertEqual(controller.state,'INSERTED',controller.reason)
            pivots=[r for r in trace if r[6]=='CHECKPOINT_APPROACH' and r[8].startswith('TURN')]
            self.assertEqual({r[8] for r in pivots},{'TURN_TO_PATH','TURN_TO_FACE'})
            self.assertLess(max(r[9] for r in pivots),.005)
            self.assertLessEqual(max(abs(r[4]) for r in pivots),.012)
            self.assertTrue(any(r[4]*bias<0 for r in pivots))
            self.assertIsNone(controller.lift_target)

    def test_204212_right_side_opening_reaches_inserted_with_delayed_pivot_response(self):
        opening=self.opening_from_204212_log()
        for bias in (.0045,-.0045):
            controller,pose,trace=self.simulate(opening,pivot_bias=bias,
                pivot_linear_gain=2.,pivot_command_delay_s=.20)
            self.assertEqual(controller.state,'INSERTED',controller.reason)
            pivots=[r for r in trace if r[6]=='CHECKPOINT_APPROACH' and r[8].startswith('TURN')]
            self.assertLess(max(r[9] for r in pivots),.005)
            self.assertTrue(any(r[4]*bias<0 for r in pivots))
            self.assertIsNone(controller.lift_target)

    def test_delayed_overresponsive_base_stops_on_measured_pivot_speed(self):
        opening=Opening(1.7505331325448037,.10064299064503585,.09557382135231517,.2447548806667328)
        controller,pose,trace=self.simulate(opening,pivot_bias=-.0045,
            pivot_linear_gain=2.5,pivot_command_delay_s=.25)
        self.assertEqual(controller.state,'BLOCKED')
        self.assertIn('15 mm/s',controller.reason)
        self.assertLess(trace[-1][9],.005)
        self.assertEqual((controller.v,controller.w),(0.,0.))
        self.assertIsNone(controller.lift_target)

    def test_unresponsive_base_cannot_be_hidden_by_pivot_compensation(self):
        controller,pose,trace=self.simulate(self.latest_opening(),pivot_bias=.0045,pivot_linear_gain=0.)
        self.assertEqual(controller.state,'BLOCKED')
        self.assertIn('hãm quay',controller.reason)
        self.assertIsNone(controller.lift_target)

    def test_logged_2p61mm_offset_5p30mm_prediction_brakes_then_recenters_same_anchor(self):
        cfg=config();p=checkpoint_plan((0.,0.,0.),self.latest_opening(),0.,cfg)
        control=CheckpointApproach(p,cfg,0.)
        control.tick(.5,(.00261,0.,0.),.01375,-.06,0.)
        self.assertFalse(control.issue);self.assertEqual(control.pivot_pause,'BRAKE')
        self.assertEqual((control.v,control.w),(0.,0.))
        anchor=control.pivot_anchor
        for t in (1.,1.9):control.tick(t,(.0028,0.,0.),0.,0.,0.)
        self.assertEqual(control.pivot_pause,'RECENTER')
        control.tick(1.95,(.0028,0.,0.),0.,0.,0.)
        self.assertLess(control.v,0.);self.assertEqual(control.w,0.)
        for t in (2.,2.9):control.tick(t,(.001,0.,0.),0.,0.,0.)
        self.assertEqual(control.pivot_pause,'');self.assertFalse(control.issue)
        self.assertEqual(control.pivot_anchor,anchor)
        control.tick(2.95,(.001,0.,0.),0.,0.,0.)
        self.assertNotEqual(control.w,0.)

    def test_operator_stop_during_pivot_brake_cannot_restart_motion(self):
        c=VisualPickupController(config());o=self.opening_from_204212_log()
        c.start(0.,(0.,0.,0.),o,(0.,0.,0.),0.,False)
        c.tick(.5,(.00261,0.,0.),.01375,-.06,lift(),ros_now=.5)
        self.assertEqual(c.checkpoint_approach.pivot_pause,'BRAKE')
        c.stop('Người vận hành dừng')
        c.tick(2.,(.0028,0.,0.),0.,0.,lift(),ros_now=2.)
        self.assertFalse(c.active)
        self.assertEqual((c.v,c.w),(0.,0.))
        self.assertIsNone(c.lift_target)

    def test_pivot_command_slew_prevents_abrupt_sign_reversals(self):
        cfg=config();p=checkpoint_plan((0.,0.,0.),self.latest_opening(),0.,cfg)
        control=CheckpointApproach(p,cfg,0.);last=0.
        for i in range(1,20):
            sign=(-1)**i
            control.tick(i*.05,(sign*.001,0.,0.),sign*.006,0.,0.)
            self.assertFalse(control.issue)
            self.assertLessEqual(abs(control.v-last),.020*.05+1e-9)
            last=control.v

    def test_coupled_pivot_command_respects_laser_braking_budget(self):
        cfg=config()
        self.assertTrue(checkpoint_plan((0.,0.,0.),self.latest_opening(),0.,cfg))
        slow_brakes=replace(cfg,insertion=replace(cfg.insertion,angular_braking_rps2=.1))
        with self.assertRaisesRegex(ValueError,'vùng phanh laser'):
            checkpoint_plan((0.,0.,0.),self.latest_opening(),0.,slow_brakes)


if __name__=='__main__':unittest.main()
