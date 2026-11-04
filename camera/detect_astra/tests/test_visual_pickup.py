"""Straight local insertion: geometry, capture barrier, drift and stop guards."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_insertion_core import InsertionConfig, Opening, opening_coordinates, fork_vertices, wrap
from pallet_pickup_core import PickupConfig
from pallet_visual_pickup import VisualPickupController, final_config, straight_plan, depth_issue, accepted_straight_config


def config():
    values=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/pallet_pickup.yaml').read_text())
    values['insertion']=InsertionConfig(**values['insertion'])
    values['quarter_turn_lateral']=False
    values['checkpoint_root_gap_m']=1.10
    return PickupConfig(**values)


def lift():return dict(position=0.,state='IDLE',velocity=0.,upper_active=False,lower_active=True)


class VisualPickupTest(unittest.TestCase):
    def simulate(self,y=.003,yaw=0.,dt=.05,stop_at_final=False,measure=True,decel=.15):
        c=VisualPickupController(config());o=Opening(1.66,0.,0.,.245)
        pose=[0.,y,yaw];v=w=0.;trace=[]
        c.start(0.,pose,o,(99,99,2),0.,True)
        for i in range(int(70/dt)):
            now=i*dt
            if measure and c.phase in ('REMEASURE','FINAL_ALIGN'):c.observe(now,now,o)
            c.tick(now,pose,v,w,lift(),ros_now=now)
            if stop_at_final and c.phase=='FINAL_INSERTION':return c,pose,now,trace
            v+=max(-decel*dt,min(.30*dt,c.v-v))
            w+=max(-.4*dt,min(.4*dt,c.w-w))
            pose[0]+=v*math.cos(pose[2]+w*dt/2)*dt
            pose[1]+=v*math.sin(pose[2]+w*dt/2)*dt;pose[2]+=w*dt
            trace.append((now,tuple(pose),v,w,c.phase,c.state,c.w))
            if not c.active:return c,pose,now,trace
        self.fail('controller did not finish')

    def test_unknown_clearance_blocks_before_checkpoint_motion(self):
        c=VisualPickupController(PickupConfig())
        with self.assertRaisesRegex(ValueError,'Chưa đo độ sâu'):
            c.start(0.,(0.,0.,0.),Opening(1.66,0.,0.,.245),(1.,0.,0.),0.,False)
        self.assertFalse(c.active);self.assertEqual((c.v,c.w),(0.,0.))
        with self.assertRaises(ValueError):final_config(PickupConfig(usable_depth_m=float('nan')))

    def test_measured_depth_produces_safe_root_stop(self):
        cfg=final_config(config())
        self.assertAlmostEqual(cfg.usable_width_m,.255)
        self.assertAlmostEqual(cfg.usable_depth_m,.355)
        self.assertAlmostEqual(cfg.insertion.opening_top_m,.100)
        self.assertAlmostEqual(cfg.rear_clearance_m,.300)
        self.assertAlmostEqual(cfg.insertion.root_stop_m,.020)
        self.assertLess(cfg.insertion.minimum_root_gap_m,cfg.insertion.root_stop_m)
        self.assertAlmostEqual(cfg.tracking_tolerance_m,.005)

    def test_open_rear_allows_tips_past_rear_posts_within_outside_clearance(self):
        cfg=final_config(config())
        opening=Opening(1.66,0.,0.,.255)
        plan=straight_plan((0.,0.,0.),opening,0.,cfg)
        depth=max(opening_coordinates(opening,p)[0] for p in fork_vertices(plan.goal,cfg.insertion))
        self.assertAlmostEqual(depth,.370)
        self.assertGreater(depth,cfg.usable_depth_m)
        self.assertFalse(depth_issue(plan.goal,opening,cfg))
        # The post columns still bound the entire fork sweep laterally.
        with self.assertRaises(ValueError):
            straight_plan((0.,.05,0.),opening,0.,cfg)
        # Declaring no outside clearance retains the conservative old bound.
        closed=final_config(replace(config(),rear_clearance_m=0.))
        self.assertTrue(.061<closed.insertion.root_stop_m<.063)
        self.assertTrue(depth_issue(plan.goal,opening,closed))

    def test_close_outside_obstacle_limits_travel_and_invalid_clearance_fails(self):
        cfg=final_config(replace(config(),rear_clearance_m=.015))
        self.assertGreater(cfg.insertion.root_stop_m,.040)
        opening=Opening(1.66,0.,0.,.255)
        plan=straight_plan((0.,0.,0.),opening,0.,cfg)
        self.assertFalse(depth_issue(plan.goal,opening,cfg))
        for clearance in (-.1,float('nan')):
            with self.assertRaises(ValueError):replace(config(),rear_clearance_m=clearance)

    def test_measured_box_width_caps_but_does_not_widen_live_measurement(self):
        cfg=final_config(config())
        for measured,expected in ((.280,.255),(.245,.245)):
            opening=Opening(1.66,0.,0.,measured)
            controller=VisualPickupController(config())
            preview=controller.preview((0.,0.,0.),opening,0.)
            self.assertAlmostEqual(preview.opening.width_m,expected)
            controller.start(0.,(0.,0.,0.),opening,preview.nominal_goal,0.,False)
            self.assertAlmostEqual(controller.plan.opening.width_m,expected)
            plan=straight_plan((0.,0.,0.),opening,0.,cfg)
            self.assertAlmostEqual(plan.opening.width_m,expected)

    def test_whole_sequence_stops_for_manual_lift_and_keeps_line(self):
        for y,yaw,dt,decel in ((.003,0.,.05,.15),(.01,math.radians(.5),.05,.15),
                               (-.01,math.radians(-.5),.10,.15),(.003,0.,.05,.25)):
            with self.subTest(y=y,yaw=yaw,dt=dt,decel=decel):
                c,pose,now,trace=self.simulate(y,yaw,dt,decel=decel)
                self.assertEqual(c.state,'INSERTED',c.reason)
                self.assertIsNone(c.lift_target)
                self.assertTrue(any(p=='REMEASURE' for *_,p,s,w in trace))
                final=[r for r in trace if r[4]=='FINAL_INSERTION']
                for _,p,v,w,phase,state,cmd_w in final:
                    self.assertFalse(depth_issue(p,c.plan.opening,c.config))
                    self.assertLessEqual(abs(wrap(p[2]-c.plan.start[2])),math.radians(.5))
                    cross=-(p[0]-c.plan.start[0])*math.sin(c.plan.start[2])+(p[1]-c.plan.start[1])*math.cos(c.plan.start[2])
                    self.assertLess(abs(cross),.005)
                    if abs(v)<1e-9:self.assertAlmostEqual(cmd_w,0.)
                gap=-opening_coordinates(c.plan.opening,pose)[0]-c.config.insertion.fork_root_m*math.cos(pose[2])
                self.assertLess(abs(gap-c.config.insertion.root_stop_m),.005)
                c.begin_lift(now,pose,0.)
                self.assertEqual(c.state,'LIFTING');self.assertAlmostEqual(c.lift_target,.08)

    def test_checkpoint_missing_camera_never_enters_pallet(self):
        c,pose,_,_=self.simulate(measure=False)
        self.assertEqual(c.state,'BLOCKED');self.assertEqual(c.phase,'REMEASURE')
        self.assertGreater(1.66-pose[0]-.3655,.79)
        self.assertEqual((c.v,c.w),(0.,0.));self.assertIsNone(c.lift_target)

    def test_supervised_rgbd_timeout_reacquires_here_and_requires_new_window(self):
        c=VisualPickupController(config());o=Opening(1.46,0.,0.,.244)
        c.plan=c.preview((-.2,0.,0.),o,0.)
        c.active=True;c.phase=c.state='FINAL_ALIGN';c.require_checkpoint_refresh=True
        c.phase_started=0.;pose=(0.,0.,0.)
        t=0.
        for attempt in (1,2):
            for _ in range(900):
                t+=.05;c.tick(t,pose,0.,0.,lift(),ros_now=t)
                self.assertTrue(c.active,c.reason)
                self.assertEqual((c.v,c.w),(0.,0.))
                if c.phase=='CHECKPOINT_RECOGNITION':break
            self.assertEqual(c.phase,'CHECKPOINT_RECOGNITION')
            self.assertEqual(c.checkpoint_refresh_seq,attempt)
            t+=60.;c.tick(t,pose,0.,0.,lift(),ros_now=t)
            self.assertTrue(c.active,c.reason)
            c.complete_checkpoint_refresh(t,t,o)
            self.assertEqual(c.phase,'REMEASURE')
            self.assertIsNone(c.observation)
        for _ in range(180):
            t+=.05;c.observe(t,t,o);c.tick(t,pose,0.,0.,lift(),ros_now=t)
            if c.phase=='FINAL_INSERTION':break
        self.assertEqual(c.phase,'FINAL_INSERTION',c.reason)
        self.assertIsNone(c.lift_target)

    def test_replayed_captures_cannot_qualify_window(self):
        c=VisualPickupController(config());c.phase='REMEASURE';c.after_stamp=10.
        o=Opening(1.66,0.,0.,.245)
        for i in range(200):c.observe(9.,20.+i*.05,o)
        self.assertIsNone(c.stable_opening(30.));self.assertFalse(c.samples)
        for i in range(101):c.observe(11+i*.05,11+i*.05,o)
        self.assertIsNotNone(c.stable_opening(16.))
        self.assertIsNone(c.stable_opening(16.3))

    def test_camera_gap_or_unstable_heading_resets_qualification(self):
        c=VisualPickupController(config());c.phase='REMEASURE'
        o=Opening(1.66,0.,0.,.245)
        for i in range(101):c.observe(i*.05,i*.05,o)
        self.assertIsNotNone(c.stable_opening(5.))
        c.observe(5.4,5.4,o)
        self.assertIsNone(c.stable_opening(5.4))
        for i in range(1,102):c.observe(5.4+i*.05,5.4+i*.05,replace(o,yaw=math.radians(2 if i%2 else 0)))
        self.assertIsNone(c.stable_opening(10.5))

    def test_lateral_heading_and_heartbeat_failures_stop_without_lift(self):
        for fault in ('cross','heading','heartbeat','odom'):
            c,p,now,_=self.simulate(stop_at_final=True)
            if fault=='cross':p[1]+=.006
            if fault=='heading':p[2]+=math.radians(.6)
            c.tick(now+.05,p,0.,0.,lift(),issue='Odometry quá hạn' if fault=='odom' else '',
                   operator_alive=fault!='heartbeat',ros_now=now+.05)
            self.assertEqual(c.state,'BLOCKED',(fault,c.reason));self.assertEqual((c.v,c.w),(0.,0.))
            self.assertIsNone(c.lift_target)

    def test_plan_heading_constant_and_no_saved_goal(self):
        cfg=final_config(config())
        for a in (0.,math.pi/2,-1.61):
            o=Opening(1.66*math.cos(a),1.66*math.sin(a),a,.245)
            plan=straight_plan((0.,0.,a),o,0.,cfg)
            self.assertTrue(all(p[2]==a for p in plan.poses))
            self.assertEqual(plan.goal,plan.nominal_goal)
            self.assertFalse(plan.station_locked)
            self.assertTrue(all(not depth_issue(p,o,cfg) for p in plan.poses))

    def test_direct_mode_skips_translation_and_checkpoint(self):
        c=VisualPickupController(config());c.direct_mode=True
        o=Opening(1.81,.003,0.,.255)
        plan=c.preview((0.,0.,0.),o,0.)
        self.assertEqual(plan.travel_m,0.)
        c.start(0.,(0.,0.,0.),o,plan.goal,0.,False)
        self.assertEqual(c.phase,'REMEASURE')
        self.assertIsNone(c.checkpoint_approach)
        self.assertEqual((c.v,c.w),(0.,0.))
        self.assertIsNone(c.lift_target)

    def test_three_cm_straight_accepted_when_entire_fork_fits(self):
        cfg=accepted_straight_config((0.,0.,0.),Opening(1.46,.031,0.,.255),0.,config())
        self.assertIsNotNone(cfg)
        self.assertAlmostEqual(cfg.insertion.lateral_tolerance_m,.05)

    def test_five_cm_admission_does_not_bypass_actual_fork_clearance(self):
        c=VisualPickupController(config());c.direct_mode=True
        # Below the 5-cm admission gate, but the full fork sweep cannot fit.
        with self.assertRaises(ValueError):
            c.preview((0.,0.,0.),Opening(1.81,.049,0.,.239),0.)
        self.assertFalse(c.active)

    def test_direct_pivot_checks_future_axis_before_motion(self):
        c=VisualPickupController(config());c.direct_mode=True
        with self.assertRaises(ValueError):
            c.preview((0.,0.,0.),Opening(1.81,-.0038,math.radians(3.54),.2476),0.)
        self.assertFalse(c.active)
        self.assertEqual((c.v,c.w),(0.,0.))

    def test_small_residual_straight_admission_keeps_swept_geometry(self):
        self.assertIsNotNone(accepted_straight_config((0.,0.,0.),Opening(1.46,.005,0.,.255),0.,config()))
        self.assertIsNone(accepted_straight_config((0.,0.,0.),Opening(1.46,.050,0.,.255),0.,config()))
        self.assertIsNone(accepted_straight_config((0.,0.,0.),Opening(1.46,0.,math.radians(1.5),.255),0.,config()))
        self.assertIsNone(accepted_straight_config((0.,0.,0.),Opening(1.46,.019,math.radians(-1.4),.20),0.,config()))

    def test_wrong_axis_is_rejected_before_insertion(self):
        cfg=final_config(config())
        for pose in ((0,.025,0),(0,0,math.radians(.7))):
            with self.assertRaises(ValueError):straight_plan(pose,Opening(1.66,0.,0.,.245),0.,cfg)

    def test_logged_small_trim_overcomes_deadzone_and_remeasures_after_motion(self):
        for sign in (-1,1):
            c=VisualPickupController(config())
            o=Opening(1.4581,sign*1.4581*math.tan(math.radians(1.8)),sign*math.radians(1.8),.2452)
            # Begin at the stopped checkpoint after its new recognition session.
            c.plan=c.preview((-.2,0.,0.),o,0.)
            c.active=True;c.phase='REMEASURE';c.state='REMEASURING';c.phase_started=0.
            pose=[0.,0.,0.];w=0.;seen_turn=False;last_motion=0.;dt=.05
            for i in range(700):
                t=i*dt
                c.observe(t,t,o,healthy=abs(w)<.003)
                c.tick(t,pose,0.,w,lift(),ros_now=t)
                if c.phase=='FINAL_INSERTION':break
                self.assertTrue(c.active,c.reason)
                target=c.w if abs(c.w)>.045 else 0.
                self.assertLessEqual(abs(c.w),.060)
                w+=max(-.4*dt,min(.4*dt,target-w))
                pose[2]+=w*dt
                if abs(w)>.003:seen_turn=True;last_motion=t
            self.assertTrue(seen_turn)
            self.assertEqual(c.phase,'FINAL_INSERTION',c.reason)
            self.assertGreaterEqual(t-last_motion,5.)
            self.assertLess(abs(wrap(pose[2]-o.yaw)),math.radians(1.5))
            self.assertIsNone(c.lift_target)

    def test_final_trim_unresponsive_base_fails_without_insertion(self):
        c=VisualPickupController(config());o=Opening(1.4581,1.4581*math.tan(math.radians(1.8)),math.radians(1.8),.2452)
        c.plan=c.preview((-.2,0.,0.),o,0.)
        c.active=True;c.phase='REMEASURE';c.state='REMEASURING';c.phase_started=0.
        for i in range(600):
            t=i*.05;c.observe(t,t,o);c.tick(t,(0.,0.,0.),0.,0.,lift(),ros_now=t)
            if not c.active:break
        self.assertEqual(c.state,'BLOCKED')
        self.assertIn('20 giây',c.reason)
        self.assertEqual((c.v,c.w),(0.,0.));self.assertIsNone(c.lift_target)

    def test_251_degree_trim_allowed_outside_when_future_straight_line_fits(self):
        angle=math.radians(2.51)
        opening=Opening(1.4637,1.4637*math.tan(angle),angle,.2390)
        c=VisualPickupController(config());c.plan=c.preview((-.2,0.,0.),opening,0.)
        c.active=True;c.phase='REMEASURE';c.state='REMEASURING';c.phase_started=0.
        for i in range(150):
            t=i*.05;c.observe(t,t,opening);c.tick(t,(0.,0.,0.),0.,0.,lift(),ros_now=t)
            if c.w:break
        self.assertTrue(c.active,c.reason)
        self.assertGreater(c.w,0.)
        self.assertIsNone(c.lift_target)

    def test_logged_checkpoint_lateral_offset_requires_new_approach(self):
        opening=Opening(1.4637,.0312,math.radians(2.51),.2390)
        c=VisualPickupController(config());c.plan=c.preview((-.2,0.,0.),opening,0.)
        c.active=True;c.phase='REMEASURE';c.state='REMEASURING';c.phase_started=0.
        for i in range(150):
            t=i*.05;c.observe(t,t,opening);c.tick(t,(0.,0.,0.),0.,0.,lift(),ros_now=t)
            if not c.active:break
        self.assertIn('Đường càng sau quay không lọt khe',c.reason)
        self.assertEqual((c.v,c.w),(0.,0.));self.assertIsNone(c.lift_target)

    def test_final_stationary_measurement_off_axis_requests_recovery_before_insertion(self):
        # Reproduces the branch after trim, where no more rotation is needed
        # but the newly measured straight line still misses the opening axis.
        opening=Opening(1.4592,.049,0.,.2441)
        c=VisualPickupController(config());c.plan=c.preview((-.2,0.,0.),opening,0.)
        c.active=True;c.phase='REMEASURE';c.state='REMEASURING';c.phase_started=0.
        for i in range(180):
            t=i*.05;c.observe(t,t,opening);c.tick(t,(0.,0.,0.),0.,0.,lift(),ros_now=t)
            if not c.active:break
        self.assertEqual(c.phase,'FINAL_ALIGN')
        self.assertEqual(c.state,'BLOCKED')
        self.assertIn('Fork swept width would contact an opening side',c.reason)
        self.assertEqual(c.recovery_action,'return_to_recognition')
        self.assertEqual((c.v,c.w),(0.,0.));self.assertIsNone(c.lift_target)
        c.stop()
        self.assertIsNone(c.recovery_action)

    def test_sensor_fault_has_no_return_recovery_action(self):
        c=VisualPickupController(config());c.active=True;c.phase='FINAL_ALIGN'
        c.tick(1.,(0.,0.,0.),0.,0.,lift(),issue='Mất odometry',ros_now=1.)
        self.assertEqual(c.state,'BLOCKED')
        self.assertIsNone(c.recovery_action)

    def test_slow_final_trim_can_exceed_six_seconds_while_converging(self):
        # Center lies on the final heading axis, isolating slow angular response.
        c=VisualPickupController(config());o=Opening(1.4581,1.4581*math.tan(math.radians(1.45)),math.radians(1.45),.2452)
        c.plan=c.preview((-.2,0.,0.),o,0.)
        c.active=True;c.phase='REMEASURE';c.state='REMEASURING';c.phase_started=0.
        pose=[0.,0.,0.];w=0.;turn_start=None
        for i in range(2400):
            t=i*.05;c.observe(t,t,o);c.tick(t,pose,0.,w,lift(),ros_now=t)
            if c.phase=='FINAL_INSERTION':break
            self.assertTrue(c.active,c.reason)
            if c.w and turn_start is None:turn_start=t
            w=math.copysign(.001,c.w) if c.w else 0.
            pose[2]+=w*.05
        self.assertEqual(c.phase,'FINAL_INSERTION',c.reason)
        self.assertGreater(t-turn_start,20.)
        self.assertIsNone(c.lift_target)


if __name__=='__main__':unittest.main()
