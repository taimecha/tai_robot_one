"""Final insertion/lift state machine with real stopping and visibility loss."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_pickup_core import PickupConfig, PickupController
from pallet_insertion_core import Opening, opening_coordinates


def lift(position=0.,state='IDLE',velocity=0.):
    return dict(position=position,state=state,velocity=velocity,upper_active=False,lower_active=position<.001)


class PickupTest(unittest.TestCase):
    def start(self,raise_after=True,y=.01,yaw=math.radians(.5),config=None):
        core=PickupController(config) if config is not None else PickupController()
        pose=[0.,y,yaw]
        core.start(0.,pose,Opening(1.6555,0.,0.,.245),(1.30,.04,0.),0.,raise_after)
        return core,pose

    def simulate(self,deceleration=.2,raise_after=True,config=None,dt=.05):
        core,pose=self.start(raise_after,config=config)
        v=w=h=0.;trace=[]
        for i in range(1200):
            now=i*dt
            lf=lift(h)
            if core.lift_target is not None:
                distance=core.lift_target-h
                if distance>.0001:
                    h+=min(.01*dt,distance);lf=lift(h,'MOVING',.01)
                else:lf=lift(h)
            core.tick(now,pose,v,w,lf)
            target_v,target_w=core.v,core.w
            accel=.3 if target_v>v else deceleration
            v+=max(-accel*dt,min(accel*dt,target_v-v))
            w+=max(-.4*dt,min(.4*dt,target_w-w))
            pose[0]+=v*math.cos(pose[2]+w*dt/2)*dt
            pose[1]+=v*math.sin(pose[2]+w*dt/2)*dt
            pose[2]+=w*dt
            trace.append((now,tuple(pose),v,w,h,core.state))
            if core.state in ('COMPLETE','BLOCKED','TIMEOUT') or (core.state=='INSERTED' and not raise_after):break
        self.assertEqual(core.state,'COMPLETE' if raise_after else 'INSERTED',core.reason)
        self.assertGreaterEqual(min(-opening_coordinates(core.plan.opening,p)[0]-.3655*math.cos(p[2])
                                    for _,p,*_ in trace),core.config.insertion.minimum_root_gap_m)
        self.assertTrue(all(v>=0 for _,p,v,w,h,s in trace))
        self.assertTrue(all(abs(w)<1e-8 and abs(v)<1e-8 for _,p,v,w,h,s in trace if s=='LIFTING'))
        return core,pose,h,trace

    def test_insert_then_lift_with_braking_at_several_decelerations(self):
        for decel in (.15,.25,100.):
            with self.subTest(decel=decel):
                core,pose,h,trace=self.simulate(decel)
                self.assertAlmostEqual(h,.08,places=3)
                self.assertLess(abs(pose[1]),.02)
                self.assertLess(abs(pose[2]),math.radians(2))

    def test_insert_only_does_not_raise(self):
        core,pose,h,_=self.simulate(raise_after=False)
        self.assertIsNone(core.lift_target)
        self.assertEqual(h,0.)

    def test_real_config_reaches_deeper_stop_then_lifts_after_stillness(self):
        import yaml
        from pallet_insertion_core import InsertionConfig
        values=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/pallet_pickup.yaml').read_text())
        values['insertion']=InsertionConfig(**values['insertion'])
        config=PickupConfig(**values)
        for decel in (.15,.25,100.):
            for dt in (.05,.10):
                with self.subTest(decel=decel,dt=dt):
                    core,pose,h,trace=self.simulate(decel,config=config,dt=dt)
                    gap=-opening_coordinates(core.plan.opening,pose)[0]-.3655*math.cos(pose[2])
                    self.assertGreaterEqual(gap,.01)
                    self.assertLessEqual(gap,.025)
                    self.assertAlmostEqual(h,.08,places=3)
                    # The close approach must actually slow down, rather
                    # than relying on an abrupt stop or accepting a short run.
                    near=[v for _,p,v,_,_,state in trace if state=='INSERTING'
                          and -opening_coordinates(core.plan.opening,p)[0]-.3655<.045]
                    self.assertTrue(near)
                    self.assertLess(max(near),.06)

    def test_close_goal_does_not_allow_early_braking_stop_to_lift(self):
        from dataclasses import replace
        from pallet_insertion_core import InsertionConfig
        config=PickupConfig(insertion=replace(InsertionConfig(),root_stop_m=.025,
            minimum_root_gap_m=.020,longitudinal_tolerance_m=.005))
        core,pose=self.start(config=config)
        pose[:]=[core.plan.opening.x-.3655-.06,0.,0.]
        core.tick(.1,pose,.10,0.,lift())
        self.assertEqual(core.state,'STOPPING')
        core.tick(.2,pose,0.,0.,lift())
        core.tick(1.1,pose,0.,0.,lift())
        self.assertEqual(core.state,'BLOCKED')
        self.assertIsNone(core.lift_target)

    def test_lower_after_complete_holds_base_and_waits_for_separate_resume(self):
        core,pose,h,_=self.simulate()
        core.begin_lower(30.,pose,h)
        for i in range(250):
            h=max(0.,h-.0005)
            core.tick(30.+i*.05,pose,0.,0.,lift(h,'MOVING' if h else 'IDLE',-.01 if h else 0.))
            self.assertEqual((core.v,core.w),(0.,0.))
            if not core.active:break
        self.assertEqual(core.state,'LOWERED',core.reason)
        self.assertIsNone(core.lift_target)
        core.tick(50.,pose,0.,0.,lift())
        self.assertEqual(core.state,'LOWERED')

    def test_lower_aborts_on_sensor_loss_or_base_drift(self):
        for fault in ('sensor','base','lease'):
            core,pose,h,_=self.simulate()
            core.begin_lower(30.,pose,h)
            if fault=='base':pose[0]+=.006
            core.tick(30.1,pose,0.,0.,lift(h),
                      'Laser quá hạn' if fault=='sensor' else '',fault!='lease')
            self.assertEqual(core.state,'BLOCKED')
            self.assertIsNone(core.lift_target)

    def test_short_advance_finishes_with_low_speed_drive_deadband(self):
        import yaml
        from pallet_insertion_core import InsertionConfig
        values=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/pallet_pickup.yaml').read_text())
        values['insertion']=InsertionConfig(**values['insertion'])
        core=PickupController(PickupConfig(**values));pose=[0.,0.,0.]
        core.start(0.,pose,Opening(.41623,0.,0.,.245),(.03073,0.,0.),0.,False,preserve_lateral=True)
        v=0.;dt=.05
        for i in range(400):
            core.tick(i*dt,pose,v,0.,lift())
            target=core.v if core.v>=.005 else 0.
            v+=max(-.15*dt,min(.3*dt,target-v));pose[0]+=v*dt
            if not core.active:break
        self.assertEqual(core.state,'INSERTED',core.reason)
        self.assertAlmostEqual(pose[0],.03,delta=.003)
        self.assertIsNone(core.lift_target)

    def test_live_near_station_offset_follows_line_before_entry(self):
        core=PickupController();opening=Opening(1.64244,-.00586,math.radians(-.25),.245)
        c=core.config.insertion
        goal=(opening.x-(c.fork_root_m+c.root_stop_m)*math.cos(opening.yaw),
              opening.y-(c.fork_root_m+c.root_stop_m)*math.sin(opening.yaw),opening.yaw)
        pose=[0.,0.,0.];v=w=0.;dt=.05
        core.start(0.,pose,opening,goal,0.,False)
        for i in range(800):
            core.tick(i*dt,pose,v,w,lift())
            v+=max(-.2*dt,min(.3*dt,core.v-v))
            w+=max(-.4*dt,min(.4*dt,core.w-w))
            pose[0]+=v*math.cos(pose[2]+w*dt/2)*dt
            pose[1]+=v*math.sin(pose[2]+w*dt/2)*dt;pose[2]+=w*dt
            if not core.active:break
        self.assertEqual(core.state,'INSERTED',core.reason)
        self.assertLess(abs(opening_coordinates(opening,pose)[1]),.02)
        self.assertIsNone(core.lift_target)

    def test_operator_lease_and_sensor_loss_stop_without_auto_resume(self):
        for issue,alive in (('Mất odometry',True),('',False),('Laser quá hạn',True)):
            core,pose=self.start()
            core.tick(.1,pose,0.,0.,lift(),issue,alive)
            self.assertEqual(core.state,'BLOCKED')
            self.assertFalse(core.active)
            core.tick(.2,pose,0.,0.,lift())
            self.assertEqual((core.v,core.w),(0.,0.))
            self.assertIsNone(core.lift_target)

    def test_lateral_drift_and_wrong_entry_height_stop(self):
        for y,height in ((.03,0.),(.01,.025)):
            core,pose=self.start();pose[1]=y
            core.tick(.1,pose,0.,0.,lift(height))
            self.assertEqual(core.state,'BLOCKED')

    def test_cannot_lift_before_insertion_or_after_base_drift(self):
        core,pose=self.start()
        with self.assertRaises(ValueError):core.begin_lift(.1,pose,0.)
        core,pose,h,_=self.simulate(raise_after=False)
        pose[0]+=.01
        with self.assertRaises(ValueError):core.begin_lift(20.,pose,0.)

    def test_lift_stops_on_base_motion_or_upper_limit(self):
        for fault in ('base','limit'):
            core,pose,h,_=self.simulate(raise_after=False)
            core.begin_lift(25.,pose,0.)
            lf=lift(.01,'MOVING',.01)
            if fault=='base':pose[0]+=.01
            else:lf['upper_active']=True
            core.tick(25.1,pose,0.,0.,lf)
            self.assertEqual(core.state,'BLOCKED')
            self.assertIsNone(core.lift_target)

    def test_aligned_threshold_alone_does_not_waive_tip_clearance(self):
        core=PickupController()
        with self.assertRaises(ValueError):
            core.start(0.,(0.,.019,math.radians(-1.9)),Opening(1.65,0.,0.,.18),
                       (1.30,0.,0.),0.,True)

    def test_no_progress_and_lift_timeout_do_not_report_success(self):
        core,pose=self.start()
        core.tick(6.,pose,0.,0.,lift())
        self.assertEqual(core.state,'BLOCKED')
        core,pose,_,_=self.simulate(raise_after=False)
        core.begin_lift(25.,pose,0.)
        core.tick(51.,pose,0.,0.,lift())
        self.assertEqual(core.state,'TIMEOUT')


if __name__=='__main__':unittest.main()
