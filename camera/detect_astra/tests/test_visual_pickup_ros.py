"""Real ROS services + Collision Monitor, simulated drives only, domain 219."""
import json
import math
from dataclasses import replace
import unittest
import pickup_ros_fixture as fixture
from test_visual_pickup import config
from pallet_pickup_safety import LaserTransformPending


class VisualPickupRosTest(unittest.TestCase):
    # Reuse fake sensor/actuator plumbing, without inheriting retired workflow tests.
    setUp=fixture.PickupRosFixture.setUp
    tearDown=fixture.PickupRosFixture.tearDown
    on_status=fixture.PickupRosFixture.on_status
    lift_command=fixture.PickupRosFixture.lift_command
    stop_lift=fixture.PickupRosFixture.stop_lift
    receive=fixture.PickupRosFixture.receive
    inputs=fixture.PickupRosFixture.inputs
    spin=fixture.PickupRosFixture.spin
    service=fixture.PickupRosFixture.service
    stack=fixture.PickupRosFixture.stack
    start=fixture.PickupRosFixture.start

    def test_unstable_final_rgbd_reacquires_at_same_checkpoint_then_inserts(self):
        self.camera_visibility_limit=.85
        self.stack(pickup_config=replace(config(),remeasure_timeout_s=6.))
        self.handler.pickup.require_checkpoint_refresh=True;self.start(False)
        self.spin(lambda:self.status and self.status['phase']=='CHECKPOINT_RECOGNITION',timeout=25.)
        self.recognition_session+=1
        self.spin(lambda:self.handler.recognition.get('session')==self.recognition_session
                  and self.handler.core.confirmed>=3,timeout=5.)
        self.assertTrue(self.service('finish_checkpoint_recognition').success)
        self.spin(lambda:self.handler.pickup.phase=='FINAL_ALIGN',timeout=10.)
        anchor=tuple(self.pose);self.rgbd_stable=False
        self.spin(lambda:self.status and self.status['phase']=='CHECKPOINT_RECOGNITION'
                  and self.status['checkpoint_refresh_seq']==2,timeout=10.)
        self.assertTrue(self.status['active'],self.status)
        self.assertEqual((self.status['v'],self.status['w']),(0.,0.))
        self.assertLess(math.hypot(self.pose[0]-anchor[0],self.pose[1]-anchor[1]),.005)
        self.assertFalse(self.lift_seen)
        self.assertFalse(self.service('finish_checkpoint_recognition').success)
        self.rgbd_stable=True;self.recognition_session+=1
        self.spin(lambda:self.handler.recognition.get('session')==self.recognition_session
                  and self.handler.core.confirmed>=3,timeout=5.)
        self.assertTrue(self.service('finish_checkpoint_recognition').success)
        self.spin(lambda:self.status and not self.status['active'],timeout=40.)
        self.assertEqual(self.status['state'],'INSERTED',self.status)
        self.assertFalse(self.lift_seen)

    def test_final_axis_failure_publishes_recovery_without_inserting_or_lifting(self):
        self.camera_visibility_limit=.85
        self.stack(pickup_config=config());self.start(False)
        self.spin(lambda:self.handler.pickup.phase=='REMEASURE',timeout=25.)
        stopped_pose=tuple(self.pose)
        self.pallet_y=.025;self.pallet_heading=0.
        self.spin(lambda:self.status and not self.status['active'],timeout=15.)
        self.assertEqual(self.status['phase'],'FINAL_ALIGN',self.status)
        self.assertEqual(self.status['recovery_action'],'return_to_recognition',self.status)
        self.assertIn('Body is off the opening axis',self.status['reason'])
        self.assertLess(math.hypot(self.pose[0]-stopped_pose[0],self.pose[1]-stopped_pose[1]),.005)
        self.assertFalse(self.lift_seen)
        self.assertTrue(self.service('stop').success)
        self.spin(lambda:self.status and self.status['state']=='STOPPED',timeout=2.)
        self.assertIsNone(self.status['recovery_action'])

    def test_checkpoint_fresh_measurement_straight_entry_and_explicit_lift(self):
        self.camera_visibility_limit=.85
        self.stack(pickup_config=config())
        preview=self.service('preview');self.assertTrue(preview.success,preview.message)
        ref=json.loads(preview.message)['reference']
        self.assertFalse(ref['saved_map_goal_used'])
        self.assertEqual(ref['preview_scope'],'checkpoint_only')
        self.assertFalse(self.service('start',True).success)
        self.start(False)
        self.spin(lambda:self.status and not self.status['active'],timeout=55)
        self.assertEqual(self.status['state'],'INSERTED',self.status)
        self.assertFalse(self.lift_seen)
        self.assertTrue(any(s=='REMEASURING' for *_,s in self.trace))
        self.assertTrue(any(s=='FINAL_ALIGN' for *_,s in self.trace))
        self.assertEqual(self.status['phase'],'FINAL_INSERTION')
        self.assertFalse(self.status['reference']['saved_map_goal_used'])
        self.assertLessEqual(abs(self.pose[2]-self.handler.pickup.plan.start[2]),math.radians(.5))
        response=self.service('lift');self.assertTrue(response.success,response.message)
        self.run_id=response.message
        self.spin(lambda:self.status and self.status['state']=='COMPLETE',timeout=16)
        self.assertAlmostEqual(self.height,.08,places=3)

    def test_short_capture_tf_wait_publishes_zero_and_preserves_active_plan(self):
        self.stack(pickup_config=config());self.start(False)
        original=self.handler.laser_points;plan=self.handler.pickup.plan;held=[]
        zero=self.handler.zero;calls=[0]
        def pending():
            calls[0]+=1
            if calls[0]<=2:raise LaserTransformPending('42 ms future TF test')
            return original()
        def record_zero():
            held.append((self.handler.pickup.v,self.handler.pickup.w));zero()
        self.handler.laser_points=pending;self.handler.zero=record_zero
        self.spin(lambda:calls[0]>=3 and self.handler.laser_hold_since is None,timeout=3.)
        self.assertTrue(self.handler.pickup.active,self.status)
        self.assertTrue(held);self.assertTrue(all(v==0 and w==0 for v,w in held))
        self.assertIs(self.handler.pickup.plan,plan)
        self.assertFalse(self.lift_seen)
        self.assertTrue(self.service('stop').success)

    def test_persistent_capture_tf_loss_stops_without_lifting(self):
        self.stack(pickup_config=config());self.start(False)
        def missing():raise LaserTransformPending('missing capture pose')
        self.handler.laser_points=missing
        self.spin(lambda:self.status and not self.status['active'],timeout=3.)
        self.assertEqual(self.status['state'],'BLOCKED',self.status)
        self.assertIn('120 ms',self.status['reason'])
        self.assertFalse(self.lift_seen)

    def test_checkpoint_without_camera_stays_out_and_never_lifts(self):
        self.camera_visibility_limit=.10
        self.stack(pickup_config=config());self.start(False)
        self.spin(lambda:self.status and not self.status['active'],timeout=38)
        self.assertEqual(self.status['state'],'BLOCKED',self.status)
        self.assertEqual(self.status['phase'],'REMEASURE')
        self.assertFalse(self.lift_seen)
        self.assertLess(self.pose[0],.52)

    def test_short_scan_loss_holds_zero_then_rechecks_same_checkpoint_plan(self):
        self.stack(pickup_config=config());self.start(False)
        plan=self.handler.pickup.plan
        self.emit_scan=False
        self.spin(lambda:self.handler.scan_pause_started is not None,timeout=2.)
        self.assertTrue(self.handler.pickup.active)
        self.assertEqual((self.handler.pickup.v,self.handler.pickup.w),(0.,0.))
        self.assertFalse(self.lift_seen)
        self.emit_scan=True
        self.spin(lambda:self.handler.scan_pause_started is None or not self.handler.pickup.active,timeout=4.)
        self.assertTrue(self.handler.pickup.active,self.status)
        self.assertIs(self.handler.pickup.plan,plan)
        self.assertTrue(self.service('stop').success)

    def test_persistent_scan_loss_aborts_after_zero_hold(self):
        self.stack(pickup_config=config());self.start(False);self.emit_scan=False
        self.spin(lambda:self.handler.scan_pause_started is not None,timeout=2.)
        self.assertEqual((self.handler.pickup.v,self.handler.pickup.w),(0.,0.))
        self.spin(lambda:not self.handler.pickup.active,timeout=4.)
        self.assertIn('3 giây',self.handler.pickup.reason)
        self.assertFalse(self.lift_seen)

    def test_stop_during_scan_hold_never_resumes(self):
        self.stack(pickup_config=config());self.start(False);self.emit_scan=False
        self.spin(lambda:self.handler.scan_pause_started is not None,timeout=2.)
        self.assertTrue(self.service('stop').success);self.emit_scan=True
        self.spin(lambda:self.handler.sensor_issue()=='',timeout=3.)
        self.assertFalse(self.handler.pickup.active)
        self.assertEqual((self.handler.pickup.v,self.handler.pickup.w),(0.,0.))
        self.assertFalse(self.lift_seen)

    def test_obstacle_on_scan_recovery_blocks_resume(self):
        self.stack(pickup_config=config());self.start(False);self.emit_scan=False
        self.spin(lambda:self.handler.scan_pause_started is not None,timeout=2.)
        self.obstacle=True;self.emit_scan=True
        self.spin(lambda:not self.handler.pickup.active,timeout=3.)
        self.assertIn('Laser chặn',self.handler.pickup.reason)
        self.assertFalse(self.lift_seen)

    def test_small_final_trim_with_wheel_deadzone_reaches_insertion(self):
        self.camera_visibility_limit=.85;self.angular_deadzone_rps=.022
        self.stack(pickup_config=config());self.start(False)
        self.spin(lambda:self.handler.pickup.phase=='REMEASURE',timeout=25.)
        self.pallet_heading=math.radians(.62);self.pallet_y=.0153
        self.spin(lambda:self.status and not self.status['active'],timeout=50.)
        self.assertEqual(self.status['state'],'INSERTED',self.status)
        self.assertLessEqual(abs(self.pose[2]-self.pallet_heading),math.radians(.5))
        self.assertFalse(self.lift_seen)

    def test_101444_aligned_start_uses_checkpoint_controller_and_new_measurement(self):
        self.pallet_x=1.7434209859350533;self.pallet_y=.06216299385940547
        self.pallet_heading=.05146388077589474
        self.pose=[-.018396698883757816,-.01259590098038211,.035184791137135456]
        self.camera_visibility_limit=.85
        self.stack(pickup_config=config())
        self.handler.pickup.require_checkpoint_refresh=True
        preview=self.service('preview');self.assertTrue(preview.success,preview.message)
        self.start(False)
        self.assertIsNotNone(self.handler.pickup.checkpoint_approach)
        self.spin(lambda:self.status and (self.status['phase']=='CHECKPOINT_RECOGNITION'
            or not self.status['active']),timeout=40.)
        self.assertEqual(self.status['phase'],'CHECKPOINT_RECOGNITION',self.status)
        self.assertFalse(self.lift_seen)
        self.recognition_session+=1
        self.spin(lambda:self.handler.recognition.get('session')==self.recognition_session
            and self.handler.core.confirmed>=3,timeout=8.)
        self.assertTrue(self.service('finish_checkpoint_recognition').success)
        self.spin(lambda:self.status and not self.status['active'],timeout=50.)
        self.assertEqual(self.status['state'],'INSERTED',self.status)
        self.assertFalse(self.lift_seen)

    def test_farther_checkpoint_refreshes_new_session_before_straight_entry_and_lift(self):
        # The 20:42 opening is 10.1 cm to the right with a -1.47-degree face.
        self.pallet_x=1.7603509298005904;self.pallet_y=-.10103790361464139
        self.pallet_heading=-.025724050347053412
        self.camera_visibility_limit=.40
        self.pivot_bias_mps=.0045
        self.pivot_linear_gain=1.2
        self.pivot_command_delay_s=.15
        self.pose=[0.,0.,0.];self.stack(pickup_config=config())
        self.handler.pickup.require_checkpoint_refresh=True
        self.assertFalse(self.handler.core.start(self.handler.now(),__import__('time').monotonic())[0])
        self.handler.prepare_checkpoint_route();self.start(False)
        self.spin(lambda:self.status and (self.status['phase']=='CHECKPOINT_RECOGNITION'
            or not self.status['active']),timeout=45.)
        self.assertEqual(self.status['phase'],'CHECKPOINT_RECOGNITION',self.status)
        self.assertTrue(self.status['active']);self.assertFalse(self.lift_seen)
        self.assertFalse(self.service('finish_checkpoint_recognition').success)
        anchor=list(self.pose)
        # Old tracking is lost; reacquisition must occur while base stays put.
        self.force_camera_lost=True;self.rgbd_stable=False
        self.spin(lambda:not self.handler.last_target_msg.valid)
        self.assertTrue(self.handler.pickup.active,self.status)
        self.assertFalse(self.service('finish_checkpoint_recognition').success)
        self.force_camera_lost=False;self.rgbd_stable=True;self.recognition_session+=1
        self.spin(lambda:self.handler.last_target_msg.valid and
                  self.handler.recognition.get('session')==self.recognition_session
                  and self.handler.core.confirmed>=3,timeout=8.)
        self.assertTrue(self.service('finish_checkpoint_recognition').success)
        self.assertLess(math.hypot(self.pose[0]-anchor[0],self.pose[1]-anchor[1]),.005)
        self.spin(lambda:self.status and not self.status['active'],timeout=50.)
        self.assertEqual(self.status['state'],'INSERTED',self.status)
        self.assertFalse(self.lift_seen)
        self.assertEqual({row[0] for row in self.pivot_offsets},{'TURN_TO_PATH','TURN_TO_FACE'})
        self.assertLess(max(row[1] for row in self.pivot_offsets),.005)
        self.assertTrue(any(row[2]<-.001 for row in self.pivot_offsets))
        self.assertLessEqual(max(abs(row[2]) for row in self.pivot_offsets),.012)
        response=self.service('lift');self.assertTrue(response.success,response.message);self.run_id=response.message
        self.spin(lambda:self.status and self.status['state']=='COMPLETE',timeout=16.)
        self.assertAlmostEqual(self.height,.08,places=3)

    def test_checkpoint_refresh_pose_motion_aborts_without_inserting(self):
        self.camera_visibility_limit=.40;self.stack(pickup_config=config())
        self.handler.pickup.require_checkpoint_refresh=True;self.start(False)
        self.spin(lambda:self.status and self.status['phase']=='CHECKPOINT_RECOGNITION',timeout=25.)
        self.pose[1]+=.006
        self.spin(lambda:self.status and not self.status['active'],timeout=3.)
        self.assertEqual(self.status['state'],'BLOCKED')
        self.assertFalse(self.lift_seen)

    def test_persistent_checkpoint_pivot_creep_aborts_before_entry_or_lift(self):
        self.pallet_x=1.357564+.3655;self.pallet_y=.0446176
        self.pallet_heading=math.radians(3.10107);self.pose=[0.,0.,0.]
        self.pivot_bias_mps=.0045;self.pivot_linear_gain=0.
        self.stack(pickup_config=config())
        self.assertFalse(self.handler.core.start(self.handler.now(),__import__('time').monotonic())[0])
        self.handler.prepare_checkpoint_route();self.start(False)
        self.spin(lambda:self.status and not self.status['active'],timeout=40.)
        self.assertEqual(self.status['state'],'BLOCKED',self.status)
        self.assertIn('hãm quay',self.status['reason'])
        self.assertFalse(self.lift_seen)
        self.assertLess(self.pose[0],.4)

    def test_logged_4cm_misalignment_checkpoint_entry_and_confirmed_lift(self):
        self.pallet_x=1.357564+.3655;self.pallet_y=.0446176
        self.pallet_heading=math.radians(3.10107);self.camera_visibility_limit=.90
        self.pose=[0.,0.,0.]
        self.stack(pickup_config=config())
        # Exercise the exact NO_SAFE_GOAL -> checked outside route admission.
        self.assertFalse(self.handler.core.start(self.handler.now(),__import__('time').monotonic())[0])
        self.assertEqual(self.handler.core.state,'NO_SAFE_GOAL')
        self.handler.prepare_checkpoint_route()
        self.assertEqual(self.handler.core.state,'CHECKPOINT_READY')
        self.assertFalse(self.handler.pickup.active)
        preview=self.service('preview');self.assertTrue(preview.success,preview.message)
        self.assertTrue(json.loads(preview.message)['reference']['checkpoint_correction_only'])
        self.start(False)
        self.spin(lambda:self.status and not self.status['active'],timeout=70.)
        self.assertEqual(self.status['state'],'INSERTED',self.status)
        self.assertFalse(self.lift_seen)
        self.assertTrue(any(row[-1]=='REMEASURING' for row in self.trace))
        self.assertLess(max(abs(row[1][2]) for row in self.trace),math.radians(12))
        self.assertEqual(self.status['reference']['preview_scope'],'final_straight_odom')
        response=self.service('lift');self.assertTrue(response.success,response.message)
        self.run_id=response.message
        self.spin(lambda:self.status and self.status['state']=='COMPLETE',timeout=16.)
        self.assertAlmostEqual(self.height,.08,places=3)

    def test_checkpoint_correction_cannot_bypass_an_obstacle(self):
        self.pallet_x=1.357564+.3655;self.pallet_y=.0446176
        self.pallet_heading=math.radians(3.10107);self.pose=[0.,0.,0.]
        self.stack(pickup_config=config());self.obstacle=True
        self.spin(lambda:len(self.handler.laser_points())>0)
        self.assertFalse(self.handler.core.start(self.handler.now(),__import__('time').monotonic())[0])
        # Wait for the obstructed scan, not the previous clear measurement.
        from pallet_pickup_safety import footprint_hit
        self.spin(lambda:footprint_hit(self.handler.laser_points(),(0.,0.,0.),padding=.21) is not None)
        with self.assertRaisesRegex(ValueError,'laser chặn'):self.handler.prepare_checkpoint_route()
        self.assertNotEqual(self.handler.core.state,'CHECKPOINT_READY')
        self.assertFalse(self.handler.pickup.active);self.assertFalse(self.lift_seen)

    def test_yaw_corrected_can_prepare_checked_lateral_checkpoint_route(self):
        self.pallet_x=1.6715;self.pallet_y=-.0613
        self.pallet_heading=math.radians(-.08);self.pose=[0.,0.,0.]
        self.stack(pickup_config=config())
        self.handler.core.heading_only_trim=True
        self.handler.core.stop('yaw complete','YAW_CORRECTED')
        self.spin(lambda:self.handler.core.state!='YAW_CORRECTED',timeout=3.)
        self.assertEqual(self.handler.core.state,'CHECKPOINT_READY',self.handler.core.reason)
        self.assertFalse(self.handler.pickup.active)
        self.assertFalse(self.lift_seen)

    def test_alignment_service_admits_checked_checkpoint_without_sending_motion(self):
        from std_srvs.srv import Trigger,SetBool
        self.pallet_x=1.357564+.3655;self.pallet_y=.0446176
        self.pallet_heading=math.radians(3.10107);self.pose=[0.,0.,0.]
        self.stack(pickup_config=config())
        def acknowledge(request,response):response.success=True;response.message='YOLO stopped';return response
        self.input.create_service(Trigger,'/pallet/prepare_alignment',acknowledge)
        # No planner/controller is started by this pickup fixture. Keep all
        # sensor/CM/lift/heartbeat interlocks and exercise the real async
        # preparation service, core projection and fallback route checks.
        def interlock():
            issue=self.handler.sensor_issue()
            return not bool(issue),issue
        self.handler.interlock=interlock
        client=self.input.create_client(SetBool,'/pallet/alignment/set_enabled')
        self.spin(client.service_is_ready)
        future=client.call_async(SetBool.Request(data=True));self.spin(future.done,timeout=12.)
        self.assertTrue(future.result().success,future.result().message)
        self.assertEqual(self.handler.core.state,'CHECKPOINT_READY')
        self.assertFalse(self.handler.core.active);self.assertFalse(self.handler.pickup.active)
        self.assertFalse(self.lift_seen)
        self.assertTrue(all(abs(row[2])+abs(row[3])==0 for row in self.trace))


if __name__=='__main__':unittest.main()
