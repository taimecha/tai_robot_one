"""Actual Nav2 planner/controller + collision monitor, simulated base in isolated domains 220–224."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest
import copy
import threading
os.environ['ROS_DOMAIN_ID']='231'
import test_alignment_ros as fixture
from stage_pallet import PalletStagingNode
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from detected_pallets_interfaces.msg import PalletTarget
from pallet_alignment_core import wrap, OdomSample
from pallet_staging_core import StagingController,on_approach_axis,plane_clearance
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]


class StagingRosTest(unittest.TestCase):
    _domain_index=0

    def setUp(self):
        self.previous_domain=os.environ.get('ROS_DOMAIN_ID','0')
        os.environ['ROS_DOMAIN_ID']=str(220+type(self)._domain_index)
        type(self)._domain_index+=1
        fixture.AlignmentRosTest.setUp(self)
        # Real odom/laser producers run independently of the Python route
        # validator. Keep simulated inputs independent too, so a planning
        # callback cannot fabricate an odometry outage in the same executor.
        self.executor.remove_node(self.node)
        self.inputs_executor=fixture.SingleThreadedExecutor()
        self.inputs_executor.add_node(self.node)
        self.inputs_stop=threading.Event()
        def spin_inputs():
            while not self.inputs_stop.is_set():self.inputs_executor.spin_once(timeout_sec=.02)
        self.inputs_thread=threading.Thread(target=spin_inputs)
        self.inputs_thread.start()
    spin_until=fixture.AlignmentRosTest.spin_until
    service=fixture.AlignmentRosTest.service
    def on_status(self,msg):
        status=json.loads(msg.data)
        if self.status is None or status['state']!=self.status['state']:
            print('Staging transition:',status['state'],status['reason'],flush=True)
        self.status=status
    prepare=fixture.AlignmentRosTest.prepare

    def inputs(self):
        if not hasattr(self,'pose'):
            self.pose=[0.,0.,0.]; self.velocity=(0.,0.); self.last_input=time.monotonic()
            self.received_velocity=0.
        now=time.monotonic(); dt=min(.1,now-self.last_input); self.last_input=now
        v,w=self.velocity if now-self.received_velocity<.2 else (0.,0.)
        v*=getattr(self,'measured_linear_scale',1.)
        w*=getattr(self,'measured_yaw_scale',1.)
        if abs(w)<getattr(self,'angular_deadband_rps',0.):
            w=0.
        if abs(w)>.03 and abs(v)<.001:
            v+=getattr(self,'pivot_drift_mps',0.)
        if getattr(self,'integrate',False):
            self.pose[0]+=v*math.cos(self.pose[2])*dt
            self.pose[1]+=v*math.sin(self.pose[2])*dt
            self.pose[2]=wrap(self.pose[2]+w*dt)
        else:v=w=0.
        if not hasattr(self,'trajectory'):self.trajectory=[]
        self.trajectory.append(dict(t=now,pose=list(self.pose),v=v,w=w,
            state=self.status['state'] if self.status else None))
        if self.publish_recognition:
            self.yolo.publish(Bool(data=self.yolo_enabled))
            self.recognition.publish(String(data=json.dumps(dict(enabled=self.yolo_enabled,busy=False,
                locked=True,measurement_ready=True,phase='TRACKING',rgbd_health=dict(stable=True)))))
        stamp=self.node.get_clock().now().to_msg()
        odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id='odom'; odom.child_frame_id='base_footprint'
        odom.pose.pose.position.x,odom.pose.pose.position.y=self.pose[:2]
        yaw=self.pose[2]
        odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
        odom.twist.twist.linear.x,odom.twist.twist.angular.z=v,w
        self.odom.publish(odom)
        tf=TransformStamped(); tf.header=odom.header; tf.child_frame_id='base_footprint'
        tf.transform.translation.x,tf.transform.translation.y=self.pose[:2]
        tf.transform.rotation=odom.pose.pose.orientation
        self.tf.sendTransform(tf)
        scan=LaserScan(); scan.header.stamp=stamp; scan.header.frame_id='base_footprint'
        scan.angle_min=-math.pi; scan.angle_max=math.pi; scan.angle_increment=2*math.pi/getattr(self,'scan_count',3600)
        scan.range_min=.05; scan.range_max=10.; scan.ranges=[3.5]*(getattr(self,'scan_count',3600)+1)
        if self.emit_scan:self.scan.publish(scan)
        # Simulate losing the pallet while Nav2 turns sideways, then seeing it again.
        if self.emit_targets and abs(yaw)<math.radians(getattr(self,'target_visibility_yaw_deg',5)):
            dx,dy=getattr(self,'pallet_x',2.)-self.pose[0],getattr(self,'pallet_y',.05)-self.pose[1]
            forward=math.cos(yaw)*dx+math.sin(yaw)*dy
            lateral=-math.sin(yaw)*dx+math.cos(yaw)*dy
            msg=PalletTarget(); msg.header.stamp=stamp; msg.header.frame_id='camera_color_optical_frame'
            msg.valid=True; msg.status='TRACKED_RGBD; target_bias_applied'
            if getattr(self,'invalidate_targets',False):
                msg.valid=False;msg.status='LOST_RGBD: geometry jumped; target_bias_applied'
            msg.camera_forward_m=forward; msg.chassis_front_forward_m=forward-.3005
            msg.opening_lateral_m=lateral
            msg.face_yaw_rad=wrap(yaw-getattr(self,'pallet_heading',0.)); msg.entry_heading_rad=-msg.face_yaw_rad
            msg.opening_width_m=.252; msg.leg_center_separation_m=.31
            msg.left_depth_samples=msg.right_depth_samples=100
            self.target.publish(msg); self.last_target=msg
        elif self.emit_targets:
            msg=PalletTarget(); msg.header.stamp=stamp
            msg.header.frame_id='camera_color_optical_frame'
            msg.valid=False; msg.status='LOST_RGBD: pallet outside view'
            self.target.publish(msg)

    def start_stack(self,execute):
        self.processes=[]
        self.logs=[]
        log=open('/tmp/pallet_staging_nav_test.log','w'); self.logs.append(log)
        self.processes.append(subprocess.Popen(['ros2','launch',str(ROOT/'launch/pallet_staging_nav.launch.py')],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        self.alignment=PalletStagingNode(execute=execute)
        self.executor.add_node(self.alignment); self.extra.append(self.alignment)
        if execute:
            self.monitor=subprocess.Popen(['/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor',
                '--ros-args','-r','__node:=pallet_alignment_collision_monitor','--params-file',
                str(ROOT/'config/pallet_alignment_collision.yaml')],stdout=log,stderr=subprocess.STDOUT)
            change=self.node.create_client(ChangeState,'/pallet_alignment_collision_monitor/change_state')
            self.spin_until(change.service_is_ready)
            for transition in (1,3):
                req=ChangeState.Request(); req.transition.id=transition
                f=change.call_async(req); self.spin_until(f.done); self.assertTrue(f.result().success)
            stamper=Node('cmd_vel_stamper')
            for name,value in [('input_topic','/cmd_vel_safe'),('output_topic','/base_controller/cmd_vel'),
                               ('require_single_publisher',True)]:stamper.declare_parameter(name,value)
            pub=stamper.create_publisher(TwistStamped,'/base_controller/cmd_vel',1)
            def receive(msg):
                self.velocity=(msg.linear.x,msg.angular.z); self.received_velocity=time.monotonic()
                m=TwistStamped(); m.twist=msg; pub.publish(m)
            stamper.create_subscription(fixture.Twist,'/cmd_vel_safe',receive,1)
            self.executor.add_node(stamper); self.extra.append(stamper)
        self.spin_until(lambda:self.status is not None and self.status['ready'],timeout=25)


    def test_preview_plans_without_following_or_motor_publishers(self):
        self.start_stack(False)
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:not self.status['active'],timeout=15)
        self.assertEqual(self.status['state'],'PREVIEW_READY')
        self.assertIsNone(self.alignment.command)
        self.assertFalse(any(abs(v)+abs(w)>0 for v,w in self.commands))

    def test_transient_unknown_coverage_waits_for_new_grid_then_preview_passes(self):
        self.assert_unknown_grid_retry(persistent=False)

    def test_persistent_unknown_coverage_never_becomes_a_valid_path(self):
        self.assert_unknown_grid_retry(persistent=True)

    def assert_unknown_grid_retry(self,persistent):
        self.start_stack(False)
        original=self.alignment.schedule_route_validation;calls=[];unknown_stamps=[]
        def validate(path,retry_deadline=None):
            calls.append(self.alignment.grid_stamp)
            if persistent or len(calls)==1:
                grid=copy.copy(self.alignment.grid);grid.data=list(grid.data);grid.costs=None
                p=self.alignment.core.odom.samples[-1]
                i=int((p.x-grid.ox)/grid.res);j=int((p.y-grid.oy)/grid.res)
                grid.data[j*grid.width+i]=-1
                self.alignment.grid=grid
                unknown_stamps.append(self.alignment.grid_stamp)
            return original(path,retry_deadline)
        self.alignment.schedule_route_validation=validate
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:not self.status['active'],timeout=30)
        self.assertEqual(self.status['state'],'NO_SAFE_GOAL' if persistent else 'PREVIEW_READY')
        self.assertTrue(unknown_stamps)
        self.assertGreater(len(calls),1)
        self.assertGreater(calls[1],calls[0])
        if not persistent:self.assertEqual(self.status['candidate_index'],0)
        self.assertIsNone(self.alignment.command)
        self.assertFalse(any(abs(v)+abs(w)>0 for v,w in self.commands))

    def test_live_action_chain_reaches_normal_and_stops_with_new_rgbd(self):
        self.assert_live_alignment()
        self.assertLessEqual(max(abs(p['pose'][2]) for p in self.trajectory),math.radians(18))

    def test_larger_diagonal_is_refused_without_publishing_motion(self):
        self.pallet_y=-.11; self.pallet_heading=math.radians(1.2)
        self.start_stack(True);self.integrate=True
        self.assertFalse(self.service(True).success)
        self.spin_until(lambda:self.status['state']=='NO_SAFE_GOAL',timeout=5.)
        self.assertTrue(all(abs(v)+abs(w)==0 for v,w in self.commands))
        self.assertIn('≤12°',self.status['reason'])

    def test_actual_near_centred_observation_only_corrects_yaw_then_stops(self):
        self.pallet_x=1.7999;self.pallet_y=.0122933;self.pallet_heading=math.radians(3.1941)
        self.start_stack(True);self.integrate=True
        # Same lateral tolerance as the real staging YAML.
        from dataclasses import replace
        self.alignment.core.config=replace(self.alignment.core.config,lateral_tolerance_m=.015,
                                           heading_tolerance_deg=2.)
        followed=[]
        self.alignment.follow_checked_route=lambda path:followed.append(path)
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:self.status['state'] in ('YAW_CORRECTED','BLOCKED','TIMEOUT'),timeout=25.)
        self.assertEqual(self.status['state'],'YAW_CORRECTED',self.status)
        self.assertTrue(all(abs(v)==0 for v,w in self.commands))
        self.assertTrue(any(abs(w)>0 for v,w in self.commands))
        self.assertFalse(followed)
        self.assertLess(abs(self.pose[2]-self.pallet_heading),math.radians(.5))
        self.assertGreater(abs(self.status['errors']['lateral_m']),.015)

    def test_direct_diagonal_has_no_pinned_rotation_cusps_and_handles_pivot_drift(self):
        self.pallet_x=2.5
        self.pallet_y=-.11;self.pallet_heading=math.radians(1.2)
        self.pivot_drift_mps=.004
        self.start_stack(True);self.integrate=True
        captured=[];original=self.alignment.follow_checked_route
        def follow(path):
            captured.append(path);original(path)
        self.alignment.follow_checked_route=follow
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:self.status['state'] in ('ALIGNED','BLOCKED','NAV_FAILED','TIMEOUT',
            'NO_SAFE_GOAL','REACQUIRE_REQUIRED','LATERAL_NOT_REACHED'),timeout=85)
        self.assertEqual(self.status['state'],'ALIGNED',self.status)
        self.assertTrue(captured)
        for path in captured:
            for a,b in zip(path.poses,path.poses[1:]):
                self.assertGreater(math.hypot(a.pose.position.x-b.pose.position.x,
                                              a.pose.position.y-b.pose.position.y),.001)

    def test_stock_rpp_can_start_the_realistic_angular_deadband_without_stalling(self):
        self.pallet_x=2.5
        self.pallet_y=-.11;self.pallet_heading=math.radians(1.2)
        self.angular_deadband_rps=.06
        self.assert_live_alignment()
        self.assertGreater(max(abs(p['w']) for p in self.trajectory),.06)

    def test_already_on_axis_at_safe_standoff_only_trims_yaw(self):
        self.pallet_x=1.67;self.pallet_heading=math.radians(3)
        self.pallet_y=self.pallet_x*math.tan(self.pallet_heading)
        self.assert_live_alignment()
        self.assertTrue(all(abs(p['v'])<1e-9 for p in self.trajectory))

    def test_measured_yaw_overshoot_within_reserve_completes_alignment(self):
        self.pallet_x=1.67;self.pallet_heading=math.radians(12)
        self.pallet_y=self.pallet_x*math.tan(self.pallet_heading)
        self.target_visibility_yaw_deg=18;self.measured_yaw_scale=1.18
        self.assert_live_alignment()
        self.assertGreater(max(abs(p['w']) for p in self.trajectory),.10)
        self.assertTrue(all(abs(p['v'])<1e-9 for p in self.trajectory))

    def test_linear_and_yaw_tracking_overshoot_completes_alignment(self):
        self.pallet_y=-.11;self.pallet_heading=math.radians(1.2)
        self.measured_linear_scale=1.18;self.measured_yaw_scale=1.8
        self.assert_live_alignment()
        self.assertGreater(max(p['v'] for p in self.trajectory),.10)
        self.assertGreater(max(abs(p['w']) for p in self.trajectory),.10)

    def test_genuine_excess_velocity_stops_with_measured_values(self):
        self.pallet_x=1.67;self.pallet_heading=math.radians(12)
        self.pallet_y=self.pallet_x*math.tan(self.pallet_heading)
        self.target_visibility_yaw_deg=18;self.measured_yaw_scale=3.
        self.start_stack(True);self.integrate=True
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:self.status['state']=='BLOCKED',timeout=20)
        self.assertIn('v=',self.status['reason']);self.assertIn('w=',self.status['reason'])
        self.assertIn('18.00 cm',self.status['reason'])
        self.assertEqual(self.commands[-1],(0.,0.))

    def test_near_station_already_within_tolerances_settles_without_navigation(self):
        self.pallet_x=1.64244;self.pallet_y=-.00586;self.pallet_heading=math.radians(-.2)
        self.assert_live_alignment()
        self.assertTrue(all(abs(p['v'])+abs(p['w'])<1e-9 for p in self.trajectory))

    def test_live_stop_with_tiny_axis_offset_centres_opening_by_yaw_only(self):
        self.pallet_x=1.71;self.pallet_y=-.169;self.pallet_heading=math.radians(-5.25)
        self.target_visibility_yaw_deg=18
        self.assert_live_alignment()
        self.assertTrue(all(abs(p['v'])<1e-9 for p in self.trajectory))

    def assert_live_alignment(self):
        self.start_stack(True); self.integrate=True
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:self.status['state'] in ('ALIGNED','NAV_FAILED','BLOCKED','NO_SAFE_GOAL',
            'TIMEOUT','REACQUIRE_REQUIRED','LATERAL_NOT_REACHED','LOST_DATA'),timeout=85)
        Path('/tmp/pallet_diagonal_mock_trajectory.json').write_text(json.dumps(self.trajectory))
        self.assertEqual(self.status['state'],'ALIGNED',str(self.status))
        theta=getattr(self,'pallet_heading',0.)
        self.assertGreaterEqual(min((getattr(self,'pallet_x',2.)-p['pose'][0])*math.cos(theta)
                                    +(getattr(self,'pallet_y',.05)-p['pose'][1])*math.sin(theta)
                                    -.3655*math.cos(p['pose'][2]-theta)
                                    for p in self.trajectory),1.2)
        self.assertLess(abs(self.status['errors']['lateral_m']),.01)
        self.assertLess(abs(self.status['errors']['heading_deg']),1.)
        self.assertGreaterEqual(self.status['fork_plane_distance_m'],1.2)
        self.assertFalse(self.status['active']); self.assertEqual(self.commands[-1],(0.,0.))

    def test_stop_cancels_followpath_and_ignores_late_commands(self):
        self.start_stack(True)
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:self.status['state']=='NAVIGATING',timeout=15)
        self.assertTrue(self.service(False).success)
        self.spin_until(lambda:not self.alignment.cancel_pending and self.alignment.operation is None)
        msg=fixture.Twist(); msg.linear.x=.08
        self.alignment.on_nav_command(msg)
        self.assertFalse(self.alignment.core.active)
        self.assertEqual((self.alignment.core.v,self.alignment.core.w),(0.,0.))
        self.assertEqual(self.commands[-1],(0.,0.))

    def test_reverse_at_axis_cancels_real_followpath_then_trims_with_fresh_rgbd(self):
        self.target_visibility_yaw_deg=45
        self.start_stack(True);self.integrate=True
        self.assertTrue(self.service(True).success)
        def at_axis():
            c=self.alignment.core
            if c.state!='NAVIGATING' or self.alignment.operation!='following':return False
            p=c.odom.samples[-1]
            return on_approach_axis(c.frozen_target,(p.x,p.y,p.yaw))
        self.spin_until(at_axis,timeout=25)
        command=fixture.Twist();command.linear.x=-.04
        self.alignment.on_nav_command(command)
        self.assertEqual(self.alignment.core.state,'NAV_CANCELING')
        self.assertEqual((self.alignment.core.v,self.alignment.core.w),(0.,0.))
        command.linear.x=.08;self.alignment.on_nav_command(command)
        self.assertEqual((self.alignment.core.nav_v,self.alignment.core.nav_w),(0.,0.))
        self.spin_until(lambda:self.status['state'] in ('ALIGNED','BLOCKED','NAV_FAILED',
            'REACQUIRE_REQUIRED','NO_SAFE_GOAL','TIMEOUT','LATERAL_NOT_REACHED'),timeout=50)
        self.assertEqual(self.status['state'],'ALIGNED',str(self.status))
        self.assertTrue(self.status['stationary_arrival_confirmed'])
        self.assertFalse(self.alignment.cancel_pending)
        self.assertIsNone(self.alignment.handoff_result_future)
        self.assertLess(abs(self.status['errors']['lateral_m']),.02)
        self.assertLess(abs(self.status['errors']['heading_deg']),2.)
        self.assertGreaterEqual(self.status['fork_plane_distance_m'],1.2)
        self.assertTrue(all(v>=0 for v,w in self.commands))
        self.assertEqual(self.commands[-1],(0.,0.))

    def test_laser_loss_cancels_active_nav(self):
        self.start_stack(True)
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:self.status['state']=='NAVIGATING',timeout=15)
        self.emit_scan=False
        self.spin_until(lambda:not self.status['active'])
        self.assertEqual(self.status['state'],'BLOCKED')
        self.assertEqual(self.commands[-1],(0.,0.))
        self.spin_until(lambda:not self.alignment.cancel_pending and self.alignment.operation is None)

    def test_arrival_with_invalid_rgbd_requests_reacquisition_without_navigation_timeout(self):
        self.start_stack(True);self.integrate=True
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:self.status['state']=='NAVIGATING',timeout=15)
        self.invalidate_targets=True
        self.spin_until(lambda:self.status['state']=='REACQUIRE_REQUIRED',timeout=40)
        self.assertTrue(self.status['stationary_arrival_confirmed'])
        self.assertFalse(self.status['active']);self.assertFalse(self.status['nav_snapshot'])
        self.assertEqual(self.commands[-1],(0.,0.))

    def test_sparse_unobserved_cells_do_not_become_free_space(self):
        self.scan_count=720
        self.start_stack(False)
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:not self.status['active'],timeout=15)
        self.assertEqual(self.status['state'],'NO_SAFE_GOAL')
        self.assertFalse(any(abs(v)+abs(w)>0 for v,w in self.commands))

    def test_heading_deviation_stops_navigation_instead_of_pivoting_sideways(self):
        self.start_stack(True)
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda:self.status['state']=='NAVIGATING',timeout=15)
        # Drift in small increments so the separate odometry-reset guard
        # does not mask the heading-envelope guard being exercised here.
        for degrees in (6,12,19):
            self.pose[2]=math.radians(degrees)
            self.spin_until(lambda:bool(self.alignment.core.odom.samples) and
                abs(self.alignment.core.odom.samples[-1].yaw-self.pose[2])<.001)
        self.spin_until(lambda:not self.status['active'])
        self.assertEqual(self.status['state'],'BLOCKED')
        self.assertIn('Hướng xe lệch khỏi đường đã kiểm tra',self.status['reason'])
        self.assertEqual(self.commands[-1],(0.,0.))
        self.spin_until(lambda:not self.alignment.cancel_pending and self.alignment.operation is None)

    def tearDown(self):
        if hasattr(self,'alignment'):
            self.alignment.core.stop(); self.alignment.zero(); self.alignment.cancel_actions()
        for p in getattr(self,'processes',[]):
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL); p.wait()
        self.inputs_stop.set();self.inputs_thread.join(timeout=3.)
        self.inputs_executor.remove_node(self.node)
        self.inputs_executor.shutdown()
        fixture.AlignmentRosTest.tearDown(self)
        for log in getattr(self,'logs',[]):log.close()
        os.environ['ROS_DOMAIN_ID']=self.previous_domain


class ReverseHandoffTest(unittest.TestCase):
    def test_reverse_is_never_published_and_only_safe_axis_arrival_is_handed_off(self):
        for x,y,expected in ((1.68,0.,'NAV_CANCELING'),(1.68,.03,'BLOCKED'),
                             (1.5,0.,'BLOCKED')):
            with self.subTest(x=x,y=y):
                core=StagingController();core.active=True;core.state='NAVIGATING'
                core.frozen_target=(x,y,0.);core.odom.add(OdomSample(1.,0.,0.,0.))
                zeros=[];canceled=[]
                node=SimpleNamespace(core=core,operation='following',raw_after=0.,
                                     result_future=object(),
                                     cancel_actions=lambda:canceled.append(True),
                                     now=lambda:1.,zero=lambda:zeros.append(True))
                command=fixture.Twist();command.linear.x=-.08
                PalletStagingNode.on_nav_command(node,command)
                self.assertEqual(core.state,expected)
                self.assertEqual(core.active,expected=='NAV_CANCELING')
                self.assertEqual(bool(canceled),expected=='NAV_CANCELING')
                self.assertEqual((core.v,core.w),(0.,0.))
                self.assertTrue(zeros)

    def test_handoff_waits_for_both_cancel_response_and_terminal_result(self):
        for terminal,expected in ((5,'NAV_STOPPING'),(4,'NAV_STOPPING'),(6,'BLOCKED')):
            core=StagingController();core.active=True;core.state='NAVIGATING'
            core.frozen_target=(1.68,0.,0.);core.odom.add(OdomSample(1.,0.,0.,0.))
            self.assertTrue(core.begin_axis_handoff(time.monotonic()))
            done=[False];ack=[False]
            result=SimpleNamespace(done=lambda:done[0],result=lambda:SimpleNamespace(status=terminal))
            cancel=SimpleNamespace(done=lambda:ack[0],result=lambda:SimpleNamespace(return_code=0))
            n=SimpleNamespace(core=core,cancel_pending=[(cancel,time.monotonic()+2.)],cancel_failed=False,
                handoff_result_future=result,now=lambda:1.,zero=lambda:None)
            PalletStagingNode.sync_navigation(n)
            self.assertEqual(core.state,'NAV_CANCELING')
            done[0]=True;PalletStagingNode.sync_navigation(n)
            self.assertEqual(core.state,'NAV_CANCELING')
            ack[0]=True;PalletStagingNode.sync_navigation(n)
            self.assertEqual(core.state,expected)
            self.assertFalse(core.stationary_arrival_confirmed)
            self.assertEqual((core.v,core.w),(0.,0.))

    def test_failed_cancel_cannot_become_a_stationary_arrival(self):
        core=StagingController();core.active=True;core.state='NAVIGATING'
        core.frozen_target=(1.68,0.,0.);core.odom.add(OdomSample(1.,0.,0.,0.))
        self.assertTrue(core.begin_axis_handoff(time.monotonic()))
        cancel=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(return_code=1))
        n=SimpleNamespace(core=core,cancel_pending=[(cancel,time.monotonic()+2.)],cancel_failed=False,
            handoff_result_future=None,now=lambda:1.,zero=lambda:None)
        PalletStagingNode.sync_navigation(n)
        self.assertEqual(core.state,'BLOCKED')
        self.assertFalse(core.stationary_arrival_confirmed)
        self.assertEqual((core.v,core.w),(0.,0.))


if __name__=='__main__':unittest.main()
