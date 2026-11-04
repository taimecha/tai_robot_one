# Historical tests for the retired pickup workflow.
"""Actual Collision Monitor + fake base/lift in domain 219. No device drivers."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest

os.environ['ROS_DOMAIN_ID']='219'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist,TwistStamped,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String,Float64MultiArray,Empty
from std_srvs.srv import Trigger,SetBool
from lifecycle_msgs.srv import ChangeState
from tf2_ros import TransformBroadcaster
from detected_pallets_interfaces.msg import PalletTarget
from handle_pallet import PalletHandlingNode


class PickupRosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.exe=SingleThreadedExecutor()
        self.input=Node('pickup_test_inputs')
        self.nodes=[self.input];self.monitor=None;self.extra=[]
        self.pose=[0.,.003,math.radians(.1)];self.velocity=(0.,0.)
        self.command=(0.,0.);self.command_wall=0.
        self.height=0.;self.lift_target=None;self.lift_wall=0.;self.lift_seen=[];self.stops=0
        self.status=None;self.last=time.monotonic();self.emit_scan=True;self.emit_odom=True
        self.emit_operator=True;self.emit_recognition=True;self.raise_fault=False;self.obstacle=False
        self.rgbd_stable=True
        self.cycle_token=None;self.emit_cycle=True
        self.force_camera_lost=False
        self.trace=[];self.run_id=None
        self.odom=self.input.create_publisher(Odometry,'/odom',1)
        self.scan=self.input.create_publisher(LaserScan,'/scan_filtered',1)
        self.target=self.input.create_publisher(PalletTarget,'/pallet/target',1)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.yolo=self.input.create_publisher(Bool,'/pallet/enabled',latched)
        self.recognition=self.input.create_publisher(String,'/pallet/recognition/status',latched)
        self.heartbeat=self.input.create_publisher(String,'/pallet/handling/heartbeat',1)
        self.cycle_heartbeat=self.input.create_publisher(String,'/pallet/cycle/heartbeat',1)
        self.tf=TransformBroadcaster(self.input)
        self.input.create_subscription(String,'/pallet/handling/status',self.on_status,latched)
        self.input.create_timer(.05,self.inputs)
        self.bridge=Node('esp32_imu_bridge');self.nodes.append(self.bridge)
        self.feedback=self.bridge.create_publisher(String,'/lift/status',1)
        self.bridge.create_subscription(Float64MultiArray,'/lift_controller/commands',self.lift_command,1)
        self.bridge.create_subscription(Empty,'/lift/stop',self.stop_lift,1)
        # On the robot, odometry/laser/lift producers run independently of the
        # handling executor. Do the same here: a collision-check callback must
        # not prevent the fake sensors from producing their next timestamp.
        self.inputs_exe=SingleThreadedExecutor()
        self.inputs_exe.add_node(self.input);self.inputs_exe.add_node(self.bridge)
        self.inputs_stop=threading.Event()
        def spin_inputs():
            while not self.inputs_stop.is_set():self.inputs_exe.spin_once(timeout_sec=.02)
        self.inputs_thread=threading.Thread(target=spin_inputs)
        self.inputs_thread.start()

    def on_status(self,msg):self.status=json.loads(msg.data)

    def lift_command(self,msg):
        self.lift_target=msg.data[0];self.lift_wall=time.monotonic();self.lift_seen.append(self.lift_target)

    def stop_lift(self,msg):self.lift_target=None;self.stops+=1

    def receive(self,msg):self.command=(msg.linear.x,msg.angular.z);self.command_wall=time.monotonic()

    def inputs(self):
        now=time.monotonic();dt=min(.08,now-self.last);self.last=now
        v,w=self.velocity;tv,tw=self.command if now-self.command_wall<.25 else (0.,0.)
        v+=max(-.15*dt,min(.3*dt,tv-v));w+=max(-.4*dt,min(.4*dt,tw-w))
        self.velocity=(v,w)
        self.pose[0]+=v*math.cos(self.pose[2]+w*dt/2)*dt
        self.pose[1]+=v*math.sin(self.pose[2]+w*dt/2)*dt;self.pose[2]+=w*dt
        moving_lift=False
        lift_velocity=0.
        if self.lift_target is not None and now-self.lift_wall<.35 and abs(self.height-self.lift_target)>.00001:
            delta=self.lift_target-self.height
            self.height+=math.copysign(min(.01*dt,abs(delta)),delta)
            moving_lift=abs(self.height-self.lift_target)>.0001
            lift_velocity=math.copysign(.01,delta) if moving_lift else 0.
        self.feedback.publish(String(data=json.dumps(dict(serial_connected=True,sample_ms=int(now*1000)%2**32,
            homed=True,state='MOVING' if moving_lift else 'IDLE',position=self.height,
            target=self.lift_target if self.lift_target is not None else self.height,velocity=lift_velocity,
            lower_active=self.height<.001,upper_active=False,fault='TEST' if self.raise_fault else 'NONE'))))
        stamp=self.input.get_clock().now().to_msg()
        od=Odometry();od.header.frame_id='odom';od.header.stamp=stamp;od.child_frame_id='base_footprint'
        od.pose.pose.position.x,od.pose.pose.position.y=self.pose[:2]
        od.pose.pose.orientation.z,od.pose.pose.orientation.w=math.sin(self.pose[2]/2),math.cos(self.pose[2]/2)
        od.twist.twist.linear.x,od.twist.twist.angular.z=v,w
        if self.emit_odom:self.odom.publish(od)
        tf=TransformStamped();tf.header=od.header;tf.child_frame_id='base_footprint'
        tf.transform.translation.x,tf.transform.translation.y=self.pose[:2];tf.transform.rotation=od.pose.pose.orientation
        self.tf.sendTransform(tf)
        scan=LaserScan();scan.header=od.header;scan.header.frame_id='base_footprint'
        scan.angle_min=-math.pi;scan.angle_increment=math.pi/720;scan.angle_max=math.pi
        scan.range_min=.05;scan.range_max=10.;ranges=[5.]*1441
        if self.obstacle:ranges[718:723]=[.9]*5
        scan.ranges=ranges
        if self.emit_scan:self.scan.publish(scan)
        visible=self.pose[0]<getattr(self,'camera_visibility_limit',.4) and not self.force_camera_lost
        msg=PalletTarget();msg.header=scan.header;msg.header.frame_id='camera_color_optical_frame'
        msg.valid=visible;msg.status='TRACKED_RGBD; target_bias_applied' if visible else 'LOST: clipped by near view'
        dx,dy=1.6555-self.pose[0],-self.pose[1]
        msg.chassis_front_forward_m=math.cos(self.pose[2])*dx+math.sin(self.pose[2])*dy-.3005
        msg.opening_lateral_m=-math.sin(self.pose[2])*dx+math.cos(self.pose[2])*dy
        msg.entry_heading_rad=-self.pose[2];msg.opening_width_m=.245
        msg.face_yaw_rad=self.pose[2];msg.leg_center_separation_m=.31
        msg.camera_forward_m=msg.chassis_front_forward_m+.0665
        msg.left_depth_samples=msg.right_depth_samples=100
        self.target.publish(msg)
        if self.emit_recognition:
            self.yolo.publish(Bool(data=False))
            self.recognition.publish(String(data=json.dumps(dict(enabled=False,busy=False,locked=visible,
                phase='TRACKING' if visible else 'LOST',measurement_ready=visible,
                rgbd_health=dict(stable=visible and self.rgbd_stable)))))
        if self.emit_operator and self.run_id:self.heartbeat.publish(String(data=self.run_id))
        if self.emit_cycle and self.cycle_token:
            self.cycle_heartbeat.publish(String(data=self.cycle_token))
        self.trace.append((now,list(self.pose),v,w,self.height,self.status['state'] if self.status else None))

    def spin(self,condition,timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.exe.spin_once(timeout_sec=.01)
            if condition():return
        self.fail('Timed out: '+str(self.status)+'; sensor='+self.handler.sensor_issue()+
                  '; confirmed='+str(self.handler.core.confirmed))

    def service(self,name,raise_after=None):
        typ=Trigger if raise_after is None else SetBool
        client=self.input.create_client(typ,'/pallet/handling/'+name)
        self.spin(client.service_is_ready)
        req=typ.Request()
        if raise_after is not None:req.data=raise_after
        future=client.call_async(req);self.spin(future.done)
        return future.result()

    def stack(self,execute=True,pickup_config=None,cycle_token=None):
        options={'pickup_config':pickup_config} if pickup_config is not None else {}
        if cycle_token:options['cycle_token']=cycle_token
        self.handler=PalletHandlingNode(execute=execute,**options);self.nodes.append(self.handler);self.exe.add_node(self.handler)
        if execute:
            self.log=open('/tmp/pallet_pickup_collision_test.log','w')
            self.monitor=subprocess.Popen(['/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor',
                '--ros-args','-r','__node:=pallet_alignment_collision_monitor','--params-file',
                str(ROOT/'config/pallet_alignment_collision.yaml')],stdout=self.log,stderr=subprocess.STDOUT)
            change=self.input.create_client(ChangeState,'/pallet_alignment_collision_monitor/change_state')
            self.spin(change.service_is_ready)
            for transition in (1,3):
                req=ChangeState.Request();req.transition.id=transition
                future=change.call_async(req);self.spin(future.done);self.assertTrue(future.result().success)
            stamper=Node('cmd_vel_stamper');self.nodes.append(stamper);self.exe.add_node(stamper)
            for key,value in [('input_topic','/cmd_vel_safe'),('output_topic','/base_controller/cmd_vel'),
                    ('require_single_publisher',True)]:stamper.declare_parameter(key,value)
            final=stamper.create_publisher(TwistStamped,'/base_controller/cmd_vel',1)
            def recv(msg):
                self.receive(msg);m=TwistStamped();m.twist=msg;final.publish(m)
            stamper.create_subscription(Twist,'/cmd_vel_safe',recv,1)
        self.spin(lambda:self.handler.core.confirmed>=3 and self.handler.sensor_issue()=='',timeout=15)
        # Fixture represents a completed staging phase; staging itself is
        # covered separately by test_staging_ros with actual Nav2.
        self.handler.core.state='ALIGNED'

    def start(self,raise_after=True):
        result=self.service('start',raise_after)
        self.assertTrue(result.success,result.message);self.run_id=result.message
        self.spin(lambda:self.status and self.status['active'])

    def test_preview_has_no_motor_or_lift_publishers_and_recomputes_on_start(self):
        self.stack(False)
        self.assertIsNone(self.handler.command);self.assertIsNone(self.handler.lift_command)
        self.assertTrue(self.service('preview').success)
        self.assertFalse(self.service('start',True).success)
        self.assertFalse(self.handler.pickup.active)

    def test_stale_aligned_state_cannot_authorize_outside_one_cm_or_one_degree(self):
        self.stack(False)
        for lateral,heading in ((.012,0.),(0.,math.radians(1.2))):
            self.pose[1]=lateral;self.pose[2]=heading
            self.spin(lambda:self.handler.core.odom.samples and
                abs(self.handler.core.odom.samples[-1].y-lateral)<.001 and
                abs(self.handler.core.odom.samples[-1].yaw-heading)<.001)
            # Wait for new targets to catch up with independent odometry.
            end=time.monotonic()+.8
            while time.monotonic()<end:self.exe.spin_once(timeout_sec=.01)
            self.handler.core.state='ALIGNED'
            result=self.service('preview')
            self.assertFalse(result.success)
            self.assertIn('1 cm',result.message)
            self.assertFalse(self.handler.pickup.active)
            self.assertIsNone(self.handler.lift_target if hasattr(self.handler,'lift_target') else None)

    def test_actual_collision_chain_completes_after_camera_loses_pallet(self):
        self.stack();self.emit_operator=False
        self.cli_log=open('/tmp/pallet_pickup_operator_test.log','w')
        self.cli=subprocess.Popen(['bash',str(ROOT/'scripts/open_pallet_pickup.sh'),'insert-and-lift'],
            stdout=self.cli_log,stderr=subprocess.STDOUT)
        self.spin(lambda:self.status['active'],timeout=18)
        self.spin(lambda:self.status['state'] in ('COMPLETE','BLOCKED','TIMEOUT'),timeout=36)
        self.assertEqual(self.status['state'],'COMPLETE',self.status)
        self.assertGreater(self.pose[0],1.20);self.assertLess(abs(self.pose[1]),.02)
        self.assertAlmostEqual(self.height,.08,delta=.003)
        self.assertTrue(self.lift_seen)
        self.assertTrue(all(abs(v)<.005 and abs(w)<.025 for _,p,v,w,h,s in self.trace if h>.001))
        self.assertFalse(self.handler.recognition['measurement_ready'])
        self.spin(lambda:self.cli.poll() is not None)
        self.assertEqual(self.cli.returncode,0)

    def test_lost_operator_lease_stops_without_lifting(self):
        self.stack();self.start();self.spin(lambda:self.pose[0]>.02)
        self.emit_operator=False
        self.spin(lambda:self.status['state']=='BLOCKED')
        self.spin(lambda:abs(self.velocity[0])<.003)
        self.assertFalse(self.lift_seen)
        self.assertIn('heartbeat',self.status['reason'])

    def test_cycle_lease_loss_stops_insertion_and_does_not_resume(self):
        self.cycle_token='cycle-test-identity'
        self.stack(cycle_token=self.cycle_token);self.start()
        self.spin(lambda:self.pose[0]>.02)
        self.emit_cycle=False
        self.spin(lambda:self.status['state']=='BLOCKED')
        self.spin(lambda:abs(self.velocity[0])<.003)
        self.assertIn('heartbeat chu trình',self.status['reason'])
        stopped=self.pose[0];self.emit_cycle=True
        end=time.monotonic()+1.
        while time.monotonic()<end:self.exe.spin_once(timeout_sec=.01)
        self.assertFalse(self.handler.pickup.active)
        self.assertLess(abs(self.pose[0]-stopped),.005)
        self.assertFalse(self.lift_seen)

    def test_cycle_lease_cancels_pending_alignment_before_motion(self):
        from rclpy.task import Future
        self.cycle_token='cycle-prepare-test'
        self.stack(False,cycle_token=self.cycle_token)
        waiter=Future();self.handler.preparing=True
        self.handler.start_waiter=(waiter,time.monotonic()+5.)
        epoch=self.handler.start_epoch
        self.emit_cycle=False
        self.spin(lambda:self.handler.core.state=='BLOCKED')
        self.assertTrue(waiter.done());self.assertFalse(waiter.result()[0])
        self.assertFalse(self.handler.preparing)
        self.assertGreater(self.handler.start_epoch,epoch)
        self.assertFalse(self.lift_seen)

    def test_real_config_fine_approach_completes_through_collision_monitor(self):
        import yaml
        from pallet_insertion_core import InsertionConfig,opening_coordinates
        from pallet_pickup_core import PickupConfig
        values=yaml.safe_load((ROOT/'config/pallet_pickup.yaml').read_text())
        values['insertion']=InsertionConfig(**values['insertion'])
        self.stack(pickup_config=PickupConfig(**values));self.start()
        self.spin(lambda:self.status['state'] in ('COMPLETE','BLOCKED','TIMEOUT'),timeout=40)
        self.assertEqual(self.status['state'],'COMPLETE',self.status)
        opening=self.handler.pickup.plan.opening
        gap=-opening_coordinates(opening,self.pose)[0]-.3655*math.cos(self.pose[2]-opening.yaw)
        self.assertGreaterEqual(gap,.01)
        self.assertLessEqual(gap,.025)
        self.assertAlmostEqual(self.height,.08,delta=.003)
        self.assertTrue(all(abs(v)<.005 and abs(w)<.025 for _,p,v,w,h,s in self.trace if h>.001))

    def test_completed_pickup_lower_then_advance_three_cm_and_raise(self):
        import yaml
        from pallet_insertion_core import InsertionConfig,Opening,make_plan
        from pallet_pickup_core import PickupConfig
        values=yaml.safe_load((ROOT/'config/pallet_pickup.yaml').read_text())
        values['insertion']=InsertionConfig(**values['insertion'])
        self.pose=[0.,0.,0.];self.height=.08
        self.stack(pickup_config=PickupConfig(**values))
        # Actual stopped geometry includes ~5-mm lateral error and .36-deg
        # yaw. A 3-cm depth adjustment must not try to centre this by turning.
        self.handler.core.state='IDLE'
        self.handler.recovery_checkpoint=dict(completed_insertion=True,frozen_trajectory=True,
            state='COMPLETE',run_id='completed_test_run',created_wall=time.monotonic(),
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            relative_opening=dict(x=.4162488,y=.00234794,yaw=-.0062327,width_m=.245),
            entry_height_m=0.,completed_height_m=.08)
        self.handler.recovery_anchor=tuple(self.pose)
        self.assertTrue(self.service('lower_preview').success)
        result=self.service('lower');self.assertTrue(result.success,result.message);self.run_id=result.message
        self.spin(lambda:self.status['state'] in ('LOWERED','BLOCKED','TIMEOUT'),timeout=16)
        self.assertEqual(self.status['state'],'LOWERED',self.status)
        self.assertLess(abs(self.pose[0]),.001)
        self.assertAlmostEqual(self.height,0.,delta=.002)
        result=self.service('resume',True);self.assertTrue(result.success,result.message);self.run_id=result.message
        self.spin(lambda:self.status['state'] in ('COMPLETE','BLOCKED','TIMEOUT'),timeout=20)
        self.assertEqual(self.status['state'],'COMPLETE',self.status)
        self.assertAlmostEqual(self.pose[0],.03,delta=.003)
        self.assertAlmostEqual(self.height,.08,delta=.002)

    def test_explicit_lift_checkpoint_requires_terminal_geometry_and_no_base_motion(self):
        import yaml
        from pallet_insertion_core import InsertionConfig
        from pallet_pickup_core import PickupConfig
        values=yaml.safe_load((ROOT/'config/pallet_pickup.yaml').read_text())
        values['insertion']=InsertionConfig(**values['insertion'])
        self.pose=[0.,0.,0.]
        self.stack(pickup_config=PickupConfig(**values));self.handler.core.state='IDLE'
        self.handler.recovery_anchor=tuple(self.pose)
        cp=dict(lift_only=True,frozen_trajectory=True,state='BLOCKED',run_id='terminal_test',
            created_wall=time.monotonic(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            relative_opening=dict(x=.41665,y=.002,yaw=0.,width_m=.245),entry_height_m=0.,
            source_state={'reason':'Xe không tiến; có thể bị LiDAR chặn hoặc truyền động đứng'})
        self.handler.recovery_checkpoint=cp
        result=self.service('lift');self.assertFalse(result.success)
        self.assertFalse(self.lift_seen)
        cp['relative_opening']['x']=.38665
        self.spin(lambda:len(self.handler.core.odom.samples)>30)
        result=self.service('lift');self.assertTrue(result.success,result.message);self.run_id=result.message
        self.spin(lambda:self.status['state'] in ('COMPLETE','BLOCKED','TIMEOUT'),timeout=14)
        self.assertEqual(self.status['state'],'COMPLETE',self.status)
        self.assertLess(abs(self.pose[0]),.001)
        self.assertAlmostEqual(self.height,.08,delta=.002)

    def test_invalid_scan_metadata_is_rejected_before_motion(self):
        self.stack(False)
        self.handler.pickup_scan.angle_min=float('nan')
        with self.assertRaisesRegex(ValueError,'Laser không hợp lệ'):
            self.handler.laser_points()

    def test_scan_waits_for_capture_time_tf_without_accepting_stale_data(self):
        import copy
        self.stack(False)
        self.spin(lambda:len(self.handler.pickup_scans)>=3)
        self.handler.laser_points()
        msg=copy.deepcopy(self.handler.pickup_scan)
        stamp_ns=int(self.handler.now()*1e9)+30000000
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp_ns,1000000000)
        self.handler.on_pickup_scan(msg)
        # Current scan leads TF by 30 ms; use the previous fresh mapped scan.
        self.assertGreater(len(self.handler.laser_points()),0)
        for scan,received in self.handler.pickup_scans:scan.header.stamp.sec-=1
        with self.assertRaises(ValueError):self.handler.laser_points()

    def test_laser_loss_and_new_obstacle_stop_before_any_lift(self):
        self.stack();self.start();self.spin(lambda:self.pose[0]>.02)
        self.emit_scan=False
        self.spin(lambda:self.status['state']=='BLOCKED')
        self.assertFalse(self.lift_seen)
        self.assertIn('laser',self.status['reason'].lower())

    def test_new_obstacle_in_full_fork_footprint_stops(self):
        self.stack();self.start();self.spin(lambda:self.pose[0]>.02)
        self.obstacle=True
        self.spin(lambda:abs(self.command[0])<.001)
        # CM's larger 15-cm StopZone may stop before the smaller full-body
        # checker. The no-progress watchdog then disarms the operation.
        self.spin(lambda:self.status['state']=='BLOCKED',timeout=8)
        self.assertFalse(self.lift_seen)

    def test_stop_service_during_lift_stops_both_axes(self):
        self.stack();self.start(False)
        self.spin(lambda:self.status['state'] in ('INSERTED','BLOCKED'),timeout=25)
        self.assertEqual(self.status['state'],'INSERTED',self.status)
        self.assertFalse(self.lift_seen)
        result=self.service('lift');self.assertTrue(result.success,result.message);self.run_id=result.message
        self.spin(lambda:self.height>.002 or self.status['state']=='BLOCKED')
        self.assertGreater(self.height,.002,self.status)
        self.assertTrue(self.service('stop').success)
        self.spin(lambda:self.status['state']=='STOPPED')
        self.assertIsNone(self.handler.pickup.lift_target)
        self.spin(lambda:self.stops>0)
        self.assertEqual(self.command,(0.,0.))

    def test_ctrl_c_operator_terminal_sends_stop(self):
        self.stack();self.emit_operator=False
        self.cli_log=open('/tmp/pallet_pickup_operator_interrupt_test.log','w')
        self.cli=subprocess.Popen(['bash',str(ROOT/'scripts/open_pallet_pickup.sh'),'insert'],
            stdout=self.cli_log,stderr=subprocess.STDOUT)
        self.spin(lambda:self.pose[0]>.025,timeout=8)
        import signal
        self.cli.send_signal(signal.SIGINT)
        self.spin(lambda:self.status['state']=='STOPPED')
        self.spin(lambda:abs(self.velocity[0])<.003)
        self.assertFalse(self.lift_seen)

    def waiting_cli(self,timeout=15):
        self.rgbd_stable=False
        self.stack();self.emit_operator=False
        self.cli_log=open('/tmp/pallet_pickup_operator_wait_test.log','w')
        self.cli=subprocess.Popen(['bash',str(ROOT/'scripts/open_pallet_pickup.sh'),
            'insert-and-lift','--ready-timeout',str(timeout)],
            stdout=self.cli_log,stderr=subprocess.STDOUT)
        self.spin(lambda:'Chờ cửa sổ RGB-D' in Path(self.cli_log.name).read_text())
        self.assertEqual(self.handler.pickup.state,'IDLE')
        self.assertEqual(self.handler.core.state,'ALIGNED')
        self.assertEqual(self.command,(0.,0.))
        self.assertFalse(self.lift_seen)

    def test_operator_waits_for_rgbd_then_starts_once(self):
        self.waiting_cli()
        self.rgbd_stable=True
        self.spin(lambda:self.pose[0]>.025,timeout=8)
        import signal
        self.cli.send_signal(signal.SIGINT)
        self.spin(lambda:self.status['state']=='STOPPED')
        self.spin(lambda:self.cli.poll() is not None)
        text=Path(self.cli_log.name).read_text()
        self.assertEqual(text.count('Đã nhận lệnh.'),1)
        self.assertFalse(self.lift_seen)

    def test_ctrl_c_while_waiting_never_sends_start_or_stop(self):
        self.waiting_cli()
        stops=self.stops
        import signal
        self.cli.send_signal(signal.SIGINT)
        self.spin(lambda:self.cli.poll() is not None)
        self.rgbd_stable=True
        end=time.monotonic()+.7
        self.spin(lambda:time.monotonic()>=end)
        self.assertEqual(self.handler.pickup.state,'IDLE')
        self.assertEqual(self.handler.core.state,'ALIGNED')
        self.assertEqual(self.stops,stops)
        self.assertEqual(self.command,(0.,0.))
        self.assertFalse(self.lift_seen)

    def test_operator_readiness_timeout_never_sends_start(self):
        self.waiting_cli(timeout=1)
        self.spin(lambda:self.cli.poll() is not None)
        self.assertNotEqual(self.cli.returncode,0)
        self.assertIn('Hết thời gian chờ',Path(self.cli_log.name).read_text())
        self.assertEqual(self.handler.pickup.state,'IDLE')
        self.assertEqual(self.handler.core.state,'ALIGNED')
        self.assertFalse(self.lift_seen)

    def test_resume_requires_prior_run_fresh_rgbd_and_rechecks_remaining_path(self):
        self.stack()
        denied=self.service('resume',True)
        self.assertFalse(denied.success)
        self.start();self.spin(lambda:self.pose[0]>.08)
        self.emit_scan=False;self.spin(lambda:self.status['state']=='BLOCKED')
        self.spin(lambda:abs(self.velocity[0])<.003)
        self.assertFalse(self.lift_seen)
        denied=self.service('resume',True)
        self.assertFalse(denied.success)
        self.emit_scan=True
        self.spin(lambda:self.handler.sensor_issue()=='' and self.handler.core.confirmed>=3)
        self.obstacle=True
        self.spin(lambda:any(r<1 for r in self.handler.pickup_scan.ranges))
        self.assertFalse(self.service('resume',True).success)
        self.obstacle=False
        self.spin(lambda:all(r>=1 for r in self.handler.pickup_scan.ranges))
        self.force_camera_lost=True
        self.spin(lambda:self.handler.recognition.get('phase')=='LOST')
        result=self.service('resume',True)
        self.assertTrue(result.success,result.message);self.run_id=result.message
        self.spin(lambda:self.status.get('run_id')==self.run_id
                  and self.status['state'] in ('COMPLETE','BLOCKED','TIMEOUT'),timeout=35)
        self.assertEqual(self.status['state'],'COMPLETE',self.status)
        self.assertAlmostEqual(self.height,.08,delta=.003)

    def test_restarted_owner_needs_valid_checkpoint_and_fresh_pallet(self):
        from pallet_pickup_recovery import relative_opening
        from pallet_insertion_core import Opening
        self.stack();self.handler.core.state='IDLE'
        opening=Opening(*self.handler.core.target,.245)
        expected=relative_opening(opening,tuple(self.pose))
        checkpoint=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            created_wall=time.monotonic(),state='BLOCKED',run_id='old_operator_run',
            relative_opening=expected,entry_height_m=0.)
        self.handler.recovery_checkpoint=checkpoint
        self.assertTrue(self.service('resume_preview').success)
        self.assertEqual(self.command,(0.,0.))
        self.assertFalse(self.lift_seen)
        checkpoint['created_wall']-=1801
        self.assertFalse(self.service('resume_preview').success)
        checkpoint['created_wall']=time.monotonic();checkpoint['relative_opening']['y']+=.04
        self.assertFalse(self.service('resume_preview').success)

    def test_frozen_restart_is_anchored_expires_and_does_not_need_new_camera_pose(self):
        from pallet_pickup_recovery import relative_opening
        from pallet_insertion_core import Opening
        self.stack();self.handler.core.state='IDLE'
        expected=relative_opening(Opening(*self.handler.core.target,.245),tuple(self.pose))
        checkpoint=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            created_wall=time.monotonic(),state='BLOCKED',run_id='stopped_run',frozen_trajectory=True,
            relative_opening=expected,entry_height_m=0.)
        self.handler.recovery_checkpoint=checkpoint;self.force_camera_lost=True
        self.spin(lambda:self.handler.recovery_anchor is not None
                  and self.handler.recognition.get('phase')=='LOST')
        self.assertTrue(self.service('resume_preview').success)
        self.assertEqual(self.command,(0.,0.));self.assertFalse(self.lift_seen)
        checkpoint['created_wall']-=601
        self.assertFalse(self.service('resume_preview').success)
        checkpoint['created_wall']=time.monotonic();self.handler.recovery_anchor_invalid=True
        self.assertFalse(self.service('resume_preview').success)

    def tearDown(self):
        if hasattr(self,'cli'):
            if self.cli.poll() is None:self.cli.terminate()
            try:self.cli.wait(timeout=3)
            except subprocess.TimeoutExpired:self.cli.kill();self.cli.wait()
            self.cli_log.close()
        if hasattr(self,'handler'):
            self.handler.core.stop();self.handler.abort_pickup('test teardown','STOPPED')
        if self.monitor:
            self.monitor.terminate()
            try:self.monitor.wait(timeout=3)
            except subprocess.TimeoutExpired:self.monitor.kill();self.monitor.wait()
            self.log.close()
        self.inputs_stop.set();self.inputs_exe.wake();self.inputs_thread.join(timeout=2)
        self.inputs_exe.shutdown()
        for node in reversed(self.nodes):
            self.exe.remove_node(node);node.destroy_node()
        self.exe.shutdown();rclpy.shutdown()


if __name__=='__main__':unittest.main()
