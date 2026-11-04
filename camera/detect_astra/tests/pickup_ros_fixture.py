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


class PickupRosFixture(unittest.TestCase):
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
        self.recognition_session=1
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
        if getattr(self,'pivot_bias_mps',0.) and hasattr(self,'handler'):
            approach=self.handler.pickup.checkpoint_approach
            if approach and self.handler.pickup.phase=='CHECKPOINT_APPROACH' and approach.phase.startswith('TURN'):
                rotating=abs(tw)>.005 or abs(w)>.015
                if not hasattr(self,'pivot_commands'):self.pivot_commands=[]
                self.pivot_commands.append((now,tv,tw))
                due=[row for row in self.pivot_commands
                    if row[0]<=now-getattr(self,'pivot_command_delay_s',0.)+1e-9]
                _,tv,tw=due[-1] if due else (now,0.,0.)
                tv*=getattr(self,'pivot_linear_gain',.65)
                tw*=getattr(self,'pivot_angular_gain',.55)
                if rotating:tv+=self.pivot_bias_mps
        # Optional physical startup creep; integrate it into real pose and
        # odometry rather than injecting a velocity-only outlier.
        if getattr(self,'pivot_creep_mps',0.) and hasattr(self,'handler'):
            approach=self.handler.pickup.checkpoint_approach
            if approach and approach.phase.startswith('TURN') and abs(tw)>.05:
                if not hasattr(self,'pivot_creep_started'):self.pivot_creep_started={}
                self.pivot_creep_started.setdefault(approach.phase,now)
                if now-self.pivot_creep_started[approach.phase]<getattr(self,'pivot_creep_duration_s',.35):
                    tv=self.pivot_creep_mps
        if abs(tw)<getattr(self,'angular_deadzone_rps',0.):tw=0.
        v+=max(-.15*dt,min(.3*dt,tv-v));w+=max(-.4*dt,min(.4*dt,tw-w))
        self.velocity=(v,w)
        self.pose[0]+=v*math.cos(self.pose[2]+w*dt/2)*dt
        self.pose[1]+=v*math.sin(self.pose[2]+w*dt/2)*dt;self.pose[2]+=w*dt
        if getattr(self,'pivot_bias_mps',0.) and hasattr(self,'handler'):
            approach=self.handler.pickup.checkpoint_approach
            if approach and self.handler.pickup.phase=='CHECKPOINT_APPROACH' and approach.phase.startswith('TURN'):
                if not hasattr(self,'pivot_offsets'):self.pivot_offsets=[]
                self.pivot_offsets.append((approach.phase,
                    math.hypot(self.pose[0]-approach.pivot_anchor[0],self.pose[1]-approach.pivot_anchor[1]),
                    self.command[0]))
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
        dx,dy=getattr(self,'pallet_x',1.6555)-self.pose[0],getattr(self,'pallet_y',0.)-self.pose[1]
        msg.chassis_front_forward_m=math.cos(self.pose[2])*dx+math.sin(self.pose[2])*dy-.3005
        msg.opening_lateral_m=-math.sin(self.pose[2])*dx+math.cos(self.pose[2])*dy
        msg.entry_heading_rad=getattr(self,'pallet_heading',0.)-self.pose[2];msg.opening_width_m=.245
        msg.face_yaw_rad=-msg.entry_heading_rad;msg.leg_center_separation_m=.31
        msg.camera_forward_m=msg.chassis_front_forward_m+.0665
        msg.left_depth_samples=msg.right_depth_samples=100
        self.target.publish(msg)
        if self.emit_recognition:
            self.yolo.publish(Bool(data=False))
            self.recognition.publish(String(data=json.dumps(dict(enabled=False,busy=False,locked=visible,
                session=self.recognition_session,
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
