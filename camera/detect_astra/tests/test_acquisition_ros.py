"""Actual acquisition/alignment services with synthetic inputs in ROS domain 231."""
import json,math,os
from pathlib import Path
import sys,time,unittest
from types import SimpleNamespace
os.environ['ROS_DOMAIN_ID']='231'
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT)]
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from detected_pallets_interfaces.msg import PalletTarget,PalletDetection
from publish_pallet_detection import DetectionNode
from publish_pallet_target import TargetNode
from pallet_perception_config import parse_args
from align_pallet import PalletAlignmentNode
from align_pallet import rgbd_wait_reason,START_DATA_TIMEOUT_S


class QualifiedAlignmentNode(PalletAlignmentNode):
    """Use real RGB-D health with the staging admission gate, without Nav2/motors."""
    def interlock(self):
        ready, reason = super().interlock()
        if ready and not self.core.active and not self.recognition.get('rgbd_health', {}).get('stable', False):
            return False, rgbd_wait_reason(self.recognition.get('rgbd_health'))
        return ready, reason


class AcquisitionRosTest(unittest.TestCase):
    def test_checkpoint_permission_allows_yolo_only_under_stationary_live_hold(self):
        self.detector.set_enabled(SetBool.Request(data=False),SetBool.Response())
        active=dict(active=True,preparing=False,state='INSERTING',strategy='pallet_insertion_odom',
                    linear_mps=.08,angular_rps=0.,perception_refresh_allowed=False)
        self.detector.on_alignment(String(data=json.dumps(active)))
        self.assertFalse(self.detector.set_enabled(SetBool.Request(data=True),SetBool.Response()).success)
        checkpoint={**active,'state':'CHECKPOINT_RECOGNITION','linear_mps':0.,'perception_refresh_allowed':True}
        self.detector.on_alignment(String(data=json.dumps(checkpoint)))
        self.assertTrue(self.detector.set_enabled(SetBool.Request(data=True),SetBool.Response()).success)
        self.assertTrue(self.detector.inference_enabled)
        self.detector.checkpoint_permission_until=time.monotonic()-.01
        self.detector.control_tick()
        self.assertFalse(self.detector.inference_enabled)
        self.assertTrue(self.detector.alignment_active)
        for fields in ({'linear_mps':.01},{'angular_rps':.01},{'perception_refresh_allowed':False},
                       {'state':'REMEASURING'}):
            self.detector.on_alignment(String(data=json.dumps({**checkpoint,**fields})))
            self.assertFalse(self.detector.set_enabled(SetBool.Request(data=True),SetBool.Response()).success)

    def setUp(self):
        rclpy.init()
        self.node=Node('acquisition_synthetic_inputs')
        self.detector=DetectionNode(SimpleNamespace(start_paused=False,max_age_ms=250))
        self.tracker=TargetNode(parse_args('tracker',[]))
        self.alignment=QualifiedAlignmentNode()
        self.executor=SingleThreadedExecutor()
        for node in (self.node,self.detector,self.tracker,self.alignment):self.executor.add_node(node)
        self.target=self.node.create_publisher(PalletTarget,'/pallet/target',1)
        self.odom=self.node.create_publisher(Odometry,'/odom',1)
        self.state=None;self.commands=[];self.emit_semantic=True;self.last_detection=0.
        self.emit_targets=True;self.target_valid=True
        self.node.create_subscription(String,'/pallet/recognition/status',self.on_state,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(Twist,'/pallet/alignment/proposed_cmd_vel',
            lambda m:self.commands.append((m.linear.x,m.angular.z)),1)
        self.client=self.node.create_client(SetBool,'/pallet/alignment/set_enabled')
        self.recognition_client=self.node.create_client(SetBool,'/pallet/set_enabled')
        self.timer=self.node.create_timer(.05,self.inputs)

    def on_state(self,msg):self.state=json.loads(msg.data)

    def inputs(self):
        stamp=self.node.get_clock().now().to_msg()
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_footprint'
        odom.pose.pose.orientation.w=1.;self.odom.publish(odom)
        target=PalletTarget();target.header.stamp=stamp;target.header.frame_id='camera_color_optical_frame'
        target.valid=self.target_valid;target.status='TRACKED_RGBD; class=pallets; target_bias_applied'
        target.camera_forward_m=1.8;target.chassis_front_forward_m=1.8665;target.opening_lateral_m=.01
        target.entry_heading_rad=math.radians(1.);target.face_yaw_rad=-target.entry_heading_rad
        target.opening_width_m=.252;target.leg_center_separation_m=.32
        target.left_depth_samples=target.right_depth_samples=100
        if self.emit_targets:self.target.publish(target)
        wall=time.monotonic()
        enabled,generation=self.detector.inference_state()
        if self.emit_semantic and enabled and wall-self.last_detection>=.5:
            self.last_detection=wall
            detection=PalletDetection();detection.header=target.header;detection.valid=True
            detection.confidence=.9;detection.class_name='pallets';detection.bbox_xyxy=[270.,170.,390.,300.]
            self.detector.publish_result(detection,generation)

    def spin_until(self,predicate,timeout=10):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.executor.spin_once(timeout_sec=.02)
            if predicate():return
        self.fail('Timed out: '+str(self.state))

    def request(self,client,enabled):
        self.spin_until(client.service_is_ready)
        req=SetBool.Request();req.data=enabled
        return client.call_async(req)

    def result(self,future):
        self.spin_until(future.done,timeout=START_DATA_TIMEOUT_S+3.)
        return future.result()

    def acquired(self):
        self.spin_until(lambda:self.state is not None and self.state['phase']=='TRACKING'
            and not self.state['enabled'] and not self.state['busy'])
        self.spin_until(lambda:self.alignment.interlock()[0])

    def test_automatic_handoff_waits_for_manual_start_and_resets_on_retry(self):
        self.acquired()
        self.assertGreaterEqual(self.state['elapsed_s'],3.)
        self.assertLessEqual(self.state['elapsed_s'],5.)
        self.assertTrue(self.state['locked']);self.assertFalse(self.alignment.core.active)
        self.assertTrue(self.commands);self.assertTrue(all(v==w==0 for v,w in self.commands))
        generation=self.detector.inference_generation-1
        self.assertFalse(self.detector.publish_result(PalletDetection(),generation))
        session=self.state['session'];start=self.tracker.recognition_start_ns
        self.assertTrue(self.result(self.request(self.recognition_client,True)).success)
        self.spin_until(lambda:self.state['session']>session and self.state['phase']=='ACQUIRING')
        self.spin_until(lambda:self.tracker.recognition_start_ns>start)
        self.assertFalse(self.state['locked']);self.assertFalse(self.alignment.core.active)
        self.assertTrue(self.tracker.recognition_reset_pending)

    def test_timeout_stops_yolo_without_authorizing_alignment(self):
        self.emit_semantic=False
        with self.detector.lock:self.detector.acquisition.started-=5.1
        self.spin_until(lambda:self.state is not None and self.state['phase']=='ACQUIRE_FAILED')
        self.assertFalse(self.state['enabled']);self.assertFalse(self.state['locked'])
        self.assertFalse(self.result(self.request(self.client,True)).success)
        self.assertFalse(self.alignment.core.active)
        self.assertTrue(all(v==w==0 for v,w in self.commands))

    def test_start_waits_for_inflight_yolo_and_rejects_reenable_during_alignment(self):
        self.acquired()
        with self.detector.lock:self.detector.inference_busy=True
        future=self.request(self.client,True)
        self.spin_until(lambda:self.alignment.preparing and bool(self.detector.prepare_waiters))
        self.assertFalse(future.done());self.assertFalse(self.alignment.core.active)
        self.assertEqual((self.alignment.core.v,self.alignment.core.w),(0.,0.))
        self.detector.end_inference()
        self.assertTrue(self.result(future).success)
        self.spin_until(lambda:self.alignment.core.v>0)
        self.assertGreater(self.alignment.core.target_stamp,self.alignment.measure_after_s)
        self.assertFalse(self.result(self.request(self.recognition_client,True)).success)
        self.assertTrue(self.result(self.request(self.client,False)).success)
        self.assertFalse(self.alignment.core.active)

    def test_stop_cancels_pending_start_and_late_ack_cannot_restart(self):
        self.acquired()
        with self.detector.lock:self.detector.inference_busy=True
        start=self.request(self.client,True)
        self.spin_until(lambda:self.alignment.preparing and bool(self.detector.prepare_waiters))
        self.assertTrue(self.result(self.request(self.client,False)).success)
        self.detector.end_inference()
        self.assertFalse(self.result(start).success)
        self.assertFalse(self.alignment.core.active)
        self.assertEqual((self.alignment.core.v,self.alignment.core.w),(0.,0.))
        self.assertTrue(all(v==w==0 for v,w in self.commands))

    def test_start_waits_for_fresh_measurements_then_full_health_window(self):
        self.acquired()
        self.commands.clear()
        # An invalid frame retains the semantic lock, but clears the short
        # measurement window and spoils the five-second health window.
        self.target_valid=False
        self.spin_until(lambda:not self.detector.acquisition.measurement_ready(
            time.monotonic(),self.alignment.now()) and not self.alignment.interlock()[0])
        start=self.request(self.client,True)
        self.spin_until(lambda:bool(self.detector.prepare_waiters))
        self.assertFalse(start.done())
        self.assertFalse(self.alignment.core.active)
        self.target_valid=True
        self.spin_until(lambda:self.alignment.start_waiter is not None)
        self.assertFalse(start.done())
        # Hold beyond the old 1.2-second timeout: no motion while health is bad.
        until=time.monotonic()+1.5
        self.spin_until(lambda:time.monotonic()>=until)
        self.assertFalse(start.done())
        self.assertFalse(self.alignment.core.active)
        self.assertTrue(all(v==w==0 for v,w in self.commands))
        self.assertTrue(self.result(start).success)
        self.assertTrue(self.alignment.recognition['rgbd_health']['stable'])
        self.assertGreater(self.alignment.core.target_stamp,self.alignment.measure_after_s)

    def test_stop_cancels_rgbd_wait_and_recovery_does_not_restart(self):
        self.acquired()
        self.target_valid=False
        self.spin_until(lambda:not self.alignment.interlock()[0])
        start=self.request(self.client,True)
        self.spin_until(lambda:bool(self.detector.prepare_waiters))
        self.target_valid=True
        self.spin_until(lambda:self.alignment.start_waiter is not None)
        self.assertFalse(start.done())
        self.assertTrue(self.result(self.request(self.client,False)).success)
        self.assertFalse(self.result(start).success)
        self.spin_until(lambda:self.alignment.interlock()[0])
        self.assertFalse(self.alignment.core.active)
        self.assertFalse(self.alignment.preparing)
        self.assertTrue(all(v==w==0 for v,w in self.commands))

    def test_late_580ms_gap_recovers_in_same_request_after_old_seven_second_deadline(self):
        self.acquired();self.commands.clear()
        self.target_valid=False
        self.spin_until(lambda:not self.alignment.interlock()[0])
        request=self.request(self.client,True)
        self.spin_until(lambda:bool(self.detector.prepare_waiters))
        self.target_valid=True
        self.spin_until(lambda:self.alignment.start_waiter is not None)
        begun=time.monotonic()
        self.spin_until(lambda:time.monotonic()-begun>=3.)
        self.emit_targets=False
        gap_start=time.monotonic()
        self.spin_until(lambda:time.monotonic()-gap_start>=.58)
        self.emit_targets=True
        self.spin_until(lambda:time.monotonic()-begun>=7.1)
        self.assertFalse(request.done());self.assertFalse(self.alignment.core.active)
        self.assertTrue(all(v==w==0 for v,w in self.commands))
        result=self.result(request)
        self.assertTrue(result.success,result.message)
        self.assertTrue(self.alignment.recognition['rgbd_health']['stable'])
        self.assertGreater(self.alignment.core.target_stamp,self.alignment.measure_after_s)

    def test_persistent_bad_rgbd_times_out_without_motion(self):
        self.acquired()
        self.commands.clear()
        # Keep short-term target measurements valid, but make the real health
        # policy reject every frame. Admission must remain bounded and closed.
        original=self.detector.rgbd_health.sample
        def invalid_health(stamp,wall,ros,valid,x,y,heading):
            original(stamp,wall,ros,False,x,y,heading)
        self.detector.rgbd_health.sample=invalid_health
        self.spin_until(lambda:not self.alignment.interlock()[0])
        start=self.request(self.client,True)
        self.spin_until(lambda:self.alignment.start_waiter is not None)
        result=self.result(start)
        self.assertFalse(result.success)
        self.assertIn('RGB-D',result.message)
        self.assertFalse(self.alignment.core.active)
        self.assertFalse(self.alignment.preparing)
        self.assertTrue(all(v==w==0 for v,w in self.commands))

    def tearDown(self):
        self.alignment.core.stop();self.alignment.zero()
        for node in (self.alignment,self.tracker,self.detector,self.node):
            self.executor.remove_node(node);node.destroy_node()
        self.executor.shutdown();rclpy.shutdown()

if __name__=='__main__':unittest.main()
