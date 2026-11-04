"""Session acknowledgement with a lost service response, isolated ROS domain 228."""
import json
import os
import time
import unittest
from unittest.mock import patch

os.environ['ROS_DOMAIN_ID']='228'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import SetBool
from detected_pallets_interfaces.msg import PalletTarget
import run_pallet_cycle as cycle


class AcquisitionAckTest(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.operator=cycle.Cycle();self.server=Node('acquisition_ack_fixture')
        rclpy.get_global_executor().add_node(self.server)
        self.requests=0;self.advance=True;self.reject=False;self.unstable_first=False
        self.state=dict(session=0,started_ros_s=None,phase='PAUSED',camera_ready=True,
            enabled=False,busy=False,locked=False,measurement_ready=False,rgbd_health=dict(stable=False))
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher=self.server.create_publisher(String,'/pallet/recognition/status',qos)
        self.target_publisher=self.server.create_publisher(PalletTarget,'/pallet/target',1)
        self.server.create_timer(.05,self.publish)
        self.service=self.server.create_service(SetBool,'/pallet/set_enabled',self.request)

    def publish(self):
        self.publisher.publish(String(data=json.dumps(self.state)))
        msg=PalletTarget();msg.header.stamp=self.server.get_clock().now().to_msg()
        msg.header.frame_id='camera_color_optical_frame';msg.valid=True;msg.status='target_bias_applied'
        msg.camera_forward_m=1.5;msg.chassis_front_forward_m=1.4555
        msg.opening_lateral_m=.0723;msg.entry_heading_rad=.088;msg.face_yaw_rad=-.088
        msg.opening_width_m=.255;msg.leg_center_separation_m=.30
        msg.left_depth_samples=100;msg.right_depth_samples=100
        self.target_publisher.publish(msg)

    def request(self,request,response):
        self.requests+=1
        response.success=not self.reject;response.message='accepted' if response.success else 'fixture rejected'
        if response.success and self.advance:
            self.state.update(session=self.state['session']+1,
                started_ros_s=self.server.get_clock().now().nanoseconds/1e9,phase='TRACKING',
                locked=True,measurement_ready=True,rgbd_health=dict(stable=True))
            if self.unstable_first and self.requests==1:
                self.state.update(measurement_ready=False,rgbd_health=dict(stable=False))
            self.publish()
        return response

    def lose_response(self):
        actual=self.operator.service_client(SetBool,'/pallet/set_enabled')
        pending=Future()
        class LostReply:
            service_is_ready=actual.service_is_ready
            def call_async(proxy,request):
                proxy.real_future=actual.call_async(request)
                return pending
            def remove_pending_request(proxy,future):pass
        self.operator.service_clients[(SetBool,'/pallet/set_enabled')]=LostReply()
        return pending

    def test_lost_rpc_reply_uses_new_live_session_and_sends_only_once(self):
        pending=self.lose_response()
        self.operator.acquire(watch=())
        self.assertEqual(self.requests,1)
        self.assertTrue(pending.cancelled())
        self.assertTrue(self.operator.fresh('recognition')['measurement_ready'])

    def test_rpc_success_with_old_tracking_session_cannot_acknowledge_new_start(self):
        self.advance=False
        self.state.update(session=8,started_ros_s=self.server.get_clock().now().nanoseconds/1e9-1.,
            phase='TRACKING',locked=True,measurement_ready=True,rgbd_health=dict(stable=True))
        with patch.object(cycle,'ACQUISITION_ACK_TIMEOUT_S',.8),\
                self.assertRaisesRegex(RuntimeError,'xác nhận phiên nhận diện mới'):
            self.operator.acquire(watch=())
        self.assertEqual(self.requests,1)

    def test_explicit_rejection_fails_without_another_request(self):
        self.reject=True
        with self.assertRaisesRegex(RuntimeError,'fixture rejected'):
            self.operator.acquire(watch=())
        self.assertEqual(self.requests,1)

    def test_acknowledged_unstable_window_retries_new_session_at_checkpoint(self):
        self.unstable_first=True;self.operator.pickup_run_id='checkpoint-run'
        wait=self.operator.wait;fresh=self.operator.fresh
        def bounded(predicate,timeout,label,watch=()):
            if label=='pallet xanh và cửa sổ RGB-D ổn định':timeout=.3
            return wait(predicate,timeout,label,watch)
        def states(key,*args):
            if key=='alignment':return dict(active=True,preparing=False,perception_refresh_allowed=True)
            if key=='handling':return dict(active=True,phase='CHECKPOINT_RECOGNITION',run_id='checkpoint-run')
            return fresh(key,*args)
        with patch.object(self.operator,'wait',bounded),patch.object(self.operator,'fresh',states),\
                patch.object(self.operator,'still',return_value=True),patch.object(self.operator,'sensors'):
            self.operator.acquire(watch=())
        self.assertEqual(self.requests,2)
        self.assertEqual(self.operator.fresh('recognition')['session'],2)

    def tearDown(self):
        rclpy.get_global_executor().remove_node(self.server)
        self.server.destroy_node();self.operator.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
