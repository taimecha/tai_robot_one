"""Synthetic ROS integration in domain 231. No motor hardware is launched."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '231'
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy
from lifecycle_msgs.srv import ChangeState
from tf2_ros import TransformBroadcaster
from detected_pallets_interfaces.msg import PalletTarget
from align_pallet import PalletAlignmentNode, COMMAND


class AlignmentRosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('alignment_synthetic_inputs')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.odom = self.node.create_publisher(Odometry, '/odom', 1)
        self.target = self.node.create_publisher(PalletTarget, '/pallet/target', 1)
        self.scan = self.node.create_publisher(LaserScan, '/scan_filtered', 1)
        self.tf = TransformBroadcaster(self.node)
        self.commands = []
        self.status = None
        self.node.create_subscription(Twist, '/pallet/alignment/proposed_cmd_vel',
                                      lambda m: self.commands.append((m.linear.x, m.angular.z)), 1)
        self.node.create_subscription(String, '/pallet/alignment/status', self.on_status, 1)
        self.client = self.node.create_client(SetBool, '/pallet/alignment/set_enabled')
        self.extra = []
        self.monitor = None
        self.emit_targets = True
        self.emit_scan = True
        self.obstacle = False
        self.target_frame = 'camera_color_optical_frame'
        self.target_valid = True
        self.last_target = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.yolo = self.node.create_publisher(Bool, '/pallet/enabled', latched)
        self.recognition = self.node.create_publisher(String, '/pallet/recognition/status', latched)
        self.yolo_enabled = False
        self.prepare_calls = 0
        self.publish_recognition = True
        self.node.create_service(Trigger, '/pallet/prepare_alignment', self.prepare)
        self.timer = self.node.create_timer(.05, self.inputs)

    def prepare(self, request, response):
        self.prepare_calls += 1
        self.yolo_enabled = False
        response.success, response.message = True, 'YOLO stopped'
        return response

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def inputs(self):
        if self.publish_recognition:
            self.yolo.publish(Bool(data=self.yolo_enabled))
            self.recognition.publish(String(data=json.dumps(dict(enabled=self.yolo_enabled,
                busy=False, locked=True, measurement_ready=True, phase='TRACKING'))))
        stamp = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        self.odom.publish(odom)
        tf = TransformStamped(); tf.header = odom.header
        tf.child_frame_id = 'base_footprint'; tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        scan = LaserScan(); scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, 2*math.pi/360
        scan.range_min, scan.range_max = .05, 10.
        ranges = [5.] * 361
        if self.obstacle:
            ranges[180:184] = [.9]*4
        scan.ranges = ranges
        if self.emit_scan:
            self.scan.publish(scan)
        if self.emit_targets:
            msg = PalletTarget(); msg.header.stamp, msg.header.frame_id = stamp, self.target_frame
            msg.valid, msg.status = self.target_valid, 'TRACKED_RGBD; target_bias_applied'
            msg.camera_forward_m, msg.chassis_front_forward_m = 1.8, 1.865
            msg.opening_lateral_m = .048
            msg.opening_width_m, msg.leg_center_separation_m = .252, .31
            msg.left_depth_samples = msg.right_depth_samples = 100
            self.target.publish(msg)
            self.last_target = msg

    def spin_until(self, condition, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)
            if condition():
                return
        self.fail('Timed out; status=' + str(self.status))

    def service(self, enabled):
        req = SetBool.Request(); req.data = enabled
        future = self.client.call_async(req)
        self.spin_until(future.done)
        return future.result()

    def start_node(self, execute=False):
        self.alignment = PalletAlignmentNode(execute=execute)
        self.executor.add_node(self.alignment)
        self.extra.append(self.alignment)
        self.spin_until(lambda: self.client.service_is_ready() and self.status is not None)

    def test_default_has_no_motor_publisher_and_stale_data_stops(self):
        self.start_node()
        self.spin_until(lambda: self.status['ready'])
        self.assertFalse(self.status['execute'])
        self.assertEqual(self.node.get_publishers_info_by_topic(COMMAND), [])
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda: any(v > 0 for v, w in self.commands))
        self.emit_targets = False
        self.spin_until(lambda: self.status['state'] == 'LOST_DATA')
        self.assertEqual(self.commands[-1], (0., 0.))
        self.assertFalse(self.status['active'])

    def test_camera_invalidates_after_start_waiter_then_reconfirms_without_second_request(self):
        class InterruptedStart(PalletAlignmentNode):
            injected = False
            def tick(node):
                pending = node.start_waiter
                super().tick()
                if (pending is not None and pending[0].done() and node.preparing
                        and not node.injected):
                    node.injected = True
                    node.core.invalidate_target('synthetic frame lost just before coroutine resumes')
        self.alignment = InterruptedStart(execute=False)
        self.executor.add_node(self.alignment)
        self.extra.append(self.alignment)
        self.spin_until(lambda: self.client.service_is_ready() and self.status and self.status['ready'])
        response = self.service(True)
        self.assertTrue(self.alignment.injected)
        self.assertTrue(response.success, response.message)
        self.assertEqual(self.prepare_calls, 1)
        self.assertGreaterEqual(self.alignment.core.confirmed, 3)

    def test_invalid_frame_stops_immediately_and_requires_new_confirmation(self):
        self.start_node()
        self.spin_until(lambda: self.status['ready'])
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda: any(v > 0 for v, w in self.commands))
        self.emit_targets = False
        self.last_target.header.frame_id = 'wrong_frame'
        self.alignment.on_target(self.last_target)
        self.assertEqual((self.alignment.core.v, self.alignment.core.w), (0., 0.))
        self.assertIsNone(self.alignment.core.target)
        self.assertEqual(self.alignment.core.confirmed, 0)
        self.assertTrue(self.service(False).success)
        self.assertFalse(self.alignment.core.active)

    def test_execute_requires_verified_chain_and_monitor_stops_obstacles(self):
        self.start_node(execute=True)
        self.spin_until(lambda: self.alignment.core.confirmed >= 3)
        self.assertFalse(self.service(True).success)
        self.assertEqual((self.alignment.core.v, self.alignment.core.w), (0., 0.))
        self.last_target.status = 'TRACKED_RGBD'  # Known bias must be applied before real commands.
        self.alignment.on_target(self.last_target)
        self.assertIsNone(self.alignment.core.target)
        self.assertEqual((self.alignment.core.v, self.alignment.core.w), (0., 0.))
        # Real Nav2 collision monitor, fake final receiver: domain 231 has no actuators.
        self.monitor = subprocess.Popen([
            '/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor', '--ros-args',
            '-r', '__node:=pallet_alignment_collision_monitor', '--params-file',
            str(ROOT/'config/pallet_alignment_collision.yaml')],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        change = self.node.create_client(ChangeState, '/pallet_alignment_collision_monitor/change_state')
        self.spin_until(change.service_is_ready)
        for transition in (1, 3):
            req = ChangeState.Request(); req.transition.id = transition
            future = change.call_async(req); self.spin_until(future.done)
            self.assertTrue(future.result().success)
        stamper = Node('cmd_vel_stamper')
        for name, value in [('input_topic', '/cmd_vel_safe'), ('output_topic', '/base_controller/cmd_vel'),
                            ('require_single_publisher', True)]:
            stamper.declare_parameter(name, value)
        final_pub = stamper.create_publisher(TwistStamped, '/base_controller/cmd_vel', 1)
        safe = []
        def receive(msg):
            safe.append((msg.linear.x, msg.angular.z))
            stamped = TwistStamped(); stamped.twist = msg
            final_pub.publish(stamped)
        stamper.create_subscription(Twist, '/cmd_vel_safe', receive, 1)
        self.executor.add_node(stamper); self.extra.append(stamper)
        self.spin_until(lambda: self.status['ready'])
        self.assertTrue(self.service(True).success)
        self.spin_until(lambda: any(v > 0 for v, w in safe))
        self.obstacle = True
        safe.clear()
        self.spin_until(lambda: len(safe) >= 8)
        self.assertEqual(safe[-1], (0., 0.))
        self.emit_scan = False
        self.spin_until(lambda: self.status['state'] == 'BLOCKED')
        self.assertFalse(self.status['active'])
        self.assertEqual(self.commands[-1], (0., 0.))
        self.emit_scan = True
        self.obstacle = False
        self.spin_until(lambda: self.status['ready'])
        self.assertTrue(self.service(True).success)
        # A competing source must disarm the alignment node.
        competing = self.node.create_publisher(Twist, '/cmd_vel_safe', 1)
        self.spin_until(lambda: self.status['state'] == 'BLOCKED')
        self.assertFalse(self.status['active'])
        self.assertEqual(self.commands[-1], (0., 0.))
        self.node.destroy_publisher(competing)

    def test_odometry_jump_disarms_and_invalidates_world_target(self):
        self.start_node()
        self.spin_until(lambda: self.status['ready'])
        self.assertTrue(self.service(True).success)
        odom = Odometry()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x = .5
        self.alignment.on_odom(odom)
        self.assertFalse(self.alignment.core.active)
        self.assertIsNone(self.alignment.core.target)
        self.assertEqual((self.alignment.core.v, self.alignment.core.w), (0., 0.))

    def test_stale_odometry_disarms_then_recovers_without_starting_motion(self):
        self.start_node()
        self.spin_until(lambda:self.status['ready'])
        self.assertTrue(self.service(True).success)
        msg=Odometry();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.header.stamp.sec-=1
        msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
        msg.pose.pose.orientation.w=1.
        self.alignment.on_odom(msg)
        self.assertEqual(self.alignment.core.state,'LOST_DATA')
        self.assertIn('thời gian',self.alignment.core.reason)
        self.assertFalse(self.alignment.core.active)
        self.spin_until(lambda:self.status['ready'] and self.alignment.core.confirmed>=3)
        self.assertFalse(self.alignment.core.active)
        self.assertEqual((self.alignment.core.v,self.alignment.core.w),(0.,0.))

    def test_start_turns_yolo_off_then_waits_for_new_measurements(self):
        self.start_node()
        self.spin_until(lambda: self.status['ready'])
        self.yolo_enabled = True
        self.spin_until(lambda: not self.status['ready'])
        self.commands.clear()
        self.assertTrue(self.service(True).success)
        self.assertEqual(self.prepare_calls,1)
        self.assertFalse(self.yolo_enabled)
        self.assertGreater(self.alignment.core.target_stamp,self.alignment.measure_after_s)
        self.assertGreaterEqual(self.alignment.core.confirmed,3)
        # Recognition heartbeat loss must stop even if target/odom keep arriving.
        self.publish_recognition = False
        self.spin_until(lambda: self.status['state']=='BLOCKED')
        self.assertFalse(self.status['active'])
        self.assertEqual(self.commands[-1],(0.,0.))

    def tearDown(self):
        for node in self.extra:
            if isinstance(node, PalletAlignmentNode):
                node.core.stop(); node.zero()
            self.executor.remove_node(node)
            node.destroy_node()
        if self.monitor:
            self.monitor.terminate()
            try:
                self.monitor.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.monitor.kill(); self.monitor.wait()
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
