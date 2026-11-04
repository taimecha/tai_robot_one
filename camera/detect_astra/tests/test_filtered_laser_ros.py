"""Latest-only filter retains real capture stamps and drops stale sweeps."""
import os
from pathlib import Path
import runpy
import time
import unittest
os.environ['ROS_DOMAIN_ID']='215'
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from rclpy.parameter import Parameter

ROOT=Path(__file__).resolve().parents[3]
ScanSelfFilter=runpy.run_path(str(ROOT/'ros2_ws/scripts/scan_self_filter'))['ScanSelfFilter']


class FilteredLaserTest(unittest.TestCase):
    def test_old_scan_is_not_republished_or_restamped(self):
        rclpy.init();source=Node('filter_test');filt=ScanSelfFilter(parameter_overrides=[Parameter('max_scan_age_s',value=.4)]);exe=SingleThreadedExecutor()
        exe.add_node(source);exe.add_node(filt);received=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        pub=source.create_publisher(LaserScan,'/scan',qos)
        source.create_subscription(LaserScan,'/scan_filtered',received.append,qos)
        try:
            end=time.monotonic()+3
            while not pub.get_subscription_count() and time.monotonic()<end:exe.spin_once(timeout_sec=.02)
            self.assertTrue(pub.get_subscription_count())
            self.assertEqual(filt.subscription.qos_profile.depth,1)
            scan=LaserScan();scan.header.frame_id='lidar_link';scan.header.stamp=source.get_clock().now().to_msg()
            scan.header.stamp.sec-=1;scan.angle_increment=.01;scan.range_min=.05;scan.range_max=10.;scan.ranges=[2.]*10
            pub.publish(scan);end=time.monotonic()+.3
            while time.monotonic()<end:exe.spin_once(timeout_sec=.02)
            self.assertEqual(received,[])
            scan.header.stamp=(source.get_clock().now()-rclpy.duration.Duration(seconds=.35)).to_msg()
            original=scan.header.stamp
            pub.publish(scan);end=time.monotonic()+.15
            while not received and time.monotonic()<end:exe.spin_once(timeout_sec=.01)
            self.assertTrue(received);self.assertEqual(received[-1].header.stamp,original)
            received.clear()
            scan.header.stamp=(source.get_clock().now()-rclpy.duration.Duration(seconds=.45)).to_msg()
            pub.publish(scan);end=time.monotonic()+.15
            while time.monotonic()<end:exe.spin_once(timeout_sec=.01)
            self.assertEqual(received,[])
            scan.header.stamp=source.get_clock().now().to_msg();pub.publish(scan)
            end=time.monotonic()+1
            while not received and time.monotonic()<end:exe.spin_once(timeout_sec=.02)
            self.assertTrue(received)
            self.assertEqual(received[-1].header,scan.header)
            self.assertEqual(received[-1].ranges,scan.ranges)
        finally:
            exe.remove_node(filt);exe.remove_node(source);filt.destroy_node();source.destroy_node()
            exe.shutdown();rclpy.shutdown()


if __name__=='__main__':unittest.main()
