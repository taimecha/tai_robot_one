"""Recover only the sensor relay after a crash, in isolated ROS domain 229."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '229'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

REPO = Path(__file__).resolve().parents[3]


class FilterSupervisionTest(unittest.TestCase):
    def test_filter_respawns_and_only_forwards_fresh_real_stamps(self):
        rclpy.init();node = Node('filter_supervision_test')
        pub = node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        received=[]
        sub = node.create_subscription(LaserScan,'/scan_filtered',received.append,qos_profile_sensor_data)
        def wait(predicate,seconds=10):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.02)
                if predicate():return
            self.fail('Timed out waiting for filter recovery')
        def children():
            return [int(p.name) for p in Path('/proc').iterdir() if p.name.isdecimal()
                    and self.is_filter_child(p,process.pid)]
        with tempfile.TemporaryFile(mode='w+') as log:
            process=subprocess.Popen(['ros2','launch',str(REPO/'ros2_ws/launch/scan_self_filter.launch.py'),
                'max_scan_age_s:=0.40'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                wait(lambda:children() and pub.get_subscription_count()>0)
                old=children()[0]
                os.kill(old,signal.SIGTERM)
                wait(lambda:bool(children()) and old not in children())
                wait(lambda:pub.get_subscription_count()>0)
                scan=LaserScan();scan.header.frame_id='lidar_link'
                scan.angle_increment=.01;scan.range_min=.05;scan.range_max=10.;scan.ranges=[2.]*10
                scan.header.stamp=(node.get_clock().now()-rclpy.duration.Duration(seconds=1.)).to_msg()
                pub.publish(scan)
                end=time.monotonic()+.3
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
                self.assertEqual(received,[])
                scan.header.stamp=node.get_clock().now().to_msg()
                pub.publish(scan);wait(lambda:bool(received),2.)
                self.assertEqual(received[-1].header,scan.header)
                self.assertEqual(received[-1].ranges,scan.ranges)
            finally:
                os.killpg(process.pid,signal.SIGINT)
                process.wait(timeout=6.)
                node.destroy_node();rclpy.shutdown()

    @staticmethod
    def is_filter_child(path,parent):
        try:
            args=path.joinpath('cmdline').read_bytes().split(b'\0')
            ppid=int(path.joinpath('stat').read_text().rsplit(')',1)[1].split()[1])
            return ppid==parent and any(Path(a.decode()).name=='scan_self_filter' for a in args if a)
        except (OSError,ValueError,UnicodeError):return False


if __name__=='__main__':unittest.main()
