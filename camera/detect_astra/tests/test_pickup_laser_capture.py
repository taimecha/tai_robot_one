"""Capture-time laser geometry remains valid when duplicate dynamic TF lags."""
from collections import deque
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from tf2_ros import Buffer,TransformException
from handle_pallet import PalletHandlingNode
from pallet_alignment_core import OdomHistory,OdomSample
from pallet_pickup_safety import LaserTransformPending,capture_laser_points


class CaptureLaserTest(unittest.TestCase):
    def scan(self,stamp=100.1,ranges=(1.,1.1)):
        m=LaserScan();m.header.frame_id='lidar_link';m.header.stamp=Time(seconds=stamp).to_msg()
        m.angle_min=0.;m.angle_increment=math.pi/2;m.range_min=.05;m.range_max=10.;m.ranges=list(ranges)
        return m

    def fixture(self):
        buf=Buffer();tf=TransformStamped();tf.header.frame_id='base_footprint';tf.child_frame_id='lidar_link'
        tf.transform.translation.x=.14;tf.transform.rotation.w=1.;buf.set_transform_static(tf,'mount')
        for stamp in (100.,100.058):
            tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_footprint'
            tf.header.stamp=Time(seconds=stamp).to_msg();tf.transform.rotation.w=1.
            buf.set_transform(tf,'late_ekf_tf')
        history=OdomHistory();history.add(OdomSample(100.05,1.,2.,0.));history.add(OdomSample(100.15,1.1,2.,0.))
        m=self.scan();node=NS(pickup_scan=m,pickup_scans=deque([(m,100.2)]),
                            now=lambda:100.3,core=NS(odom=history),tf_buffer=buf)
        return node

    def points(self,node):
        with patch('handle_pallet.time.monotonic',return_value=100.3):
            return PalletHandlingNode.laser_points(node)

    def test_capture_pose_interpolates_odom_despite_42_ms_late_dynamic_tf(self):
        n=self.fixture()
        with self.assertRaises(TransformException):
            n.tf_buffer.lookup_transform('odom','lidar_link',Time(seconds=100.1))
        np.testing.assert_allclose(self.points(n),[[2.19,2.],[1.19,3.1]],atol=1e-6)

    def test_uses_previous_fresh_scan_when_newest_pose_is_not_yet_bracketed(self):
        n=self.fixture();latest=self.scan(100.25,(3.,4.))
        n.pickup_scans.append((latest,100.28));n.pickup_scan=latest
        np.testing.assert_allclose(self.points(n),[[2.19,2.],[1.19,3.1]],atol=1e-6)

    def test_never_extrapolates_odom_or_uses_expired_source_or_receipt(self):
        for failure in ('future','old_source','old_receipt','odom_gap'):
            n=self.fixture()
            if failure=='future':n.pickup_scan.header.stamp=Time(seconds=100.2).to_msg()
            elif failure=='old_source':n.now=lambda:100.6
            elif failure=='old_receipt':n.pickup_scans=deque([(n.pickup_scan,99.8)])
            else:
                n.core.odom=OdomHistory();n.core.odom.add(OdomSample(99.9,0.,0.,0.));n.core.odom.add(OdomSample(100.2,0.,0.,0.))
            with self.assertRaises(LaserTransformPending):self.points(n)

    def test_composes_sensor_mount_at_rotated_capture_pose(self):
        n=self.fixture();tf=n.tf_buffer.lookup_transform('base_footprint','lidar_link',Time(seconds=100.1))
        points=capture_laser_points(n.pickup_scan,(1.,2.,math.pi/2),tf.transform)
        np.testing.assert_allclose(points,[[1.,3.14],[-.1,2.14]],atol=1e-6)

    def test_bad_scan_geometry_is_rejected_without_a_transform_wait(self):
        n=self.fixture();n.pickup_scan.angle_increment=0.
        with self.assertRaises(ValueError) as error:self.points(n)
        self.assertNotIsInstance(error.exception,LaserTransformPending)


if __name__=='__main__':unittest.main()
