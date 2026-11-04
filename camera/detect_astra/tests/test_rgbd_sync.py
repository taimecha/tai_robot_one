"""RGB/Depth/CameraInfo callback ordering must not consume incomplete sets."""
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

os.environ['ROS_DOMAIN_ID'] = '231'
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'scripts')]
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from publish_pallet_target import TargetNode
from astra_rgbd_stream import stamp_ns, validate_intrinsics


class RGBDSyncTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = TargetNode(SimpleNamespace(max_age_ms=250, max_delta_ms=50))
        self.node.source_diagnostics = {'endpoints': {'rgb': [{}, {}]}}
        self.node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_200_000_000))

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def msg(self, kind, ns):
        msg = CameraInfo() if kind == 'info' else Image()
        msg.header.stamp.sec = ns//1_000_000_000
        msg.header.stamp.nanosec = ns%1_000_000_000
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height, msg.width = 480, 640
        if kind == 'info':
            msg.k = [500., 0., 320., 0., 500., 240., 0., 0., 1.]
            msg.d = [0.]*5
            msg.distortion_model = 'plumb_bob'
        return msg

    def add_images(self, ns):
        self.node.receive('rgb', self.msg('rgb', ns))
        self.node.receive('depth', self.msg('depth', ns+16_000_000))

    def test_info_callback_can_arrive_after_images(self):
        self.add_images(1_000_000_000)
        self.assertIsNone(self.node.take_pair())
        self.assertEqual(len(self.node.pending_images['rgb']), 1)
        self.node.receive_info('rgb', self.msg('info', 1_000_000_000))
        pair = self.node.take_pair()
        self.assertEqual(stamp_ns(pair[0]), stamp_ns(pair[3]))
        self.assertEqual(pair[2], 16.)

    def test_prefer_complete_set_and_keep_newer_rgb_waiting(self):
        self.add_images(1_000_000_000)
        self.node.receive_info('rgb', self.msg('info', 1_000_000_000))
        self.add_images(1_100_000_000)
        pair = self.node.take_pair()
        self.assertEqual(stamp_ns(pair[0]), 1_000_000_000)
        self.assertEqual(len(self.node.pending_images['rgb']), 1)
        self.node.receive_info('rgb', self.msg('info', 1_100_000_000))
        pair = self.node.take_pair()
        self.assertEqual(stamp_ns(pair[0]), 1_100_000_000)

    def test_metadata_timeout_does_not_relax_50_ms_validation(self):
        self.add_images(1_000_000_000)
        self.node.receive_info('rgb', self.msg('info', 930_000_000))
        self.assertIsNone(self.node.take_pair())
        self.node.info_wait_started_at = time.monotonic()-.06
        self.node.registration = dict(verified=True, parameter='depth_registration', value=True)
        self.node.registration_checked = time.monotonic()
        pair = self.node.take_pair()
        intrinsics, reason = validate_intrinsics(pair[0], pair[1], pair[3], pair[5])
        self.assertIsNone(intrinsics)
        self.assertIn('within 50 ms', reason)

    def test_late_callback_does_not_override_newer_capture_or_rewind_tracking(self):
        for stamp in (1_000_000_000,1_100_000_000):
            self.add_images(stamp)
            self.node.receive_info('rgb',self.msg('info',stamp))
        self.node.receive('depth',self.msg('depth',1_016_000_000))
        pair=self.node.take_pair()
        self.assertEqual(stamp_ns(pair[0]),1_100_000_000)
        self.node.receive('rgb',self.msg('rgb',1_000_000_000))
        self.assertIsNone(self.node.take_pair())

    def test_recent_callback_with_old_capture_is_rejected(self):
        self.add_images(900_000_000)
        self.node.receive_info('rgb',self.msg('info',900_000_000))
        self.assertIsNone(self.node.take_pair())
        self.assertFalse(self.node.pending_images['rgb'])
