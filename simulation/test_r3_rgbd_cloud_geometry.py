"""Obstacle projection shares the current pallet correction and rejects stale fits."""
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'camera/detect_astra/scripts'),
                str(ROOT/'camera/detect_astra'),
                str(ROOT/'camera/detect_astra/tests')]
import test_plane_heading as scenarios
from pallet_plane_heading import floor_depth_skew
from pallet_target_geometry import measure_pallet_target

spec = importlib.util.spec_from_file_location('r3_cloud_projection',
    ROOT/'ros2_ws/scripts/publish_rgb_calibrated_cloud.py')
cloud = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cloud)


class CloudProjectionTest(unittest.TestCase):
    def scene(self, yaw=0., skew=.000045):
        return scenarios.PlaneHeadingTest().scene(yaw=yaw, skew=skew)

    def rays(self, intrinsics, stride=1):
        v, u = np.mgrid[0:480:stride, 0:640:stride]
        rays = cv2.undistortPoints(np.stack((u, v), axis=-1).astype(float).reshape(-1, 1, 2),
            *intrinsics).reshape(u.shape + (2,))
        return rays, u

    def plane_yaw(self, xyz):
        centered = xyz - np.mean(xyz, axis=0)
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[-1]
        pitch = math.radians(20.)
        forward = normal[2]*math.cos(pitch)-normal[1]*math.sin(pitch)
        if forward < 0:
            normal = -normal
            forward = -forward
        return math.degrees(math.atan2(normal[0], forward))

    def test_cloud_and_pallet_front_plane_share_the_same_capture_geometry(self):
        for yaw in (-5., 0., 5.):
            with self.subTest(yaw=yaw):
                mask, bbox, depth, intrinsics = self.scene(yaw)
                slope, _ = floor_depth_skew(depth, intrinsics)
                rays, u = self.rays(intrinsics)
                xyz = cloud.project_corrected_depth(depth, rays, u, slope, 370.)
                pallet = measure_pallet_target(mask, bbox, depth, intrinsics,
                    heading_floor_reference=True)
                self.assertEqual(slope, pallet['position_depth_x_slope'])
                self.assertEqual(slope, pallet['heading_depth_x_slope'])
                self.assertAlmostEqual(self.plane_yaw(xyz[mask]), yaw, delta=.15)
                self.assertAlmostEqual(self.plane_yaw(xyz[mask]),
                    math.degrees(pallet['face_yaw_rad']), delta=.15)

    def test_old_static_preview_adds_several_degrees_to_a_straight_pallet(self):
        mask, _, depth, intrinsics = self.scene()
        rays, u = self.rays(intrinsics)
        slope, _ = floor_depth_skew(depth, intrinsics)
        old = cloud.project_corrected_depth(depth, rays, u, .00015807, 370.)
        new = cloud.project_corrected_depth(depth, rays, u, slope, 370.)
        self.assertGreater(abs(self.plane_yaw(old[mask])), 5.)
        self.assertLess(abs(self.plane_yaw(new[mask])), .15)

    def probe(self, depth, intrinsics, now=100.):
        rays, u = self.rays(intrinsics, 4)
        probe = SimpleNamespace(rays=rays, u_coords=u, info_key=(640, 480),
            frame_id='camera_color_optical_frame', intrinsics=intrinsics,
            next_publish=0., max_rate_hz=4., stride=4, floor_reference=True,
            inverse_depth_x_slope=.00015807, reference_u=370., max_input_age_s=.5,
            publisher=Mock(), status_pub=Mock(), last_publish=0.,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(now*1e9))),
            get_logger=Mock(return_value=Mock()))
        probe.publish_status = MethodType(cloud.CalibratedCloud.publish_status, probe)
        msg = Image()
        msg.header.frame_id=probe.frame_id
        msg.header.stamp.sec=100
        msg.width=640; msg.height=480; msg.step=1280; msg.encoding='16UC1'
        msg.data=np.asarray(depth,dtype='<u2').tobytes()
        return probe, msg

    def test_invalid_floor_never_falls_back_to_the_old_static_slope(self):
        _, _, depth, intrinsics = self.scene()
        probe, msg = self.probe(depth, intrinsics)
        cloud.CalibratedCloud.on_depth(probe, msg)
        self.assertEqual(probe.publisher.publish.call_count, 1)
        self.assertTrue(json.loads(probe.status_pub.publish.call_args.args[0].data)['ready'])
        broken = depth.copy(); broken[350:470, :] = 0
        msg.data=broken.astype('<u2').tobytes(); probe.next_publish=0.
        cloud.CalibratedCloud.on_depth(probe, msg)
        self.assertEqual(probe.publisher.publish.call_count, 1)
        status=json.loads(probe.status_pub.publish.call_args.args[0].data)
        self.assertFalse(status['ready'])
        self.assertNotIn('inverse_depth_x_slope', status)
        self.assertEqual(status['source'], 'pallet_floor_depth_skew')

    def test_stale_depth_cannot_renew_cloud_or_correction_evidence(self):
        _, _, depth, intrinsics = self.scene()
        probe, msg = self.probe(depth, intrinsics, now=100.6)
        with patch.object(cloud,'floor_depth_skew') as fit:
            cloud.CalibratedCloud.on_depth(probe, msg)
        fit.assert_not_called(); probe.publisher.publish.assert_not_called()
        self.assertFalse(json.loads(probe.status_pub.publish.call_args.args[0].data)['ready'])

    def test_invalid_raw_depth_stays_invalid_after_inverse_depth_correction(self):
        raw=np.array([[0., float('nan'), float('inf'), 65000., 1400.]])
        xyz=cloud.project_corrected_depth(raw, np.ones((1,5,2)), np.zeros((1,5)), .00015807, 370.)
        self.assertTrue(np.isnan(xyz[0,:4]).all())
        self.assertTrue(np.isfinite(xyz[0,4]).all())


if __name__=='__main__':
    unittest.main()
