"""Track actual image motion and current depth; reject lost/occluded targets."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]
from pallet_rgb_tracker import PalletRGBTracker
from pallet_rgbd_estimator import RGBDTargetEstimator


class RGBDTrackingTest(unittest.TestCase):
    def setUp(self):
        cv2.setNumThreads(1)
        self.mask = np.zeros((480, 640), bool)
        self.mask[170:260, 270:390] = True
        self.mask[260:300, 270:288] = True
        self.mask[260:300, 372:390] = True
        self.bbox = np.array([270, 170, 390, 300], np.float32)
        self.image = np.full((480, 640, 3), 45, np.uint8)
        rng = np.random.default_rng(3)
        self.image[self.mask] = rng.integers(85, 230, (self.mask.sum(), 3), np.uint8)
        self.depth = np.full((480, 640), 1500, np.uint16)
        self.intrinsics = (np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]]), np.zeros(5))
        self.args = SimpleNamespace(confirm_frames=3, pitch_deg=20,
                                    inverse_depth_x_slope=0, reference_u=370,
                                    camera_ahead_of_chassis_m=.0665,
                                    expected_opening_width_m=.252, opening_width_tolerance_m=.08,
                                    measurement_bias=None)

    def seed(self):
        return dict(rgb=self.image, mask=self.mask, bbox=self.bbox,
                    stamp_ns=1_000_000_000, class_name='pallets', confidence=.9)

    def test_rgb_mask_follows_translation_and_scale(self):
        tracker = PalletRGBTracker()
        tracker.seed(self.image, self.mask, self.bbox, 1_000_000_000)
        transform = cv2.getRotationMatrix2D((330, 235), 0, 1.02)
        transform[:, 2] += (7, 3)
        image = cv2.warpAffine(self.image, transform, (640, 480), borderValue=(45, 45, 45))
        mask, bbox = tracker.update(image, 1_100_000_000)
        expected = cv2.warpAffine(self.mask.astype(np.uint8), transform, (640, 480),
                                  flags=cv2.INTER_NEAREST).astype(bool)
        self.assertGreater(np.sum(mask & expected) / np.sum(mask | expected), .98)
        np.testing.assert_allclose((bbox[:2]+bbox[2:])/2, [337, 238], atol=.4)

    def test_occlusion_one_leg_and_timestamp_gap_invalidate(self):
        for mode in ('blank', 'one_leg', 'gap'):
            with self.subTest(mode=mode):
                tracker = PalletRGBTracker()
                tracker.seed(self.image, self.mask, self.bbox, 1_000_000_000)
                image = self.image.copy()
                stamp = 1_100_000_000
                if mode == 'blank':
                    image[:] = 45
                elif mode == 'one_leg':
                    image[255:310, 365:400] = 45
                else:
                    stamp = 1_600_000_000
                with self.assertRaises(ValueError):
                    tracker.update(image, stamp)
                if mode != 'gap':
                    self.assertTrue(tracker.active)
                    with self.assertRaises(ValueError):
                        tracker.update(image,1_600_000_000)
                self.assertFalse(tracker.active)

    def test_tracking_neighbourhood_follows_pallet_at_different_image_locations(self):
        for dx,dy in ((-220,-100),(0,0),(200,130)):
            with self.subTest(offset=(dx,dy)):
                place=np.float32([[1,0,dx],[0,1,dy]])
                image=cv2.warpAffine(self.image,place,(640,480),borderValue=(45,45,45))
                mask=cv2.warpAffine(self.mask.astype(np.uint8),place,(640,480),
                                    flags=cv2.INTER_NEAREST).astype(bool)
                bbox=self.bbox+np.array([dx,dy,dx,dy])
                tracker=PalletRGBTracker()
                tracker.seed(image,mask,bbox,1_000_000_000)
                center=tuple((bbox[:2]+bbox[2:])/2)
                move=cv2.getRotationMatrix2D(center,2,1.02)
                move[:,2]+=(8,-4)
                current=cv2.warpAffine(image,move,(640,480),borderValue=(45,45,45))
                tracked,_=tracker.update(current,1_100_000_000)
                expected=cv2.warpAffine(mask.astype(np.uint8),move,(640,480),
                                       flags=cv2.INTER_NEAREST).astype(bool)
                self.assertGreater(np.sum(tracked&expected)/np.sum(tracked|expected),.97)
                self.assertTrue(all(score>=.75 for score in tracker.scores))

    def test_short_gap_reverifies_both_legs_and_requires_new_current_depth_confirmations(self):
        self.args.rgb_gap_recovery=True
        estimator=RGBDTargetEstimator(self.args)
        for i in range(3):
            try:
                first=estimator.process(self.image,self.depth,self.intrinsics,
                    1_000_000_000+i*33_000_000,self.seed() if i==0 else None)
            except ValueError as exc:
                self.assertIn('confirming front legs',str(exc))
        moved=cv2.warpAffine(self.image,np.float32([[1,0,5],[0,1,2]]),
                            (640,480),borderValue=(45,45,45))
        with self.assertRaisesRegex(ValueError,'confirming front legs 1/3'):
            estimator.process(moved,self.depth+100,self.intrinsics,1_700_000_000)
        self.assertTrue(estimator.tracker.reverified)
        self.assertTrue(estimator.tracker.active)
        with self.assertRaisesRegex(ValueError,'confirming front legs 2/3'):
            estimator.process(moved,self.depth+100,self.intrinsics,1_733_000_000)
        current=estimator.process(moved,self.depth+100,self.intrinsics,1_766_000_000)
        self.assertGreater(current['camera_forward_m']-first['camera_forward_m'],.08)
        self.assertEqual(estimator.seed_stamp_ns,1_000_000_000)
        self.assertEqual(estimator.tracker.stamp_ns,1_766_000_000)

    def test_gap_reverification_rejects_hidden_leg_inconsistent_motion_and_expires(self):
        for mode in ('blank','one_leg','different_motion'):
            with self.subTest(mode=mode):
                tracker=PalletRGBTracker(allow_gap_recovery=True)
                tracker.seed(self.image,self.mask,self.bbox,1_000_000_000)
                image=self.image.copy()
                if mode=='blank': image[:]=45
                elif mode=='one_leg': image[255:310,365:400]=45
                else:
                    for a,b,shift in ((265,295,8),(365,395,-8)):
                        patch=self.image[255:310,a:b].copy()
                        image[255:310,a:b]=45
                        image[255:310,a+shift:b+shift]=patch
                with self.assertRaisesRegex(ValueError,'RGB re-verification pending'):
                    tracker.update(image,1_700_000_000)
                self.assertTrue(tracker.active)
                with self.assertRaisesRegex(ValueError,'RGB tracking gap'):
                    tracker.update(image,2_600_000_000)
                self.assertFalse(tracker.active)

    def test_no_yolo_needed_after_seed_and_depth_is_current(self):
        estimator = RGBDTargetEstimator(self.args)
        for i in range(2):
            with self.assertRaisesRegex(ValueError, 'confirming front legs'):
                estimator.process(self.image, self.depth, self.intrinsics,
                                  1_000_000_000+i*33_000_000, self.seed() if i == 0 else None)
        first = estimator.process(self.image, self.depth, self.intrinsics, 1_066_000_000)
        for i in range(3, 33):
            last = estimator.process(self.image, self.depth+10, self.intrinsics,
                                     1_000_000_000+i*33_000_000)
        # Changes in new depth must affect the measurement, despite no new YOLO seed.
        self.assertGreater(last['camera_forward_m']-first['camera_forward_m'], .008)
        self.assertEqual(estimator.seed_stamp_ns, 1_000_000_000)
        self.assertTrue(estimator.tracker.active)

        for i in range(33, 36):
            with self.assertRaisesRegex(ValueError, 'insufficient valid depth'):
                estimator.process(self.image, self.depth*0, self.intrinsics,
                                  1_000_000_000+i*33_000_000)
        self.assertTrue(estimator.tracker.active)
        # Depth loss clears poses, while independent RGB identity remains.
        for i in range(36, 38):
            with self.assertRaisesRegex(ValueError, 'confirming front legs'):
                estimator.process(self.image, self.depth, self.intrinsics,
                                  1_000_000_000+i*33_000_000)
        recovered = estimator.process(self.image, self.depth, self.intrinsics,
                                      1_000_000_000+38*33_000_000)
        self.assertAlmostEqual(recovered['camera_forward_m'], first['camera_forward_m'])

    def test_fresh_yolo_masks_do_not_replace_a_healthy_rgb_reference(self):
        estimator = RGBDTargetEstimator(self.args)
        for i in range(3):
            try:
                first = estimator.process(self.image, self.depth, self.intrinsics,
                    1_000_000_000+i*33_000_000, self.seed() if i == 0 else None)
            except ValueError as exc:
                self.assertIn('confirming front legs', str(exc))
        for i in range(3, 16):
            stamp = 1_000_000_000+i*33_000_000
            seed = self.seed()
            seed['stamp_ns'] = stamp
            seed['mask'] = np.roll(self.mask, 2 if i % 2 else -2, axis=0)
            seed['bbox'] = self.bbox+np.array([0, 2 if i % 2 else -2, 0, 2 if i % 2 else -2])
            current = estimator.process(self.image, self.depth+10, self.intrinsics, stamp, seed)
            self.assertEqual(estimator.seed_stamp_ns, 1_000_000_000)
            np.testing.assert_array_equal(estimator.tracker.seed_mask.astype(bool), self.mask)
            self.assertGreater(current['camera_forward_m']-first['camera_forward_m'], .008)
        estimator.reset('target lost')
        seed = self.seed(); seed['stamp_ns'] = 1_600_000_000
        with self.assertRaisesRegex(ValueError, 'confirming front legs 1/3'):
            estimator.process(self.image, self.depth, self.intrinsics, seed['stamp_ns'], seed)
        self.assertEqual(estimator.seed_stamp_ns, seed['stamp_ns'])

    def test_camera_info_skew_rejects_pose_without_losing_rgb_target(self):
        from sensor_msgs.msg import Image, CameraInfo
        from builtin_interfaces.msg import Time
        from publish_pallet_target import process_pair
        self.args.max_result_age_ms = 500
        estimator = RGBDTargetEstimator(self.args)
        current_stamp = [1_000_000_000]
        seed_pending = [self.seed()]
        def take_seed(_):
            return seed_pending.pop() if seed_pending else None
        node = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=current_stamp[0])),
            take_seed=take_seed)
        def pair(index, bad_info=False):
            ns = 1_000_000_000+index*33_000_000
            current_stamp[0] = ns
            stamp = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)
            rgb = Image(height=480, width=640, encoding='bgr8', step=640*3,
                        data=self.image.tobytes())
            depth = Image(height=480, width=640, encoding='16UC1', step=640*2,
                          data=self.depth.tobytes())
            info = CameraInfo(height=480, width=640, distortion_model='plumb_bob',
                              k=self.intrinsics[0].ravel().tolist(), d=[0.]*5)
            for msg in (rgb, depth, info):
                msg.header.frame_id = 'camera_color_optical_frame'
                msg.header.stamp = stamp
            if bad_info:
                old = ns-70_000_000
                info.header.stamp = Time(sec=old//1_000_000_000, nanosec=old%1_000_000_000)
            registration = dict(verified=True, parameter='depth_registration', value=True,
                                check_age_ms=0, source_diagnostics={'endpoints': {'rgb': [{}, {}]}})
            return rgb, depth, 0., info, None, registration
        for i in range(2):
            with self.assertRaisesRegex(ValueError, 'confirming front legs'):
                process_pair(pair(i), self.args, node, estimator)
        self.assertTrue(process_pair(pair(2), self.args, node, estimator).valid)
        for i in range(3, 7):
            with self.assertRaisesRegex(ValueError, 'SYNC_WAIT.*within 50 ms'):
                process_pair(pair(i, bad_info=True), self.args, node, estimator)
            self.assertTrue(estimator.tracker.active)
            self.assertEqual(estimator.tracker.stamp_ns, current_stamp[0])
            self.assertEqual(len(estimator.history), 0)
        for i in (7, 8):
            with self.assertRaisesRegex(ValueError, 'confirming front legs'):
                process_pair(pair(i), self.args, node, estimator)
        restored = process_pair(pair(9), self.args, node, estimator)
        self.assertTrue(restored.valid)
        self.assertEqual(estimator.seed_stamp_ns, 1_000_000_000)
        self.assertFalse(seed_pending)

    def test_stale_seed_never_initializes_tracking(self):
        estimator = RGBDTargetEstimator(self.args)
        with self.assertRaisesRegex(ValueError, '500 ms'):
            estimator.process(self.image, self.depth, self.intrinsics, 1_600_000_000, self.seed())
        self.assertFalse(estimator.tracker.active)

    def test_delayed_semantic_seed_replays_rgb_and_measures_current_depth(self):
        estimator = RGBDTargetEstimator(self.args)
        # No identity yet: retain observed RGB while semantic inference runs.
        for i in range(7):
            with self.assertRaisesRegex(ValueError, 'LOST'):
                estimator.process(self.image, self.depth, self.intrinsics,
                                  1_000_000_000+i*100_000_000)
        # A 700-ms-old semantic mask is brought to the present through RGB.
        for i in range(7, 9):
            with self.assertRaisesRegex(ValueError, 'confirming front legs'):
                estimator.process(self.image, self.depth+100, self.intrinsics,
                                  1_000_000_000+i*100_000_000,
                                  self.seed() if i == 7 else None)
        current = estimator.process(self.image, self.depth+100, self.intrinsics, 1_900_000_000)
        expected = estimator.measure(self.mask, self.bbox, self.depth+100, self.intrinsics)
        old_depth = estimator.measure(self.mask, self.bbox, self.depth, self.intrinsics)
        self.assertAlmostEqual(current['camera_forward_m'], expected['camera_forward_m'], places=3)
        self.assertGreater(current['camera_forward_m']-old_depth['camera_forward_m'], .08)
        self.assertEqual(estimator.tracker.stamp_ns, 1_900_000_000)

    def test_seed_over_one_second_rejected_even_with_rgb_history(self):
        estimator = RGBDTargetEstimator(self.args)
        for i in range(11):
            estimator.remember_rgb(self.image, 1_000_000_000+i*100_000_000)
        with self.assertRaisesRegex(ValueError, 'semantic seed outside acquisition age limit'):
            estimator.process(self.image, self.depth, self.intrinsics, 2_100_000_000, self.seed())
        self.assertFalse(estimator.tracker.active)

    def test_static_noise_does_not_accumulate_mask_drift(self):
        tracker = PalletRGBTracker()
        tracker.seed(self.image, self.mask, self.bbox, 1_000_000_000)
        rng = np.random.default_rng(7)
        for i in range(1, 61):
            noise = rng.normal(0, 2, self.image.shape)
            image = np.clip(self.image.astype(float)+noise, 0, 255).astype(np.uint8)
            mask, bbox = tracker.update(image, 1_000_000_000+i*33_000_000)
        self.assertGreater(np.sum(mask & self.mask)/np.sum(mask | self.mask), .99)
        np.testing.assert_allclose(bbox, self.bbox, atol=.3)

    def test_slow_exposure_change_compensates_reference_and_keeps_motion(self):
        from unittest.mock import patch
        tracker = PalletRGBTracker()
        tracker.seed(self.image, self.mask, self.bbox, 1_000_000_000)
        with patch.object(tracker, '_flow_transform', wraps=tracker._flow_transform) as flow:
            for i in range(1, 41):
                transform = np.array([[1.,0.,i*.1], [0.,1.,0.]])
                shifted = cv2.warpAffine(self.image, transform, (640,480), borderValue=(45,45,45))
                image = np.clip(shifted.astype(float)+i*1.5, 0, 255).astype(np.uint8)
                mask, bbox = tracker.update(image,1_000_000_000+i*33_000_000)
            self.assertGreater(flow.call_count, 40)
        self.assertTrue(tracker.active)
        np.testing.assert_allclose(bbox,self.bbox+np.array([4,0,4,0]),atol=.6)
        # Real disappearance must still invalidate after the exposure recovery.
        with self.assertRaises(ValueError):
            tracker.update(np.full_like(self.image, 45),2_400_000_000)
        self.assertTrue(tracker.active)
        with self.assertRaises(ValueError):
            tracker.update(np.full_like(self.image,45),2_900_000_000)
        self.assertFalse(tracker.active)

    def test_one_bad_rgb_frame_clears_pose_then_recovers_without_new_yolo(self):
        estimator = RGBDTargetEstimator(self.args)
        for i in range(3):
            try:
                estimator.process(self.image,self.depth,self.intrinsics,
                    1_000_000_000+i*33_000_000,self.seed() if i==0 else None)
            except ValueError as exc:
                self.assertIn('confirming front legs',str(exc))
        with self.assertRaises(ValueError):
            estimator.process(np.full_like(self.image,45),self.depth,self.intrinsics,1_099_000_000)
        self.assertFalse(estimator.history)
        self.assertTrue(estimator.tracker.active)
        self.assertEqual(estimator.tracker.stamp_ns,1_066_000_000)
        for stamp in (1_132_000_000,1_165_000_000):
            with self.assertRaisesRegex(ValueError,'confirming front legs'):
                estimator.process(self.image,self.depth,self.intrinsics,stamp)
        current=estimator.process(self.image,self.depth+10,self.intrinsics,1_198_000_000)
        self.assertEqual(estimator.seed_stamp_ns,1_000_000_000)
        self.assertAlmostEqual(current['camera_forward_m'],
            estimator.measure(self.mask,self.bbox,self.depth+10,self.intrinsics)['camera_forward_m'],places=4)
