import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
import cv2
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'scripts')]
from pallet_target_geometry import measure_pallet_target
from pallet_plane_heading import floor_depth_skew
from pallet_rgbd_estimator import RGBDTargetEstimator


class PlaneHeadingTest(unittest.TestCase):
    def scene(self,yaw=0.,skew=.0001):
        k=np.array([[613.,0,323.],[0,611.,238.],[0,0,1.]])
        v,u=np.mgrid[:480,:640]
        rx=(u-k[0,2])/k[0,0];ry=(v-k[1,2])/k[1,1]
        pitch=math.radians(20)
        inverse_floor=(math.cos(pitch)*ry+math.sin(pitch))/.75
        inverse_pallet=(math.tan(math.radians(yaw))*rx-math.sin(pitch)*ry+math.cos(pitch))/1.8
        mask=np.zeros((480,640),bool)
        mask[170:260,270:390]=True
        mask[260:300,270:288]=True;mask[260:300,372:390]=True
        inverse=np.where(mask,inverse_pallet,inverse_floor)-skew*(370-u)
        depth=np.clip(np.rint(1000/np.maximum(inverse,.0001)),0,65535).astype(np.uint16)
        return mask,np.array([270,170,390,300]),depth,(k,np.zeros(5))

    def test_independent_floor_skew_and_real_positive_negative_plane_angles(self):
        for skew in (.00007,.00010,.00015807):
            for yaw in (0.,5.,-5.):
                with self.subTest(skew=skew,yaw=yaw):
                    mask,box,depth,intrinsics=self.scene(yaw,skew)
                    measured_skew,_=floor_depth_skew(depth,intrinsics)
                    self.assertAlmostEqual(measured_skew,skew,delta=2e-6)
                    m=measure_pallet_target(mask,box,depth,intrinsics,heading_floor_reference=True)
                    self.assertAlmostEqual(math.degrees(m['face_yaw_rad']),yaw,delta=.15)
                    self.assertAlmostEqual(m['entry_heading_rad'],-m['face_yaw_rad'])

    def test_position_and_depth_sample_fields_are_identical(self):
        mask,box,depth,intrinsics=self.scene()
        old=measure_pallet_target(mask,box,depth,intrinsics)
        new=measure_pallet_target(mask,box,depth,intrinsics,heading_floor_reference=True)
        for key in ('camera_forward_m','chassis_front_forward_m','opening_lateral_m',
                    'opening_width_m','leg_center_separation_m','left_depth_samples','right_depth_samples'):
            self.assertEqual(old[key],new[key])
        args=SimpleNamespace(pitch_deg=20,inverse_depth_x_slope=.00015807,reference_u=370,
            camera_ahead_of_chassis_m=.0665,expected_opening_width_m=.252,
            opening_width_tolerance_m=.08,confirm_frames=3,heading_floor_reference=True,
            measurement_bias=dict(forward_bias_m=.03843,lateral_bias_m=.02053,face_yaw_bias_deg=.89627))
        result=RGBDTargetEstimator(args).measure(mask,box,depth,intrinsics)
        self.assertAlmostEqual(math.degrees(result['face_yaw_rad']),0.,delta=.15)
        self.assertEqual(result['applied_heading_bias_deg'],0.)
        self.assertEqual(result['camera_forward_m'],old['camera_forward_m']+.03843)
        self.assertEqual(result['opening_lateral_m'],old['opening_lateral_m']+.02053)

    def test_missing_floor_and_missing_leg_are_rejected(self):
        mask,box,depth,intrinsics=self.scene()
        for mode in ('floor','leg'):
            broken=depth.copy()
            if mode=='floor':broken[350:470,:]=0
            else:broken[260:300,270:288]=0
            with self.subTest(mode=mode),self.assertRaises(ValueError):
                measure_pallet_target(mask,box,broken,intrinsics,heading_floor_reference=True)

    def test_near_pallet_is_excluded_from_independent_floor_fit(self):
        _,_,depth,intrinsics=self.scene()
        depth[340:470,190:450]=1400
        exclude=np.zeros_like(depth,bool);exclude[328:480,178:462]=True
        slope,_=floor_depth_skew(depth,intrinsics,exclude)
        self.assertAlmostEqual(slope,.0001,delta=2e-6)
        depth[350:470,:]=0
        with self.assertRaises(ValueError):floor_depth_skew(depth,intrinsics,exclude)

    def test_another_object_in_floor_band_does_not_tilt_the_reference(self):
        _,_,depth,intrinsics=self.scene()
        depth[350:470,30:170]=1250
        slope,quality=floor_depth_skew(depth,intrinsics)
        self.assertAlmostEqual(slope,.0001,delta=2e-6)
        self.assertLess(quality['heading_floor_mad_inv_m'],.006)
        depth[350:470,:430]=1250
        with self.assertRaises(ValueError):floor_depth_skew(depth,intrinsics)


if __name__=='__main__':unittest.main()
