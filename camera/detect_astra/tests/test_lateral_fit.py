import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fit_pallet_lateral import fit


class LateralFitTest(unittest.TestCase):
    def fixture(self):
        rows=[dict(stamp_s=100+i*.05,lateral_raw_m=.040+(i%3-1)*.0001,status='TRACKED_RGBD') for i in range(241)]
        odom=[dict(stamp_s=r['stamp_s'],x=0.,y=0.,yaw=0.,v=0.,w=0.) for r in rows]
        return dict(samples=rows,odometry=odom,latest_age_s=.06,
            recognition=dict(rgbd_health=dict(stable=True))),dict(
                forward_bias_m=.0384298849,lateral_bias_m=.0205314774,face_yaw_bias_deg=.896271969)

    def test_refit_preserves_forward_and_heading_and_checks_distinct_time_window(self):
        capture,base=self.fixture();saved=copy.deepcopy(base)
        profile=fit(capture,base,.025)
        self.assertEqual(base,saved)
        self.assertAlmostEqual(profile['lateral_bias_m'],-.015)
        self.assertEqual(profile['forward_bias_m'],base['forward_bias_m'])
        self.assertEqual(profile['face_yaw_bias_deg'],base['face_yaw_bias_deg'])
        self.assertFalse(profile['motion_qualified'])
        self.assertLess(profile['lateral_calibration']['check_p95_absolute_error_m'],.0002)

    def test_corrected_data_motion_stale_and_unstable_data_cannot_fit(self):
        for failure in ('biased','motion','stale','unstable','jitter'):
            capture,base=self.fixture()
            if failure=='biased':capture['samples'][0]['status']+='; target_bias_applied'
            if failure=='motion':capture['odometry'][50]['x']=.006
            if failure=='stale':capture['latest_age_s']=1.
            if failure=='unstable':capture['recognition']['rgbd_health']['stable']=False
            if failure=='jitter':
                for r in capture['samples'][150:]:r['lateral_raw_m']+=.02
            with self.subTest(failure=failure),self.assertRaises(ValueError):fit(capture,base,.025)


if __name__=='__main__':unittest.main()
