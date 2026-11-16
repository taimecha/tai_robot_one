"""Navigation uses a short hold; teaching retains the full stable window."""
import math
from types import SimpleNamespace,MethodType
import unittest
from unittest.mock import patch

from r3_recognition_goal import RecognitionGoal,NAV_QUALITY_HOLD_S,A_QUALITY_HOLD_S


class NavigationHoldTest(unittest.TestCase):
    def probe(self,continuous):
        now=[100.]
        pose=(1.,2.,.3)
        covariance=[0.]*36
        covariance[0]=covariance[7]=.01**2
        covariance[35]=math.radians(.5)**2
        result=SimpleNamespace(continuous_localization=continuous,stationary_mode=True,
            stationary_evidence=SimpleNamespace(fault=None),a_scan_window_start=None,
            a_scan_window_last=None,a_scan_window_count=0,a_scan_quality_reason='',
            map_match=dict(ready=True,inlier_ratio=.83,median_distance_m=.01),
            amcl=SimpleNamespace(pose=SimpleNamespace(covariance=covariance)),
            particle_gate=SimpleNamespace(metrics=dict(xy_rms_m=.015,yaw_std_deg=.5,
                unique_pose_bins=100,largest_pose_bin_mass=.02)),
            recent_a_poses=lambda:[pose]*3,current_pose=lambda require_still:pose,
            now=lambda:now[0],readiness_diagnostics={})
        for name in ('required_quality_hold_s','_reset_a_scan_quality','_observe_a_scan_quality',
                     'a_scan_quality_duration','a_scan_quality_ready','wait_pose'):
            setattr(result,name,MethodType(getattr(RecognitionGoal,name),result))
        return result,now

    def feed(self,probe,now,stamp,ratio=.83):
        now[0]=stamp
        fit=dict(ready=True,inlier_ratio=ratio,median_distance_m=.01)
        probe.map_match=fit
        probe._observe_a_scan_quality(stamp,fit)

    def test_navigation_half_second_boundary_and_teaching_two_seconds(self):
        for continuous,required in ((True,.5),(False,2.)):
            with self.subTest(continuous=continuous):
                probe,now=self.probe(continuous)
                self.assertEqual(probe.required_quality_hold_s(),required)
                for stamp in (100.,100.2,100.4,100.49):
                    self.feed(probe,now,stamp)
                self.assertFalse(probe.a_scan_quality_ready())
                self.feed(probe,now,100.5)
                self.assertEqual(probe.a_scan_quality_ready(),continuous)
                if not continuous:
                    for stamp in (100.75,101.,101.25,101.5,101.75,102.):
                        self.feed(probe,now,stamp)
                    self.assertTrue(probe.a_scan_quality_ready())
        self.assertEqual(NAV_QUALITY_HOLD_S,.5)
        self.assertEqual(A_QUALITY_HOLD_S,2.)

    def test_bad_scan_resets_navigation_half_second_at_the_runtime_fit_limit(self):
        probe,now=self.probe(True)
        for stamp in (100.,100.2,100.4,100.5):
            self.feed(probe,now,stamp)
        self.assertTrue(probe.a_scan_quality_ready())
        self.feed(probe,now,100.6,ratio=.649)
        self.assertFalse(probe.a_scan_quality_ready())
        for stamp in (100.7,100.9,101.1,101.19):
            self.feed(probe,now,stamp)
        self.assertFalse(probe.a_scan_quality_ready())
        self.feed(probe,now,101.2)
        self.assertTrue(probe.a_scan_quality_ready())

    def test_wait_pose_uses_selected_hold_for_both_timer_and_distinct_scans(self):
        for continuous,required in ((True,.5),(False,2.)):
            with self.subTest(continuous=continuous):
                probe,now=self.probe(continuous)
                def spin():
                    self.feed(probe,now,round(now[0]+.1,6))
                probe.spin_once=spin
                with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                        patch('r3_recognition_goal.time.monotonic',side_effect=lambda:now[0]):
                    pose=probe.wait_pose(5.)
                self.assertEqual(pose,(1.,2.,.3))
                self.assertGreaterEqual(probe.a_scan_quality_duration(),required)
                self.assertLessEqual(probe.a_scan_quality_duration(),required+.11)

    def test_duplicate_timestamps_cannot_complete_the_shorter_window(self):
        probe,now=self.probe(True)
        for _ in range(30):
            self.feed(probe,now,100.)
        self.assertEqual(probe.a_scan_window_count,1)
        self.assertFalse(probe.a_scan_quality_ready())


if __name__=='__main__':
    unittest.main()
