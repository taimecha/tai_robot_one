"""Stationary bootstrap guards and seed contract; no ROS nodes or hardware."""
import math
from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

from builtin_interfaces.msg import Time
from r3_global_localization import (agree_candidates,candidate_pose,fresh,motion_issue,
                                     seed_message,GlobalLocalization)


def report(pose):
    return dict(accepted=True,best=dict(pose_xyyaw=list(pose)))


class GlobalLocalizationTest(unittest.TestCase):
    def run_probe(self,apply,accepted=True):
        wall=[10.];first=Future();first.set_result(report((1.,2.,0.)) if accepted else
                                                 dict(accepted=False,reason='ambiguous',best={}))
        probe=SimpleNamespace(grid=object(),scan=object(),odom=object(),odom_stamp=10.,
            scan_stamp=10.,still_since=9.,future=first,reports=[],capture_stamp=9.9,
            after_scan_stamp=-math.inf,active=False,active_wall=0.,now=lambda:wall[0],
            check_running=Mock(),poll_active=Mock(),get_logger=Mock(),publisher=Mock(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=10))))
        probe.publisher.get_subscription_count.return_value=0
        captures=[];waited_without_active=[]
        def begin():
            captures.append((probe.scan_stamp,probe.after_scan_stamp))
            probe.capture_stamp=probe.scan_stamp
            probe.future=Future();probe.future.set_result(report((1.01,2.,.01)))
        probe.begin_match=begin
        def spin(*args,**kwargs):
            wall[0]=round(wall[0]+.05,5)
            probe.odom_stamp=wall[0]
            # A scan captured exactly when the first match completes cannot
            # stand in for the required independent subsequent capture.
            probe.scan_stamp=10.05 if wall[0]<=10.1 else wall[0]
            if len(probe.reports)==2:
                waited_without_active.append(True)
                probe.active=True;probe.active_wall=wall[0]
                probe.publisher.get_subscription_count.return_value=1
        with patch('r3_global_localization.time.monotonic',side_effect=lambda:wall[0]), \
                patch('r3_global_localization.rclpy.ok',return_value=True), \
                patch('r3_global_localization.rclpy.spin_once',side_effect=spin):
            if not accepted:
                with self.assertRaisesRegex(ValueError,'ambiguous'):
                    GlobalLocalization.run(probe,2.,apply)
                return probe,captures,waited_without_active,None
            result=GlobalLocalization.run(probe,2.,apply)
        return probe,captures,waited_without_active,result

    def test_second_capture_is_after_first_completes_and_report_mode_never_seeds(self):
        probe,captures,_,result=self.run_probe(False)
        self.assertEqual(result['status'],'CANDIDATE')
        self.assertEqual(len(captures),1)
        self.assertGreater(captures[0][0],captures[0][1])
        probe.publisher.publish.assert_not_called()

    def test_apply_waits_for_active_amcl_subscriber_then_publishes_once(self):
        probe,_,waited,result=self.run_probe(True)
        self.assertTrue(waited)
        self.assertEqual(result['status'],'SEEDED')
        probe.publisher.publish.assert_called_once()
        self.assertEqual(probe.publisher.publish.call_args.args[0].header.frame_id,'map')

    def test_rejected_candidate_remains_in_evidence_and_never_seeds(self):
        probe,captures,_,_=self.run_probe(True,accepted=False)
        self.assertEqual(probe.reports[0]['match']['reason'],'ambiguous')
        self.assertEqual(captures,[])
        probe.publisher.publish.assert_not_called()

    def test_independent_candidates_must_agree_with_wrapped_heading(self):
        a=report((1.,2.,math.radians(179.)))
        diff=agree_candidates(a,report((1.05,2.,math.radians(-179.))))
        self.assertAlmostEqual(diff['position_difference_m'],.05)
        self.assertAlmostEqual(diff['heading_difference_deg'],2.)
        for b in (report((1.051,2.,a['best']['pose_xyyaw'][2])),
                  report((1.,2.,math.radians(-178.9))),dict(accepted=False,best=a['best'])):
            with self.assertRaises(ValueError):agree_candidates(a,b)

    def test_invalid_candidate_cannot_be_published(self):
        for value in ({},dict(accepted=True),report([0.,math.nan,0.]),
                      report([0.,0.,True]),dict(accepted=False,best=dict(pose_xyyaw=[0.,0.,0.]))):
            with self.subTest(value=value),self.assertRaises(ValueError):candidate_pose(value)

    def test_motion_and_accumulated_displacement_abort_stationary_search(self):
        base=(1.,2.,math.radians(179.))
        self.assertIsNone(motion_issue((1.004,2.,math.radians(179.4)),(0.,0.,.014),base))
        for pose,twist in ((base,(.005,0.,0.)),(base,(0.,.005,0.)),(base,(0.,0.,.015)),
                           ((1.006,2.,base[2]),(0.,0.,0.)),
                           ((1.,2.,base[2]+math.radians(.6)),(0.,0.,0.)),
                           ((math.nan,2.,base[2]),(0.,0.,0.))):
            with self.subTest(pose=pose,twist=twist):self.assertIsNotNone(motion_issue(pose,twist,base))

    def test_freshness_has_no_fallback_after_solver_begins(self):
        self.assertTrue(fresh(10.,10.2,.25))
        for stamp in (None,9.,10.3,math.nan):self.assertFalse(fresh(stamp,10.,.25))
        probe=SimpleNamespace(started=True,fault=None,odom_stamp=9.,scan_stamp=10.,now=lambda:10.1)
        with self.assertRaisesRegex(ValueError,'gap exceeds'):
            GlobalLocalization.check_running(probe)
        probe.odom_stamp=10.;GlobalLocalization.check_running(probe)
        probe.fault='robot motion detected'
        with self.assertRaisesRegex(ValueError,'motion'):
            GlobalLocalization.check_running(probe)

    def test_seed_has_map_frame_current_timestamp_and_explicit_covariance(self):
        stamp=Time(sec=123,nanosec=450)
        msg=seed_message((1.,2.,math.radians(30.)),stamp)
        self.assertEqual(msg.header.frame_id,'map');self.assertEqual(msg.header.stamp,stamp)
        self.assertEqual((msg.pose.pose.position.x,msg.pose.pose.position.y),(1.,2.))
        self.assertAlmostEqual(msg.pose.pose.orientation.z,math.sin(math.radians(15.)))
        self.assertAlmostEqual(msg.pose.covariance[0],.05**2)
        self.assertAlmostEqual(msg.pose.covariance[7],.05**2)
        self.assertAlmostEqual(msg.pose.covariance[35],math.radians(2.)**2)


if __name__=='__main__':
    unittest.main()
