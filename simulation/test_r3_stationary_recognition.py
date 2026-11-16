"""Late A readers validate live scans without resampling or re-dating AMCL."""
import copy
import json
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.context import Context
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformException

from r3_core import compose
from r3_recognition_goal import RecognitionGoal,validate_a_localization


class StationaryRecognitionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context=Context()
        rclpy.init(context=cls.context,domain_id=229)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def setUp(self):
        self.now=100.
        self.node=RecognitionGoal(node_name='stationary_recognition_'+self._testMethodName,
                                 selected_map_file='/tmp/selected.yaml',context=self.context)
        self.node.now=lambda:self.now
        self.node.tf=Mock()
        self.node.tf.lookup_transform.return_value=SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=.3,y=.2),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        self.node.map_check=Mock()
        self.node.map_check.score.side_effect=lambda *a,**k:dict(
            ready=True,inlier_ratio=.95,median_distance_m=.01,reason='matching live scan')
        covariance=[0.]*36
        covariance[0]=covariance[7]=.01**2
        covariance[35]=math.radians(.5)**2
        self.anchor=[.1,.2,-.1]
        map_odom=[2.,-1.,.2]
        self.snapshot=dict(run_id='boot-session',map_file='/tmp/selected.yaml',
            amcl_stamp_s=10.,cloud_stamp_s=10.,covariance_6x6=covariance,
            map_pose_xyyaw=list(compose(map_odom,self.anchor)),
            map_odom_xyyaw=map_odom,odom_anchor_xyyaw=self.anchor,
            particle_metrics=dict(converged=True,xy_rms_m=.015,yaw_std_deg=.5,
                                  unique_pose_bins=100,largest_pose_bin_mass=.02))

    def tearDown(self):
        self.node.destroy_node()

    def status(self,*,snapshot=None,validated=True,ready=True):
        value=dict(run_id='boot-session',map_file='/tmp/selected.yaml',
            published_ros_s=self.now,ready=ready,stationary_inspection=True,
            stationary_cache=dict(enabled=True,validated=validated,
                original_covariance_preserved=True,repeated_nomotion_updates=False),
            stationary_snapshot=copy.deepcopy(snapshot or self.snapshot))
        return value

    def feed(self,stamp,*,velocity=0.,position=None,update_status=True):
        self.now=stamp
        if update_status:
            self.node.on_localization_status(String(data=json.dumps(self.status())))
        odom=Odometry()
        odom.header.frame_id='odom';odom.child_frame_id='base_footprint'
        odom.header.stamp=Time(nanoseconds=round(stamp*1e9)).to_msg()
        x,y,yaw=position or self.anchor
        odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
        odom.pose.pose.orientation.z=math.sin(yaw/2.)
        odom.pose.pose.orientation.w=math.cos(yaw/2.)
        odom.twist.twist.linear.x=velocity
        self.node.on_odom(odom)
        scan=LaserScan()
        scan.header.frame_id='lidar_link';scan.header.stamp=odom.header.stamp
        self.node.on_scan(scan)

    def settle(self,start=100.):
        for index in range(13):
            self.feed(start+index*.1)

    def test_late_join_uses_original_pose_covariance_and_only_live_scan_tf(self):
        self.settle()
        pose=self.node.current_pose()
        for actual,expected in zip(pose,self.snapshot['map_pose_xyyaw']):
            self.assertAlmostEqual(actual,expected)
        self.assertEqual(RecognitionGoal.stamp(self.node.amcl),10.)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])
        self.assertEqual(len(self.node.observations),0)
        self.assertGreaterEqual(len(self.node.recent_a_poses()),3)
        for call in self.node.tf.lookup_transform.call_args_list:
            self.assertEqual(call.args[:2],('odom','lidar_link'))
            self.assertGreaterEqual(call.args[2].nanoseconds,100_000_000_000)
        self.assertFalse(self.node.request_nomotion)
        evidence=self.node.localization_evidence()
        validation=evidence['stationary_snapshot_validation']
        self.assertEqual(validation['original_amcl_stamp_ros_s'],10.)
        self.assertGreater(validation['original_amcl_age_s'],90.)
        self.assertTrue(validation['live_scan_odom_validated'])
        self.assertFalse(validation['physical_accuracy_verified'])
        self.assertEqual(evidence['amcl_covariance_6x6'],self.snapshot['covariance_6x6'])
        self.assertEqual(evidence['recent_amcl_poses'],[])
        self.assertGreaterEqual(len(evidence['recent_scan_validated_poses']),3)

    def test_duplicates_never_create_temporal_evidence(self):
        for _ in range(20):
            self.feed(100.)
        self.assertEqual(len(self.node.stationary_evidence.scans),1)
        self.assertEqual(len(self.node.recent_a_poses()),1)
        with self.assertRaisesRegex(ValueError,'distinct matching scans'):
            self.node.current_pose()

    def test_capture_time_tf_arriving_after_scan_is_retried_without_new_amcl(self):
        self.node.tf.lookup_transform.side_effect=TransformException('TF is 11 milliseconds behind scan')
        self.feed(100.)
        self.assertEqual(list(self.node.stationary_evidence.scans),[])
        self.node.tf.lookup_transform.side_effect=None
        self.node.poll_stationary_scan()
        self.assertEqual(list(self.node.stationary_evidence.scans),[100.])
        self.settle(100.1)
        self.node.current_pose()
        self.assertEqual(RecognitionGoal.stamp(self.node.amcl),10.)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])

    def test_motion_invalidates_snapshot_until_new_original_observation(self):
        self.settle()
        self.node.current_pose()
        self.feed(101.3,velocity=.006)
        self.feed(101.4)
        with self.assertRaisesRegex(ValueError,'motion'):
            self.node.current_pose()
        self.assertEqual(self.node.stationary_snapshot['amcl_stamp_s'],10.)
        self.snapshot['amcl_stamp_s']=self.snapshot['cloud_stamp_s']=101.5
        self.settle(101.5)
        self.node.current_pose()
        self.assertEqual(RecognitionGoal.stamp(self.node.amcl),101.5)

    def test_stale_bridge_or_streams_block_reuse(self):
        self.settle()
        self.node.current_pose()
        self.now=102.
        with self.assertRaisesRegex(ValueError,'status.*stale'):
            self.node.current_pose()
        self.node.on_localization_status(String(data=json.dumps(self.status())))
        with self.assertRaisesRegex(ValueError,'odom unavailable'):
            self.node.current_pose()

    def test_bridge_not_ready_is_not_bypassed_by_good_local_scan(self):
        self.settle()
        self.node.on_localization_status(String(data=json.dumps(self.status(ready=False))))
        with self.assertRaisesRegex(ValueError,'bridge is not validating'):
            self.node.current_pose()

    def test_bad_live_scan_clears_evidence_and_strict_a_limits_are_preserved(self):
        self.settle()
        self.node.current_pose()
        self.node.map_check.score.side_effect=lambda *a,**k:dict(
            ready=False,inlier_ratio=.2,median_distance_m=.4,reason='scan mismatch')
        self.feed(101.3)
        with self.assertRaisesRegex(ValueError,'scan mismatch'):
            self.node.current_pose()
        self.assertFalse(self.node.stationary_evidence.ready(self.now)[0])
        self.assertEqual(self.node.recent_a_poses(),[])

    def test_wide_original_covariance_cannot_be_shrunk_by_live_static_scans(self):
        self.snapshot['covariance_6x6'][0]=self.snapshot['covariance_6x6'][7]=.05**2
        self.settle()
        pose=self.node.current_pose()
        with self.assertRaisesRegex(ValueError,'uncertainty at A exceeds'):
            validate_a_localization(self.node.amcl.pose.covariance,
                self.node.particle_gate.metrics,self.node.recent_a_poses(),pose)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])

    def test_snapshot_wrong_map_invalid_covariance_and_inconsistent_transforms_are_rejected(self):
        cases=[]
        wrong=copy.deepcopy(self.snapshot);wrong['map_file']='/tmp/other.yaml'
        cases.append(wrong)
        broken=copy.deepcopy(self.snapshot);broken['covariance_6x6'][1]=math.nan
        cases.append(broken)
        inconsistent=copy.deepcopy(self.snapshot);inconsistent['map_odom_xyyaw'][0]+=.1
        cases.append(inconsistent)
        stale_cloud=copy.deepcopy(self.snapshot);stale_cloud['cloud_stamp_s']=9.
        cases.append(stale_cloud)
        for snapshot in cases:
            with self.subTest(snapshot=snapshot):
                with self.assertRaises(ValueError):
                    self.node._validate_stationary_snapshot(self.status(snapshot=snapshot))

    def test_same_timestamp_cannot_replace_snapshot_and_hide_a_motion_fault(self):
        self.settle()
        self.feed(101.3,velocity=.006)
        value=self.status()
        value['stationary_snapshot']['covariance_6x6'][0]/=100.
        self.node.on_localization_status(String(data=json.dumps(value)))
        self.assertIn('without a new observation',self.node.stationary_error)
        self.assertEqual(self.node.stationary_snapshot['covariance_6x6'],self.snapshot['covariance_6x6'])

    def test_one_bad_scan_between_good_display_samples_resets_strict_a_window(self):
        self.node.map_check.score.side_effect=lambda *a,**k:dict(
            ready=True,inlier_ratio=.79 if abs(self.now-102.3)<1e-6 else .95,
            median_distance_m=.01,reason='tracking fit')
        for index in range(23):
            self.feed(100.+index*.1)
        self.assertTrue(self.node.a_scan_quality_ready())
        # Displays sample at102.0,102.5,...; a rejected scan at102.3 must
        # interrupt readiness even though no display tick saw that bad scan.
        for index in range(23,44):
            self.feed(100.+index*.1)
            if index%5==0:
                self.node.current_pose()
                self.assertGreaterEqual(self.node.map_match['inlier_ratio'],.80)
                self.assertFalse(self.node.a_scan_quality_ready())
        self.assertLess(self.node.a_scan_quality_duration(),2.)
        self.feed(104.4)
        self.assertTrue(self.node.a_scan_quality_ready())
        self.assertAlmostEqual(self.node.a_scan_quality_duration(),2.)

    def test_duplicate_or_stale_scans_cannot_extend_strict_a_duration(self):
        for _ in range(10):
            self.feed(100.)
        self.assertEqual(self.node.a_scan_quality_duration(),0.)
        self.assertFalse(self.node.a_scan_quality_ready())
        for index in range(1,22):
            self.feed(100.+index*.1)
        self.assertTrue(self.node.a_scan_quality_ready())
        duration=self.node.a_scan_quality_duration()
        self.now+=.1
        self.assertEqual(self.node.a_scan_quality_duration(),duration)
        self.now+=.5
        self.assertFalse(self.node.a_scan_quality_ready())
        self.assertEqual(self.node.a_scan_quality_duration(),0.)

    def test_new_snapshot_and_motion_interrupt_strict_a_scan_window(self):
        for index in range(23):
            self.feed(100.+index*.1)
        self.assertTrue(self.node.a_scan_quality_ready())
        self.snapshot['amcl_stamp_s']=self.snapshot['cloud_stamp_s']=102.3
        self.feed(102.3)
        self.assertFalse(self.node.a_scan_quality_ready())
        for index in range(1,23):
            self.feed(102.3+index*.1)
        self.assertTrue(self.node.a_scan_quality_ready())
        self.feed(104.6,velocity=.006)
        self.assertFalse(self.node.a_scan_quality_ready())
        self.assertEqual(self.node.a_scan_quality_duration(),0.)

    def test_map_change_interrupts_strict_a_scan_window(self):
        from nav_msgs.msg import OccupancyGrid
        for index in range(23):
            self.feed(100.+index*.1)
        self.assertTrue(self.node.a_scan_quality_ready())
        self.node.map_signature=('previous map',)
        grid=OccupancyGrid();grid.info.width=grid.info.height=2
        grid.info.resolution=1.;grid.info.origin.orientation.w=1.
        grid.data=[100,0,0,0]
        self.node.on_map(grid)
        self.assertFalse(self.node.a_scan_quality_ready())
        self.assertEqual(self.node.a_scan_quality_duration(),0.)


if __name__=='__main__':
    unittest.main()
