"""Navigation reads parked/live AMCL evidence without resampling it itself."""
import copy
import json
import math
import time
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

import rclpy
from rclpy.context import Context
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformException

from r3_core import compose
from r3_recognition_goal import (RecognitionGoal,MotionPredictionPending,
    LocalizationDataPending,NAV_DATA_RECOVERY_S,SCHEMA,map_identity)
from r3_scan_map_check import LIVE_SCAN_MAX_SAMPLES


class ContinuousRecognitionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context=Context()
        rclpy.init(context=cls.context,domain_id=225)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def setUp(self):
        self.now=100.
        self.node=RecognitionGoal(node_name='continuous_recognition_'+self._testMethodName,
            request_nomotion=True,continuous_localization=True,
            selected_map_file='/tmp/selected.yaml',context=self.context)
        self.node.now=lambda:self.now
        self.node.stationary_since=0.
        self.node.tf=Mock()
        self.node.tf.lookup_transform.return_value=SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=.3,y=.2),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        self.node.map_check=Mock()
        self.node.map_check.score.side_effect=lambda *a,**k:dict(
            ready=True,inlier_ratio=.95,median_distance_m=.01,reason='matching live scan')
        self.anchor=[.1,.2,-.1]
        self.map_odom=[2.,-1.,.2]
        self.prediction=None
        covariance=[0.]*36
        covariance[0]=covariance[7]=(.031/math.sqrt(2.))**2
        covariance[35]=math.radians(.74)**2
        self.snapshot=dict(run_id='boot-session',map_file='/tmp/selected.yaml',
            amcl_stamp_s=10.,cloud_stamp_s=10.,covariance_6x6=covariance,
            map_pose_xyyaw=list(compose(self.map_odom,self.anchor)),
            map_odom_xyyaw=self.map_odom,odom_anchor_xyyaw=self.anchor,
            particle_metrics=dict(converged=True,xy_rms_m=.031,yaw_std_deg=.74,
                                  unique_pose_bins=100,largest_pose_bin_mass=.02))

    def tearDown(self):
        self.node.destroy_node()

    def status(self,*,parked=True,live=False,ready=True):
        value=dict(run_id=self.snapshot['run_id'],map_file='/tmp/selected.yaml',
            published_ros_s=self.now,ready=ready,stationary_inspection=False,
            continuous_localization=True,navigation_monitor=True,robot_stationary=parked,
            live_observation_validated=live,moving_live_validated=live and not parked,
            stationary_cache=dict(enabled=True,validated=parked,
                original_covariance_preserved=True,repeated_nomotion_updates=False),
            stationary_snapshot=copy.deepcopy(self.snapshot))
        if self.prediction is not None:
            value['motion_prediction']=copy.deepcopy(self.prediction)
            value['motion_prediction_validated']=self.prediction.get('validated') is True
        return value

    def feed(self,stamp,*,velocity=0.,position=None,parked=True,live=False):
        self.now=stamp
        self.node.on_localization_status(String(data=json.dumps(self.status(parked=parked,live=live))))
        odom=Odometry()
        odom.header.frame_id='odom';odom.child_frame_id='base_footprint'
        odom.header.stamp=Time(nanoseconds=round(stamp*1e9)).to_msg()
        x,y,yaw=position or self.anchor
        odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
        odom.pose.pose.orientation.z=math.sin(yaw/2.)
        odom.pose.pose.orientation.w=math.cos(yaw/2.)
        odom.twist.twist.linear.x=velocity
        self.node.on_odom(odom)
        scan=LaserScan();scan.header.frame_id='lidar_link';scan.header.stamp=odom.header.stamp
        self.node.on_scan(scan)

    def settle(self,start=100.,count=23):
        for index in range(count):
            self.feed(start+index*.1)

    def fresh_snapshot(self,stamp,anchor=None):
        if anchor is not None:
            self.snapshot['odom_anchor_xyyaw']=list(anchor)
            self.snapshot['map_pose_xyyaw']=list(compose(self.map_odom,anchor))
        self.snapshot['amcl_stamp_s']=self.snapshot['cloud_stamp_s']=stamp

    def prediction_proof(self,start=102.3):
        return dict(validated=True,started_ros_s=start,anchor_odom_xyyaw=list(self.anchor),
            source_amcl_stamp_s=10.,elapsed_s=self.now-start,displacement_m=0.,yaw_change_rad=0.,
            max_duration_s=2.,max_distance_m=.08,max_yaw_rad=.08)

    def test_parked_three_point_one_cm_passes_full_wait_without_nomotion_updates(self):
        self.node.spin_once=lambda:self.feed(round(self.now+.1,6))
        with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                patch('r3_recognition_goal.time.monotonic',side_effect=lambda:self.now):
            pose=self.node.wait_pose(5.)
        self.assertFalse(self.node.request_nomotion)
        self.assertGreaterEqual(self.node.a_scan_quality_duration(),self.node.required_quality_hold_s())
        self.assertEqual(self.node.stamp(self.node.amcl),10.)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])
        self.assertAlmostEqual(self.node.a_quality['amcl_xy_rms_m'],.031)
        self.assertAlmostEqual(self.node.a_quality['amcl_yaw_std_deg'],.74)
        for actual,expected in zip(pose,self.snapshot['map_pose_xyyaw']):
            self.assertAlmostEqual(actual,expected)
        self.assertEqual(self.node.map_check.score.call_args.kwargs['max_samples'],LIVE_SCAN_MAX_SAMPLES)

    def test_runtime_seventy_nine_percent_fit_passes_without_new_amcl_or_covariance(self):
        self.node.map_check.score.side_effect=lambda *a,**kw:dict(
            ready=True,inlier_ratio=.79,median_distance_m=.012,reason='stable walls plus unmapped returns')
        self.node.spin_once=lambda:self.feed(round(self.now+.1,6))
        with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                patch('r3_recognition_goal.time.monotonic',side_effect=lambda:self.now):
            self.node.wait_pose(5.)
        self.assertEqual(self.node.stamp(self.node.amcl),10.)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])
        self.assertAlmostEqual(self.node.map_match['inlier_ratio'],.79)

    def test_startup_tf_backlog_does_not_arm_a_false_capture_gap(self):
        self.node.tf.lookup_transform.side_effect=TransformException('initial TF queue is not drained')
        self.feed(100.)
        self.feed(101.)
        self.assertTrue(self.node.stationary_startup_pending)
        self.assertIsNone(self.node.stationary_evidence.observation_stamp)
        self.assertIsNone(self.node.stationary_evidence.fault)
        self.node.tf.lookup_transform.side_effect=None
        self.feed(101.1)
        self.assertFalse(self.node.stationary_startup_pending)
        self.assertEqual(self.node.stationary_evidence.observation_stamp,10.)
        self.settle(start=101.2,count=16)
        self.node.current_pose(True)
        self.assertEqual(self.node.stamp(self.node.amcl),10.)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])
        # Once this reader adopts a fresh baseline, an actual capture gap
        # still latches until a new AMCL observation, even if the peer is OK.
        self.feed(103.5)
        with self.assertRaisesRegex(ValueError,'(?:odom|LiDAR) capture gap/clock change|odom gap/clock change'):
            self.node.current_pose(True)
        self.assertFalse(self.node.stationary_startup_pending)

    def test_startup_needs_both_bridge_parked_proof_and_own_good_scan(self):
        self.feed(100.,parked=False)
        self.assertTrue(self.node.stationary_startup_pending)
        self.assertIsNone(self.node.stationary_evidence.observation_stamp)
        self.node.map_check.score.side_effect=lambda *a,**k:dict(
            ready=False,inlier_ratio=.2,median_distance_m=.4,reason='startup map mismatch')
        self.feed(100.1)
        with self.assertRaisesRegex(ValueError,'startup map mismatch'):
            self.node.current_pose(True)
        self.assertTrue(self.node.stationary_startup_pending)
        self.assertIsNone(self.node.stationary_evidence.observation_stamp)

    def test_old_parked_snapshot_cannot_be_used_as_live_pose_after_motion(self):
        self.settle()
        self.node.current_pose(True)
        self.feed(102.3,velocity=.05,position=(.12,.2,-.1),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'live AMCL observation.*stale'):
            self.node.current_pose(False)

    def test_live_motion_uses_fresh_original_snapshot_and_rejects_stale_evidence(self):
        self.settle()
        position=(.13,.2,-.1)
        self.fresh_snapshot(102.3,position)
        self.feed(102.3,velocity=.05,position=position,parked=False,live=True)
        pose=self.node.current_pose(False)
        for actual,expected in zip(pose,compose(self.map_odom,position)):
            self.assertAlmostEqual(actual,expected)
        self.assertEqual(self.node.stamp(self.node.amcl),102.3)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])
        for call in self.node.tf.lookup_transform.call_args_list:
            self.assertEqual(call.args[:2],('odom','lidar_link'))
        self.feed(104.4,velocity=.05,position=(.2,.2,-.1),parked=False,live=True)
        with self.assertRaisesRegex(ValueError,'live AMCL observation.*stale'):
            self.node.current_pose(False)

    def test_fresh_live_pose_covers_deceleration_but_does_not_qualify_arrival(self):
        self.settle()
        position=(.14,.2,-.1)
        self.fresh_snapshot(102.3,position)
        self.feed(102.3,velocity=.05,position=position,parked=False,live=True)
        self.node.current_pose(False)
        self.feed(102.4,velocity=0.,position=position,parked=False,live=True)
        self.node.current_pose(False)
        with self.assertRaisesRegex(ValueError,'bridge is not validating|motion'):
            self.node.current_pose(True)

    def test_new_scan_tf_lag_uses_only_fresh_previously_checked_scan_then_retries(self):
        self.settle()
        self.node.current_pose(True)
        previous=self.node.map_match['scan_stamp_s']
        self.node.tf.lookup_transform.side_effect=TransformException('capture TF is 11 ms behind scan')
        self.feed(102.3)
        self.node.current_pose(True)
        self.assertEqual(self.node.map_match['scan_stamp_s'],previous)
        self.node.tf.lookup_transform.side_effect=None
        self.node.poll_stationary_scan()
        self.node.current_pose(True)
        self.assertAlmostEqual(self.node.map_match['scan_stamp_s'],102.3)
        self.assertEqual(self.node.stamp(self.node.amcl),10.)

    def test_actual_bad_scan_is_never_hidden_by_last_good_scan(self):
        self.settle()
        self.node.map_check.score.side_effect=lambda *a,**k:dict(
            ready=False,inlier_ratio=.2,median_distance_m=.4,reason='actual scan mismatch')
        self.feed(102.3)
        with self.assertRaisesRegex(ValueError,'actual scan mismatch'):
            self.node.current_pose(False)
        self.assertIsNone(self.node.last_validated_scan)
        self.node.tf.lookup_transform.side_effect=TransformException('next scan TF waiting')
        self.feed(102.4)
        with self.assertRaisesRegex(ValueError,'transform unavailable'):
            self.node.current_pose(False)

    def test_tf_wait_cannot_extend_previous_scan_past_its_freshness_limit(self):
        self.settle()
        original_scan_stamp=self.node.last_validated_scan[0].header.stamp
        self.node.tf.lookup_transform.side_effect=TransformException('TF unavailable')
        for index in range(1,7):
            self.feed(102.2+index*.1)
        self.assertEqual(self.node.last_validated_scan[0].header.stamp,original_scan_stamp)
        with self.assertRaisesRegex(ValueError,'transform unavailable'):
            self.node.current_pose(False)

    def test_continuous_raw_scans_reacquire_after_tf_scoring_gap_without_new_amcl(self):
        self.settle()
        self.node.current_pose(True)
        self.node.tf.lookup_transform.side_effect=TransformException('TF callback backlog')
        for index in range(1,9):
            self.feed(102.2+index*.1)
        self.assertIsNone(self.node.stationary_evidence.fault)
        self.assertFalse(self.node.stationary_evidence.ready(self.now)[0])
        self.node.tf.lookup_transform.side_effect=None
        for index in range(1,13):
            self.feed(103.+index*.1)
        self.assertIsNone(self.node.stationary_evidence.fault)
        self.node.current_pose(True)
        self.assertEqual(self.node.stamp(self.node.amcl),10.)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])

    def test_real_raw_capture_gap_still_latches_until_a_new_amcl_observation(self):
        self.settle()
        self.feed(102.8)
        self.assertIn('raw LiDAR capture gap',self.node.stationary_evidence.fault)
        for index in range(1,13):
            self.feed(102.8+index*.1)
        with self.assertRaisesRegex(ValueError,'raw LiDAR capture gap'):
            self.node.current_pose(True)
        self.fresh_snapshot(104.1)
        for index in range(13):
            self.feed(104.1+index*.1)
        self.node.current_pose(True)
        self.assertEqual(self.node.stamp(self.node.amcl),104.1)

    def test_active_navigation_rejects_a_large_map_correction_jump(self):
        self.settle()
        self.node.current_pose(False)
        self.node.goal_handle=SimpleNamespace(accepted=True)
        self.node.result_future=SimpleNamespace(done=lambda:False)
        self.map_odom=[2.2,-1.,.2]
        self.snapshot['map_odom_xyyaw']=self.map_odom
        self.fresh_snapshot(102.3,(.13,.2,-.1))
        self.feed(102.3,velocity=.05,position=(.13,.2,-.1),parked=False,live=True)
        with self.assertRaisesRegex(ValueError,'map correction jumped'):
            self.node.current_pose(False)

    def test_moving_pose_requires_validated_spread_but_allows_concentrated_particles(self):
        self.settle()
        self.fresh_snapshot(102.3,(.13,.2,-.1))
        self.feed(102.3,velocity=.05,position=(.13,.2,-.1),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'not validating live AMCL'):
            self.node.current_pose(False)
        self.snapshot['particle_metrics']['unique_pose_bins']=1
        self.snapshot['particle_metrics']['largest_pose_bin_mass']=.9
        self.fresh_snapshot(102.4,(.14,.2,-.1))
        self.feed(102.4,velocity=.05,position=(.14,.2,-.1),parked=False,live=True)
        self.node.current_pose(False)
        self.snapshot['particle_metrics']['xy_rms_m']=.05
        self.fresh_snapshot(102.5,(.15,.2,-.1))
        self.feed(102.5,velocity=.05,position=(.15,.2,-.1),parked=False,live=True)
        with self.assertRaisesRegex(ValueError,'particle spread requires <5 cm / 2 deg'):
            self.node.current_pose(False)

    def test_navigation_session_change_is_rejected_even_with_consistent_pose(self):
        self.settle()
        self.node.navigation_run_id='boot-session'
        self.snapshot['run_id']='replacement-session'
        self.feed(102.3)
        with self.assertRaisesRegex(ValueError,'session changed during navigation'):
            self.node.current_pose(False)

    def test_old_parked_amcl_predicts_only_the_bounded_first_motion_transition(self):
        self.settle()
        self.node.current_pose(True)
        self.prediction=self.prediction_proof()
        self.prediction['elapsed_s']=0.
        self.feed(102.3,velocity=.05,position=(.102,.2,-.1),parked=False,live=False)
        pose=self.node.current_pose(False)
        self.assertEqual(self.node.current_localization_mode,'bounded_motion_prediction')
        self.assertEqual(self.node.stamp(self.node.amcl),10.)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])
        for actual,expected in zip(pose,compose(self.map_odom,(.102,.2,-.1))):
            self.assertAlmostEqual(actual,expected)
        evidence=self.node.localization_evidence()
        self.assertFalse(evidence['live_amcl_observation_fresh'])
        self.assertTrue(evidence['motion_prediction']['validated'])
        with self.assertRaisesRegex(ValueError,'bridge is not validating|motion'):
            self.node.current_pose(True)
        self.prediction['elapsed_s']=.2
        self.fresh_snapshot(102.5,(.112,.2,-.1))
        self.prediction=None
        self.feed(102.5,velocity=.05,position=(.112,.2,-.1),parked=False,live=True)
        self.node.current_pose(False)
        self.assertEqual(self.node.current_localization_mode,'fresh_live_amcl')
        self.assertEqual(self.node.stamp(self.node.amcl),102.5)

    def test_prediction_fixed_limits_source_and_own_odom_bounds_are_independent(self):
        self.settle()
        self.node.current_pose(True)
        proof=self.prediction_proof()
        proof['elapsed_s']=0.
        cases=[('larger time allowance','max_duration_s',3.),
               ('larger distance allowance','max_distance_m',.1),
               ('wrong source','source_amcl_stamp_s',9.),
               ('reported distance already outside bound','displacement_m',.081),
               ('reported rotation already outside bound','yaw_change_rad',.081)]
        for label,key,value in cases:
            with self.subTest(case=label):
                self.prediction=copy.deepcopy(proof);self.prediction[key]=value
                self.feed(102.3,velocity=.05,position=(.102,.2,-.1),parked=False,live=False)
                with self.assertRaises(ValueError):
                    self.node.current_pose(False)
        self.prediction=copy.deepcopy(proof)
        self.feed(102.3,velocity=.05,position=(.181,.2,-.1),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'distance or heading'):
            self.node.current_pose(False)
        self.feed(102.3,velocity=.05,position=(.102,.2,-.009),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'distance or heading'):
            self.node.current_pose(False)

    def test_prediction_expires_and_cannot_rearm_the_same_original_observation(self):
        self.settle()
        self.node.current_pose(True)
        self.prediction=self.prediction_proof();self.prediction['elapsed_s']=0.
        self.feed(102.3,velocity=.05,position=(.102,.2,-.1),parked=False,live=False)
        self.node.current_pose(False)
        self.prediction['started_ros_s']=102.4;self.prediction['elapsed_s']=0.
        self.feed(102.4,velocity=.05,position=(.107,.2,-.1),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'rearmed'):
            self.node.current_pose(False)
        self.prediction['started_ros_s']=102.3
        for index in range(5,24):
            stamp=102.+index*.1
            self.prediction['elapsed_s']=stamp-102.3
            self.feed(stamp,velocity=.01,position=(.13,.2,-.1),parked=False,live=False)
        self.prediction['elapsed_s']=2.01
        self.feed(104.31,velocity=.01,position=(.13,.2,-.1),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'two-second'):
            self.node.current_pose(False)

    def test_prediction_needs_own_recent_parked_validation_and_real_good_scan(self):
        self.settle()
        self.prediction=self.prediction_proof();self.prediction['elapsed_s']=0.
        self.feed(102.3,velocity=.05,position=(.102,.2,-.1),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'recent independently validated parked pose'):
            self.node.current_pose(False)
        self.node.last_parked_validation=dict(validated_ros_s=102.2,run_id='boot-session',
            source_amcl_stamp_s=10.,odom_xyyaw=list(self.anchor))
        self.node.map_check.score.side_effect=lambda *a,**k:dict(
            ready=False,inlier_ratio=.2,median_distance_m=.4,reason='prediction scan mismatch')
        self.feed(102.4,velocity=.05,position=(.107,.2,-.1),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'prediction scan mismatch'):
            self.node.current_pose(False)

    def test_motion_status_ordering_wait_is_limited_to_original_150_ms_onset(self):
        self.settle()
        self.node.current_pose(True)
        self.feed(102.3,velocity=.05,position=(.102,.2,-.1),parked=True,live=False)
        with self.assertRaises(MotionPredictionPending):
            self.node.current_pose(False)
        self.feed(102.4,velocity=.05,position=(.107,.2,-.1),parked=True,live=False)
        with self.assertRaises(MotionPredictionPending):
            self.node.current_pose(False)
        self.feed(102.46,velocity=.05,position=(.11,.2,-.1),parked=True,live=False)
        with self.assertRaisesRegex(ValueError,'did not arrive within 0.15'):
            self.node.current_pose(False)

    def test_action_wait_retries_only_the_dedicated_pending_proof(self):
        calls=[0]
        future=SimpleNamespace(done=lambda:calls[0]>=2,result=lambda:'complete')
        def spin():calls[0]+=1
        probe=SimpleNamespace(spin_once=spin,
            current_pose=Mock(side_effect=[MotionPredictionPending('wait'),None]))
        with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                patch('r3_recognition_goal.time.monotonic',return_value=100.):
            self.assertEqual(RecognitionGoal.await_future(probe,future,110.,True),'complete')
        probe.current_pose=Mock(side_effect=ValueError('real scan mismatch'))
        calls[0]=0
        with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                patch('r3_recognition_goal.time.monotonic',return_value=100.):
            with self.assertRaisesRegex(ValueError,'real scan mismatch'):
                RecognitionGoal.await_future(probe,future,110.,True)

    def test_queued_odom_gap_recovers_without_canceling_or_accepting_a_stale_pose(self):
        self.settle()
        self.node.current_pose(True)
        steps=[0]
        future=SimpleNamespace(done=lambda:steps[0]>=3,result=lambda:'same goal completed')
        def spin():
            steps[0]+=1
            if steps[0]==1:
                # A 250 ms delivery gap reproduces the two real cancellations.
                self.now=102.45
            else:
                self.feed(102.5+(steps[0]-2)*.1)
        self.node.spin_once=spin
        with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                patch('r3_recognition_goal.time.monotonic',side_effect=lambda:self.now):
            result=self.node.await_future(future,110.,True)
        self.assertEqual(result,'same goal completed')
        self.assertEqual(len(self.node.navigation_data_waits),1)
        wait=self.node.navigation_data_waits[0]
        self.assertEqual(wait['reason'],'fresh EKF odom unavailable')
        self.assertTrue(wait['recovered'])
        self.assertGreater(wait['duration_s'],0.)
        self.assertEqual(self.node.stamp(self.node.amcl),10.)
        self.assertEqual(list(self.node.amcl.pose.covariance),self.snapshot['covariance_6x6'])

    def test_persistent_odom_loss_still_ends_the_action_wait(self):
        self.settle()
        self.node.current_pose(True)
        future=SimpleNamespace(done=lambda:False)
        def spin():
            self.now=round(self.now+.1,6)
            self.node.stationary_status['published_ros_s']=self.now
        self.node.spin_once=spin
        with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                patch('r3_recognition_goal.time.monotonic',side_effect=lambda:self.now):
            with self.assertRaisesRegex(RuntimeError,'did not recover within 2 seconds.*EKF odom'):
                self.node.await_future(future,110.,True)
        wait=self.node.navigation_data_waits[0]
        self.assertFalse(wait['recovered'])
        self.assertGreaterEqual(wait['duration_s'],NAV_DATA_RECOVERY_S)
        with self.assertRaises(LocalizationDataPending):
            self.node.current_pose(False)

    def test_hard_fault_during_a_data_wait_is_not_given_a_recovery_grace(self):
        steps=[0]
        future=SimpleNamespace(done=lambda:False)
        def spin():steps[0]+=1
        probe=SimpleNamespace(spin_once=spin,current_pose=Mock(side_effect=[
            LocalizationDataPending('fresh EKF odom unavailable'),
            ValueError('localization session changed during navigation')]))
        with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                patch('r3_recognition_goal.time.monotonic',return_value=100.):
            with self.assertRaisesRegex(ValueError,'session changed'):
                RecognitionGoal.await_future(probe,future,110.,True)
        self.assertEqual(steps[0],2)
        self.assertFalse(probe.navigation_data_waits[0]['recovered'])

    def arm_native_navigation(self):
        self.settle()
        self.node.current_pose(True)
        self.node.navigation_run_id=self.snapshot['run_id']
        self.node.navigation_map_signature=self.node.map_signature
        self.node.navigation_observation_stamp=self.snapshot['amcl_stamp_s']
        self.node.navigation_action_active=True

    def test_native_goal_survives_delayed_python_quality_report(self):
        self.arm_native_navigation()
        # Reproduce the old 2 s timeout and false 0.2 s odom-age failures.
        # Native travel must use Nav2's TF/scan checks rather than these
        # late Python caches. Arrival still performs current_pose(True).
        self.now+=3.
        self.node.stationary_status.update(ready=False,
            reason='continuous odom is missing/stale',live_observation_validated=False)
        self.node.current_pose=Mock(side_effect=AssertionError('no parked quality gate during travel'))
        steps=[0]
        future=SimpleNamespace(done=lambda:steps[0]>=3,result=lambda:'same native goal completed')
        def spin():steps[0]+=1
        self.node.spin_once=spin
        with patch('r3_recognition_goal.rclpy.ok',return_value=True):
            self.assertEqual(self.node.await_future(future,time.monotonic()+5.,True),
                             'same native goal completed')
        self.node.current_pose.assert_not_called()
        self.assertFalse(self.node.navigation_data_waits)

    def test_native_travel_still_rejects_map_session_role_and_latched_fault_changes(self):
        self.arm_native_navigation()
        original=copy.deepcopy(self.node.stationary_status)
        for name,value in [('run_id','other-session'),('map_file','/tmp/other-map.yaml'),
                ('navigation_monitor',False),('localization_fault','AMCL map transform jumped')]:
            with self.subTest(field=name):
                self.node.stationary_status=copy.deepcopy(original)
                self.node.stationary_status[name]=value
                with self.assertRaises(ValueError):self.node.monitor_navigation_session()
        self.node.stationary_status=original
        self.node.navigation_map_signature='changed occupancy image'
        with self.assertRaisesRegex(ValueError,'occupancy map changed'):
            self.node.monitor_navigation_session()

    def test_native_travel_keeps_large_map_correction_jump_fatal(self):
        self.arm_native_navigation()
        self.node.stationary_snapshot=copy.deepcopy(self.snapshot)
        self.node.stationary_snapshot['amcl_stamp_s']+=1.
        self.node.stationary_snapshot['map_odom_xyyaw'][0]+=.16
        with self.assertRaisesRegex(ValueError,'jumped'):
            self.node.monitor_navigation_session()

    def test_active_native_action_does_not_repeat_parked_scan_scoring(self):
        self.arm_native_navigation()
        self.node.map_check.score.reset_mock()
        self.node.poll_stationary_scan()
        self.node.map_check.score.assert_not_called()
        self.node.navigation_action_active=False
        self.node.stationary_fit_key=None
        self.node.poll_stationary_scan()
        self.node.map_check.score.assert_called_once()

    def test_prediction_cannot_mask_a_raw_sensor_gap_or_ended_transition(self):
        self.settle()
        self.node.current_pose(True)
        self.prediction=self.prediction_proof();self.prediction['elapsed_s']=0.
        self.feed(102.3,velocity=.05,position=(.102,.2,-.1),parked=False,live=False)
        self.node.current_pose(False)
        self.prediction['ended']=True
        self.feed(102.4,velocity=.05,position=(.107,.2,-.1),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'already ended'):
            self.node.current_pose(False)
        self.prediction['ended']=False;self.prediction['elapsed_s']=.7
        self.feed(103.,velocity=.05,position=(.13,.2,-.1),parked=False,live=False)
        with self.assertRaisesRegex(ValueError,'prediction blocked by.*gap'):
            self.node.current_pose(False)

    def test_timeout_preserves_primary_failure_and_longest_valid_window(self):
        self.node.spin_once=lambda:setattr(self,'now',round(self.now+.1,6))
        self.node.current_pose=Mock(side_effect=ValueError('actual scan mismatch'))
        with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                patch('r3_recognition_goal.time.monotonic',side_effect=lambda:self.now):
            with self.assertRaisesRegex(RuntimeError,'actual scan mismatch.*last failed check'):
                self.node.wait_pose(.5)
        diagnostic=self.node.readiness_diagnostics
        self.assertEqual(diagnostic['last_failed_check'],'actual scan mismatch')
        self.assertGreater(diagnostic['failure_counts']['actual scan mismatch'],0)

    def test_check_only_checks_both_readiness_phases_without_sending_goal(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            (folder/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
            map_file=folder/'map.yaml'
            map_file.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
            goal_file=folder/'goal.json'
            goal_file.write_text(json.dumps(dict(schema=SCHEMA,purpose='pallet_recognition_A',
                frame_id='map',goal_id='test-A',saved_at_utc='2026-11-14T00:00:00Z',
                pose_xyyaw=[4.,5.,.2],map=map_identity(map_file))))
            client=Mock();client.wait_for_server.return_value=True
            probe=SimpleNamespace(wait_pose=Mock(return_value=(1.,2.,.3)),client=client,
                localization_evidence=Mock(return_value={'original_covariance_preserved':True}))
            result=RecognitionGoal.navigate(probe,map_file,goal_file,60.,check_only=True)
            self.assertEqual(result['status'],'READY_FOR_NAVIGATION')
            self.assertEqual(result['robot_commands_from_this_process'],0)
            self.assertFalse(result['navigation_goal_attempted'])
            self.assertEqual(result['direct_velocity_commands_from_this_process'],0)
            self.assertEqual(result['current_pose_xyyaw'],[1.,2.,.3])
            self.assertEqual(result['goal_pose_xyyaw'],[4.,5.,.2])
            self.assertEqual([call.args[0] for call in probe.wait_pose.call_args_list],[30.,10.])
            client.send_goal_async.assert_not_called()

    def test_rejected_goal_is_reported_as_attempted_without_claiming_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            (folder/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
            map_file=folder/'map.yaml'
            map_file.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
            goal_file=folder/'goal.json'
            goal_file.write_text(json.dumps(dict(schema=SCHEMA,purpose='pallet_recognition_A',
                frame_id='map',goal_id='test-A',saved_at_utc='2026-11-14T00:00:00Z',
                pose_xyyaw=[4.,5.,.2],map=map_identity(map_file))))
            client=Mock();client.wait_for_server.return_value=True
            probe=SimpleNamespace(wait_pose=Mock(return_value=(1.,2.,.3)),client=client,
                get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time().to_msg())),
                await_future=Mock(return_value=SimpleNamespace(accepted=False)),cancel=Mock(),
                navigation_goal_attempted=False,localization_evidence=Mock(return_value={}))
            with self.assertRaisesRegex(RuntimeError,'rejected'):
                RecognitionGoal.navigate(probe,map_file,goal_file,60.)
            self.assertTrue(probe.navigation_goal_attempted)
            client.send_goal_async.assert_called_once()


if __name__=='__main__':
    unittest.main()
