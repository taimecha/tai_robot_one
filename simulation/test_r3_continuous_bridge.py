"""Continuous localization transitions without hardware or motion commands."""
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

import rclpy
from rclpy.context import Context
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from r3_amcl_bridge import AmclTrialBridge,TrialCorrection,particle_cloud_metrics
from r3_core import compose


def stamp(seconds):
    return Time(nanoseconds=round(seconds*1e9)).to_msg()


def tf(pose):
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=pose[0],y=pose[1]),
        rotation=SimpleNamespace(x=0.,y=0.,z=math.sin(pose[2]/2.),w=math.cos(pose[2]/2.))))


class ContinuousBridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context=Context()
        rclpy.init(context=cls.context,domain_id=225)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.nodes=[]
        self.now=100.
        self.wall=100.

    def tearDown(self):
        for node in self.nodes:node.destroy_node()
        self.temp.cleanup()

    def bridge(self,monitor=False,seed=True):
        args=SimpleNamespace(map=Path(self.temp.name)/'map.yaml',initial_pose=None,
            wait_ready=None,global_seed_ready=seed,global_localization=False,
            stationary_inspection=False,continuous_localization=True,navigation_monitor=monitor)
        node=AmclTrialBridge(args,context=self.context)
        self.nodes.append(node)
        node.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=round(self.now*1e9)))
        node.global_requested_ros_s=99.
        node.active=node.map_received=True
        node.next_state=float('inf')
        node.status_pub=Mock();node.marker_pub=Mock();node.odom_pub=Mock()
        node.amcl_subscription=Mock();node.cloud_subscription=Mock()
        node.amcl_subscription.get_publisher_count.return_value=1
        node.cloud_subscription.get_publisher_count.return_value=1
        node.nomotion_interfaces_since=0.
        node.tf_buffer=Mock()
        node.tf_buffer.can_transform.return_value=True
        node.tf_buffer.lookup_transform.return_value=tf((0.,0.,0.))
        node.nomotion_client=Mock()
        node.nomotion_client.service_is_ready.return_value=True
        node.nomotion_client.call_async.return_value=Mock(done=lambda:True)
        node.last_tick=self.wall
        return node

    def odom(self,node,x=0.,y=0.,yaw=0.,v=0.,w=0.):
        message=Odometry()
        message.header.frame_id='odom';message.child_frame_id='base_footprint'
        message.header.stamp=stamp(self.now)
        message.pose.pose.position.x=x;message.pose.pose.position.y=y
        message.pose.pose.orientation.z=math.sin(yaw/2.)
        message.pose.pose.orientation.w=math.cos(yaw/2.)
        message.twist.twist.linear.x=v;message.twist.twist.angular.z=w
        with patch('r3_amcl_bridge.time.monotonic',return_value=self.wall):
            node.on_odom(message)
        scan=LaserScan();scan.header.frame_id='lidar_link';scan.header.stamp=stamp(self.now)
        node.on_scan(scan)
        node.scan_match=dict(ready=True,inlier_ratio=.9,median_distance_m=.02,
                             scan_stamp_s=self.now)

    def observe(self,node,odom=(0.,0.,0.),correction=(1.,2.,.3),collapsed=False,concentrated=False):
        map_base=compose(correction,odom)
        observation=PoseWithCovarianceStamped()
        observation.header.frame_id='map';observation.header.stamp=stamp(self.now)
        observation.pose.pose.position.x,observation.pose.pose.position.y=map_base[:2]
        observation.pose.pose.orientation.z=math.sin(map_base[2]/2.)
        observation.pose.pose.orientation.w=math.cos(map_base[2]/2.)
        observation.pose.covariance[0]=observation.pose.covariance[7]=.015**2
        observation.pose.covariance[35]=math.radians(.5)**2
        particles=[(map_base[0]+dx,map_base[1]+dy,map_base[2]+dy,1.)
                   for dx in (-.02,-.01,0.,.01,.02) for dy in (-.02,-.01,0.,.01,.02)]
        if collapsed:particles=[(*map_base,1.)]*2000
        if concentrated:
            # 95% duplicate poses, five occupied bins, about 7 mm / 0.1 deg.
            # This models the run that was rejected despite an 86% map fit.
            particles=[(*map_base,1.)]*1900
            for dx,dy,angle in ((.03,0.,.5),(-.03,0.,-.5),(0.,.03,.5),(0.,-.03,-.5)):
                particles.extend([(map_base[0]+dx,map_base[1]+dy,
                                   map_base[2]+math.radians(angle),1.)]*25)
            metrics=particle_cloud_metrics(particles,map_base)
            observation.pose.covariance[0]=observation.pose.covariance[7]=metrics['xy_rms_m']**2/2.
            observation.pose.covariance[35]=math.radians(metrics['yaw_std_deg'])**2
        node.latest_cloud=(self.now,particles)
        node.tf_buffer.lookup_transform.return_value=tf(odom)
        node.on_amcl(observation)
        node.process_pending(self.now)
        return observation

    def tick(self,node):
        with patch.object(node,'poll_services'),patch.object(node,'check_scan_fit'), \
                patch('r3_amcl_bridge.time.monotonic',return_value=self.wall):
            node.tick()
        return json.loads(node.status_pub.publish.call_args.args[0].data)

    def settle(self,node,count=16):
        for step in range(count):
            self.now=100.+step*.2;self.wall=self.now
            self.odom(node)
            if step==0:self.observe(node)
            status=self.tick(node)
        return status

    def test_parked_snapshot_keeps_original_covariance_and_never_resamples_periodically(self):
        node=self.bridge()
        status=self.settle(node)
        original=copy.deepcopy(node.stationary_snapshot)
        self.assertTrue(status['ready'],status['reason'])
        self.assertTrue(status['stationary_cache']['validated'])
        self.assertFalse(status['live_observation_validated'])
        self.assertEqual(status['amcl_stamp_s'],100.)
        self.assertGreater(status['stationary_cache']['amcl_observation_age_s'],2.)
        for index in range(30):node.poll_services(103.+index*.1)
        self.assertEqual(node.nomotion_request_count,1)
        self.assertEqual(node.stationary_snapshot,original)
        node.odom_pub.publish.assert_called()

    def test_motion_has_a_bounded_onset_then_requires_fresh_amcl_and_live_odom(self):
        node=self.bridge()
        self.settle(node)
        self.now=self.wall=103.2
        self.odom(node,x=.01,v=.05)
        stale=self.tick(node)
        self.assertTrue(stale['ready'])
        self.assertFalse(stale['stationary_cache']['validated'])
        self.assertFalse(stale['moving_live_validated'])
        self.assertTrue(stale['motion_prediction_validated'])
        observation=self.observe(node,odom=(.01,0.,0.))
        live=self.tick(node)
        self.assertTrue(live['ready'],live['reason'])
        self.assertTrue(live['moving_live_validated'])
        self.assertTrue(live['live_observation_validated'])
        self.assertFalse(live['stationary_cache']['validated'])
        self.assertEqual(live['stationary_snapshot']['amcl_stamp_s'],103.2)
        self.assertEqual(live['stationary_snapshot']['covariance_6x6'],list(observation.pose.covariance))
        self.now=self.wall=103.5
        # Missing live odom must stop output even with a recent AMCL pose.
        self.assertFalse(self.tick(node)['ready'])

    def test_external_control_odom_owns_capture_freshness_only_after_handoff(self):
        node=self.bridge(monitor=True)
        node.external_control_odom=True
        self.settle(node)
        self.now=self.wall=103.3
        self.observe(node)
        node.scan_stamp=self.now
        node.scan_match.update(scan_stamp_s=self.now)
        # Nav2 monitoring retains its original freshness rules.
        self.assertFalse(self.tick(node)['ready'])
        node.navigation_monitor=False
        status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertEqual(status['ekf_freshness_owner'],'r3_control_odom')
        self.assertGreater(status['supervisor_ekf_capture_age_s'],.2)
        node.odom_pub.publish.assert_not_called()
        # Delegation does not hide a bad map fit or an AMCL correction fault.
        node.scan_match.update(ready=False,reason='scan/map mismatch')
        self.assertFalse(self.tick(node)['ready'])

    def test_motion_onset_uses_old_original_pose_only_after_recent_parked_validation(self):
        node=self.bridge(monitor=True)
        self.settle(node,count=66)  # Original observation is now 13 seconds old.
        original=copy.deepcopy(node.stationary_snapshot)
        self.assertAlmostEqual(self.now,113.)
        self.now=self.wall=113.1
        self.odom(node,x=.001,v=.05)
        status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertTrue(status['motion_prediction_validated'])
        self.assertFalse(status['stationary_cache']['validated'])
        self.assertFalse(status['live_observation_validated'])
        self.assertFalse(status['moving_live_validated'])
        proof=status['motion_prediction']
        self.assertEqual(proof['source_amcl_stamp_s'],100.)
        self.assertGreater(proof['source_amcl_age_s'],13.)
        self.assertEqual(proof['max_duration_s'],2.)
        self.assertEqual(proof['max_distance_m'],.08)
        self.assertEqual(proof['max_yaw_rad'],.08)
        self.assertEqual(node.stationary_snapshot,original)
        # A brief stop during the maneuver cannot consume the same parked
        # proof again and extend the onset window.
        for now,v in ((113.2,0.),(113.3,.05),(113.5,.05),(113.7,.05),(113.9,.05),(114.1,.05)):
            self.now=self.wall=now
            self.odom(node,x=(now-113.1)*.03,v=v)
            status=self.tick(node)
        self.assertEqual(status['motion_prediction']['started_ros_s'],113.1)
        self.now=self.wall=114.2
        self.odom(node,x=.033,v=.05)
        self.observe(node,odom=(.033,0.,0.))
        status=self.tick(node)
        self.assertTrue(status['moving_live_validated'])
        self.assertFalse(status['motion_prediction_validated'])
        self.assertTrue(status['motion_prediction']['ended'])
        # Once fresh AMCL replaced it, the old parked proof may never rescue
        # a later stale AMCL stream during continued movement.
        for step in range(1,13):
            self.now=self.wall=114.2+step*.2
            self.odom(node,x=.033+step*.002,v=.01)
            status=self.tick(node)
        self.assertFalse(status['ready'])
        self.assertFalse(status['motion_prediction_validated'])

    def test_motion_prediction_time_distance_and_yaw_are_hard_bounds(self):
        for limit in ('time','distance','yaw'):
            node=self.bridge(monitor=True)
            self.now=self.wall=100.
            self.settle(node)
            self.now=self.wall=103.1
            self.odom(node,x=.001,v=.05)
            self.assertTrue(self.tick(node)['motion_prediction_validated'])
            if limit=='time':
                for step in range(1,12):
                    self.now=self.wall=103.1+step*.2
                    self.odom(node,x=.001+step*.002,v=.01)
                    status=self.tick(node)
            else:
                self.now=self.wall=103.2
                self.odom(node,x=.081 if limit=='distance' else .001,
                          yaw=.081 if limit=='yaw' else 0.,v=.05)
                status=self.tick(node)
            self.assertFalse(status['ready'])
            self.assertTrue(status['motion_prediction']['ended'])
            self.assertFalse(status['motion_prediction_validated'])

    def test_delayed_pose_uses_its_retained_cloud_and_ends_onset_prediction(self):
        node=self.bridge(monitor=True)
        self.settle(node)
        self.now=self.wall=103.1
        self.odom(node,x=.001,yaw=.01,w=.1)
        self.assertTrue(self.tick(node)['motion_prediction_validated'])
        capture_odom=(.001,0.,.025)
        reference=compose((1.,2.,.3),capture_odom)
        observation=PoseWithCovarianceStamped()
        observation.header.frame_id='map';observation.header.stamp=stamp(103.2)
        observation.pose.pose.position.x,observation.pose.pose.position.y=reference[:2]
        observation.pose.pose.orientation.z=math.sin(reference[2]/2.)
        observation.pose.pose.orientation.w=math.cos(reference[2]/2.)
        cloud=[(reference[0]+dx,reference[1]+dy,reference[2]+da,1.)
            for dx in (-.015,-.005,.005,.015) for dy in (-.01,.01) for da in (-.005,.005)]
        observation.pose.covariance[0]=.000125
        observation.pose.covariance[7]=.0001
        observation.pose.covariance[35]=-2*math.log(math.cos(.005))
        for now in (103.2,103.3,103.4,103.5,103.6,103.7):
            self.now=self.wall=now
            self.odom(node,x=.001,yaw=.01+(now-103.1)*.075,w=.1)
            self.tick(node)
        self.now=self.wall=103.75
        node.on_amcl(observation)
        node.latest_cloud=(103.65,[(x+.02,y,a+.04,w) for x,y,a,w in cloud])
        node.particle_cloud_history.extend([(103.35,cloud),node.latest_cloud])
        node.tf_buffer.lookup_transform.return_value=tf(capture_odom)
        node.process_pending(self.now)
        status=self.tick(node)
        self.assertTrue(status['moving_live_validated'],status['reason'])
        self.assertFalse(status['motion_prediction_validated'])
        self.assertTrue(status['motion_prediction']['ended'])
        self.assertEqual(status['stationary_snapshot']['amcl_stamp_s'],103.2)
        self.assertEqual(status['stationary_snapshot']['cloud_stamp_s'],103.35)
        self.assertEqual(status['stationary_snapshot']['covariance_6x6'],list(observation.pose.covariance))
        self.assertTrue(status['stationary_snapshot']['particle_pairing']['retained_cloud_posterior_verified'])
        from r3_amcl_observation_pair import validate_pair_evidence
        validate_pair_evidence(status['stationary_snapshot'])
        node.odom_pub.publish.assert_not_called()

    def test_motion_prediction_cannot_arm_from_old_or_failed_parked_proof(self):
        for failure in ('old','no_ready','map_change'):
            node=self.bridge(monitor=True)
            self.now=self.wall=100.
            self.settle(node)
            if failure=='old':
                self.now=self.wall=103.4  # still fresh scan/odom below .5 s gap
            else:
                self.now=self.wall=103.1
                if failure=='no_ready':node.last_parked_validation=None
                else:node.map_signature='changed-map'
            self.odom(node,x=.001,v=.05)
            status=self.tick(node)
            self.assertFalse(status['motion_prediction_validated'])
            self.assertIsNone(status['motion_prediction'])

    def test_real_sensor_gap_permanently_invalidates_an_active_onset_prediction(self):
        node=self.bridge(monitor=True)
        self.settle(node)
        self.now=self.wall=103.1
        self.odom(node,x=.001,v=.05)
        self.assertTrue(self.tick(node)['motion_prediction_validated'])
        self.now=self.wall=103.7  # Actual raw odom and scan capture gap.
        self.odom(node,x=.02,v=.05)
        status=self.tick(node)
        self.assertFalse(status['ready'])
        self.assertFalse(status['motion_prediction_validated'])
        self.assertTrue(status['motion_prediction']['ended'])

    def test_one_refresh_for_each_real_motion_then_stop_and_fresh_cache_reacquires(self):
        node=self.bridge()
        self.odom(node)
        self.now=self.wall=100.6
        self.odom(node)
        node.poll_services(100.6)
        self.assertEqual(node.nomotion_request_count,1)
        self.assertFalse(node.parking_refresh_armed)
        self.observe(node)
        self.now=self.wall=100.8
        self.odom(node,x=.04,v=.05)
        node.poll_services(100.8)
        self.assertTrue(node.parking_refresh_armed)
        self.assertEqual(node.nomotion_request_count,1)
        self.observe(node,odom=(.04,0.,0.))
        self.tick(node)
        self.now=self.wall=101.
        self.odom(node,x=.04)
        node.poll_services(101.3)
        self.assertEqual(node.nomotion_request_count,1)
        node.poll_services(101.55)
        self.assertEqual(node.nomotion_request_count,2)
        for index in range(20):node.poll_services(101.6+index*.1)
        self.assertEqual(node.nomotion_request_count,2)
        self.observe(node,odom=(.04,0.,0.))
        for step in range(7):
            self.now=self.wall=101.+step*.2
            self.odom(node,x=.04)
            status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertTrue(status['stationary_cache']['validated'])

    def test_deceleration_still_has_fresh_live_pose_while_arrival_cache_prepares(self):
        node=self.bridge(monitor=True)
        self.odom(node,v=.05)
        self.observe(node)
        self.assertTrue(self.tick(node)['moving_live_validated'])
        self.now=self.wall=100.1
        self.odom(node)
        status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertTrue(status['live_observation_validated'])
        self.assertFalse(status['moving_live_validated'])
        self.assertFalse(status['stationary_cache']['validated'])
        node.odom_pub.publish.assert_not_called()

    def test_identical_particles_cannot_pair_with_an_inconsistent_reported_covariance(self):
        node=self.bridge()
        self.odom(node,v=.05)
        self.observe(node,collapsed=True)
        status=self.tick(node)
        self.assertFalse(status['ready'])
        self.assertFalse(status['moving_live_validated'])
        node.odom_pub.publish.assert_not_called()

    def test_concentrated_posterior_passes_parked_and_live_checks_with_original_evidence(self):
        node=self.bridge(monitor=True)
        original=None
        for step in range(16):
            self.now=self.wall=100.+step*.2
            self.odom(node)
            node.scan_match['inlier_ratio']=.86
            if step==0:
                self.observe(node,concentrated=True)
                original=copy.deepcopy(node.stationary_snapshot)
            status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertTrue(status['stationary_cache']['validated'])
        metrics=original['particle_metrics']
        self.assertEqual(metrics['unique_pose_bins'],5)
        self.assertGreater(metrics['largest_pose_bin_mass'],.94)
        self.assertLess(metrics['xy_rms_m'],.007)
        self.assertLess(metrics['yaw_std_deg'],.12)
        self.assertTrue(original['particle_pairing']['full_cloud_state_verified'])
        self.assertTrue(original['particle_pairing']['concentrated_cloud_posterior_verified'])
        self.assertEqual(node.stationary_snapshot,original)
        self.assertEqual(node.nomotion_request_count,0)
        # The same policy applies after motion, rather than cancelling on
        # the duplicate-pose ratio once a goal has begun.
        self.now=self.wall=103.2
        self.odom(node,x=.01,v=.05)
        self.observe(node,odom=(.01,0.,0.),concentrated=True)
        status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertTrue(status['moving_live_validated'])
        node.scan_match.update(ready=False,inlier_ratio=.60,reason='scan/map mismatch')
        self.assertFalse(self.tick(node)['ready'])

    def test_sensor_capture_gap_needs_new_original_observation(self):
        node=self.bridge()
        self.odom(node,v=.05);self.observe(node)
        self.assertTrue(self.tick(node)['ready'])
        self.now=self.wall=100.6
        self.odom(node,x=.03,v=.05)
        status=self.tick(node)
        self.assertFalse(status['ready'])
        self.assertIn('new AMCL',status['reason'])
        self.now=self.wall=100.7
        self.odom(node,x=.035,v=.05)
        self.observe(node,odom=(.035,0.,0.))
        self.assertTrue(self.tick(node)['ready'])

    def test_parked_capture_gap_recovers_once_but_needs_a_new_original_amcl_pair(self):
        node=self.bridge(monitor=True)
        self.settle(node)
        node.poll_services(self.wall)
        self.assertEqual(node.nomotion_request_count,1)
        original=copy.deepcopy(node.stationary_snapshot)
        self.now=self.wall=104.
        self.odom(node)
        self.tick(node)
        node.poll_services(self.wall)
        for step in range(1,24):
            self.now=self.wall=104.+step*.1
            self.odom(node)
            status=self.tick(node)
            node.poll_services(self.wall)
        self.assertEqual(node.nomotion_request_count,2)
        self.assertEqual(node.stationary_snapshot,original)
        self.assertFalse(status['ready'])
        self.assertLess(node.stationary_snapshot['amcl_stamp_s'],node.continuous_invalid_after_stamp)
        self.observe(node)
        self.assertTrue(self.tick(node)['ready'])
        self.assertGreater(node.stationary_snapshot['amcl_stamp_s'],original['amcl_stamp_s'])
        self.assertEqual(node.stationary_snapshot['covariance_6x6'],original['covariance_6x6'])

    def test_capture_recovery_does_not_run_for_frozen_trial_bad_clock_or_missing_tf(self):
        for problem in ['trial','clock','tf','map','moving']:
            with self.subTest(problem=problem):
                self.now=self.wall=100.
                node=self.bridge(monitor=True)
                self.odom(node);self.observe(node)
                node.seed_refresh_requested=True
                node.continuous_invalid_after_stamp=100.
                node.stationary_evidence.invalidate('odom gap/clock change invalidated stationary AMCL snapshot')
                if problem=='trial':node.navigation_monitor=False
                if problem=='clock':node.stream_clock_reversed=True
                if problem=='map':node.map_signature='different-map'
                if problem=='moving':node.raw_odom.twist.twist.linear.x=.02
                self.assertFalse(node.stream_recovery_ready(100.,100.,problem!='tf'))
                self.assertFalse(node.stream_recovery_ready(100.1,101.1,problem!='tf'))

    def test_navigation_monitor_omits_short_trial_total_bound_but_retains_jump_guard(self):
        monitor=self.bridge(monitor=True)
        trial=self.bridge(monitor=False)
        self.assertFalse(monitor.correction.bound_total)
        self.assertTrue(trial.correction.bound_total)
        for node in (monitor,trial):
            self.now=self.wall=100.
            self.odom(node);self.observe(node);self.tick(node)
            for index in range(1,5):
                self.now=self.wall=100.+index*.1
                self.odom(node,v=.05)
                self.observe(node,correction=(1.+index*.06,2.,.3))
                self.tick(node)
        self.assertIsNone(monitor.correction.fault)
        self.assertIn('exceeds',trial.correction.fault)
        monitor.odom_pub.publish.assert_not_called()
        self.now=self.wall=101.
        self.odom(monitor,v=.05)
        self.observe(monitor,correction=(2.,2.,.3))
        self.assertIn('jumped',monitor.correction.fault)

    def test_continuous_scan_score_uses_more_actual_endpoints_without_changing_thresholds(self):
        node=self.bridge()
        self.odom(node)
        node.scan_checker=Mock()
        node.scan_checker.score.return_value=dict(ready=True,inlier_ratio=.81,median_distance_m=.05)
        result=node.score_scan_fit(self.now,(1.,2.,.3))
        self.assertTrue(result['ready'])
        self.assertEqual(node.scan_checker.score.call_args.kwargs,dict(max_samples=720))

    def prepare_completed_scan(self,node):
        self.settle(node)
        fit=dict(ready=True,inlier_ratio=.9,median_distance_m=.02,scan_stamp_s=self.now)
        node.score_scan_fit=Mock(return_value=fit)
        node.check_scan_fit(self.now)
        return self.now

    def test_tf_late_new_scan_keeps_only_the_last_actual_fresh_score(self):
        node=self.bridge()
        completed=self.prepare_completed_scan(node)
        original_guard=list(node.stationary_evidence.scans)
        self.now=self.wall=completed+.1
        self.odom(node)
        node.score_scan_fit.return_value=dict(ready=False,pending_capture_tf=True,reason='TF is 11ms behind new scan')
        node.check_scan_fit(self.now)
        status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertEqual(status['scan_map_fit']['scan_stamp_s'],completed)
        self.assertTrue(status['scan_map_fit']['latest_scan_pending_tf'])
        self.assertEqual(list(node.stationary_evidence.scans),original_guard)
        for step in range(2,8):
            self.now=self.wall=completed+step*.1
            self.odom(node)
            node.check_scan_fit(self.now)
            status=self.tick(node)
        self.assertFalse(status['ready'])
        self.assertFalse(status['stationary_cache']['validated'])

    def test_bad_completed_scan_replaces_good_fit_and_is_not_hidden_by_next_pending_tf(self):
        node=self.bridge()
        completed=self.prepare_completed_scan(node)
        self.now=self.wall=completed+.1
        self.odom(node)
        node.score_scan_fit.return_value=dict(ready=False,inlier_ratio=.3,
            median_distance_m=.4,scan_stamp_s=self.now,reason='genuine scan mismatch')
        node.check_scan_fit(self.now)
        self.assertFalse(self.tick(node)['ready'])
        self.now=self.wall=completed+.2
        self.odom(node)
        node.score_scan_fit.return_value=dict(ready=False,pending_capture_tf=True,reason='TF pending')
        node.check_scan_fit(self.now)
        status=self.tick(node)
        self.assertFalse(status['ready'])
        self.assertEqual(status['scan_map_fit']['reason'],'genuine scan mismatch')
        self.assertAlmostEqual(status['scan_map_fit']['scan_stamp_s'],completed+.1)

    def test_pending_tf_after_changed_amcl_correction_rescores_old_actual_scan(self):
        node=self.bridge()
        completed=self.prepare_completed_scan(node)
        original_scan=node.last_scored_scan
        self.now=self.wall=completed+.1
        self.odom(node)
        node.latest_observation=(self.now,(1.1,2.,.3))
        node.score_scan_fit.side_effect=[dict(ready=False,pending_capture_tf=True,reason='TF pending'),
            dict(ready=False,inlier_ratio=.3,median_distance_m=.4,
                scan_stamp_s=completed,reason='old scan rejects new AMCL correction')]
        node.check_scan_fit(self.now)
        self.assertFalse(node.scan_match['ready'])
        self.assertEqual(node.score_scan_fit.call_args.kwargs,dict(scan=original_scan))
        self.assertEqual(node.score_scan_fit.call_args.args[1],(1.1,2.,.3))

    def test_invalid_scan_geometry_does_not_reuse_a_previous_good_score(self):
        node=self.bridge()
        completed=self.prepare_completed_scan(node)
        self.now=self.wall=completed+.1
        self.odom(node)
        # Exercise the real scorer's ValueError distinction, not only a
        # fabricated failed match dictionary.
        node.score_scan_fit=AmclTrialBridge.score_scan_fit.__get__(node)
        node.scan_checker=Mock()
        node.scan_checker.score.side_effect=ValueError('invalid new scan geometry')
        node.check_scan_fit(self.now)
        status=self.tick(node)
        self.assertFalse(status['ready'])
        self.assertEqual(status['scan_map_fit']['scan_stamp_s'],self.now)
        self.assertNotIn('latest_scan_pending_tf',status['scan_map_fit'])

    def test_current_or_explicit_seed_validation_does_not_randomize_global_pose(self):
        node=self.bridge(seed=False)
        self.assertTrue(node.stationary_cache_enabled)
        self.assertTrue(node.global_initialized)
        node.global_client=Mock()
        node.global_client.service_is_ready.return_value=True
        self.odom(node)
        node.poll_services(100.6)
        node.global_client.call_async.assert_not_called()
        self.assertEqual(node.nomotion_request_count,1)

    def test_capture_clock_rollback_rejects_live_motion_even_if_old_scan_fit_is_fresh(self):
        node=self.bridge()
        self.odom(node,v=.05);self.observe(node)
        self.assertTrue(self.tick(node)['ready'])
        self.now=self.wall=100.1
        self.odom(node,x=.005,v=.05)
        backwards=LaserScan()
        backwards.header.frame_id='lidar_link';backwards.header.stamp=stamp(100.05)
        node.on_scan(backwards)
        status=self.tick(node)
        self.assertFalse(status['ready'])
        self.assertFalse(status['moving_live_validated'])

    def test_scoring_pause_with_continuous_raw_scans_rechecks_without_resampling(self):
        node=self.bridge()
        completed=self.prepare_completed_scan(node)
        original=copy.deepcopy(node.stationary_snapshot)
        count=node.nomotion_request_count
        for step in range(1,9):
            self.now=self.wall=completed+step*.1
            self.odom(node)
            node.score_scan_fit.return_value=dict(ready=False,pending_capture_tf=True,
                                                  reason='TF processing is temporarily behind')
            node.check_scan_fit(self.now)
            status=self.tick(node)
        self.assertFalse(status['ready'])
        self.assertIsNone(node.stationary_evidence.fault)
        for step in range(7):
            self.now=self.wall=completed+.9+step*.2
            self.odom(node)
            node.score_scan_fit.return_value=dict(ready=True,inlier_ratio=.9,
                median_distance_m=.02,scan_stamp_s=self.now)
            node.check_scan_fit(self.now)
            status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertTrue(status['stationary_cache']['validated'])
        self.assertIsNone(node.stationary_evidence.fault)
        self.assertEqual(node.stationary_snapshot,original)
        self.assertEqual(node.nomotion_request_count,count)

    def test_two_mm_confirmed_nav2_adjustment_requests_one_arrival_refresh(self):
        node=self.bridge(monitor=True)
        self.odom(node)
        self.now=self.wall=100.6
        self.odom(node)
        node.poll_services(self.wall)
        self.assertEqual(node.nomotion_request_count,1)
        self.assertFalse(node.parking_refresh_armed)
        for stamp_s,x in ((100.8,0.),(100.85,.001),(100.9,.002)):
            self.now=self.wall=stamp_s
            self.odom(node,x=x,v=.02)
            node.poll_services(self.wall)
        self.assertTrue(node.parking_refresh_armed)
        self.assertEqual(node.nomotion_request_count,1)
        self.now=self.wall=101.
        self.odom(node,x=.002)
        node.poll_services(101.4)
        self.assertEqual(node.nomotion_request_count,1)
        node.poll_services(101.51)
        self.assertEqual(node.nomotion_request_count,2)
        for step in range(1,101):
            self.now=self.wall=101.+step*.1
            self.odom(node,x=.002)
            node.poll_services(self.wall)
        self.assertEqual(node.nomotion_request_count,2)

    def test_single_twist_spike_or_sustained_twist_without_pose_change_does_not_rearm(self):
        for sustained in (False,True):
            node=self.bridge(monitor=True)
            self.now=self.wall=100.
            self.odom(node)
            node.poll_services(100.6)
            self.assertEqual(node.nomotion_request_count,1)
            for step in range(1,11):
                self.now=self.wall=100.+step*.05
                self.odom(node,v=.02 if sustained or step==1 else 0.)
            self.now=self.wall=101.
            self.odom(node)
            for step in range(11):node.poll_services(101.6+step*.1)
            self.assertFalse(node.parking_refresh_armed)
            self.assertEqual(node.nomotion_request_count,1)


if __name__=='__main__':unittest.main()
