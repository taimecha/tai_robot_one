"""A validated navigation posterior becomes a trial frame without resampling."""
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
from std_srvs.srv import Trigger

from r3_amcl_bridge import AmclTrialBridge


def stamp(seconds):
    return Time(nanoseconds=round(seconds*1e9)).to_msg()


class TrialHandoffTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context=Context()
        rclpy.init(context=cls.context,domain_id=223)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.nodes=[]
        self.now=100.

    def tearDown(self):
        for node in self.nodes:node.destroy_node()
        self.temp.cleanup()

    def tick(self,node):
        with patch.object(node,'poll_services'),patch.object(node,'check_scan_fit'):
            node.tick()
        return json.loads(node.status_pub.publish.call_args.args[0].data)

    def ready_monitor(self):
        args=SimpleNamespace(map=Path(self.temp.name)/'map.yaml',initial_pose=None,
            wait_ready=None,global_seed_ready=True,global_localization=False,
            stationary_inspection=False,continuous_localization=True,navigation_monitor=True)
        node=AmclTrialBridge(args,context=self.context)
        self.nodes.append(node)
        node.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=round(self.now*1e9)))
        node.global_requested_ros_s=99.
        node.active=node.map_received=True
        node.map_signature='same-occupancy-map'
        node.scan_checker=Mock()
        node.status_pub=Mock();node.marker_pub=Mock();node.odom_pub=Mock()
        node.get_publishers_info_by_topic=Mock(return_value=[])
        node.tf_buffer=Mock()
        node.tf_buffer.lookup_transform.return_value=SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        node.nomotion_client=Mock()
        node.nomotion_request_count=1
        node.seed_refresh_requested=True
        node.parking_refresh_armed=False
        node.refresh_odom_anchor=(0.,0.,0.)
        observation=PoseWithCovarianceStamped()
        observation.header.frame_id='map';observation.header.stamp=stamp(self.now)
        observation.pose.pose.position.x=3.1;observation.pose.pose.position.y=1.4
        observation.pose.pose.orientation.z=math.sin(-1.55/2.)
        observation.pose.pose.orientation.w=math.cos(-1.55/2.)
        observation.pose.covariance[0]=observation.pose.covariance[7]=.015**2
        observation.pose.covariance[35]=math.radians(.5)**2
        node.latest_cloud=(self.now,[(3.1+dx,1.4+dy,-1.55+dy,1.)
            for dx in (-.02,-.01,0.,.01,.02) for dy in (-.02,-.01,0.,.01,.02)])
        for step in range(16):
            self.now=100.+step*.2
            odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_footprint'
            odom.header.stamp=stamp(self.now);odom.pose.pose.orientation.w=1.
            node.on_odom(odom)
            scan=LaserScan();scan.header.frame_id='lidar_link';scan.header.stamp=stamp(self.now)
            node.on_scan(scan)
            node.scan_match=dict(ready=True,inlier_ratio=.85,median_distance_m=.015,scan_stamp_s=self.now)
            if step==0:
                node.on_amcl(observation)
                node.process_pending(self.now)
            status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertTrue(status['stationary_cache']['validated'])
        self.assertGreater(self.now-node.stationary_snapshot['amcl_stamp_s'],2.)
        node.odom_pub.publish.assert_not_called()
        return node

    @staticmethod
    def enable(node):
        return node.enable_trial(Trigger.Request(),Trigger.Response())

    def test_old_validated_posterior_is_reanchored_without_new_amcl_or_identity(self):
        node=self.ready_monitor()
        original=copy.deepcopy(node.stationary_snapshot)
        original_run=node.run_id
        current=tuple(node.latest_observation[1])
        # Nav2's monitor frame may have begun elsewhere. The trial begins at
        # the actually arrived, already checked current map correction.
        node.correction.baseline=(3.,1.3,-1.55)
        node.correction.target=node.correction.applied=(.1,.1,0.)
        response=self.enable(node)
        self.assertTrue(response.success,response.message)
        evidence=json.loads(response.message)
        self.assertEqual(evidence['status'],'TRIAL_LOCALIZATION_ENABLED')
        self.assertEqual(node.run_id,original_run)
        self.assertFalse(node.navigation_monitor)
        self.assertTrue(node.correction.bound_total)
        self.assertEqual(node.correction.baseline,current)
        self.assertEqual(node.correction.last_transform,current)
        self.assertEqual(node.correction.target,(0.,0.,0.))
        self.assertEqual(node.correction.applied,(0.,0.,0.))
        self.assertEqual(node.correction.amcl_stamp,original['amcl_stamp_s'])
        self.assertEqual(node.stationary_snapshot,original)
        self.assertEqual(node.nomotion_request_count,1)
        self.assertFalse(node.parking_refresh_armed)
        node.nomotion_client.call_async.assert_not_called()
        self.assertEqual(node.trial_handoff['source_amcl_stamp_s'],100.)
        self.assertEqual(node.trial_handoff['previous_role'],'navigation_monitor')
        # The normal timer only starts publishing control odometry after the
        # handoff, retaining the same original covariance and observation age.
        node.odom_pub.reset_mock()
        status=self.tick(node)
        self.assertTrue(status['ready'],status['reason'])
        self.assertEqual(status['role'],'trial')
        self.assertEqual(status['trial_handoff'],node.trial_handoff)
        self.assertEqual(status['amcl_stamp_s'],100.)
        node.odom_pub.publish.assert_called_once()
        control=node.odom_pub.publish.call_args.args[0]
        self.assertEqual(control.header.frame_id,'r3_trial')
        self.assertEqual(control.pose.pose.position.x,0.)
        self.assertEqual(control.pose.pose.position.y,0.)

    def test_each_command_topic_must_have_released_all_publishers(self):
        for topic in ('/cmd_vel_nav','/cmd_vel_localized','/cmd_vel_smoothed','/cmd_vel_safe'):
            with self.subTest(topic=topic):
                self.now=100.
                node=self.ready_monitor()
                original=node.correction
                node.get_publishers_info_by_topic.side_effect=lambda name:[SimpleNamespace(node_name='nav2_owner')] if name==topic else []
                response=self.enable(node)
                self.assertFalse(response.success)
                self.assertIn(topic,response.message)
                self.assertTrue(node.navigation_monitor)
                self.assertIs(node.correction,original)
                self.assertIsNone(node.trial_handoff)
                node.odom_pub.publish.assert_not_called()

    def test_runtime_scan_fit_below_eighty_percent_can_handoff_without_resampling(self):
        node=self.ready_monitor()
        original=copy.deepcopy(node.stationary_snapshot)
        node.scan_match['inlier_ratio']=.79
        response=self.enable(node)
        self.assertTrue(response.success,response.message)
        self.assertEqual(node.stationary_snapshot,original)
        node.nomotion_client.call_async.assert_not_called()

    def test_handoff_rejects_stale_motion_map_and_bad_posterior(self):
        cases={
            'moving':lambda n:setattr(n.raw_odom.twist.twist.linear,'x',.01),
            'stale_odom':lambda n:setattr(n,'odom_stamp',self.now-.3),
            'stale_raw_scan':lambda n:setattr(n,'scan_stamp',self.now-.6),
            'stale_scored_scan':lambda n:n.scan_match.update(scan_stamp_s=self.now-.6),
            'bad_scan_ratio':lambda n:n.scan_match.update(inlier_ratio=.64),
            'bad_scan_median':lambda n:n.scan_match.update(median_distance_m=.051),
            'map_missing':lambda n:setattr(n,'map_received',False),
            'map_changed':lambda n:setattr(n,'map_signature','changed-map'),
            'capture_gap':lambda n:setattr(n,'continuous_invalid_after_stamp',101.),
            'uncertain_covariance':lambda n:n.stationary_snapshot['covariance_6x6'].__setitem__(0,.06**2),
            'ambiguous_particles':lambda n:n.stationary_snapshot['particle_metrics'].update(converged=False),
            'wide_particles':lambda n:n.stationary_snapshot['particle_metrics'].update(xy_rms_m=.05),
            'pending_refresh':lambda n:setattr(n,'nomotion_future',Mock(done=lambda:False)),
            'correction_fault':lambda n:setattr(n.correction,'fault','AMCL jumped'),
        }
        for label,change in cases.items():
            with self.subTest(label=label):
                self.now=100.
                node=self.ready_monitor()
                correction=node.correction
                snapshot=copy.deepcopy(node.stationary_snapshot)
                change(node)
                response=self.enable(node)
                self.assertFalse(response.success,(label,response.message))
                self.assertTrue(node.navigation_monitor)
                self.assertIs(node.correction,correction)
                self.assertIsNone(node.trial_handoff)
                self.assertEqual(node.nomotion_request_count,1)
                node.nomotion_client.call_async.assert_not_called()

    def test_handoff_accepts_a_concentrated_original_posterior_without_resampling(self):
        node=self.ready_monitor()
        node.stationary_snapshot['particle_metrics'].update(unique_pose_bins=5,largest_pose_bin_mass=.95)
        original=copy.deepcopy(node.stationary_snapshot)
        response=self.enable(node)
        self.assertTrue(response.success,response.message)
        self.assertFalse(node.navigation_monitor)
        self.assertEqual(node.stationary_snapshot,original)
        node.nomotion_client.call_async.assert_not_called()

    def test_pending_newer_amcl_observation_must_finish_before_handoff(self):
        node=self.ready_monitor()
        newer=PoseWithCovarianceStamped();newer.header.frame_id='map'
        newer.header.stamp=stamp(self.now);newer.pose.pose.orientation.w=1.
        node.on_amcl(newer)
        response=self.enable(node)
        self.assertFalse(response.success)
        self.assertIn('newer pending AMCL',response.message)
        self.assertTrue(node.navigation_monitor)



    def test_repeated_request_cannot_reanchor_a_captured_or_moving_trial(self):
        node=self.ready_monitor()
        first=self.enable(node)
        self.assertTrue(first.success,first.message)
        baseline=node.correction.baseline
        handoff=copy.deepcopy(node.trial_handoff)
        correction=node.correction
        node.raw_odom.twist.twist.linear.x=.05
        node.raw_odom.pose.pose.position.x=.1
        node.correction.target=(.01,.02,.003)
        node.correction.applied=(.005,.01,.001)
        node.get_publishers_info_by_topic.return_value=[SimpleNamespace(node_name='r3_collision_monitor')]
        again=self.enable(node)
        self.assertTrue(again.success,again.message)
        self.assertEqual(json.loads(first.message),json.loads(again.message))
        self.assertIs(node.correction,correction)
        self.assertEqual(node.correction.baseline,baseline)
        self.assertEqual(node.correction.target,(.01,.02,.003))
        self.assertEqual(node.correction.applied,(.005,.01,.001))
        self.assertEqual(node.trial_handoff,handoff)
        self.assertEqual(node.nomotion_request_count,1)


if __name__=='__main__':
    unittest.main()
