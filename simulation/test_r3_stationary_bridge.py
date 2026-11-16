"""Parked snapshot integration: no repeated resampling or synthetic freshness."""
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.context import Context
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from r3_amcl_bridge import AmclTrialBridge


def covariance():
    result=[0.]*36
    result[0]=.04**2
    result[7]=.05**2
    result[35]=math.radians(1.5)**2
    return result


def stamp(seconds):
    return Time(nanoseconds=round(seconds*1e9)).to_msg()


class StationaryBridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context=Context()
        rclpy.init(context=cls.context,domain_id=231)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.nodes=[]

    def tearDown(self):
        for node in self.nodes:
            node.destroy_node()
        self.temp.cleanup()

    def bridge(self, inspection=True):
        args=SimpleNamespace(map=Path(self.temp.name)/'selected.yaml',initial_pose=None,
                             wait_ready=None,global_seed_ready=True,
                             global_localization=False,stationary_inspection=inspection)
        bridge=AmclTrialBridge(args,context=self.context)
        self.nodes.append(bridge)
        bridge.active=bridge.map_received=True
        bridge.next_state=float('inf')
        bridge.state_client=Mock()
        bridge.state_client.service_is_ready.return_value=False
        bridge.global_client=Mock()
        bridge.global_client.service_is_ready.return_value=False
        bridge.nomotion_client=Mock()
        bridge.nomotion_client.service_is_ready.return_value=True
        future=Mock()
        future.done.return_value=True
        bridge.nomotion_client.call_async.return_value=future
        bridge.amcl_subscription=Mock()
        bridge.cloud_subscription=Mock()
        bridge.amcl_subscription.get_publisher_count.return_value=1
        bridge.cloud_subscription.get_publisher_count.return_value=1
        bridge.nomotion_interfaces_since=0.
        bridge.tf_buffer=Mock()
        bridge.tf_buffer.can_transform.return_value=True
        bridge.scan_stamp=bridge.get_clock().now().nanoseconds*1e-9
        bridge.latest_scan=LaserScan()
        bridge.latest_scan.header.frame_id='lidar_link'
        bridge.latest_scan.header.stamp=stamp(bridge.scan_stamp)
        bridge.status_pub=Mock()
        bridge.marker_pub=Mock()
        bridge.odom_pub=Mock()
        return bridge

    @staticmethod
    def odom(seconds):
        result=Odometry()
        result.header.frame_id='odom'
        result.child_frame_id='base_footprint'
        result.header.stamp=stamp(seconds)
        result.pose.pose.orientation.w=1.
        return result

    def test_seeded_inspection_requests_exactly_one_refresh_over_five_seconds(self):
        bridge=self.bridge()
        now=bridge.get_clock().now().nanoseconds*1e-9
        bridge.on_odom(self.odom(now))
        for step in range(51):
            bridge.poll_services(100.+step*.1)
        self.assertTrue(bridge.stationary_cache_enabled)
        self.assertEqual(bridge.nomotion_client.call_async.call_count,1)
        self.assertEqual(bridge.nomotion_request_count,1)
        bridge.global_client.call_async.assert_not_called()

    def test_normal_trial_keeps_its_periodic_nomotion_updates(self):
        bridge=self.bridge(inspection=False)
        now=bridge.get_clock().now().nanoseconds*1e-9
        bridge.on_odom(self.odom(now))
        for step in range(51):
            bridge.poll_services(100.+step*.1)
        self.assertFalse(bridge.stationary_cache_enabled)
        self.assertEqual(bridge.nomotion_client.call_async.call_count,11)
        self.assertEqual(bridge.nomotion_request_count,11)

    def test_single_refresh_waits_for_both_data_interfaces_to_match(self):
        bridge=self.bridge()
        now=bridge.get_clock().now().nanoseconds*1e-9
        bridge.on_odom(self.odom(now))
        bridge.nomotion_interfaces_since=None
        bridge.cloud_subscription.get_publisher_count.return_value=0
        bridge.poll_services(100.)
        bridge.nomotion_client.call_async.assert_not_called()
        bridge.cloud_subscription.get_publisher_count.return_value=1
        bridge.poll_services(101.)
        bridge.poll_services(101.9)
        bridge.nomotion_client.call_async.assert_not_called()
        bridge.poll_services(102.01)
        bridge.poll_services(104.)
        bridge.nomotion_client.call_async.assert_called_once()

    def test_single_refresh_waits_for_capture_time_tf_history(self):
        bridge=self.bridge()
        now=bridge.get_clock().now().nanoseconds*1e-9
        bridge.on_odom(self.odom(now))
        bridge.tf_buffer.can_transform.return_value=False
        bridge.poll_services(100.)
        bridge.nomotion_client.call_async.assert_not_called()
        bridge.tf_buffer.can_transform.return_value=True
        bridge.poll_services(100.1)
        bridge.nomotion_client.call_async.assert_called_once()
        capture_times=[call.args[2].nanoseconds*1e-9
                       for call in bridge.tf_buffer.can_transform.call_args_list]
        self.assertTrue(any(value<=now-1.9 for value in capture_times))
        self.assertTrue(any(abs(value-bridge.scan_stamp)<.01 for value in capture_times))

    def seeded_snapshot(self, bridge, now):
        bridge.on_odom(self.odom(now))
        observation=PoseWithCovarianceStamped()
        observation.header.frame_id='map'
        observation.header.stamp=stamp(now)
        observation.pose.pose.position.x=1.
        observation.pose.pose.position.y=2.
        observation.pose.pose.orientation.z=math.sin(.3/2.)
        observation.pose.pose.orientation.w=math.cos(.3/2.)
        observation.pose.covariance=covariance()
        particles=[(1.+dx,2.+dy,.3+dy,1.)
                   for dx in (-.02,-.01,0.,.01,.02)
                   for dy in (-.02,-.01,0.,.01,.02)]
        bridge.latest_cloud=(now,particles)
        transform=SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        bridge.tf_buffer=Mock()
        bridge.tf_buffer.lookup_transform.return_value=transform
        bridge.on_amcl(observation)
        bridge.process_pending(now)
        self.assertIsNotNone(bridge.stationary_snapshot)
        return observation

    def test_old_snapshot_ready_only_with_live_odom_and_scans_preserving_covariance(self):
        bridge=self.bridge()
        now=bridge.get_clock().now().nanoseconds*1e-9
        observation=self.seeded_snapshot(bridge,now)
        original=copy.deepcopy(bridge.stationary_snapshot)
        self.assertIsNone(bridge.correction.baseline)
        with patch.object(bridge,'get_clock') as clock, \
                patch.object(bridge,'poll_services'), \
                patch.object(bridge,'check_scan_fit'):
            for step in range(14):
                current=now+step*.25
                clock.return_value.now.return_value=Time(nanoseconds=round(current*1e9))
                bridge.on_odom(self.odom(current))
                scan=LaserScan()
                scan.header.frame_id='lidar_link'
                scan.header.stamp=stamp(current)
                bridge.on_scan(scan)
                bridge.scan_match=dict(ready=True,inlier_ratio=.9,median_distance_m=.02)
                bridge.tick()
            status=json.loads(bridge.status_pub.publish.call_args.args[0].data)
            self.assertTrue(status['ready'],status['reason'])
            self.assertTrue(status['stationary_cache']['validated'])
            self.assertGreater(status['stationary_cache']['amcl_observation_age_s'],2.)
            self.assertEqual(bridge.stationary_snapshot,original)
            self.assertEqual(original['covariance_6x6'],list(observation.pose.covariance))
            self.assertEqual(original['amcl_stamp_s'],bridge.stamp(observation))
            self.assertEqual(bridge.correction.amcl_stamp,bridge.stamp(observation))
            bridge.odom_pub.publish.assert_not_called()

            # Advancing time without fresh streams cannot preserve readiness.
            current+=.75
            clock.return_value.now.return_value=Time(nanoseconds=round(current*1e9))
            bridge.tick()
            stale=json.loads(bridge.status_pub.publish.call_args.args[0].data)
            self.assertFalse(stale['ready'])
            self.assertFalse(stale['stationary_cache']['validated'])
            self.assertEqual(bridge.stationary_snapshot,original)
            bridge.odom_pub.publish.assert_not_called()

    def test_identical_map_delivery_keeps_snapshot_but_changed_content_invalidates(self):
        bridge=self.bridge()
        grid=OccupancyGrid()
        grid.header.frame_id='map'
        grid.info.width=grid.info.height=4
        grid.info.resolution=.05
        grid.info.origin.orientation.w=1.
        grid.data=[100,0,0,0]*4
        bridge.on_map(grid)
        now=bridge.get_clock().now().nanoseconds*1e-9
        self.seeded_snapshot(bridge,now)
        original=copy.deepcopy(bridge.stationary_snapshot)
        repeated=copy.deepcopy(grid)
        repeated.header.stamp=stamp(now+1.)
        bridge.on_map(repeated)
        self.assertIsNone(bridge.stationary_evidence.fault)
        self.assertEqual(bridge.stationary_snapshot,original)
        changed=copy.deepcopy(repeated)
        changed.data[1]=100
        bridge.on_map(changed)
        self.assertIn('map content changed',bridge.stationary_evidence.fault)
        self.assertFalse(bridge.stationary_evidence.ready(now)[0])
        self.assertEqual(bridge.stationary_snapshot,original)


if __name__=='__main__':
    unittest.main()
