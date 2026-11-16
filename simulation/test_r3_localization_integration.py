"""AMCL control/obstacle frame integration without hardware or motion output."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from r3_core import FrozenPlan, compose, goal_base
from r3_live_prepare import provisional_geometry
from r3_real_trial import RealTrial
from r3_real_obstacle_filter import PalletObstacleFilter


class LocalizationIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.owns_ros = not rclpy.ok()
        if self.owns_ros:
            rclpy.init()
        self.temp = tempfile.TemporaryDirectory()
        self.spec = dict(mode='amcl',run_id='test-run',map_file='/test/map.yaml',
                         baseline_map_odom=[1.,2.,.1],correction_at_capture_xyyaw=[0.,0.,0.])
        self.target = dict(frame_id='r3_trial',pose=[2.,0.,0.],localization=self.spec)
        self.g = provisional_geometry()
        c=goal_base(self.target['pose'],self.g,.535)
        b=goal_base(self.target['pose'],self.g)
        self.plan=FrozenPlan(json.dumps(self.target),json.dumps(self.g),json.dumps([
            dict(type='APPROACH',poses=[[0.,0.,0.],c]),
            dict(type='STRAIGHT',poses=[c,b])]),json.dumps(dict(localization=self.spec)))

    def tearDown(self):
        self.temp.cleanup()
        if self.owns_ros:
            rclpy.shutdown()

    def status(self,node,**updates):
        data=dict(frame_id='r3_trial',run_id='test-run',map_file='/test/map.yaml',
                  baseline_map_odom=[1.,2.,.1],correction_xyyaw=[.02,-.01,.03],
                  correction_settled=True,ready=True,
                  published_ros_s=node.get_clock().now().nanoseconds*1e-9)
        data.update(updates)
        return data

    def ready_trial(self):
        node=RealTrial(self.plan,Path(self.temp.name),stop_at_c=True)
        node.armed=node.started=True
        node.follow.start(True)
        node.pose=(.3,0.,0.)
        node.localized_pose=(.32,-.01,.03)
        node.localized_wall=node.odom_wall=node.imu_wall=node.yolo_wall=100.
        node.scan_wall['lidar']=100.
        node.imu_yaw=0.
        node.imu_anchor=(0.,0.)
        node.twist=(.05,0.)
        node.yolo_enabled=False
        node.last_tick=node.last_progress_wall=99.95
        node.localization_status=self.status(node)
        return node

    def test_follower_uses_localized_pose_and_keeps_raw_imu_check_independent(self):
        node=self.ready_trial()
        try:
            # A map yaw correction is not an IMU/EKF disagreement.
            node.localized_pose=(.32,-.01,math.radians(3.))
            with patch('r3_real_trial.time.monotonic',return_value=100.), \
                    patch.object(node,'send'),patch.object(node.follow,'step',return_value=(.05,.01)) as follow:
                node.tick()
            self.assertFalse(node.finished)
            self.assertEqual(follow.call_args.args[0],node.localized_pose)
            row=node.rows[-1]
            self.assertEqual(row['heading_source'],'amcl_odom_ekf')
            self.assertAlmostEqual(row['x_m'],.32)
            self.assertAlmostEqual(row['raw_odom_x_m'],.3)
            self.assertAlmostEqual(row['imu_odom_disagreement_deg'],0.)
        finally:
            node.destroy_node()

    def test_stale_or_restarted_localization_never_falls_back_to_raw_odom(self):
        for updates in ({'published_ros_s':0.},{'run_id':'another-run'},{'ready':False}):
            node=self.ready_trial()
            try:
                node.localization_status.update(updates)
                with patch('r3_real_trial.time.monotonic',return_value=100.), \
                        patch.object(node,'send') as send,patch.object(node.follow,'step') as follow:
                    node.tick()
                self.assertTrue(node.finished)
                self.assertEqual(node.state,'STOP_LATCHED')
                follow.assert_not_called()
                send.assert_called_with()
            finally:
                node.destroy_node()

    def test_obstacle_mask_uses_same_trial_transform_as_localized_robot(self):
        node=PalletObstacleFilter(self.target,self.g)
        try:
            status=self.status(node)
            node.on_localization(String(data=json.dumps(status)))
            q=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)
            tf=SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=.3,y=.1),rotation=q))
            scan=LaserScan()
            scan.header.frame_id='lidar_link'
            scan.header.stamp=node.get_clock().now().to_msg()
            scan.ranges=[2.,2.]
            scan.angle_increment=.1
            with patch.object(node.tf,'lookup_transform',return_value=tf), \
                    patch('r3_real_obstacle_filter.classify_ranges',return_value=([2.,2.],0,2)) as classify:
                node.on_scan('lidar',scan)
            expected=compose(status['correction_xyyaw'],(.3,.1,0.))
            self.assertEqual(classify.call_args.args[3],expected)
            node.localization_status['ready']=False
            with patch('r3_real_obstacle_filter.classify_ranges') as classify:
                node.on_scan('lidar',scan)
                classify.assert_not_called()
        finally:
            node.destroy_node()


if __name__=='__main__':
    unittest.main()
