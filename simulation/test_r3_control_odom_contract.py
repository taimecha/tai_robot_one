"""Motion and obstacle projection share the continuing EKF control frame."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import rclpy
from rclpy.time import Time
from std_msgs.msg import String

from r3_real_trial import RealTrial
from r3_real_obstacle_filter import PalletObstacleFilter


class ControlOdomContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=219)

    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def test_motion_and_obstacle_filter_read_the_same_held_control_reference(self):
        spec=dict(mode='amcl',run_id='one-run',map_file='/test/edited.yaml',
            baseline_map_odom=[3.,1.4,-1.55],correction_at_capture_xyyaw=[.01,-.02,.003],
            control_status_topic='/pallet/r3/control_odom/status')
        target=dict(frame_id='r3_trial',pose=[1.,0.,0.],localization=spec)
        plan=SimpleNamespace(geometry_json='{}',target_json=json.dumps(target),
            phases_json=json.dumps([dict(type='APPROACH',poses=[[0.,0.,0.],[.5,0.,0.]]),
                                    dict(type='STRAIGHT',poses=[[.5,0.,0.],[.7,0.,0.]])]))
        nodes=[]
        try:
            with tempfile.TemporaryDirectory() as tmp:
                follower=RealTrial(plan,Path(tmp),stop_at_c=True)
                nodes.append(follower)
                obstacle_filter=PalletObstacleFilter(target,{})
                nodes.append(obstacle_filter)
                now=Time(nanoseconds=100_600_000_000)
                state=dict(ready=True,frame_id='r3_trial',run_id='one-run',map_file=spec['map_file'],
                    baseline_map_odom=spec['baseline_map_odom'],correction_xyyaw=[.01,-.02,.003],
                    published_ros_s=100.6,control_status_topic=spec['control_status_topic'],
                    control_mode='ekf_tracking',amcl_stamp_s=40.,amcl_update_validated=False,
                    correction_settled=True,reason='AMCL delayed; fresh EKF continues')
                for node in nodes:
                    node.get_clock=lambda:SimpleNamespace(now=lambda:now)
                    names={sub.topic_name for sub in node.subscriptions}
                    self.assertIn('/pallet/r3/control_odom/status',names)
                    self.assertNotIn('/pallet/r3/localization/status',names)
                follower.started=True
                message=String(data=json.dumps(state))
                follower.on_localization(message)
                obstacle_filter.on_localization(message)
                self.assertFalse(follower.finished)
                self.assertEqual(follower.localization_status['amcl_stamp_s'],40.)
                self.assertEqual(obstacle_filter.localization_status['correction_xyyaw'],[.01,-.02,.003])
                self.assertEqual(obstacle_filter.correction_history.at(100.6),(.01,-.02,.003))
        finally:
            for node in reversed(nodes):node.destroy_node()


if __name__=='__main__':unittest.main()
