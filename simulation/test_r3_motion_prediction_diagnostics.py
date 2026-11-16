"""Hard bounds retain measured cancellation evidence instead of hiding it."""
import copy
import json
import math
from types import SimpleNamespace
import unittest

from nav_msgs.msg import Odometry

from r3_recognition_goal import RecognitionGoal,MotionPredictionLimitError


class MotionPredictionDiagnosticsTest(unittest.TestCase):
    def probe(self,distance=0.,yaw=0.):
        proof=dict(validated=True,started_ros_s=100.,source_amcl_stamp_s=10.,
            elapsed_s=.05,displacement_m=.001,yaw_change_rad=.01,
            max_duration_s=2.,max_distance_m=.08,max_yaw_rad=.08,
            anchor_odom_xyyaw=[0.,0.,0.])
        odom=Odometry()
        odom.pose.pose.position.x=distance
        odom.pose.pose.orientation.z=math.sin(yaw/2.)
        odom.pose.pose.orientation.w=math.cos(yaw/2.)
        return SimpleNamespace(stationary_status=dict(run_id='this-run',published_ros_s=100.05,
                motion_prediction=proof,amcl_pipeline=dict(pending_pose_count=1,
                    pending_reason='waiting for matching particle cloud')),
            stationary_evidence=SimpleNamespace(fault=None),
            stationary_snapshot=dict(amcl_stamp_s=10.),
            last_parked_validation=dict(run_id='this-run',source_amcl_stamp_s=10.,
                validated_ros_s=99.95,odom_xyyaw=[0.,0.,0.]),odom=odom,
            stamp=lambda message:100.08,motion_prediction_identity=None)

    def test_own_newer_odom_rotation_still_cancels_and_reports_precise_evidence(self):
        probe=self.probe(.002,.081)
        old_proof=copy.deepcopy(probe.stationary_status['motion_prediction'])
        with self.assertRaises(MotionPredictionLimitError) as failure:
            RecognitionGoal._validate_motion_prediction(probe,100.1)
        evidence=failure.exception.evidence
        self.assertAlmostEqual(evidence['own_yaw_change_rad'],.081)
        self.assertAlmostEqual(evidence['own_yaw_change_deg'],math.degrees(.081))
        self.assertAlmostEqual(evidence['source_amcl_age_s'],90.1)
        self.assertAlmostEqual(evidence['status_age_s'],.05)
        self.assertEqual(evidence['odom_stamp_ros_s'],100.08)
        self.assertEqual(evidence['amcl_pipeline']['pending_pose_count'],1)
        self.assertEqual(evidence['max_yaw_rad'],.08)
        self.assertIn('limits 0.080 m / 4.58 deg',str(failure.exception))
        self.assertEqual(probe.stationary_status['motion_prediction'],old_proof)
        self.assertEqual(json.loads(json.dumps(evidence))['bridge_transition_proof'],old_proof)

    def test_distance_limit_and_peer_reported_limit_are_still_independent(self):
        for own_distance,reported_distance in ((.081,.001),(.001,.081)):
            with self.subTest(own=own_distance,reported=reported_distance):
                probe=self.probe(own_distance)
                probe.stationary_status['motion_prediction']['displacement_m']=reported_distance
                with self.assertRaises(MotionPredictionLimitError) as failure:
                    RecognitionGoal._validate_motion_prediction(probe,100.1)
                self.assertEqual(failure.exception.evidence['own_displacement_m'],own_distance)
                self.assertEqual(failure.exception.evidence['max_distance_m'],.08)

    def test_valid_transition_keeps_exact_hard_bounds(self):
        probe=self.probe(.079,.079)
        proof=copy.deepcopy(probe.stationary_status['motion_prediction'])
        self.assertTrue(RecognitionGoal._validate_motion_prediction(probe,100.1))
        self.assertEqual(probe.stationary_status['motion_prediction'],proof)


if __name__=='__main__':
    unittest.main()
