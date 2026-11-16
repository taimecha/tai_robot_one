"""A teaching persists complete evidence and never replaces another taught A."""
import json
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from r3_recognition_goal import (RecognitionGoal, create_goal_json, map_identity,
                                 validate_goal, validate_a_scan_match)


class ASaveEvidenceTest(unittest.TestCase):
    def test_loose_tracking_scan_fit_cannot_qualify_a_teaching(self):
        validate_a_scan_match(dict(inlier_ratio=.80,median_distance_m=.05))
        for match in (dict(inlier_ratio=.65,median_distance_m=.05),
                      dict(inlier_ratio=.90,median_distance_m=.051),
                      dict(inlier_ratio=.90,median_distance_m=None),{},
                      dict(inlier_ratio=math.nan,median_distance_m=.01)):
            with self.subTest(match=match),self.assertRaises(ValueError):
                validate_a_scan_match(match)

    def map(self, directory):
        directory=Path(directory)
        (directory/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
        path=directory/'map.yaml'
        path.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
                        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        return path

    def test_goal_is_complete_before_atomic_publication_and_cannot_be_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'nested'/'recognition_A.json'
            value={'pose_xyyaw':[1.,2.,.3],'evidence':{'covariance':[.001]*36}}
            real_link=os.link

            def publish(temporary, destination):
                self.assertFalse(path.exists())
                self.assertEqual(json.loads(Path(temporary).read_text()),value)
                return real_link(temporary,destination)

            with patch('r3_recognition_goal.os.link',side_effect=publish):
                create_goal_json(path,value)
            self.assertEqual(json.loads(path.read_text()),value)
            with self.assertRaises(FileExistsError):
                create_goal_json(path,{'pose_xyyaw':[8.,9.,1.]})
            self.assertEqual(json.loads(path.read_text()),value)
            self.assertEqual(list(path.parent.glob('*.tmp')),[])

    def test_competing_a_creation_wins_without_being_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'recognition_A.json'
            real_link=os.link

            def competing_save(temporary,destination):
                path.write_text('{"goal_id":"other-A"}')
                return real_link(temporary,destination)

            with patch('r3_recognition_goal.os.link',side_effect=competing_save), \
                    self.assertRaises(FileExistsError):
                create_goal_json(path,{'goal_id':'our-A'})
            self.assertEqual(json.loads(path.read_text()),{'goal_id':'other-A'})
            self.assertEqual(list(path.parent.glob('*.tmp')),[])

    def test_save_persists_full_pose_and_localization_evidence_for_exact_map(self):
        with tempfile.TemporaryDirectory() as directory:
            map_file=self.map(directory)
            path=Path(directory)/'recognition_A.json'
            pose=(1.125,-.42,math.radians(172.))
            evidence=dict(amcl_covariance_6x6=[.0001]*36,
                          particle_metrics=dict(xy_rms_m=.012,yaw_std_deg=.4),
                          scan_map_match=dict(ready=True,inlier_ratio=.95),
                          odom_pose_xyyaw=[.1,.2,.3],map_odom_xyyaw=[1.,-.5,2.7],
                          captured_ros_s=123.,stationary_duration_s=3.)
            probe=SimpleNamespace(wait_pose=Mock(return_value=pose),
                                  localization_evidence=Mock(return_value=evidence))
            result=RecognitionGoal.save(probe,map_file,path,12.)
            value=json.loads(path.read_text())
            self.assertEqual(result['status'],'NEW_A_SAVED')
            self.assertEqual(value['pose_xyyaw'],list(pose))
            self.assertEqual(value['pose']['position'],dict(x=pose[0],y=pose[1],z=0.))
            q=value['pose']['orientation']
            self.assertEqual((q['x'],q['y']),(0.,0.))
            self.assertAlmostEqual(q['z'],math.sin(pose[2]/2))
            self.assertAlmostEqual(q['w'],math.cos(pose[2]/2))
            self.assertEqual(value['localization_evidence'],evidence)
            self.assertEqual(value['map'],map_identity(map_file))
            self.assertEqual(validate_goal(value,map_identity(map_file)),pose)
            self.assertEqual(value['arrival_limits'],dict(position_m=.05,heading_deg=2.))
            self.assertFalse(value['physical_alignment_verified'])
            probe.wait_pose.assert_called_once_with(12.)
            probe.localization_evidence.assert_called_once_with()

    def test_failed_readiness_creates_no_a_or_temporary_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'recognition_A.json'
            probe=SimpleNamespace(wait_pose=Mock(side_effect=RuntimeError('AMCL ambiguous')),
                                  localization_evidence=Mock())
            with self.assertRaisesRegex(RuntimeError,'AMCL ambiguous'):
                RecognitionGoal.save(probe,Path('/unused/map.yaml'),path,12.)
            self.assertFalse(path.exists())
            self.assertEqual(list(Path(directory).iterdir()),[])
            probe.localization_evidence.assert_not_called()

    def test_live_evidence_records_pose_covariance_particles_scan_and_odom(self):
        amcl=PoseWithCovarianceStamped()
        amcl.header.stamp.sec=99
        amcl.pose.pose.position.x=1.2
        amcl.pose.pose.position.y=-.5
        amcl.pose.pose.orientation.w=1.
        amcl.pose.covariance=[float(i)*1e-5 for i in range(36)]
        odom=Odometry()
        odom.header.stamp.sec=100
        odom.pose.pose.position.x=.2
        odom.pose.pose.position.y=.3
        odom.pose.pose.orientation.w=1.
        probe=SimpleNamespace(amcl=amcl,odom=odom,now=lambda:100.,
            stamp=RecognitionGoal.stamp,last_map_odom=(1.,-.8,0.),scan_stamp=99.9,
            map_match=dict(ready=True,inlier_ratio=.9),
            particle_gate=SimpleNamespace(metrics=dict(xy_rms_m=.01,yaw_std_deg=.5)),
            a_quality=dict(amcl_xy_rms_m=.015,amcl_yaw_std_deg=.5),
            stationary_since=50.,observations=[(97.,(9.,9.,0.)),(99.,(1.2,-.5,0.))])
        with patch('r3_recognition_goal.time.monotonic',return_value=53.):
            evidence=RecognitionGoal.localization_evidence(probe)
        self.assertEqual(evidence['amcl_pose_xyyaw'],[1.2,-.5,0.])
        self.assertEqual(evidence['amcl_covariance_6x6'],list(amcl.pose.covariance))
        self.assertEqual(evidence['odom_pose_xyyaw'],[.2,.3,0.])
        self.assertEqual(evidence['map_odom_xyyaw'],[1.,-.8,0.])
        self.assertEqual(evidence['scan_map_match'],probe.map_match)
        self.assertEqual(evidence['particle_metrics'],probe.particle_gate.metrics)
        self.assertEqual(evidence['a_quality'],probe.a_quality)
        self.assertEqual(evidence['stationary_duration_s'],3.)
        self.assertEqual(evidence['recent_amcl_poses'],
                         [dict(stamp_ros_s=99.,pose_xyyaw=[1.2,-.5,0.])])
        self.assertFalse(evidence['uncertainty_is_physical_accuracy_proof'])


if __name__=='__main__':
    unittest.main()
