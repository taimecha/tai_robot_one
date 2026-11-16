"""Skip a redundant goal only after fresh localization confirms saved A."""
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from builtin_interfaces.msg import Time

from r3_recognition_goal import RecognitionGoal,SCHEMA,map_identity


class AlreadyAtATest(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.folder=Path(self.directory.name)
        (self.folder/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
        self.map_file=self.folder/'map.yaml'
        self.map_file.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
        self.goal=[3.051,1.457,math.radians(-90.15)]
        self.goal_file=self.folder/'recognition_A.json'
        self.goal_file.write_text(json.dumps(dict(schema=SCHEMA,purpose='pallet_recognition_A',
            frame_id='map',goal_id='saved-A',saved_at_utc='2026-11-14T00:00:00Z',
            pose_xyyaw=self.goal,map=map_identity(self.map_file))))
        self.goal_bytes=self.goal_file.read_bytes()
        self.evidence=dict(amcl_stamp_ros_s=10.,amcl_covariance_6x6=[.001]*36,
            particle_metrics=dict(unique_pose_bins=120,largest_pose_bin_mass=.35),
            stationary_snapshot_validation=dict(original_covariance_preserved=True,
                repeated_nomotion_updates=False,live_scan_odom_validated=True))

    def tearDown(self):
        self.assertEqual(self.goal_file.read_bytes(),self.goal_bytes)
        self.directory.cleanup()

    def probe(self,pose):
        client=Mock();client.wait_for_server.return_value=True
        return SimpleNamespace(wait_pose=Mock(return_value=pose),client=client,
            map_match=dict(ready=True,inlier_ratio=.8319444444,median_distance_m=.00845),
            particle_gate=SimpleNamespace(metrics=copy.deepcopy(self.evidence['particle_metrics'])),
            localization_evidence=Mock(return_value=copy.deepcopy(self.evidence)),
            navigation_goal_attempted=False,cancel=Mock(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=123))),
            await_future=Mock(return_value=SimpleNamespace(accepted=False)))

    def test_actual_four_point_four_eight_cm_case_proceeds_without_goal(self):
        pose=(3.013352422098309,1.4326788298911344,-1.554808388770839)
        probe=self.probe(pose)
        result=RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.)
        self.assertEqual(result['status'],'AT_NEW_A')
        self.assertTrue(result['already_at_A'])
        self.assertFalse(result['navigation_goal_attempted'])
        self.assertFalse(probe.navigation_goal_attempted)
        self.assertEqual(result['direct_velocity_commands_from_this_process'],0)
        self.assertEqual(result['arrival_pose_xyyaw'],list(pose))
        self.assertEqual(result['goal_pose_xyyaw'],self.goal)
        self.assertAlmostEqual(result['position_error_m'],.04482030162022882)
        self.assertAlmostEqual(result['heading_error_deg'],1.0660413718952277)
        self.assertEqual(result['localization_evidence'],self.evidence)
        self.assertEqual([call.args[0] for call in probe.wait_pose.call_args_list],[30.,10.])
        probe.client.wait_for_server.assert_called_once_with(timeout_sec=30.)
        probe.client.send_goal_async.assert_not_called()
        probe.cancel.assert_not_called()

    def test_under_five_cm_and_two_degrees_keeps_measured_pose(self):
        for distance,angle in ((.031,.74),(.049,2.),(.0518,.056),(.099,5.),(0.,0.)):
            with self.subTest(distance=distance,angle=angle):
                pose=(self.goal[0]+distance,self.goal[1],self.goal[2]+math.radians(angle))
                probe=self.probe(pose)
                result=RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.)
                self.assertTrue(result['already_at_A'])
                self.assertEqual(result['arrival_pose_xyyaw'],list(pose))
                probe.client.send_goal_async.assert_not_called()

    def test_equal_ten_cm_or_heading_above_five_degrees_still_sends_goal(self):
        for distance,angle in ((.10,0.),(.101,0.),(0.,5.1)):
            with self.subTest(distance=distance,angle=angle):
                pose=(self.goal[0]+distance,self.goal[1],self.goal[2]+math.radians(angle))
                probe=self.probe(pose)
                with self.assertRaisesRegex(RuntimeError,'Nav2 rejected'):
                    RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.)
                probe.client.send_goal_async.assert_called_once()
                self.assertTrue(probe.navigation_goal_attempted)

    def test_reported_second_run_starts_recognition_without_any_navigation_goal(self):
        pose=(3.102811336015215,1.4554707358659507,-1.5744025988979056)
        probe=self.probe(pose)
        result=RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.)
        self.assertTrue(result['already_at_A'])
        self.assertAlmostEqual(result['position_error_m'],.0518339,places=5)
        probe.client.send_goal_async.assert_not_called()

    def test_stale_or_collapsed_localization_is_not_replaced_by_saved_A(self):
        for issue in ('stale AMCL pose','AMCL particles collapsed'):
            with self.subTest(issue=issue):
                probe=self.probe(tuple(self.goal))
                probe.wait_pose.side_effect=RuntimeError(issue)
                with self.assertRaisesRegex(RuntimeError,issue):
                    RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.)
                probe.client.send_goal_async.assert_not_called()
                probe.localization_evidence.assert_not_called()

    def test_second_freshness_check_can_block_even_when_first_pose_was_at_A(self):
        probe=self.probe(tuple(self.goal))
        probe.wait_pose.side_effect=[tuple(self.goal),RuntimeError('new scan is stale')]
        with self.assertRaisesRegex(RuntimeError,'new scan is stale'):
            RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.)
        probe.client.wait_for_server.assert_called_once()
        probe.client.send_goal_async.assert_not_called()

    def test_check_only_remains_readiness_check_at_or_away_from_A(self):
        for pose in (tuple(self.goal),(self.goal[0]+1.,self.goal[1],self.goal[2])):
            with self.subTest(pose=pose):
                probe=self.probe(pose)
                result=RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.,check_only=True)
                self.assertEqual(result['status'],'READY_FOR_NAVIGATION')
                self.assertTrue(result['check_only'])
                self.assertEqual(result['current_pose_xyyaw'],list(pose))
                probe.client.send_goal_async.assert_not_called()

    def test_missing_action_server_is_not_bypassed_by_nearby_pose(self):
        probe=self.probe(tuple(self.goal));probe.client.wait_for_server.return_value=False
        with self.assertRaisesRegex(RuntimeError,'action server is unavailable'):
            RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.)
        probe.client.send_goal_async.assert_not_called()


if __name__=='__main__':
    unittest.main()
