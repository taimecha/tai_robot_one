"""Explicit current-place recognition must never request navigation motion."""
import copy
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

import r3_recognition_goal as goal_module
from r3_recognition_goal import RecognitionGoal,SCHEMA,map_identity


class RecognizeHereTest(unittest.TestCase):
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
        self.original=self.goal_file.read_bytes()

    def tearDown(self):
        self.assertEqual(self.goal_file.read_bytes(),self.original)
        self.directory.cleanup()

    def probe(self,pose):
        client=Mock()
        client.wait_for_server.side_effect=AssertionError('current-place mode discovered Nav2')
        client.send_goal_async.side_effect=AssertionError('current-place mode sent a goal')
        return SimpleNamespace(wait_pose=Mock(return_value=pose),client=client,
            map_match=dict(ready=True,inlier_ratio=.86,median_distance_m=.01),
            particle_gate=SimpleNamespace(metrics=dict(xy_rms_m=.03,yaw_std_deg=.7,
                unique_pose_bins=100,largest_pose_bin_mass=.02)),
            localization_evidence=Mock(return_value=dict(run_id='this-session',
                original_covariance_preserved=True,amcl_stamp_ros_s=123.)),
            navigation_goal_attempted=False,cancel=Mock())

    def test_current_place_requires_two_fresh_stationary_checks_without_any_action_client_call(self):
        measured=(3.0065882363,1.4133515625,-1.3845795118)
        probe=self.probe(measured)
        result=RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.,recognize_here=True)
        self.assertEqual(result['status'],'RECOGNIZING_HERE')
        self.assertEqual(result['reason'],'user_confirmed_current_recognition_place')
        self.assertEqual(result['current_pose_xyyaw'],list(measured))
        self.assertEqual(result['goal_pose_xyyaw'],self.goal)
        self.assertFalse(result['within_saved_A_limits'])
        self.assertGreater(result['position_error_m'],.05)
        self.assertGreater(result['heading_error_deg'],10.)
        self.assertFalse(result['navigation_goal_attempted'])
        self.assertEqual(result['robot_commands_from_this_process'],0)
        self.assertEqual([c.args[0] for c in probe.wait_pose.call_args_list],[30.,10.])
        self.assertEqual(probe.client.mock_calls,[])
        probe.cancel.assert_not_called()

    def test_small_error_is_reported_without_claiming_saved_A_was_reached(self):
        measured=(self.goal[0]+.031,self.goal[1],self.goal[2]+math.radians(.74))
        probe=self.probe(measured)
        result=RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.,recognize_here=True)
        self.assertEqual(result['status'],'RECOGNIZING_HERE')
        self.assertTrue(result['within_saved_A_limits'])
        self.assertEqual(result['current_pose_xyyaw'],list(measured))
        self.assertAlmostEqual(result['position_error_m'],.031)
        self.assertNotIn('arrival_pose_xyyaw',result)
        self.assertNotIn('already_at_A',result)

    def test_motion_stale_or_ambiguous_localization_cannot_be_bypassed(self):
        for reason in ('robot must stand still','stale LiDAR scan','AMCL particles collapsed'):
            for index in (0,1):
                with self.subTest(reason=reason,check=index):
                    probe=self.probe(tuple(self.goal))
                    probe.wait_pose.side_effect=([RuntimeError(reason)] if index==0
                        else [tuple(self.goal),RuntimeError(reason)])
                    with self.assertRaisesRegex(RuntimeError,reason):
                        RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.,recognize_here=True)
                    self.assertEqual(probe.client.mock_calls,[])
                    probe.cancel.assert_not_called()

    def test_modes_are_mutually_exclusive_in_api_and_cli(self):
        probe=self.probe(tuple(self.goal))
        with self.assertRaisesRegex(ValueError,'choose check-only or recognize-here'):
            RecognitionGoal.navigate(probe,self.map_file,self.goal_file,60.,
                                      check_only=True,recognize_here=True)
        probe.wait_pose.assert_not_called()
        with patch('sys.argv',['r3_recognition_goal.py','navigate','--map',str(self.map_file),
                '--output',str(self.folder/'result.json'),'--check-only','--recognize-here']), \
                patch('sys.stderr',io.StringIO()),patch.object(goal_module.rclpy,'init') as initialize:
            with self.assertRaises(SystemExit) as failure:
                goal_module.main()
        self.assertEqual(failure.exception.code,2)
        initialize.assert_not_called()

    def test_cli_saves_current_place_success_and_does_not_rewrite_A(self):
        output=self.folder/'result.json'
        report=dict(status='RECOGNIZING_HERE',current_pose_xyyaw=[3.,1.4,-1.38],
            goal_pose_xyyaw=self.goal,within_saved_A_limits=False,
            navigation_goal_attempted=False,robot_commands_from_this_process=0)
        node=SimpleNamespace(navigate=Mock(return_value=copy.deepcopy(report)),
            cancel=Mock(),destroy_node=Mock(),navigation_data_waits=[],navigation_result=None)
        with patch('sys.argv',['r3_recognition_goal.py','navigate','--map',str(self.map_file),
                '--goal',str(self.goal_file),'--output',str(output),'--recognize-here',
                '--continuous-localization']),patch('sys.stdout',io.StringIO()), \
                patch.object(goal_module,'RecognitionGoal',return_value=node), \
                patch.object(goal_module.rclpy,'init'),patch.object(goal_module.rclpy,'shutdown'), \
                patch.object(goal_module.rclpy,'ok',return_value=True):
            self.assertEqual(goal_module.main(),0)
        self.assertEqual(json.loads(output.read_text()),report)
        self.assertEqual(node.navigate.call_args.kwargs,
                         dict(check_only=False,recognize_here=True))


if __name__=='__main__':
    unittest.main()
