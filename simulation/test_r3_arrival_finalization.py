"""Arrival, runtime map fit and exact-A correction after native success."""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace,MethodType
import unittest
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from r3_recognition_goal import (RecognitionGoal,SCHEMA,map_identity,validate_a_scan_match,
                                  MAX_A_FINALIZATION_ATTEMPTS)
from r3_scan_map_check import validate_recognition_scan_match


class ArrivalFinalizationTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.folder=Path(self.temp.name)
        (self.folder/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
        self.map=self.folder/'map.yaml'
        self.map.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
        self.goal=(1.,2.,.3)
        self.file=self.folder/'A.json'
        self.file.write_text(json.dumps(dict(schema=SCHEMA,purpose='pallet_recognition_A',
            frame_id='map',goal_id='immutable-A',saved_at_utc='2026-11-15T00:00:00Z',
            pose_xyyaw=list(self.goal),map=map_identity(self.map))))
        self.original=self.file.read_bytes()

    def tearDown(self):
        self.assertEqual(self.file.read_bytes(),self.original)
        self.temp.cleanup()

    def probe(self,terminal_poses,action_status=4):
        complete=SimpleNamespace(done=lambda:True)
        handle=SimpleNamespace(accepted=True,get_result_async=lambda:complete,cancel_goal_async=Mock())
        result=SimpleNamespace(status=action_status)
        start=(.8,2.,.3)
        client=Mock();client.wait_for_server.return_value=True
        now=SimpleNamespace(to_msg=lambda:Time(sec=100),nanoseconds=100000000000)
        node=SimpleNamespace(continuous_localization=False,
            wait_pose=Mock(side_effect=[start,start,*terminal_poses]),client=client,
            get_clock=lambda:SimpleNamespace(now=lambda:now),
            await_future=Mock(side_effect=[item for _ in terminal_poses for item in (handle,result)]),
            goal_handle=None,result_future=None,localization_evidence=Mock(return_value={}),
            map_match=dict(ready=True,inlier_ratio=.79,median_distance_m=.01),
            particle_gate=SimpleNamespace(metrics={}))
        node.cancel=MethodType(RecognitionGoal.cancel,node)
        return node,handle

    def test_good_terminal_pose_starts_recognition_without_an_extra_goal(self):
        node,handle=self.probe([(1.02,2.,.31)])
        result=RecognitionGoal.navigate(node,self.map,self.file,30.)
        self.assertEqual(result['status'],'AT_NEW_A')
        node.client.send_goal_async.assert_called_once()
        handle.cancel_goal_async.assert_not_called()
        self.assertFalse(node.navigation_action_active)

    def test_settled_heading_inside_five_degrees_does_not_trigger_another_goal(self):
        node,handle=self.probe([(1.02,2.,.3+math.radians(4.05))])
        result=RecognitionGoal.navigate(node,self.map,self.file,30.)
        self.assertEqual(result['status'],'AT_NEW_A')
        self.assertEqual(node.client.send_goal_async.call_count,1)
        self.assertAlmostEqual(result['arrival_attempts'][0]['heading_error_deg'],4.05)
        self.assertLess(result['heading_error_deg'],5.)
        self.assertEqual(result['arrival_attempts'][0]['command_goal_pose_xyyaw'],list(self.goal))
        for call in node.client.send_goal_async.call_args_list:
            pose=call.args[0].pose
            self.assertEqual(pose.header.frame_id,'map')
        handle.cancel_goal_async.assert_not_called()

    def test_terminal_errors_cannot_trigger_unlimited_adjustments(self):
        self.assertEqual(MAX_A_FINALIZATION_ATTEMPTS,1)
        node,_=self.probe([(1.02,2.,.41)]*MAX_A_FINALIZATION_ATTEMPTS)
        with self.assertRaisesRegex(RuntimeError,'outside A arrival limits'):
            RecognitionGoal.navigate(node,self.map,self.file,30.)
        self.assertEqual(node.client.send_goal_async.call_count,MAX_A_FINALIZATION_ATTEMPTS)

    def test_large_terminal_error_and_native_abort_do_not_trigger_a_correction(self):
        # Large errors and real native failures must not be treated as the
        # small stationary finalization cases accepted by this workflow.
        for pose,status in [((1.2,2.,.3),4),((1.02,2.,.31),6)]:
            with self.subTest(status=status):
                node,_=self.probe([pose],status)
                with self.assertRaises(RuntimeError):RecognitionGoal.navigate(node,self.map,self.file,30.)
                node.client.send_goal_async.assert_called_once()

    def test_inside_A_region_only_heading_is_corrected_without_an_xy_loop(self):
        start=(1.02,2.,.3+math.radians(6.05))
        end=(1.02,2.,.31)
        node,_=self.probe([end])
        node.wait_pose=Mock(side_effect=[start,start,end])
        result=RecognitionGoal.navigate(node,self.map,self.file,30.)
        self.assertEqual(result['status'],'AT_NEW_A')
        node.client.send_goal_async.assert_called_once()
        self.assertTrue(node.navigation_start_evidence['heading_only_within_A'])
        self.assertEqual(node.navigation_start_evidence['command_goal_pose_xyyaw'],[1.02,2.,.3])
        self.assertEqual(result['goal_pose_xyyaw'],list(self.goal))

    def test_runtime_tolerates_unmapped_returns_but_teaching_remains_stricter(self):
        for ratio in [.65,.70,.79,.80]:
            fit=dict(ready=True,inlier_ratio=ratio,median_distance_m=.01)
            validate_a_scan_match(fit,teaching=False)
            validate_recognition_scan_match(fit)
            if ratio<.80:
                with self.assertRaises(ValueError):validate_a_scan_match(fit)

    def test_poor_walls_missing_fit_and_nonfinite_values_remain_rejected(self):
        for fit in [dict(ready=True,inlier_ratio=.649,median_distance_m=.01),
                    dict(ready=True,inlier_ratio=.95,median_distance_m=.051),
                    dict(ready=False,inlier_ratio=.95,median_distance_m=.01),
                    dict(ready=True,inlier_ratio=float('nan'),median_distance_m=.01),
                    dict(ready=True,inlier_ratio=.95,median_distance_m=-.01),{}]:
            with self.subTest(fit=fit),self.assertRaises(ValueError):validate_recognition_scan_match(fit)


if __name__=='__main__':unittest.main()
