"""Brake before the AMCL onset bound, then resume only on a fresh posterior."""
import copy
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from r3_nav_localization_gate import OnsetGate,NavigationGate,ZERO


class NavigationGateTest(unittest.TestCase):
    def state(self,now=100.,pose=(0.,0.,0.),twist=(0.,0.,0.),*,live=False):
        status=dict(run_id='one-session',map_file='/maps/edited.yaml',published_ros_s=now,
            ready=True,continuous_localization=True,navigation_monitor=True,
            live_observation_validated=live,stationary_cache=dict(validated=not live),
            stationary_snapshot=dict(amcl_stamp_s=now-.1 if live else 80.))
        return dict(stamp=now,pose=list(pose),twist=list(twist)),status

    def prediction(self,status,now,start=100.):
        status['stationary_cache']['validated']=False
        status['motion_prediction_validated']=True
        status['motion_prediction']=dict(validated=True,ended=False,source_amcl_stamp_s=80.,
            started_ros_s=start,anchor_odom_xyyaw=[0.,0.,0.],
            max_duration_s=2.,max_distance_m=.08,max_yaw_rad=.08)

    def test_stationary_start_is_allowed_without_moving_or_redating_its_pose(self):
        gate=OnsetGate();odom,status=self.state();before=copy.deepcopy(status)
        command,reason,_=gate.filter((0.,0.,.22),odom,status,100.)
        self.assertEqual(command,(0.,0.,.22))
        self.assertIn('bounded',reason)
        self.assertEqual(status,before)

    def test_current_speed_braking_distance_is_reserved_before_yaw_limit(self):
        gate=OnsetGate();odom,status=self.state()
        gate.filter((0.,0.,.22),odom,status,100.)
        odom,status=self.state(100.5,(0.,0.,.04),(0.,0.,.18))
        self.prediction(status,100.5)
        command,reason,evidence=gate.filter((0.,0.,.22),odom,status,100.5)
        self.assertEqual(command,ZERO)
        self.assertIn('pausing',reason)
        self.assertLess(evidence['yaw_change_rad'],.08)
        self.assertGreater(evidence['predicted_stop_yaw_rad'],.08)

    def test_smoother_deceleration_stops_rotation_within_the_original_bound(self):
        gate=OnsetGate();yaw=velocity=0.;held=False
        for i in range(36):
            now=100.+i*.05
            odom,status=self.state(now,(0.,0.,yaw),(0.,0.,velocity))
            if i:self.prediction(status,now)
            output,_,_=gate.filter((0.,0.,.22),odom,status,now)
            target=min(.2,output[2])
            velocity+=max(-.4*.05,min(.4*.05,target-velocity))
            yaw+=velocity*.05
            held|=output==ZERO
            self.assertLess(yaw,.08)
        self.assertTrue(held)
        self.assertAlmostEqual(velocity,0.)

    def test_translation_also_brakes_inside_its_eight_centimeter_bound(self):
        gate=OnsetGate();x=velocity=0.;held=False
        for i in range(36):
            now=100.+i*.05
            odom,status=self.state(now,(x,0.,0.),(velocity,0.,0.))
            if i:self.prediction(status,now)
            output,_,_=gate.filter((.1,0.,0.),odom,status,now)
            target=output[0]
            velocity+=max(-.12*.05,min(.08*.05,target-velocity))
            x+=velocity*.05
            held|=output==ZERO
            self.assertLess(x,.08)
        self.assertTrue(held)

    def test_fresh_native_posterior_releases_the_hold_without_an_action_retry(self):
        gate=OnsetGate();odom,status=self.state()
        gate.filter((0.,0.,.22),odom,status,100.)
        odom,status=self.state(100.8,(0.,0.,.06),(0.,0.,.1))
        self.prediction(status,100.8)
        self.assertEqual(gate.filter((0.,0.,.22),odom,status,100.8)[0],ZERO)
        odom,status=self.state(100.9,(0.,0.,.065),(0.,0.,.05),live=True)
        output,reason,_=gate.filter((0.,0.,.22),odom,status,100.9)
        self.assertEqual(output,(0.,0.,.22))
        self.assertIn('fresh AMCL',reason)
        self.assertIsNone(gate.onset)

    def test_stale_faulted_or_changed_session_evidence_cannot_command_motion(self):
        gate=OnsetGate();odom,status=self.state(live=True)
        gate.filter((.1,0.,0.),odom,status,100.)
        cases=[dict(status,ready=False),dict(status,published_ros_s=99.),
            dict(status,run_id='another-session'),dict(status,navigation_monitor=False)]
        for value in cases:
            self.assertEqual(gate.filter((.1,0.,0.),odom,value,100.)[0],ZERO)
        self.assertEqual(gate.filter((.1,0.,0.),dict(odom,stamp=99.),status,100.)[0],ZERO)

    def test_stopping_does_not_rearm_an_expired_onset(self):
        gate=OnsetGate();odom,status=self.state()
        gate.filter((0.,0.,.22),odom,status,100.)
        odom,status=self.state(102.1,(0.,0.,.04))
        output,_,_=gate.filter((0.,0.,.22),odom,status,102.1)
        self.assertEqual(output,ZERO)
        self.assertEqual(gate.onset['started'],100.)

    def test_no_commands_during_startup_but_stale_input_after_motion_publishes_zero(self):
        node=SimpleNamespace(now=lambda:100.,command_stamp=None,command=None,
            gate=OnsetGate(),odom=None,status=None,output=Mock(),report=Mock(),
            last_reason=None,last_report=0.,get_logger=Mock(return_value=Mock()))
        NavigationGate.tick(node)
        node.output.publish.assert_not_called()
        node.command_stamp=99.;node.command=(.1,0.,.2)
        NavigationGate.tick(node)
        node.output.publish.assert_called_once()
        command=node.output.publish.call_args.args[0]
        self.assertEqual((command.linear.x,command.linear.y,command.angular.z),ZERO)


if __name__=='__main__':unittest.main()
